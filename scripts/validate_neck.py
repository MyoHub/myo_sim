"""Reproduce the HYOID neck source and neutral experimental-strength checks.

Run `uv run python scripts/validate_neck.py`. OpenSim is needed only for the
optional `--export-source` command, not for validation or normal model loading.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import linprog

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import myo_sim  # noqa: E402

VALIDATION = myo_sim.MODELS_DIR / "head" / "validation"
COORDINATES = ("pitch1", "roll1", "yaw1", "pitch2", "roll2", "yaw2")
REFERENCE = VALIDATION / "opensim_reference.npz"
LENGTH_TOLERANCE_M = 0.001
MOMENT_ARM_TOLERANCE_M = 0.001


def load_reference(reference: Path = REFERENCE) -> dict[str, np.ndarray]:
    """Reject altered fixtures and load numerical arrays without pickle."""
    metadata = json.loads(reference.with_suffix(".json").read_text())
    if hashlib.sha256(reference.read_bytes()).hexdigest() != metadata["fixture_sha256"]:
        raise ValueError("OpenSim reference checksum mismatch")
    with np.load(reference, allow_pickle=False) as fixture:
        return {key: fixture[key] for key in fixture.files}


def export_source(source: Path, output: Path) -> None:
    """Export signed, constraint-aware lengths and arms directly from OpenSim."""
    import opensim

    provenance = json.loads((VALIDATION / "provenance.json").read_text())
    expected = provenance["files"][f"osim/Neck6D/{source.name}"]
    if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
        raise ValueError("Pinned OpenSim source checksum mismatch")
    xml = ET.parse(source)
    model = opensim.Model(str(source))
    model.setUseVisualizer(False)
    state = model.initSystem()
    coordinates = model.getCoordinateSet()
    muscles = model.getMuscles()
    names = [muscles.get(i).getName() for i in range(muscles.getSize())]
    ranges = [np.fromstring(xml.find(f'.//Coordinate[@name="{name}"]/range').text, sep=" ") for name in COORDINATES]
    # Include neutral and both endpoints of every independent coordinate, jointly.
    poses = np.array(list(itertools.product(*[(low, 0.0, high) for low, high in ranges])))
    couplings = []
    for constraint in xml.findall(".//CoordinateCouplerConstraint"):
        function = constraint.find("./coupled_coordinates_function/SimmSpline")
        xs = np.fromstring(function.findtext("x"), sep=" ")
        ys = np.fromstring(function.findtext("y"), sep=" ")
        slope = (ys[1] - ys[0]) / (xs[1] - xs[0])
        couplings.append(
            (constraint.findtext("dependent_coordinate_name"), constraint.findtext("independent_coordinate_names"), slope)
        )
    lengths = np.empty((len(poses), len(names)))
    arms = np.empty((len(poses), len(names), len(COORDINATES)))
    for row, pose in enumerate(poses):
        for name, value in zip(COORDINATES, pose):
            coordinates.get(name).setValue(state, float(value), False)
        # Exact coupled q avoids the assembler perturbing the requested controls.
        for dependent, independent, slope in couplings:
            coordinates.get(dependent).setValue(state, slope * coordinates.get(independent).getValue(state), False)
        model.realizePosition(state)
        if row % 100 == 0:
            print(f"OpenSim export: {row}/{len(poses)}", flush=True)
        for i, name in enumerate(names):
            muscle = muscles.get(name)
            lengths[row, i] = muscle.getLength(state)
            arms[row, i] = [muscle.computeMomentArm(state, coordinates.get(coordinate)) for coordinate in COORDINATES]
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, poses=poses, muscles=names, lengths=lengths, moment_arms=arms)
    metadata = {
        "source_file": source.name,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "opensim_version": opensim.GetVersionAndDate(),
        "coordinates": COORDINATES,
        "units": {"poses": "rad", "lengths": "m", "moment_arms": "m/rad"},
        "moment_arm_sign": "-d(musculotendon length)/d(independent coordinate), including source couplings",
        "sampling": "Cartesian product of low, zero, high for all six independent coordinates",
        "pose_count": len(poses),
        "muscle_count": len(names),
        "fixture_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
    }
    output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")


def coupling_matrix(model: mujoco.MjModel) -> np.ndarray:
    """Map six independent cervical velocities to all compiled model velocities."""
    matrix = np.zeros((model.nv, len(COORDINATES)))
    for k, name in enumerate(COORDINATES):
        joint = model.joint(f"myohead_{name}").id
        matrix[model.jnt_dofadr[joint], k] = 1
        for equality in range(model.neq):
            if model.eq_type[equality] == mujoco.mjtEq.mjEQ_JOINT and model.eq_obj2id[equality] == joint:
                matrix[model.jnt_dofadr[model.eq_obj1id[equality]], k] = model.eq_data[equality, 1]
    return matrix


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, pose: np.ndarray, matrix: np.ndarray) -> None:
    """Prescribe only cervical positions; leave other model coordinates at reset."""
    data.qpos[:] = model.qpos0
    values = matrix @ pose
    for joint in range(model.njnt):
        if np.any(matrix[model.jnt_dofadr[joint]]):
            data.qpos[model.jnt_qposadr[joint]] = values[model.jnt_dofadr[joint]]
    mujoco.mj_forward(model, data)


def tendon_jacobian(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Return tendon derivatives, supporting the pre-3.9 and 3.9 sparse layouts."""
    if hasattr(model, "ten_J_rowadr"):
        rowadr, rownnz, colind = model.ten_J_rowadr, model.ten_J_rownnz, model.ten_J_colind
    elif data.ten_J.size == model.ntendon * model.nv:
        return np.asarray(data.ten_J).reshape(model.ntendon, model.nv)
    else:
        rowadr, rownnz, colind = data.ten_J_rowadr, data.ten_J_rownnz, data.ten_J_colind
    matrix = np.zeros((model.ntendon, model.nv))
    for tendon in range(model.ntendon):
        start, count = rowadr[tendon], rownnz[tendon]
        matrix[tendon, colind[start : start + count]] = data.ten_J[start : start + count]
    return matrix


def muscle_ids(model: mujoco.MjModel, names: np.ndarray) -> np.ndarray:
    """Map OpenSim bilateral muscle names to canonical composed actuators."""
    return np.array(
        [model.actuator(f"myohead_{name[:-2]}_l" if name.endswith("_L") else f"myohead_{name}_r").id for name in names]
    )


def source_agreement(model: mujoco.MjModel, reference: Path = REFERENCE) -> dict:
    """Compare the right source and its reflection, as required by repo mirroring."""
    fixture = load_reference(reference)
    ids = muscle_ids(model, fixture["muscles"])
    tendons = model.actuator_trnid[ids, 0]
    data = mujoco.MjData(model)
    matrix = coupling_matrix(model)
    length_error = 0.0
    arm_error = 0.0
    original_left_error = 0.0
    reflection = np.array([1, -1, -1, 1, -1, -1])
    pose_indices = {tuple(pose): i for i, pose in enumerate(fixture["poses"])}
    source_names = list(fixture["muscles"])
    left = np.array([name.endswith("_L") for name in source_names])
    right_indices = [source_names.index(name[:-2]) for name in np.array(source_names)[left]]
    for pose, source_lengths, source_arms in zip(fixture["poses"], fixture["lengths"], fixture["moment_arms"]):
        lengths, arms = source_lengths.copy(), source_arms.copy()
        reflected_index = pose_indices[tuple(pose * reflection)]
        lengths[left] = fixture["lengths"][reflected_index, right_indices]
        arms[left] = fixture["moment_arms"][reflected_index, right_indices] * reflection
        set_pose(model, data, pose, matrix)
        measured = -(tendon_jacobian(model, data)[tendons] @ matrix)
        if not np.all(np.isfinite(measured)) or not np.all(np.isfinite(data.ten_length[tendons])):
            raise ValueError("Nonfinite muscle lengths or moment arms")
        length_error = max(length_error, float(np.max(np.abs(data.ten_length[tendons] - lengths))))
        arm_error = max(arm_error, float(np.max(np.abs(measured - arms))))
        original_left_error = max(original_left_error, float(np.max(np.abs(measured[left] - source_arms[left]))))
    return {
        "poses": len(fixture["poses"]),
        "muscles": len(ids),
        "max_length_error_m": length_error,
        "length_tolerance_m": LENGTH_TOLERANCE_M,
        "max_signed_moment_arm_error_m": arm_error,
        "moment_arm_tolerance_m": MOMENT_ARM_TOLERANCE_M,
        "reference_mapping": "OpenSim right muscles; left is their sagittal reflection at the reflected pose",
        "original_asymmetric_left_max_moment_arm_difference_m": original_left_error,
        "pass": bool(length_error <= LENGTH_TOLERANCE_M and arm_error <= MOMENT_ARM_TOLERANCE_M),
    }


def neutral_strength(model: mujoco.MjModel) -> list[dict]:
    """Maximize helmet load while balancing all six cervical coordinates.

    Muscles are bounded at activation [0, 1]; passive forces and gravity are
    included. No reserve actuators or retuning against experimental values.
    """
    fixture = load_reference()
    ids = muscle_ids(model, fixture["muscles"])
    data = mujoco.MjData(model)
    matrix = coupling_matrix(model)
    set_pose(model, data, np.zeros(6), matrix)
    arms = -(tendon_jacobian(model, data)[model.actuator_trnid[ids, 0]] @ matrix).T
    active = np.array(
        [
            -mujoco.mju_muscleGain(
                data.actuator_length[i], 0, model.actuator_lengthrange[i], model.actuator_acc0[i], model.actuator_gainprm[i, :9]
            )
            for i in ids
        ]
    )
    passive = np.array(
        [
            -mujoco.mju_muscleBias(
                data.actuator_length[i], model.actuator_lengthrange[i], model.actuator_acc0[i], model.actuator_biasprm[i, :9]
            )
            for i in ids
        ]
    )
    head = model.body("head").id
    rotation = data.xmat[head].reshape(3, 3)
    point = data.xpos[head] + rotation @ np.array([0, 0.2, 0])
    translation = np.zeros((3, model.nv))
    angular = np.zeros_like(translation)
    mujoco.mj_jac(model, data, translation, angular, point, head)
    balance = matrix.T @ data.qfrc_bias - arms @ passive
    results = []
    with (VALIDATION / "neutral_strength.csv").open() as stream:
        experiments = list(csv.DictReader(stream))
    directions = {
        "flexion": (-1, 0, 0),
        "extension": (1, 0, 0),
        "lateral_bending": (0, 0, 1),
        "axial_rotation": (0, 1, 0),
    }
    if len(experiments) != 4 or {row["direction"] for row in experiments} != set(directions):
        raise ValueError("Experimental fixture must cover exactly four neutral loading directions")
    for experiment in experiments:
        direction = directions[experiment["direction"]]
        axial = experiment["direction"] == "axial_rotation"
        load = matrix.T @ (angular if axial else translation).T @ rotation @ np.array(direction)
        solution = linprog(
            np.r_[np.zeros(len(ids)), -1],
            A_eq=np.column_stack((arms * active, load)),
            b_eq=balance,
            bounds=[(0, 1)] * len(ids) + [(0, None)],
            method="highs",
        )
        if not solution.success:
            raise RuntimeError(f"No neutral equilibrium: {solution.message}")
        moment = float(solution.x[-1] * (1 if axial else 0.32))
        mean, sd = float(experiment["mean_Nm"]), float(experiment["sd_Nm"])
        results.append(
            {
                "direction": experiment["direction"],
                "model_Nm": moment,
                "experiment_mean_Nm": mean,
                "experiment_sd_Nm": sd,
                "within_one_sd": bool(abs(moment - mean) <= sd),
                "equilibrium_residual_Nm": float(np.max(np.abs(solution.eqlin.residual))),
            }
        )
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-source", type=Path)
    parser.add_argument("--output", type=Path, default=VALIDATION / "results.json")
    args = parser.parse_args()
    if args.export_source:
        export_source(args.export_source, REFERENCE)
        return
    results = {"mujoco_version": mujoco.__version__, "source_comparison": {}, "neutral_experimental_strength": {}}
    for name in ("myohead", "myofullbody_neck"):
        model = myo_sim.load_model(name)
        results["source_comparison"][name] = source_agreement(model)
        results["neutral_experimental_strength"][name] = neutral_strength(model)
    results["pass"] = all(row["pass"] for row in results["source_comparison"].values()) and all(
        row["within_one_sd"] for rows in results["neutral_experimental_strength"].values() for row in rows
    )
    args.output.write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))
    if not results["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
