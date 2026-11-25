# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
#
# Adapted from MyoSuite PR #564 by Mark Colley (M-Colley), Apache-2.0
# (https://github.com/MyoHub/myosuite/pull/564).
"""Compare ``scale_spec`` on MyoLeg with OpenSim's own ModelScaler on Rajagopal2016.

OpenSim runs in its own interpreter and is not a dependency of myo_sim. The script writes OpenSim's
results to ``tests/data/opensim_scaling/reference.json`` (default pose, every muscle) and to
``pose_reference.json`` (a pose grid of six right-leg muscles, stored with the myo_sim half as a
baseline). ``tests/test_opensim_comparison.py`` and ``tests/test_opensim_pose_comparison.py`` then
compare them with ``scale_spec`` without OpenSim.

    python scripts/validate_opensim_scaling.py --opensim-python <python of an OpenSim 4.6 env> [--model Rajagopal2016.osim]
    python scripts/validate_opensim_scaling.py --refresh-myo-sim   # keep the OpenSim halves, remeasure myo_sim
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    import mujoco

DATA = Path(__file__).resolve().parents[1] / "tests" / "data" / "opensim_scaling"
MODEL_URL = "https://raw.githubusercontent.com/opensim-org/opensim-models/d9b05d470b1a481c222372c85b75772faf8f7792/Models/Rajagopal/Rajagopal2016.osim"
MODEL_SHA256 = "3f5c5f23e486073f2ad2aa4a4967ffe2fcdd582b1e355512bc54f70c36376bf4"
BODIES = (
    "pelvis",
    "femur_r",
    "tibia_r",
    "patella_r",
    "talus_r",
    "calcn_r",
    "toes_r",
    "femur_l",
    "tibia_l",
    "talus_l",
    "calcn_l",
)
POSITIONS = ("femur_r", "tibia_r", "talus_r", "calcn_r", "toes_r", "femur_l", "tibia_l", "talus_l", "calcn_l", "toes_l")

# Pose comparison: six right-leg muscles over a grid of hip, knee and ankle angles.
POSE_FIXTURE = DATA / "pose_fixture.json"
POSE_REFERENCE = DATA / "pose_reference.json"
POSE_TOLERANCES = DATA / "pose_tolerances.json"
COORDINATES = ("hip_flexion_r", "knee_angle_r", "ankle_angle_r")
MUSCLES = ("iliacus_r", "recfem_r", "vasmed_r", "soleus_r", "gasmed_r", "tibant_r")
POSE_BODIES = ("pelvis", "femur_r", "tibia_r", "talus_r", "calcn_r")
SOLE = np.array([[-0.04, -0.03, 0.0], [0.16, -0.03, 0.04], [0.16, -0.03, -0.04]])
METRICS = (
    "joint_centres_m",
    "sole_points_m",
    "sole_floor_offset_m",
    "moment_arms_m",
    "normalized_fibre_length",
    "normalized_tendon_force",
    "torque_nm",
)
# Power k of one length factor s in each metric: an exact similarity gives s**k.
LENGTH_POWER = {
    "joint_centres_m": 1,
    "sole_points_m": 1,
    "sole_floor_offset_m": 1,
    "moment_arms_m": 1,
    "normalized_fibre_length": 0,
    "normalized_tendon_force": 0,
    "torque_nm": 1,
}
# Deliberate errors: a factor 5 % off along one OpenSim axis of one segment, all three tibia
# factors 5 % off, the femur's x and y factors swapped, and the six muscles' lengthrange 3 % off.
FACTOR_ERROR = 1.05
LENGTHRANGE_ERROR = 1.03
MUTATED_SEGMENTS = ("pelvis", "femur_r", "tibia_r", "patella_r", "talus_r", "calcn_r", "torso")
MUTATIONS = ("tibia_r_xyz", "femur_r_swap_xy", "lengthrange", *(f"{s}_{axis}" for s in MUTATED_SEGMENTS for axis in "xyz"))


def model_path(given: str | None) -> Path:
    """The pinned Rajagopal2016 model, downloaded if no path is given; refuses changed bytes."""
    if given:
        path = Path(given)
    else:
        path = Path(tempfile.gettempdir()) / f"Rajagopal2016_{MODEL_SHA256[:12]}.osim"
        if not path.exists():
            with urllib.request.urlopen(MODEL_URL, timeout=60) as response:
                path.write_bytes(response.read())
    if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError(f"{path} does not match the pinned SHA-256.")
    return path


def opensim_half(path: Path) -> dict:
    """Scale the model with OpenSim's ModelScaler; runs inside the OpenSim interpreter."""
    import numpy as np
    import opensim as osim

    osim.Logger.setLevelString("error")
    fixture = json.loads((DATA / "fixture.json").read_text())
    output = {"opensim_version": osim.GetVersionAndDate(), "model_sha256": MODEL_SHA256, "cases": {}}
    body = osim.Body("probe", 10.0, osim.Vec3(0), osim.Inertia(0.1, 0.2, 0.25))
    body.scaleInertialProperties(osim.Vec3(1.2, 0.8, 1.1), True)
    output["inertia_probe"] = {"mass_kg": body.getMass(), "moments": body.getInertia().getMoments().to_numpy().tolist()}
    for case, factors in fixture["cases"].items():
        model = osim.Model(str(path))
        if factors:
            scaler = osim.ModelScaler()
            order = osim.ArrayStr()
            order.append("manualScale")
            scaler.setScalingOrder(order)
            scaler.setPreserveMassDist(False)
            for name, values in factors.items():
                scale = osim.Scale()
                scale.setSegmentName(name)
                scale.setScaleFactors(osim.Vec3(*values))
                scale.setApply(True)
                scaler.addScale(scale)
            if not scaler.processModel(model, "", -1.0):
                raise RuntimeError(f"OpenSim's ModelScaler failed for {case}.")
        state = model.initSystem()
        model.assemble(state)
        model.realizePosition(state)
        pelvis = model.getBodySet().get("pelvis")
        rows: dict = {"positions_m": {}, "mass_kg": {}, "inertia_moments": {}, "muscles": {}}
        for name in POSITIONS:
            location = model.getBodySet().get(name).findStationLocationInAnotherFrame(state, osim.Vec3(0), pelvis)
            rows["positions_m"][name] = np.array(location.to_numpy()).tolist()
        for name in BODIES:
            entry = model.getBodySet().get(name)
            rows["mass_kg"][name] = entry.getMass()
            rows["inertia_moments"][name] = entry.getInertia().getMoments().to_numpy().tolist()
        muscles = model.getMuscles()
        for index in range(muscles.getSize()):
            muscle = muscles.get(index)
            rows["muscles"][muscle.getName()] = {
                "optimal_fibre_length_m": muscle.getOptimalFiberLength(),
                "tendon_slack_length_m": muscle.getTendonSlackLength(),
                "path_length_m": muscle.getLength(state),
            }
        output["cases"][case] = rows
    return output


def pose_fixture() -> dict[str, Any]:
    """The synthetic factors, mass modes and pose grid of the pose comparison."""
    return json.loads(POSE_FIXTURE.read_text())


def _inertia_matrix(inertia: Any) -> list[list[float]]:
    moments = inertia.getMoments().to_numpy()
    xy, xz, yz = inertia.getProducts().to_numpy()
    return [
        [float(moments[0]), float(xy), float(xz)],
        [float(xy), float(moments[1]), float(yz)],
        [float(xz), float(yz), float(moments[2])],
    ]


def opensim_pose_half(path: Path) -> dict:
    """Scale the model with OpenSim's ModelScaler and sample the pose grid; runs inside OpenSim."""
    import opensim as osim

    osim.Logger.setLevelString("error")
    config = pose_fixture()
    output: dict = {
        "version": osim.GetVersionAndDate(),
        "python_version": platform.python_version(),
        "inertia_probe": {},
        "cases": {},
    }
    for mode in ("volume", "keep"):
        body = osim.Body("probe", 10.0, osim.Vec3(0), osim.Inertia(0.1, 0.2, 0.25))
        body.scaleInertialProperties(osim.Vec3(1.2, 0.8, 1.1), mode == "volume")
        output["inertia_probe"][mode] = {"mass_kg": body.getMass(), "inertia_kgm2": _inertia_matrix(body.getInertia())}
    for case, recipe in config["cases"].items():
        model = osim.Model(str(path))
        factors = recipe["factors"]
        if factors:
            scaler = osim.ModelScaler()
            order = osim.ArrayStr()
            order.append("manualScale")
            scaler.setScalingOrder(order)
            scaler.setPreserveMassDist(recipe["mass"] == "keep")
            for name, values in factors.items():
                scale = osim.Scale()
                scale.setSegmentName(name)
                scale.setScaleFactors(osim.Vec3(*values))
                scale.setApply(True)
                scaler.addScale(scale)
            if not scaler.processModel(model, "", -1.0):
                raise RuntimeError(f"OpenSim's ModelScaler failed for {case}.")
        state = model.initSystem()
        bodies = model.getBodySet()
        pelvis = bodies.get("pelvis")
        muscles = [model.getMuscles().get(name) for name in MUSCLES]
        coordinates = [model.getCoordinateSet().get(name) for name in COORDINATES]
        rows: dict = {name: [] for name in METRICS}
        sole = SOLE * np.array(factors.get("calcn_r", [1.0, 1.0, 1.0]))
        for angles in config["pose_grid_degrees"]:
            for coordinate, angle in zip(coordinates, angles):
                coordinate.setValue(state, np.deg2rad(angle), False)
            model.assemble(state)
            for muscle in muscles:
                muscle.setActivation(state, 1.0)
            model.equilibrateMuscles(state)
            model.realizeDynamics(state)
            centres = [
                bodies.get(n).findStationLocationInAnotherFrame(state, osim.Vec3(0), pelvis).to_numpy().tolist()
                for n in POSE_BODIES[1:4]
            ]
            points = [
                bodies.get("calcn_r").findStationLocationInAnotherFrame(state, osim.Vec3(*p), pelvis).to_numpy().tolist()
                for p in sole
            ]
            arms = np.array([[m.computeMomentArm(state, c) for c in coordinates] for m in muscles])
            forces = np.array([m.getTendonForce(state) for m in muscles])
            values = {
                "joint_centres_m": centres,
                "sole_points_m": points,
                "sole_floor_offset_m": -float(np.min(np.array(points)[:, 1])),
                "moment_arms_m": arms.tolist(),
                "normalized_fibre_length": [m.getNormalizedFiberLength(state) for m in muscles],
                "normalized_tendon_force": [f / m.getMaxIsometricForce() for f, m in zip(forces, muscles)],
                "torque_nm": (forces @ arms).tolist(),
            }
            for name, value in values.items():
                rows[name].append(value)
        rows["body_mass_kg"] = [bodies.get(n).getMass() for n in POSE_BODIES]
        rows["body_inertia_kgm2"] = [_inertia_matrix(bodies.get(n).getInertia()) for n in POSE_BODIES]
        rows["muscle_lengths_m"] = [[m.getOptimalFiberLength(), m.getTendonSlackLength()] for m in muscles]
        for record, wrap in (("pelvis_wrap_dimensions_m", "Gmax1_at_pelvis_r"), ("psoas_wrap_dimensions_m", "PS_at_brim_r")):
            cylinder = osim.WrapCylinder.safeDownCast(pelvis.getWrapObjectSet().get(wrap))
            rows[record] = [cylinder.get_radius(), cylinder.get_length()]
        rows["total_mass_kg"] = model.getTotalMass(state)
        fibre = np.array(rows["normalized_fibre_length"])
        rows["normalized_fibre_operating_ranges"] = np.column_stack([fibre.min(0), fibre.max(0)]).tolist()
        output["cases"][case] = rows
    return output


def uniform_factor(recipe: Mapping[str, Any]) -> float | None:
    """The one factor of a recipe that scales every segment alike, else None."""
    values = {float(v) for factors in recipe["factors"].values() for v in factors}
    return values.pop() if len(values) == 1 else None


def compared_cases() -> list[str]:
    """Fixture cases that ``scale_spec`` reproduces, nominal first.

    ``scale_spec`` scales mass with volume and ``total_mass`` rescales the whole subject, so
    OpenSim's preserved mass distribution has a counterpart only for one uniform factor: rescaling
    to the nominal mass keeps every body's mass. The anisotropic preserved-mass case stays in the
    reference.
    """
    return [
        name
        for name, recipe in pose_fixture()["cases"].items()
        if recipe["mass"] == "volume" or uniform_factor(recipe) is not None
    ]


def category(case: str) -> str:
    """Tolerance category of a fixture case: ``uniform`` or ``anisotropic``."""
    return "anisotropic" if uniform_factor(pose_fixture()["cases"][case]) is None else "uniform"


def write_scale_set(factors: Mapping[str, Sequence[float]], path: Path) -> Path:
    """Write segment factors as an OpenSim ScaleSet, each factor exactly."""
    document = ET.Element("OpenSimDocument", Version="40000")
    objects = ET.SubElement(ET.SubElement(document, "ScaleSet", name="opensim_scaling_fixture"), "objects")
    for segment, values in factors.items():
        scale = ET.SubElement(objects, "Scale", name=segment)
        ET.SubElement(scale, "scales").text = " ".join(repr(float(v)) for v in values)
        ET.SubElement(scale, "segment").text = segment
        ET.SubElement(scale, "apply").text = "true"
    ET.ElementTree(document).write(path, encoding="utf-8", xml_declaration=True)
    return path


def mutated_factors(factors: Mapping[str, Sequence[float]], mutation: str | None) -> dict[str, list[float]]:
    """Copy of ``factors`` with the deliberate factor error ``mutation`` applied."""
    out = {segment: [float(v) for v in values] for segment, values in factors.items()}
    if mutation in (None, "lengthrange"):
        return out
    if mutation == "femur_r_swap_xy":
        out["femur_r"][0], out["femur_r"][1] = out["femur_r"][1], out["femur_r"][0]
        return out
    segment, axis = mutation.rsplit("_", 1)
    values = out.setdefault(segment, [1.0, 1.0, 1.0])
    for index in range(3) if axis == "xyz" else ("xyz".index(axis),):
        values[index] *= FACTOR_ERROR
    return out


def build_case(recipe: Mapping[str, Any], mutation: str | None = None) -> mujoco.MjModel:
    """Scale MyoLeg through a ScaleSet of ``recipe``, with the deliberate error ``mutation`` if given."""
    import myo_sim
    from myo_sim import scale_spec

    if mutation is not None and mutation not in MUTATIONS:
        raise ValueError(f"Unknown mutation {mutation!r}.")
    spec = myo_sim.load_spec("myolegs")
    total_mass = None
    if recipe["mass"] == "keep":
        if uniform_factor(recipe) is None:
            raise ValueError("scale_spec keeps no per-body mass distribution.")
        nominal = spec.copy().compile()
        total_mass = float(nominal.body_mass[nominal.body_rootid == nominal.body("Full Body").id].sum())
    with tempfile.TemporaryDirectory() as folder:
        path = write_scale_set(mutated_factors(recipe["factors"], mutation), Path(folder) / "scale.xml")
        model = scale_spec(spec, path, total_mass=total_mass).compile()
    if mutation == "lengthrange":
        model.actuator_lengthrange[[model.actuator(name).id for name in MUSCLES]] *= LENGTHRANGE_ERROR
    return model


def _pose(model: mujoco.MjModel, angles: Sequence[float]) -> mujoco.MjData:
    """``angles`` on the coordinates, every coupling assembled, all muscles fully active."""
    import mujoco

    from myo_sim.scaling import assemble_qpos

    data = mujoco.MjData(model)
    qpos = model.qpos0.copy()
    for name, angle in zip(COORDINATES, angles):
        qpos[model.joint(name).qposadr[0]] = angle
    assemble_qpos(model, data, qpos)
    data.act[:] = 1.0
    mujoco.mj_forward(model, data)
    return data


def _inertia_in(model: mujoco.MjModel, body: int, frame: np.ndarray) -> np.ndarray:
    """Inertia of ``body`` about its centre of mass, in the axes of ``frame``."""
    import mujoco

    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, model.body_iquat[body])
    rot = frame @ rot.reshape(3, 3)
    return rot @ np.diag(model.body_inertia[body]) @ rot.T


def sample_case(model: mujoco.MjModel, calcn_factors: Sequence[float], *, moment_arm_step: float = 1e-5) -> dict[str, Any]:
    """Geometry, moment arms along the couplings and isometric muscle values over the pose grid.

    Args:
        model: Scaled MyoLeg.
        calcn_factors: Fixture factors of ``calcn_r``. They place the sole landmarks in that body's
            frame, with the same arithmetic as the OpenSim half, so the landmarks see only the
            calcaneus pose.
        moment_arm_step: Central finite-difference step (rad).

    Returns:
        Metric name -> values over the pose grid, and the records of :func:`subject_records`.
    """
    pelvis, calcn = model.body("pelvis").id, model.body("calcn_r").id
    muscles = np.array([model.actuator(name).id for name in MUSCLES])
    gain = model.actuator_gainprm[muscles]
    lengthrange = model.actuator_lengthrange[muscles]
    optimal = np.diff(lengthrange, axis=1)[:, 0] / (gain[:, 1] - gain[:, 0])
    slack = lengthrange[:, 0] - gain[:, 0] * optimal
    rows: dict[str, Any] = {name: [] for name in METRICS}
    for degrees in pose_fixture()["pose_grid_degrees"]:
        angles = np.deg2rad(degrees)
        data = _pose(model, angles)
        rot = data.xmat[pelvis].reshape(3, 3)
        centres = (data.xpos[[model.body(n).id for n in POSE_BODIES[1:4]]] - data.xpos[pelvis]) @ rot
        sole = SOLE * np.asarray(calcn_factors)
        points = (sole @ data.xmat[calcn].reshape(3, 3).T + data.xpos[calcn] - data.xpos[pelvis]) @ rot
        arms = []
        for index in range(len(angles)):
            step = np.eye(3)[index] * moment_arm_step
            longer, shorter = _pose(model, angles + step), _pose(model, angles - step)
            arms.append(-(longer.actuator_length[muscles] - shorter.actuator_length[muscles]) / (2 * moment_arm_step))
        arms = np.array(arms).T
        force = -data.actuator_force[muscles]
        values = {
            "joint_centres_m": centres.tolist(),
            "sole_points_m": points.tolist(),
            "sole_floor_offset_m": -float(points[:, 1].min()),
            "moment_arms_m": arms.tolist(),
            "normalized_fibre_length": ((data.actuator_length[muscles] - slack) / optimal).tolist(),
            "normalized_tendon_force": (force / gain[:, 2]).tolist(),
            "torque_nm": (force @ arms).tolist(),
        }
        for name, value in values.items():
            rows[name].append(value)
    ids = [model.body(name).id for name in POSE_BODIES]
    rows["body_mass_kg"] = model.body_mass[ids].tolist()
    # In each body's own frame, like OpenSim's Body inertia.
    rows["body_inertia_kgm2"] = [_inertia_in(model, body, np.eye(3)).tolist() for body in ids]
    rows["muscle_lengths_m"] = np.column_stack([optimal, slack]).tolist()
    for record, wrap in (
        ("pelvis_wrap_dimensions_m", "Gmax1_at_pelvis_wrap_r"),
        ("psoas_wrap_dimensions_m", "PS_at_brim_wrap_r"),
    ):
        size = model.geom(wrap).size
        rows[record] = [float(size[0]), float(2 * size[1])]
    rows["total_mass_kg"] = float(model.body_mass.sum())
    fibre = np.array(rows["normalized_fibre_length"])
    rows["normalized_fibre_operating_ranges"] = np.column_stack([fibre.min(0), fibre.max(0)]).tolist()
    return {**rows, **subject_records(model)}


def subject_records(model: mujoco.MjModel) -> dict[str, Any]:
    """Records that the OpenSim half has no counterpart for, kept for the self-regression.

    The root height of ``qpos0`` comes from ``scale_spec``'s ground alignment of the feet. The torso
    and the head attachment are given in the pelvis frame of the neutral pose.
    """
    pelvis, torso, head = (model.body(name).id for name in ("pelvis", "torso", "head_attach"))
    neutral = _pose(model, np.zeros(len(COORDINATES)))
    frame = neutral.xmat[pelvis].reshape(3, 3).T
    root = model.body("Full Body").id
    height = model.jnt_qposadr[model.body_jntadr[root]] + 2
    geoms = np.flatnonzero(model.geom_bodyid == torso)
    return {
        "root_heights_m": [float(model.qpos0[height]), *model.key_qpos[:, height].tolist()],
        "torso_mass_kg": float(model.body_mass[torso]),
        "torso_inertia_kgm2": _inertia_in(model, torso, frame @ neutral.xmat[torso].reshape(3, 3)).tolist(),
        "torso_geom_offsets_m": ((neutral.geom_xpos[geoms] - neutral.xpos[torso]) @ frame.T).tolist(),
        "head_attach_m": ((neutral.xpos[head] - neutral.xpos[pelvis]) @ frame.T).tolist(),
    }


def sample_myo_sim() -> dict[str, Any]:
    """The myo_sim half: every compared case scaled from its ScaleSet and sampled."""
    import mujoco

    import myo_sim

    cases = pose_fixture()["cases"]
    return {
        "provenance": {
            "platform": platform.platform(),
            "python_version": platform.python_version(),
            "mujoco_version": mujoco.__version__,
            "myo_sim_version": myo_sim.__version__,
        },
        "cases": {
            name: sample_case(build_case(cases[name]), cases[name]["factors"].get("calcn_r", [1.0, 1.0, 1.0]))
            for name in compared_cases()
        },
    }


def discrepancy(myo_sim: Mapping[str, Any], opensim: Mapping[str, Any], case: str, metric: str) -> np.ndarray:
    """Scaling discrepancy of ``metric`` in ``case``, with the nominal conversion error removed.

    For one factor ``s`` it is ``(m_s - o_s) - s**k (m_n - o_n)``, zero when both programs scale as
    exact similarities (``k`` from :data:`LENGTH_POWER`). For several factors it is
    ``(m_s - m_n) - (o_s - o_n)``. ``m`` and ``o`` are the myo_sim and OpenSim values, ``s`` and
    ``n`` the scaled and nominal cases.
    """
    m_s, o_s = (np.array(half["cases"][case][metric]) for half in (myo_sim, opensim))
    m_n, o_n = (np.array(half["cases"]["nominal"][metric]) for half in (myo_sim, opensim))
    s = uniform_factor(pose_fixture()["cases"][case])
    weight = 1.0 if s is None else s ** LENGTH_POWER[metric]
    return (m_s - o_s) - weight * (m_n - o_n)


def wrap_increment(myo_sim: Mapping[str, Any], opensim: Mapping[str, Any], case: str) -> float:
    """Change of the pelvis wrap cylinder's difference from OpenSim's, from nominal to ``case``."""
    record = "pelvis_wrap_dimensions_m"
    scaled = np.array(myo_sim["cases"][case][record]) - np.array(opensim["cases"][case][record])
    nominal = np.array(myo_sim["cases"]["nominal"][record]) - np.array(opensim["cases"]["nominal"][record])
    return float(np.abs(scaled - nominal).max())


def assessment(myo_sim: Mapping[str, Any], opensim: Mapping[str, Any]) -> dict[str, Any]:
    """Each compared case's largest discrepancy per metric against its bound in pose_tolerances.json."""
    limits = json.loads(POSE_TOLERANCES.read_text())
    measured, violations = {}, []
    for case in compared_cases()[1:]:
        values = {metric: float(np.abs(discrepancy(myo_sim, opensim, case, metric)).max()) for metric in METRICS}
        values["pelvis_wrap_dimensions_m"] = wrap_increment(myo_sim, opensim, case)
        bounds = {**limits[category(case)], "pelvis_wrap_dimensions_m": limits["pelvis_wrap_dimensions_m"]}
        violations += [(case, metric, value, bounds[metric]) for metric, value in values.items() if value >= bounds[metric]]
        measured[case] = values
    return {"measured": measured, "violations": violations}


def _dumps(value: Any, indent: int = 0) -> str:
    """JSON with one line per array, so the stored reference stays small and diffable."""
    if isinstance(value, dict) and value:
        pad = " " * (indent + 2)
        items = [f"{pad}{json.dumps(k)}: {_dumps(v, indent + 2)}" for k, v in value.items()]
        return "{\n" + ",\n".join(items) + "\n" + " " * indent + "}"
    return json.dumps(value, separators=(",", ":"), allow_nan=False)


def _opensim_worker(python: str, path: Path, flag: str) -> dict:
    command = [python, __file__, flag, "--model", str(path)]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(result.stdout.splitlines()[-1])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--opensim-python", help="Python of an environment with OpenSim 4.6: regenerate both OpenSim halves.")
    parser.add_argument(
        "--refresh-myo-sim", action="store_true", help="Keep the stored OpenSim halves; remeasure the myo_sim half."
    )
    parser.add_argument("--model", help="A local copy of the pinned Rajagopal2016.osim.")
    parser.add_argument("--opensim-half", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--opensim-pose-half", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.opensim_half or args.opensim_pose_half:
        half = opensim_half if args.opensim_half else opensim_pose_half
        print(json.dumps(half(model_path(args.model))))
        return
    if args.refresh_myo_sim:
        stored = json.loads(POSE_REFERENCE.read_text())
        if stored["fixture"] != pose_fixture():
            raise SystemExit("pose_fixture.json changed: regenerate the OpenSim half too.")
        opensim = stored["opensim"]
    elif args.opensim_python:
        path = model_path(args.model)
        reference = _opensim_worker(args.opensim_python, path, "--opensim-half")
        (DATA / "reference.json").write_text(json.dumps(reference, indent=1, sort_keys=True) + "\n")
        print(f"Wrote {DATA / 'reference.json'} with OpenSim {reference['opensim_version']}.")
        provenance = {"platform": platform.platform(), "model_url": MODEL_URL, "model_sha256": MODEL_SHA256}
        opensim = {"provenance": provenance, **_opensim_worker(args.opensim_python, path, "--opensim-pose-half")}
    else:
        parser.error("give --opensim-python or --refresh-myo-sim")
    reference = {"schema": 2, "fixture": pose_fixture(), "opensim": opensim, "myo_sim": sample_myo_sim()}
    POSE_REFERENCE.write_text(_dumps(reference) + "\n")
    result = assessment(reference["myo_sim"], reference["opensim"])
    print(f"Wrote {POSE_REFERENCE}.")
    if result["violations"]:
        raise SystemExit(f"Scaling discrepancies at or above their bounds: {result['violations']}")


if __name__ == "__main__":
    main()
