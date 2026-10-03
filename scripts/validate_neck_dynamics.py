"""Forward head-release comparison with source mechanics and calibrated feedback.

Import the public DARUS-1132 v1.1 ExtractedTrajectories directory once with
--import-data. Normal runs use the checksum-pinned fixture and need no network.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import myo_sim  # noqa: E402
from scripts.validate_neck import VALIDATION, coupling_matrix  # noqa: E402

FIXTURE = VALIDATION / "falling_heads.npz"
DT = 0.000125
DELAY = 0.025
THRESHOLD = 0.05
RELEASE_DELAY = 0.004
WINDOW = (0.020, 0.250)
CONDITIONS = ("supine", "prone")
PHYSIOLOGY = VALIDATION / "neck_physiology.json"
CALIBRATION = VALIDATION / "neck_feedback_calibration.json"
CORRESPONDENCE = VALIDATION / "controller_muscle_families.json"
DEFAULT_GAIN = 3.5


def export_physiology(source: Path) -> None:
    """Export anatomical fiber data and the actual source tendon curve once."""
    import opensim

    provenance = json.loads((VALIDATION / "provenance.json").read_text())
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    if source_hash != provenance["files"][f"osim/Neck6D/{source.name}"]:
        raise ValueError("Pinned OpenSim source checksum mismatch")
    xml = ET.parse(source)
    source_model = opensim.Model(str(source))
    source_model.initSystem()
    strain = np.linspace(0, 0.2, 2001)
    muscles = {}
    force = None
    for muscle in xml.findall(".//Millard2012EquilibriumMuscle"):
        name = muscle.get("name")
        if name.endswith("_L"):
            continue
        instance = opensim.Millard2012EquilibriumMuscle.safeDownCast(source_model.getMuscles().get(name))
        curve = instance.getTendonForceLengthCurve()
        measured = np.array([curve.calcValue(1 + float(value)) for value in strain])
        if force is not None and not np.allclose(measured, force, atol=1e-12, rtol=0):
            raise ValueError("Source tendon curves differ; need per-muscle curve fixtures")
        force = measured
        muscles[name] = {
            key: float(muscle.findtext(key))
            for key in ("optimal_fiber_length", "tendon_slack_length", "pennation_angle_at_optimal", "max_isometric_force")
        }
    np.savez_compressed(PHYSIOLOGY.with_suffix(".npz"), strain=strain, normalized_force=force)
    content = {
        "source_sha256": source_hash,
        "opensim_version": opensim.GetVersionAndDate(),
        "muscles": muscles,
        "tendon_curve_sha256": hashlib.sha256(PHYSIOLOGY.with_suffix(".npz").read_bytes()).hexdigest(),
        "units": {"length": "m", "pennation": "rad", "force": "N"},
    }
    PHYSIOLOGY.write_text(json.dumps(content, indent=2, allow_nan=False) + "\n")
    provenance.setdefault("derived_fixtures", {})[PHYSIOLOGY.name] = hashlib.sha256(PHYSIOLOGY.read_bytes()).hexdigest()
    (VALIDATION / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")


def load_physiology(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Map pinned right source physiology to generated bilateral actuators."""
    provenance = json.loads((VALIDATION / "provenance.json").read_text())
    if hashlib.sha256(PHYSIOLOGY.read_bytes()).hexdigest() != provenance["derived_fixtures"][PHYSIOLOGY.name]:
        raise ValueError("Source physiology checksum mismatch")
    fixture = json.loads(PHYSIOLOGY.read_text())
    properties = []
    for i in range(model.nu):
        name = model.actuator(i).name.removeprefix("myohead_")[:-2]
        muscle = fixture["muscles"][name]
        l0 = muscle["optimal_fiber_length"]
        properties.append(
            (
                muscle["tendon_slack_length"],
                l0 * np.sin(muscle["pennation_angle_at_optimal"]),
                muscle["max_isometric_force"],
                l0,
            )
        )
    curve_path = PHYSIOLOGY.with_suffix(".npz")
    if hashlib.sha256(curve_path.read_bytes()).hexdigest() != fixture["tendon_curve_sha256"]:
        raise ValueError("Source tendon curve checksum mismatch")
    with np.load(curve_path, allow_pickle=False) as curve:
        return np.array(properties).T, curve["normalized_force"], curve["strain"]


def fiber_estimate(
    length: np.ndarray, tension: np.ndarray, properties: np.ndarray, force_curve: np.ndarray, strain_curve: np.ndarray
) -> np.ndarray:
    """Estimate source CE length from tension, series-elastic strain and pennation.

    This observer does not replace MuJoCo's rigid-tendon force law.
    """
    slack, width, maximum_force, _ = properties
    normalized_force = np.maximum(tension, 0) / maximum_force
    if np.any(normalized_force > force_curve[-1]):
        raise ValueError("Tendon observer exceeds exported source curve")
    strain = np.interp(normalized_force, force_curve, strain_curve)
    parallel = length - slack * (1 + strain)
    if np.any(parallel <= 0):
        raise ValueError("Tendon observer produces nonpositive parallel fiber length")
    return np.sqrt(parallel**2 + width**2)


def controller_muscles(model: mujoco.MjModel) -> np.ndarray:
    """Activate only anatomical families shared with DARUS-1145's neck list."""
    provenance = json.loads((VALIDATION / "provenance.json").read_text())
    if hashlib.sha256(CORRESPONDENCE.read_bytes()).hexdigest() != provenance["derived_fixtures"][CORRESPONDENCE.name]:
        raise ValueError("Controller family correspondence checksum mismatch")
    fixture = json.loads(CORRESPONDENCE.read_text())
    mapping = fixture["source_to_published_family"]
    if not set(mapping.values()).issubset(fixture["published_families"]):
        raise ValueError("Controller family absent from published EHTM neck list")
    return np.array([model.actuator(i).name.removeprefix("myohead_")[:-2] in mapping for i in range(model.nu)])


def graded_control(delta_length: np.ndarray, optimal_length: np.ndarray, eligible: np.ndarray, gain: float) -> np.ndarray:
    """Wochner Eq. 13: bounded, graded spindle feedback about relaxed length."""
    return np.clip(gain * delta_length / optimal_length, 0, 1) * eligible


def participant_reference(experiment: dict[str, np.ndarray], ids: np.ndarray) -> dict[str, np.ndarray]:
    """Compute the same descriptive corridor for a specified participant subset."""
    reference = {"time": experiment["time"]}
    for condition in CONDITIONS:
        for quantity in ("drop_m", "rotation_rad"):
            values = experiment[f"{condition}_{quantity}"][:, ids - 1]
            reference[f"{condition}_{quantity}"] = values
            reference[f"{condition}_{quantity}_mean"] = values.mean(axis=1)
            reference[f"{condition}_{quantity}_sd"] = values.std(axis=1, ddof=1)
    return reference


def calibration_inputs() -> dict[str, str]:
    """Pin measurements, source observer and production mechanics used for fitting."""
    return {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (
            FIXTURE,
            PHYSIOLOGY,
            CORRESPONDENCE,
            myo_sim.MODELS_DIR / "head/assets/myohead_chain.xml",
            myo_sim.MODELS_DIR / "head/assets/myohead_r_muscle.xml",
            myo_sim.MODELS_DIR / "head/assets/myohead_assets.xml",
            ROOT / "scripts/validate_neck_dynamics.py",
        )
    }


def calibrate_feedback(experiment: dict[str, np.ndarray]) -> dict:
    """Select the least tested gain passing all odd-participant training criteria.

    Even participants are not accessed by this selection function. Earlier
    full-cohort exploratory work means this is not a blinded validation study.
    """
    reference = participant_reference(experiment, np.arange(1, 18, 2))
    candidates = []
    selected = None
    for gain in np.arange(2, 20.001, 0.25):
        cases = {
            condition: agreement(
                simulate("myohead", condition, True, experiment["time"], gain=float(gain)), reference, condition
            )
            for condition in CONDITIONS
        }
        passed = all(metric["pass"] for case in cases.values() for metric in case.values())
        candidates.append({"gain": float(gain), "training_agreement": cases, "pass": passed})
        print(f"Training-only gain {gain:g}: {'pass' if passed else 'fail'}", flush=True)
        if passed:
            selected = float(gain)
            break
    if selected is None:
        raise ValueError("No gain passes training; do not relax acceptance or select using evaluation participants")
    report = {
        "selected_gain": selected,
        "training_participants": list(range(1, 18, 2)),
        "evaluation_participants": list(range(2, 18, 2)),
        "input_sha256": calibration_inputs(),
        "selection": "Smallest gain on [2,20] at 0.25 increments passing all four training RMS criteria",
        "blinded": False,
        "reason": "Earlier model development inspected full-cohort curves",
        "candidates": candidates,
    }
    CALIBRATION.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report


def load_calibration() -> dict:
    """Reject stale controller calibration after changes to data or mechanics."""
    report = json.loads(CALIBRATION.read_text())
    if report["input_sha256"] != calibration_inputs():
        raise ValueError("Stale feedback calibration; rerun --calibrate")
    return report


def import_data(directory: Path) -> None:
    """Preserve raw marker coordinates and headers, verifying publisher MD5s."""
    metadata = json.loads(FIXTURE.with_suffix(".json").read_text())
    arrays = {}
    for name, digest in metadata["source_md5"].items():
        path = directory / name
        if hashlib.md5(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Publisher checksum mismatch: {name}")
        if name == "time.txt":
            arrays["time"] = np.loadtxt(path, delimiter=",")
        else:
            header = path.read_text().splitlines()[0].rstrip(",").split(",")
            values = np.genfromtxt(path, delimiter=",", skip_header=1)
            if values.shape != (150, 51) or not np.isfinite(values).all():
                raise ValueError(f"Invalid trajectory array: {name}")
            if "trials" in arrays and not np.array_equal(arrays["trials"], header):
                raise ValueError("Marker trial ordering differs")
            arrays["trials"] = np.array(header)
            arrays[name.removesuffix(".txt")] = values
    np.savez_compressed(FIXTURE, **arrays)
    metadata["fixture_sha256"] = hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
    FIXTURE.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")


def load_experiment() -> dict[str, np.ndarray]:
    """Load measured COM-proxy drop and unsigned marker rotation by participant."""
    metadata = json.loads(FIXTURE.with_suffix(".json").read_text())
    if hashlib.sha256(FIXTURE.read_bytes()).hexdigest() != metadata["fixture_sha256"]:
        raise ValueError("Experimental fixture checksum mismatch")
    with np.load(FIXTURE, allow_pickle=False) as raw:
        time = raw["time"].ravel()
        result = {"time": time[16:] - time[16]}
        for condition in CONDITIONS:
            x1, y1, x2, y2 = [
                raw[f"{axis}traj_allparticipants_{condition}_marker{marker}"] for marker in (1, 2) for axis in ("x", "y")
            ]
            drop = y2 - y2[16]
            vector = np.stack((x1 - x2, y1 - y2), axis=-1)
            cosine = np.sum(vector * vector[16], axis=-1) / (
                np.linalg.norm(vector, axis=-1) * np.linalg.norm(vector[16], axis=-1)
            )
            angle = np.arccos(np.clip(cosine, -1, 1))
            for quantity, values in (("drop_m", drop), ("rotation_rad", angle)):
                # Publisher A10 averages three repeats first, then participants.
                participants = values[16:].reshape(134, 3, 17).mean(axis=1)
                result[f"{condition}_{quantity}"] = participants
                result[f"{condition}_{quantity}_mean"] = participants.mean(axis=1)
                result[f"{condition}_{quantity}_sd"] = participants.std(axis=1, ddof=1)
    return result


def release_model(name: str, dt: float) -> mujoco.MjModel:
    """Fix the torso in a private spec; leave production assets untouched."""
    spec = myo_sim.load_spec(name)
    for element in list(spec.keys) + list(spec.sensors):
        spec.delete(element)
    for element in list(spec.actuators) + list(spec.tendons) + list(spec.equalities):
        if not element.name.startswith("myohead_"):
            spec.delete(element)
    for joint in list(spec.joints):
        if not joint.name.startswith("myohead_"):
            spec.delete(joint)
    spec.option.timestep = dt
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    model = spec.compile()
    if (model.nq, model.nu, model.neq) != (24, 72, 18):
        raise ValueError("Unexpected fixed-torso neck topology")
    return model


def reflex_control(strain: np.ndarray, previous: np.ndarray, eligible: np.ndarray) -> np.ndarray:
    """Table 5: trigger above 5%; stay on while delayed strain remains positive."""
    return (((strain > THRESHOLD) | ((strain > 0) & (previous > 0))) & eligible).astype(float)


def simulate(
    name: str,
    condition: str,
    reflex: bool,
    time: np.ndarray,
    dt: float = DT,
    *,
    controller: str = "lambda",
    gain: float = DEFAULT_GAIN,
) -> dict[str, np.ndarray | float]:
    """Integrate a free release; output coordinates never prescribe the response."""
    model = release_model(name, dt)
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0
    mujoco.mj_forward(model, data)
    head = model.body("head").id
    initial_position = data.xipos[head].copy()
    initial_rotation = data.xmat[head].reshape(3, 3).copy()
    sign = -1 if condition == "supine" else 1
    direction = initial_rotation @ np.array([sign, 0, 0])
    properties, force_curve, strain_curve = load_physiology(model)
    reference = fiber_estimate(data.actuator_length, -data.actuator_force, properties, force_curve, strain_curve)
    eligible = controller_muscles(model)
    history: deque[np.ndarray] = deque([np.zeros(model.nu)] * (round(DELAY / dt) + 1), maxlen=round(DELAY / dt) + 1)
    matrix = coupling_matrix(model)
    masters = [model.joint(f"myohead_{coordinate}").id for coordinate in ("pitch1", "roll1", "yaw1", "pitch2", "roll2", "yaw2")]
    master_q = model.jnt_qposadr[masters]
    duration = float(time[-1])
    rows = []
    diagnostics = []
    activation_integral = 0.0
    residual = 0.0
    first_activation = None
    peak_activation = 0.0
    warnings = np.zeros(len(data.warning), dtype=int)
    while data.time <= duration + dt:
        mujoco.mj_forward(model, data)
        rotation = initial_rotation.T @ data.xmat[head].reshape(3, 3)
        rows.append(
            (
                data.time,
                float((data.xipos[head] - initial_position) @ direction),
                float(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))),
            )
        )
        fiber = fiber_estimate(data.actuator_length, -data.actuator_force, properties, force_curve, strain_curve)
        diagnostics.append(
            np.r_[
                data.act,
                data.ctrl,
                data.actuator_force,
                (fiber - reference) / reference,
                matrix.T @ data.qfrc_actuator,
                matrix.T @ data.qfrc_passive,
                -matrix.T @ data.qfrc_bias,
                data.qpos[master_q],
            ]
        )
        residual = max(residual, float(np.max(np.abs(data.qpos - matrix @ data.qpos[master_q]))))
        if data.time < RELEASE_DELAY - dt / 2:
            data.time += dt
            continue
        model.opt.gravity[:] = 9.81 * direction
        history.append(fiber - reference)
        if reflex:
            if controller == "binary":
                data.ctrl[:] = reflex_control(history[0] / reference, data.ctrl, eligible)
            elif controller == "lambda":
                data.ctrl[:] = graded_control(history[0], properties[3], eligible, gain)
            else:
                raise ValueError(f"Unknown controller: {controller}")
            if first_activation is None and data.ctrl.any():
                first_activation = float(data.time)
        mujoco.mj_step(model, data)
        activation_integral += float(np.mean(data.act**2)) * dt
        peak_activation = max(peak_activation, float(data.act.max()))
        warnings = np.maximum(warnings, [w.number for w in data.warning])
        if not np.isfinite(data.qpos).all() or warnings.any():
            raise ValueError("Unstable release integration")
    trajectory = np.array(rows)
    return {
        "diagnostics": np.column_stack([np.interp(time, trajectory[:, 0], column) for column in np.array(diagnostics).T]),
        "activation_squared_integral_s": activation_integral,
        "drop_m": np.interp(time, trajectory[:, 0], trajectory[:, 1]),
        "rotation_rad": np.interp(time, trajectory[:, 0], trajectory[:, 2]),
        "max_coupling_residual_rad": residual,
        "first_stimulation_s": first_activation,
        "peak_activation": peak_activation,
    }


def agreement(simulation: dict, experiment: dict, condition: str) -> dict:
    """Descriptive one-SD RMS criterion, not a published confidence bound."""
    mask = (experiment["time"] >= WINDOW[0]) & (experiment["time"] <= WINDOW[1])
    result = {}
    for quantity in ("drop_m", "rotation_rad"):
        mean, sd = (experiment[f"{condition}_{quantity}_{stat}"][mask] for stat in ("mean", "sd"))
        measured = simulation[quantity][mask]
        rmse = float(np.sqrt(np.mean((measured - mean) ** 2)))
        spread = float(np.sqrt(np.mean(sd**2)))
        result[quantity] = {
            "rmse": rmse,
            "participant_sd_rms": spread,
            "normalized_rmse": rmse / spread,
            "within_one_sd_fraction": float(np.mean(np.abs(measured - mean) <= sd)),
            "simulated_window_max": float(measured.max()),
            "experimental_mean_window_max": float(mean.max()),
            "pass": rmse <= spread,
        }
    return result


def plot_results(experiment: dict, trajectories: dict, output: Path) -> None:
    """Plot measured participant variation alongside unfit forward predictions."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True)
    time = experiment["time"] * 1000
    baseline = np.load(VALIDATION / "dynamic_baseline.npz", allow_pickle=False)
    for column, condition in enumerate(CONDITIONS):
        for row, (quantity, factor, label) in enumerate(
            (("drop_m", 100, "Head drop (cm)"), ("rotation_rad", 180 / np.pi, "Head rotation (degrees)"))
        ):
            ax = axes[row, column]
            mean = experiment[f"{condition}_{quantity}_mean"] * factor
            sd = experiment[f"{condition}_{quantity}_sd"] * factor
            ax.plot(time, experiment[f"{condition}_{quantity}"] * factor, color="0.8", lw=0.5, alpha=0.6)
            ax.fill_between(time, mean - sd, mean + sd, color="steelblue", alpha=0.2, label="Participant mean ± SD")
            ax.plot(time, mean, color="steelblue", lw=2)
            for reflex, color, label_sim in ((False, "#b87900", "Passive"), (True, "#ae3345", "Corrected graded feedback")):
                ax.plot(
                    time,
                    trajectories[f"myofullbody_neck/{condition}/{reflex}"][quantity] * factor,
                    color=color,
                    lw=2,
                    label=label_sim,
                )
            ax.plot(
                time,
                baseline[f"myofullbody_neck/{condition}/True/{quantity}"] * factor,
                color="#ae3345",
                lw=1.3,
                ls="--",
                alpha=0.6,
                label="Previous transfer",
            )
            ax.axvspan(20, 250, color="0.5", alpha=0.04)
            ax.set(ylabel=label, xlim=(0, time[-1]))
            ax.grid(alpha=0.2)
            if row == 0:
                ax.set_title(condition.capitalize())
            else:
                ax.set_xlabel("Time after release trigger (ms)")
    axes[0, 0].legend(fontsize=8, loc="upper left")
    fig.suptitle(
        "Literature neck-release benchmark: Wochner et al. (2022)\nFixed torso; 17 participants × 3 trials; one calibrated feedback gain",
        fontsize=12,
    )
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    baseline.close()


def plot_diagnostics(trajectories: dict, model: mujoco.MjModel, output: Path) -> None:
    """Show the corrected stimulation and cervical torque balance for review."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time = load_experiment()["time"] * 1000
    fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
    muscle_names = ("stern_mast", "splen_cap_sklc6", "semi_cap_sklthx", "semi_cerv_c3thx", "long_col_c1thx")
    for column, condition in enumerate(CONDITIONS):
        values = trajectories[f"myofullbody_neck/{condition}/True"]["diagnostics"]
        for name in muscle_names:
            index = model.actuator(f"myohead_{name}_r").id
            axes[0, column].plot(time, values[:, index], label=name, lw=1.4)
        axes[0, column].set(title=condition.capitalize(), ylabel="Activation", ylim=(0, 1))
        for row, coordinate in enumerate((0, 3), start=1):
            for offset, label, style in (
                (288, "Muscles (active + passive)", "-"),
                (294, "Joint springs + damping", "--"),
                (300, "Gravity + Coriolis", ":"),
            ):
                axes[row, column].plot(time, values[:, offset + coordinate], style, label=label, lw=1.5)
            axes[row, column].axhline(0, color="0.7", lw=0.7)
            axes[row, column].set_ylabel(f"{'Upper' if coordinate == 0 else 'Lower'} generalized moment (Nm)")
        for ax in axes[:, column]:
            ax.grid(alpha=0.2)
            ax.set_xlim(0, time[-1])
        axes[2, column].set_xlabel("Time after release trigger (ms)")
    axes[0, 0].legend(fontsize=7, loc="upper left")
    axes[1, 0].legend(fontsize=7, loc="lower left")
    fig.suptitle(
        "Corrected neck feedback: activation and torque diagnostics\nMoments are conjugate to independent source coordinates",
        fontsize=12,
    )
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main() -> int:
    """Write reproducible trajectories, engineering checks and biological scores."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--import-data", type=Path)
    parser.add_argument("--export-physiology", type=Path)
    parser.add_argument("--calibrate", action="store_true")
    args = parser.parse_args()
    if args.export_physiology:
        export_physiology(args.export_physiology)
        return 0
    if args.import_data:
        import_data(args.import_data)
    experiment = load_experiment()
    calibration = calibrate_feedback(experiment) if args.calibrate else load_calibration()
    gain = calibration["selected_gain"]
    trajectories = {}
    diagnostic_model = release_model("myohead", DT)
    report = {
        "study_doi": "10.1186/s12938-022-00994-9",
        "dataset_doi": "10.18419/DARUS-1132",
        "fixture_sha256": hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
        "units": {"drop_m": "m", "rotation_rad": "rad", "time": "s"},
        "mujoco_version": mujoco.__version__,
        "timestep_s": DT,
        "comparison_window_s": WINDOW,
        "criterion": "For each direction, graded feedback displacement and rotation RMSE <= RMS participant SD",
        "criterion_origin": "Descriptive benchmark criterion chosen here, not a published tolerance",
        "controller": {
            "type": "graded fiber-length feedback, Wochner Eq. 13",
            "gain": gain,
            "binary_reference_threshold": THRESHOLD,
            "delay_s": DELAY,
            "release_delay_s": RELEASE_DELAY,
        },
        "calibration": {key: value for key, value in calibration.items() if key != "candidates"},
        "actuator_names": [diagnostic_model.actuator(i).name for i in range(72)],
        "diagnostic_columns": [
            *[f"activation/{i}" for i in range(72)],
            *[f"stimulation/{i}" for i in range(72)],
            *[f"tension_signed_N/{i}" for i in range(72)],
            *[f"fiber_strain/{i}" for i in range(72)],
            *[f"muscle_moment_Nm/{i}" for i in range(6)],
            *[f"passive_joint_moment_Nm/{i}" for i in range(6)],
            *[f"gravity_coriolis_moment_Nm/{i}" for i in range(6)],
            *[f"coordinate_rad/{i}" for i in range(6)],
        ],
        "cases": {},
    }
    for name in ("myohead", "myofullbody_neck"):
        for condition in CONDITIONS:
            for reflex in (False, True):
                key = f"{name}/{condition}/{reflex}"
                simulation = simulate(name, condition, reflex, experiment["time"], gain=gain)
                trajectories[key] = simulation
                metrics = agreement(simulation, experiment, condition)
                report["cases"][key] = {
                    "agreement": metrics,
                    **{k: v for k, v in simulation.items() if not isinstance(v, np.ndarray)},
                }
    convergence = {}
    for condition in CONDITIONS:
        key = f"myofullbody_neck/{condition}/True"
        fine = simulate("myofullbody_neck", condition, True, experiment["time"], DT / 2, gain=gain)
        convergence[condition] = {
            quantity: float(np.max(np.abs(fine[quantity] - trajectories[key][quantity])))
            for quantity in ("drop_m", "rotation_rad")
        }
    report["timestep_halving_max_difference"] = convergence
    report["composition_max_difference"] = {
        quantity: max(
            float(
                np.max(
                    np.abs(
                        trajectories[f"myohead/{condition}/{reflex}"][quantity]
                        - trajectories[f"myofullbody_neck/{condition}/{reflex}"][quantity]
                    )
                )
            )
            for condition in CONDITIONS
            for reflex in (False, True)
        )
        for quantity in ("drop_m", "rotation_rad")
    }
    report["engineering_pass"] = (
        all(x["drop_m"] < 0.001 and x["rotation_rad"] < np.deg2rad(1) for x in convergence.values())
        and all(x["max_coupling_residual_rad"] < 0.01 for x in report["cases"].values())
        and all(x < 1e-8 for x in report["composition_max_difference"].values())
    )
    report["experimental_pass"] = all(
        metric["pass"]
        for condition in CONDITIONS
        for metric in report["cases"][f"myofullbody_neck/{condition}/True"]["agreement"].values()
    )
    evaluation = participant_reference(experiment, np.array(calibration["evaluation_participants"]))
    report["evaluation_agreement"] = {
        condition: agreement(trajectories[f"myofullbody_neck/{condition}/True"], evaluation, condition)
        for condition in CONDITIONS
    }
    report["evaluation_pass"] = all(
        metric["pass"] for case in report["evaluation_agreement"].values() for metric in case.values()
    )
    report["binary_reference"] = {
        condition: agreement(
            simulate("myohead", condition, True, experiment["time"], controller="binary"), experiment, condition
        )
        for condition in CONDITIONS
    }
    (VALIDATION / "dynamic_results.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(
        VALIDATION / "dynamic_trajectories.npz",
        time_s=experiment["time"],
        **{
            key + "/" + quantity: value
            for key, sim in trajectories.items()
            for quantity, value in sim.items()
            if isinstance(value, np.ndarray)
        },
    )
    plot_results(experiment, trajectories, ROOT / "docs/images/validation/neck_dynamics.png")
    plot_diagnostics(trajectories, diagnostic_model, ROOT / "docs/images/validation/neck_dynamics_diagnostics.png")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["engineering_pass"] and report["experimental_pass"] and report["evaluation_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
