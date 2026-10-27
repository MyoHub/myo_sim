"""Fit tendon-slack (length-range) shifts of the extrinsic hand muscles to measured passive moments.

Binder-Markey & Murray (2017) optimised the tendon slack lengths of FDSI, FDPI,
EDCI and EIP so that their passive force reproduced the extrinsic component of
Knutson's MCP passive moment over wrist and MCP angles. MyoHand muscles have
rigid tendons: the normalised fibre length is ``range[0] + (L - lr[0]) / L0``
with ``L0 = (lr[1] - lr[0]) / (range[1] - range[0])``, so a slack-length change
is a shift of ``actuator_lengthrange`` at constant ``L0``.

Two fits, each from the rows' ``reference_value_m``, with MuJoCo's own passive
force-length curve, MyoHand's muscle-tendon lengths and moment arms and
Binder-Markey's penalty on relative change:

- index (FDS2, FDP2, EDC2, EIP): least squares on Knutson's extrinsic MCP moment;
- the other extrinsic finger, thumb and wrist muscles (``joint_targets``): the
  total measured or transferred joint moments, with a one-sided loss (a moment
  beyond the total, or opposite to it, costs ``EXCESS_WEIGHT`` times a deficit).

It writes the absolute slack lengths (and the reference values they change) to
``build/data/muscle_passive.json`` and the shifted ``lengthrange`` of each muscle
to ``myoarm_r_muscle.xml``, the model's source of truth (the left arm is mirrored
from it).

Run: ``uv run python scripts/fit_muscle_passive.py`` (refits, rewrites the JSON and
the ``lengthrange`` attributes); ``--xml-only`` only writes the JSON slack lengths
into the XML.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any, Callable

import mujoco
import numpy as np

import myo_sim

DATA_PATH = Path(__file__).parents[1] / "myo_sim" / "build" / "data" / "muscle_passive.json"
MUSCLE_XML = myo_sim.MODELS_DIR / "arm" / "assets" / "myoarm_r_muscle.xml"

KNUTSON_EXTRINSIC = {"A3": 1.2, "k3": 4.2e-2, "B3": -0.8, "phi3": 10.9, "A4": 0.8, "k4": 5.4e-2, "B4": -0.3, "phi4": 40.9}
MUSCLES = ("FDS2", "FDP2", "EDC2", "EIP")
JOINT_MUSCLES = (
    "FDS3", "FDS4", "FDS5", "FDP3", "FDP4", "FDP5", "EDC3", "EDC4", "EDC5", "EDM",
    "FPL", "EPL", "EPB", "APL", "ECRB", "ECU",
)  # fmt: skip
MCP_DEG = np.arange(0.0, 90.1, 5.0)  # MyoHand's MCP range (no hyperextension; Knutson measured from about -45 deg)
WRIST_DEG = np.arange(-60.0, 60.1, 10.0)
DEG = np.pi / 180.0
EXCESS_WEIGHT = 100.0
MAX_CHANGE = 0.10  # bound on |slack change| / slack


def knutson_extrinsic(theta: np.ndarray, wrist: np.ndarray, p: dict[str, float] = KNUTSON_EXTRINSIC) -> np.ndarray:
    """Knutson et al. 2000 Eqs. (3)-(4): extrinsic MCP moment (N cm, +flexion), angles in deg (+flexion)."""
    t3 = p["B3"] * wrist + p["phi3"]
    t4 = p["B4"] * wrist + p["phi4"]
    return p["A3"] * (np.exp(-p["k3"] * (theta - t3)) - 1.0) - p["A4"] * (np.exp(p["k4"] * (theta - t4)) - 1.0)


def source_moment(row: dict[str, Any], q: np.ndarray | float, wrist_deg: np.ndarray | float = 0.0) -> np.ndarray:
    """Target internal moment (N m) of a ``joint_targets`` row along the MyoHand joint axis at *q* (rad)."""
    p, sign = row["params"], row["joint_sign"]  # sign: +1 when +q is the source's positive (flexion) angle
    x = sign * np.asarray(q, dtype=float)
    if row["model"] == "linear_two_sided":  # Formica 2012: direction-specific stiffness about neutral
        return sign * np.where(x > 0.0, -p["k_positive"] * x, -p["k_negative"] * x)
    if row["model"] == "kamper2002_static":  # Kamper 2002 Table 2 cubic, held outside the measured angles
        x = np.clip(x, p["hold_range_deg"][0] * DEG, p["hold_range_deg"][1] * DEG)
        return sign * (p["c3"] * x**3 + p["c2"] * x**2 + p["c1"] * x + p["c0"])
    if row["model"] == "knutson2000_total":  # Knutson 2000 Eq. (1): intrinsic (Eq. 2) plus extrinsic (Eqs. 3-4)
        d = x / DEG
        intrinsic = p["A1"] * (np.exp(-p["k1"] * (d - p["theta1"])) - 1.0) - p["A2"] * (
            np.exp(p["k2"] * (d - p["theta2"])) - 1.0
        )
        return sign * (intrinsic + knutson_extrinsic(d, wrist_deg, p)) / 100.0
    raise ValueError(f"unknown source model {row['model']!r}")


def passive_fl(norm_len: np.ndarray, lmax: float, fpmax: float) -> np.ndarray:
    """MuJoCo's muscle passive force-length curve (``mju_muscleBias``), as a fraction of F0."""
    b = 0.5 * (1.0 + lmax)
    x1 = (norm_len - 1.0) / (b - 1.0)
    x2 = (norm_len - b) / (b - 1.0)
    return np.where(norm_len <= 1.0, 0.0, np.where(norm_len <= b, fpmax * 0.5 * x1 * x1, fpmax * (0.5 + x2)))


def passive_force(model: mujoco.MjModel, names: tuple[str, ...], lengths: np.ndarray, shift: np.ndarray) -> np.ndarray:
    """Passive force (N, negative: muscles pull) at muscle-tendon *lengths* (..., k), length ranges shifted by *shift*."""
    force = np.zeros_like(lengths)
    for k, name in enumerate(names):
        a = model.actuator(name).id
        g = model.actuator_gainprm[a]
        lr = model.actuator_lengthrange[a] + shift[k]
        L0 = (lr[1] - lr[0]) / (g[1] - g[0])
        force[..., k] = -g[2] * passive_fl(g[0] + (lengths[..., k] - lr[0]) / L0, g[5], g[7])
    return force


def geometry(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """Muscle-tendon lengths and MCP moment arms (dL/dq) of the index muscles on the (wrist, MCP) grid; IP joints at 0."""
    data = mujoco.MjData(model)
    ids = [model.actuator(n).id for n in MUSCLES]
    mcp, wrist = model.joint("mcp2_flexion_r"), model.joint("flexion_r")
    lengths = np.zeros((WRIST_DEG.size, MCP_DEG.size, len(MUSCLES)))
    arms = np.zeros_like(lengths)
    for i, u in enumerate(WRIST_DEG):
        for j, t in enumerate(MCP_DEG):
            out = []
            for dq in (0.0, 1e-5, -1e-5):
                data.qpos[:] = model.qpos0
                data.qpos[wrist.qposadr[0]] = u * DEG
                data.qpos[mcp.qposadr[0]] = t * DEG + dq
                mujoco.mj_kinematics(model, data)
                mujoco.mj_comPos(model, data)
                mujoco.mj_tendon(model, data)
                mujoco.mj_transmission(model, data)
                out.append(data.actuator_length[ids].copy())
            lengths[i, j] = out[0]
            arms[i, j] = (out[1] - out[2]) / 2e-5
    return lengths, arms


def model_torque(model: mujoco.MjModel, lengths: np.ndarray, arms: np.ndarray, shift: np.ndarray) -> np.ndarray:
    """Passive MCP moment (N cm, +flexion) of the four index muscles with length ranges shifted by *shift* (m)."""
    return 100.0 * np.sum(arms * passive_force(model, MUSCLES, lengths, shift), axis=-1)


def row_points(table: dict[str, Any], row: dict[str, Any]) -> tuple[list[dict[str, float]], np.ndarray]:
    """Postures (joint -> deg, swept joint included) and target moments (N m) of a ``joint_targets`` row."""
    lo, hi, step = row["grid_deg"]
    angles = np.arange(lo, hi + 0.5 * step, step)
    wrists = np.arange(row["wrist_deg"][0], row["wrist_deg"][1] + 1e-9, row["wrist_deg"][2]) if "wrist_deg" in row else [None]
    base = table["joint_targets"]["postures_deg"][row["posture"]]
    postures, target = [], []
    for w in wrists:
        for a in angles:
            postures.append({**base, **({} if w is None else {"flexion_r": float(w)}), row["joint"]: float(a)})
            target.append(float(source_moment(row, a * DEG, 0.0 if w is None else w)))
    return postures, np.array(target)


def row_geometry(
    model: mujoco.MjModel, postures: list[dict[str, float]], joint: str, names: tuple[str, ...]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Lengths and moment arms on *joint* of *names*, and the passive moment (N m) of every other muscle, per posture."""
    data = mujoco.MjData(model)
    ids = [model.actuator(n).id for n in names]
    dof = model.joint(joint).dofadr[0]
    moment = np.zeros((model.nu, model.nv))
    lengths, arms, rest = np.zeros((len(postures), len(names))), np.zeros((len(postures), len(names))), np.zeros(len(postures))
    for i, posture in enumerate(postures):
        data.qpos[:] = model.qpos0
        for name, deg in posture.items():
            data.qpos[model.joint(name).qposadr[0]] = deg * DEG
        data.qvel[:] = 0.0
        data.act[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
        lengths[i] = data.actuator_length[ids]
        arms[i] = moment[ids, dof]
        rest[i] = data.qfrc_actuator[dof] - data.actuator_force[ids] @ moment[ids, dof]
    return lengths, arms, rest


def excess_and_deficit(moment: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Moment beyond the target or against its sign (excess), and the shortfall inside ``[0, target]`` (deficit)."""
    inside = np.clip(moment, np.minimum(target, 0.0), np.maximum(target, 0.0))
    return moment - inside, target - inside


def implied_slack(model: mujoco.MjModel, name: str) -> float:
    """Rigid-tendon slack length implied by MuJoCo's length range: ``lr[0] - range[0] * L0``."""
    a = model.actuator(name).id
    g, lr = model.actuator_gainprm[a], model.actuator_lengthrange[a]
    return float(lr[0] - g[0] * (lr[1] - lr[0]) / (g[1] - g[0]))


def load_table() -> dict[str, Any]:
    """The passive-tension table (``build/data/muscle_passive.json``)."""
    return json.loads(DATA_PATH.read_text())


def fitted_lengthranges(model: mujoco.MjModel, table: dict[str, Any]) -> dict[str, list[float]]:
    """``lengthrange`` of each tabled muscle shifted so its implied slack length is the tabled one."""
    ranges = {}
    for row in table["muscles"]:
        name = row["names"]["r"]
        shift = row["tendon_slack_m"]["value"] - implied_slack(model, name)
        lr = model.actuator_lengthrange[model.actuator(name).id]
        ranges[name] = [float(v) for v in lr] if abs(shift) < 1e-12 else [float(v + shift) for v in lr]
    return ranges


def write_lengthranges(ranges: dict[str, list[float]], path: Path = MUSCLE_XML) -> None:
    """Replace the ``lengthrange`` attribute of each named muscle in *path*, leaving the rest of the file as is."""
    text = path.read_text()
    for name, (lo, hi) in ranges.items():
        pattern = re.compile(rf'(<muscle name="{re.escape(name)}"[^>]*?lengthrange=")[^"]*(")')
        text, count = pattern.subn(rf"\g<1>{lo!r} {hi!r}\g<2>", text)
        if count != 1:
            raise ValueError(f"{path.name}: expected one lengthrange for {name}, found {count}")
    path.write_text(text)


def _minimize(
    cost: Callable[[np.ndarray], float], starts: list[np.ndarray], bounds: list[tuple[float, float]] | None = None
) -> np.ndarray:
    """Best of Powell runs from *starts*."""
    from scipy.optimize import minimize

    runs = [
        minimize(cost, s, method="Powell", bounds=bounds, options={"xtol": 1e-7, "ftol": 1e-10, "maxiter": 40000})
        for s in starts
    ]
    return min(runs, key=lambda r: r.fun).x


def _slack_entry(slack: float, shift: float) -> dict[str, Any]:
    return {
        "value": round(slack + shift, 6),
        "reference_value_m": round(slack, 6),
        "change_percent": round(100.0 * shift / slack, 3),
        "scales_with": "length",
        "reference_segment": "forearm",
    }


def fit_index(model: mujoco.MjModel, table: dict[str, Any]) -> np.ndarray:
    """Binder-Markey's fit of the four index slack lengths to Knutson's extrinsic MCP moment; updates *table*, returns the shifts."""
    lengths, arms = geometry(model)
    target = knutson_extrinsic(MCP_DEG[None, :], WRIST_DEG[:, None])
    slack = np.array([implied_slack(model, n) for n in MUSCLES])

    def cost(shift: np.ndarray) -> float:
        err = target - model_torque(model, lengths, arms, shift)
        return float(np.sum(err**2) + np.sum(np.abs(shift / slack)) / 100.0)  # Binder-Markey Eqs. (2)-(3), N cm

    shift = _minimize(cost, [np.full(len(MUSCLES), s) for s in (0.0, -0.01, -0.02)])
    before = target - model_torque(model, lengths, arms, np.zeros(len(MUSCLES)))
    after = target - model_torque(model, lengths, arms, shift)
    for k, name in enumerate(MUSCLES):
        row = next(r for r in table["muscles"] if r["names"]["r"] == name)
        row["tendon_slack_m"] = _slack_entry(float(slack[k]), float(shift[k]))
    table["fit"] = {
        "grid": {"mcp_deg": [float(MCP_DEG[0]), float(MCP_DEG[-1]), 5.0], "wrist_deg": [-60.0, 60.0, 10.0]},
        "rmse_Ncm_before": float(np.sqrt(np.mean(before**2))),
        "rmse_Ncm_after": float(np.sqrt(np.mean(after**2))),
        "max_abs_err_Ncm_after": float(np.max(np.abs(after))),
        "target_peak_Ncm": float(np.max(np.abs(target))),
    }
    return shift


def joint_fit_terms(model: mujoco.MjModel, table: dict[str, Any]) -> list[dict[str, Any]]:
    """Per ``joint_targets`` row: geometry of ``JOINT_MUSCLES``, the other muscles' moment and the target (N m)."""
    terms = []
    for row in table["joint_targets"]["rows"]:
        postures, target = row_points(table, row)
        lengths, arms, rest = row_geometry(model, postures, row["joint"], JOINT_MUSCLES)
        terms.append({"row": row, "target": target, "lengths": lengths, "arms": arms, "rest": rest})
    return terms


def joint_moment(model: mujoco.MjModel, term: dict[str, Any], shift: np.ndarray) -> np.ndarray:
    """Passive moment (N m) of every muscle at the points of one fit term, ``JOINT_MUSCLES`` shifted by *shift*."""
    return term["rest"] + np.sum(term["arms"] * passive_force(model, JOINT_MUSCLES, term["lengths"], shift), axis=-1)


def joint_cost(model: mujoco.MjModel, terms: list[dict[str, Any]], shift: np.ndarray, slack: np.ndarray) -> float:
    """Sum over rows of the mean one-sided squared error (N cm^2) plus Binder-Markey's change penalty."""
    total = 0.0
    for term in terms:
        excess, deficit = excess_and_deficit(100.0 * joint_moment(model, term, shift), 100.0 * term["target"])
        total += float(np.mean(EXCESS_WEIGHT * excess**2 + deficit**2))
    return total + float(np.sum(np.abs(shift / slack)) / 100.0)


def fit_joints(model: mujoco.MjModel, table: dict[str, Any]) -> None:
    """Fit the ``JOINT_MUSCLES`` slack lengths to the total moments of ``joint_targets``; updates *table*."""
    terms = joint_fit_terms(model, table)
    slack = np.array([implied_slack(model, n) for n in JOINT_MUSCLES])
    bounds = [(-MAX_CHANGE * s, MAX_CHANGE * s) for s in slack]
    starts = [f * slack for f in (0.0, 0.05, 0.1)]
    shift = _minimize(lambda s: joint_cost(model, terms, s, slack), starts, bounds)
    rows = {r["names"]["r"]: r for r in table["muscles"]}
    for k, name in enumerate(JOINT_MUSCLES):
        crossed = [t for t in terms if np.any(np.abs(t["arms"][:, k]) > 1e-4)]
        rows[name] = {
            "muscle": name,
            "names": {"r": name, "l": f"{name}_l"},
            "status": "transferred target" if any(t["row"]["status"] == "transferred target" for t in crossed) else "measured",
            "source": "joint_targets",
            "targets": [t["row"]["row"] for t in crossed],
            "method": "binder_markey2017_one_sided",
            "tendon_slack_m": _slack_entry(float(slack[k]), float(shift[k])),
            "at_bound": bool(abs(shift[k]) > 0.999 * MAX_CHANGE * slack[k]),
        }
    table["muscles"] = [rows[n] for n in (*MUSCLES, *JOINT_MUSCLES)]
    for term in terms:
        before, after = joint_moment(model, term, np.zeros(len(JOINT_MUSCLES))), joint_moment(model, term, shift)
        t = term["target"]
        term["row"]["fit"] = {
            "rmse_Nm_before": float(np.sqrt(np.mean((before - t) ** 2))),
            "rmse_Nm_after": float(np.sqrt(np.mean((after - t) ** 2))),
            "max_excess_Nm_before": float(np.max(np.abs(excess_and_deficit(before, t)[0]))),
            "max_excess_Nm_after": float(np.max(np.abs(excess_and_deficit(after, t)[0]))),
            "target_peak_Nm": float(np.max(np.abs(t))),
        }
    table["joint_fit"] = {
        "muscles": list(JOINT_MUSCLES),
        "excess_weight": EXCESS_WEIGHT,
        "max_change": MAX_CHANGE,
        "cost": "sum over rows of mean(excess_weight * excess^2 + deficit^2) (N cm^2) + (1/100) sum |slack change / slack|",
        "cost_before": joint_cost(model, terms, np.zeros(len(JOINT_MUSCLES)), slack),
        "cost_after": joint_cost(model, terms, shift, slack),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--xml-only", action="store_true", help="only write the JSON slack lengths into the XML")
    args = parser.parse_args()
    model = myo_sim.load_spec("myoarm_r").compile()
    table = load_table()
    if args.xml_only:
        write_lengthranges(fitted_lengthranges(model, table))
        return
    # Fit from the reference slack lengths, whatever the XML currently holds.
    for row in table["muscles"]:
        name = row["names"]["r"]
        model.actuator_lengthrange[model.actuator(name).id] += row["tendon_slack_m"]["reference_value_m"] - implied_slack(
            model, name
        )
    shift = fit_index(model, table)
    for k, name in enumerate(MUSCLES):  # the joint fit sees the fitted index muscles
        model.actuator_lengthrange[model.actuator(name).id] += shift[k]
    fit_joints(model, table)
    DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
    model = myo_sim.load_spec("myoarm_r").compile()
    write_lengthranges(fitted_lengthranges(model, table))
    print(json.dumps(table["fit"], indent=1))
    for row in table["muscles"]:
        s = row["tendon_slack_m"]
        print(f"{row['muscle']:5s} {s['reference_value_m']:.4f} -> {s['value']:.4f} m ({s['change_percent']:+.2f} %)")
    for row in table["joint_targets"]["rows"]:
        f = row["fit"]
        print(
            f"{row['row']:20s} rmse {f['rmse_Nm_before']:.3f} -> {f['rmse_Nm_after']:.3f} Nm, "
            f"max excess {f['max_excess_Nm_before']:.3f} -> {f['max_excess_Nm_after']:.3f} Nm (target peak {f['target_peak_Nm']:.3f})"
        )


if __name__ == "__main__":
    main()
