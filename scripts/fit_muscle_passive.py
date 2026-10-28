"""Fit tendon slack lengths and passive-force scales of the extrinsic hand muscles to measured passive moments.

Binder-Markey & Murray (2017) optimised the tendon slack lengths of FDSI, FDPI,
EDCI and EIP so that their passive force reproduced the extrinsic component of
Knutson's MCP passive moment over wrist and MCP angles. MyoHand muscles have
rigid tendons: the normalised fibre length is ``range[0] + (L - lr[0]) / L0``
with ``L0 = (lr[1] - lr[0]) / (range[1] - range[0])``, so a slack-length change
is a shift of ``actuator_lengthrange`` at constant ``L0``. MuJoCo's passive
force-length curve is much stiffer than the one Binder-Markey used, so each
muscle also gets its own ``fpmax`` (the passive-force scale; it leaves the active
force unchanged).

Two fits, each from the rows' reference values, with MuJoCo's passive curve,
MyoHand's muscle-tendon lengths and moment arms, Binder-Markey's penalty on the
relative slack change and a mild penalty on ``|ln(fpmax / reference)|``:

- index (FDS2, FDP2, EDC2, EIP): least squares on Knutson's extrinsic MCP moment;
- the other extrinsic finger, thumb and wrist muscles (``joint_targets``): the
  total measured or transferred joint moments, with a one-sided loss (a moment
  beyond the total, or opposite to it, costs ``EXCESS_WEIGHT`` times a deficit).

Both are subject to ``active_constraint``: the active force-length factor of the
finger and thumb flexors in a full fist, and of the extensors in full extension,
stays at least ``min_fl``. On MuJoCo's curve that is a window on each slack length;
a slack length is shortened only as far as that window forces it.

A third step, ``release_constraint``, keeps the slack lengths and refits only the
``fpmax`` of EDC3-5 and EDM: their lower bound is the smallest value at which the
digits open when the wrist flexes (McFarland et al. 2023 tenodesis trial B, run by
``scripts/validate_hand_function.py``), and the joint loss sets them above it.

It writes the values (and the references they change) to
``build/data/muscle_passive.json`` and the ``lengthrange`` and ``fpmax`` of each
muscle to ``myoarm_r_muscle.xml``, the model's source of truth (the left arm is
mirrored from it).

Run: ``uv run python scripts/fit_muscle_passive.py`` (refits, rewrites the JSON and
the XML attributes); ``--release-only`` reruns only the release step from the tabled
values; ``--xml-only`` only writes the JSON values into the XML.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
import sys
from functools import cache
from pathlib import Path
from types import ModuleType
from typing import Any, Callable

import mujoco
import numpy as np

import myo_sim

DATA_PATH = Path(__file__).parents[1] / "myo_sim" / "build" / "data" / "muscle_passive.json"
VALIDATION_SCRIPT = Path(__file__).with_name("validate_hand_function.py")
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
FPMAX_BOUNDS = (0.002, 2.6)
FPMAX_REFERENCE = 1.3  # MuJoCo's default, used by every fitted muscle before the fit
FPMAX_PENALTY = 1.0  # N cm^2 per unit |ln(fpmax / reference)|: mild, but it settles otherwise flat directions


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


def active_fl(norm_len: np.ndarray, lmin: float, lmax: float) -> np.ndarray:
    """MuJoCo's active force-length factor (``mju_muscleGain`` at zero velocity)."""
    x = np.asarray(norm_len, dtype=float)
    a, b = 0.5 * (lmin + 1.0), 0.5 * (1.0 + lmax)
    out = np.select(
        [x <= lmin, x <= a, x <= 1.0, x <= b, x < lmax],
        [
            0.0,
            0.5 * ((x - lmin) / (a - lmin)) ** 2,
            1.0 - 0.5 * ((1.0 - x) / (1.0 - a)) ** 2,
            1.0 - 0.5 * ((x - 1.0) / (b - 1.0)) ** 2,
            0.5 * ((lmax - x) / (lmax - b)) ** 2,
        ],
        0.0,
    )
    return out


def normalised_length(model: mujoco.MjModel, name: str, length: np.ndarray, shift: float = 0.0) -> np.ndarray:
    """Normalised fibre length of muscle *name* at muscle-tendon *length*, length range shifted by *shift*."""
    a = model.actuator(name).id
    g = model.actuator_gainprm[a]
    lr = model.actuator_lengthrange[a] + shift
    return g[0] + (np.asarray(length) - lr[0]) * (g[1] - g[0]) / (lr[1] - lr[0])


def passive_force(
    model: mujoco.MjModel, names: tuple[str, ...], lengths: np.ndarray, shift: np.ndarray, fpmax: np.ndarray | None = None
) -> np.ndarray:
    """Passive force (N, negative: muscles pull) at muscle-tendon *lengths* (..., k), length ranges shifted by *shift*.

    *fpmax* (per muscle) replaces the model's passive-force scale when given.
    """
    force = np.zeros_like(lengths)
    for k, name in enumerate(names):
        g = model.actuator_gainprm[model.actuator(name).id]
        scale = g[7] if fpmax is None else fpmax[k]
        force[..., k] = -g[2] * passive_fl(normalised_length(model, name, lengths[..., k], shift[k]), g[5], scale)
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


def model_torque(
    model: mujoco.MjModel, lengths: np.ndarray, arms: np.ndarray, shift: np.ndarray, fpmax: np.ndarray | None = None
) -> np.ndarray:
    """Passive MCP moment (N cm, +flexion) of the four index muscles, length ranges shifted by *shift* (m)."""
    return 100.0 * np.sum(arms * passive_force(model, MUSCLES, lengths, shift, fpmax), axis=-1)


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


def _forward(model: mujoco.MjModel, data: mujoco.MjData, posture: dict[str, float]) -> None:
    data.qpos[:] = model.qpos0
    for name, deg in posture.items():
        data.qpos[model.joint(name).qposadr[0]] = deg * DEG
    data.qvel[:] = 0.0
    data.act[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def row_geometry(
    model: mujoco.MjModel, postures: list[dict[str, float]], joint: str, names: tuple[str, ...], others: tuple[str, ...]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Lengths and moment arms on *joint* of *names*, and the passive moment (N m) of the muscles *others*, per posture."""
    data = mujoco.MjData(model)
    ids = [model.actuator(n).id for n in names]
    rest_ids = [model.actuator(n).id for n in others]
    dof = model.joint(joint).dofadr[0]
    moment = np.zeros((model.nu, model.nv))
    lengths, arms, rest = np.zeros((len(postures), len(names))), np.zeros((len(postures), len(names))), np.zeros(len(postures))
    for i, posture in enumerate(postures):
        _forward(model, data, posture)
        mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
        lengths[i] = data.actuator_length[ids]
        arms[i] = moment[ids, dof]
        rest[i] = data.actuator_force[rest_ids] @ moment[rest_ids, dof]
    return lengths, arms, rest


def excess_and_deficit(moment: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Moment beyond the target or against its sign (excess), and the shortfall inside ``[0, target]`` (deficit)."""
    inside = np.clip(moment, np.minimum(target, 0.0), np.maximum(target, 0.0))
    return moment - inside, target - inside


def constraint_lengths(model: mujoco.MjModel, table: dict[str, Any]) -> dict[str, dict[str, float]]:
    """Muscle-tendon length (m) of each constrained muscle in each ``active_constraint`` posture."""
    spec = table["active_constraint"]
    data = mujoco.MjData(model)
    out: dict[str, dict[str, float]] = {}
    for posture, names in spec["muscles"].items():
        _forward(model, data, spec["postures_deg"][posture])
        for name in names:
            out.setdefault(name, {})[posture] = float(data.actuator_length[model.actuator(name).id])
    return out


def active_fl_at_constraints(model: mujoco.MjModel, table: dict[str, Any]) -> dict[str, dict[str, float]]:
    """Active force-length factor of each constrained muscle in each ``active_constraint`` posture (model as is)."""
    out = {}
    for name, lengths in constraint_lengths(model, table).items():
        g = model.actuator_gainprm[model.actuator(name).id]
        out[name] = {p: float(active_fl(normalised_length(model, name, L), g[4], g[5])) for p, L in lengths.items()}
    return out


def slack_window(model: mujoco.MjModel, table: dict[str, Any], name: str) -> tuple[float, float]:
    """Length-range shifts (m) allowed by ``MAX_CHANGE`` and the active constraint.

    The active factor is at least ``min_fl`` between the two lengths where it
    equals it, so each constrained posture bounds the shift from both sides.
    Raises ``ValueError`` when no shift satisfies every posture.
    """
    a = model.actuator(name).id
    g = model.actuator_gainprm[a]
    slack = implied_slack(model, name)
    lo, hi = -MAX_CHANGE * slack, MAX_CHANGE * slack
    min_fl = table["active_constraint"]["min_fl"]
    L0 = (model.actuator_lengthrange[a][1] - model.actuator_lengthrange[a][0]) / (g[1] - g[0])
    # Points of the FL curve where it equals min_fl: one on each limb.
    a_pt, b_pt = 0.5 * (g[4] + 1.0), 0.5 * (1.0 + g[5])
    up = (
        a_pt + (1.0 - a_pt) * (1.0 - np.sqrt(2.0 * (1.0 - min_fl)))
        if min_fl >= 0.5
        else g[4] + (a_pt - g[4]) * np.sqrt(2.0 * min_fl)
    )
    down = (
        b_pt - (b_pt - 1.0) * (1.0 - np.sqrt(2.0 * (1.0 - min_fl)))
        if min_fl >= 0.5
        else g[5] - (g[5] - b_pt) * np.sqrt(2.0 * min_fl)
    )
    for length in constraint_lengths(model, table).get(name, {}).values():
        norm = float(normalised_length(model, name, length))
        lo, hi = max(lo, L0 * (norm - down)), min(hi, L0 * (norm - up))
    if lo > hi:
        raise ValueError(f"{name}: no slack length within {MAX_CHANGE:.0%} keeps the active factor at {min_fl}")
    return lo, hi


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
        name, slack = row["names"]["r"], row["tendon_slack_m"]
        shift = slack["value"] - implied_slack(model, name)
        lr = model.actuator_lengthrange[model.actuator(name).id]
        unchanged = slack["value"] == slack["reference_value_m"]  # keep the XML value, not its 1 um rounding
        ranges[name] = [float(v) for v in lr] if unchanged or abs(shift) < 1e-12 else [float(v + shift) for v in lr]
    return ranges


def fitted_fpmax(table: dict[str, Any]) -> dict[str, float]:
    """Tabled ``fpmax`` of each muscle."""
    return {row["names"]["r"]: row["fpmax"]["value"] for row in table["muscles"]}


def write_muscle_xml(ranges: dict[str, list[float]], fpmax: dict[str, float], path: Path = MUSCLE_XML) -> None:
    """Set ``lengthrange`` and ``fpmax`` of each named muscle in *path* (no ``fpmax`` at the default), rest unchanged."""
    text = path.read_text()
    for name, (lo, hi) in ranges.items():
        pattern = re.compile(rf'<muscle name="{re.escape(name)}"[^>]*?/>')
        tags = pattern.findall(text)
        if len(tags) != 1:
            raise ValueError(f"{path.name}: expected one muscle {name}, found {len(tags)}")
        tag = re.sub(r'lengthrange="[^"]*"', f'lengthrange="{lo!r} {hi!r}"', tags[0])
        tag = re.sub(r' fpmax="[^"]*"', "", tag)
        if abs(fpmax[name] - FPMAX_REFERENCE) > 1e-12:
            tag = tag.replace(' class="', f' fpmax="{fpmax[name]!r}" class="', 1)
        text = text.replace(tags[0], tag)
    path.write_text(text)


def _minimize(cost: Callable[[np.ndarray], float], starts: list[np.ndarray], bounds: list[tuple[float, float]]) -> np.ndarray:
    """Best of bounded Powell runs from *starts*."""
    from scipy.optimize import minimize

    runs = [
        minimize(cost, s, method="Powell", bounds=bounds, options={"xtol": 1e-8, "ftol": 1e-12, "maxiter": 200000})
        for s in starts
    ]
    return min(runs, key=lambda r: r.fun).x


def _penalty(shift: np.ndarray, slack: np.ndarray, log_fpmax: np.ndarray) -> float:
    """Binder-Markey's (1/100) sum |slack change / slack|, plus ``FPMAX_PENALTY`` sum |ln(fpmax / reference)|."""
    return float(np.sum(np.abs(shift / slack))) / 100.0 + FPMAX_PENALTY * float(np.sum(np.abs(log_fpmax)))


def _fit(
    model: mujoco.MjModel, table: dict[str, Any], names: tuple[str, ...], error: Callable[[np.ndarray, np.ndarray], float]
) -> tuple[np.ndarray, np.ndarray]:
    """Shifts (m) and ``fpmax`` of *names* minimising ``error(shift, fpmax)`` plus the penalty, inside the windows."""
    slack = np.array([implied_slack(model, n) for n in names])
    # Shorten a slack length only as far as the active constraint forces it: shortening merely
    # fills deficits the joint springs supply, and with fpmax free it is a flat direction.
    windows = []
    for name in names:
        lo, hi = slack_window(model, table, name)
        if hi < 0.0:  # the constraint forces a shorter slack: as little as it allows (2 um room for rounding)
            windows.append((hi - 3e-6, hi - 2e-6))
        else:
            windows.append((lo + 2e-6 if lo > 0.0 else 0.0, hi - 2e-6))
    log_bounds = [tuple(np.log(np.array(FPMAX_BOUNDS) / FPMAX_REFERENCE))] * len(names)
    k = len(names)

    def cost(x: np.ndarray) -> float:
        return error(x[:k], FPMAX_REFERENCE * np.exp(x[k:])) + _penalty(x[:k], slack, x[k:])

    nearest = np.array([np.clip(0.0, lo, hi) for lo, hi in windows])
    middle = np.array([0.5 * (lo + hi) for lo, hi in windows])
    starts = [np.concatenate([s, np.full(k, f)]) for s, f in ((nearest, 0.0), (middle, 0.0), (middle, -1.0), (nearest, -2.0))]
    x = _minimize(cost, starts, [*windows, *log_bounds])
    # Round as stored: slack to 1 um, fpmax to 4 decimals; stay inside the window.
    shift = np.array([np.clip(round(s0 + s, 6) - s0, lo, hi) for s0, s, (lo, hi) in zip(slack, x[:k], windows)])
    fpmax = np.round(FPMAX_REFERENCE * np.exp(x[k:]), 4)
    fpmax[np.abs(fpmax - FPMAX_REFERENCE) < 1e-3] = FPMAX_REFERENCE
    return shift, fpmax


def _parameter_entries(slack: float, shift: float, fpmax: float, window: tuple[float, float]) -> dict[str, Any]:
    return {
        "tendon_slack_m": {
            "value": round(slack + shift, 6),
            "reference_value_m": round(slack, 6),
            "change_percent": round(100.0 * shift / slack, 3),
            "active_window_percent": [round(100.0 * w / slack, 3) for w in window],
            "scales_with": "length",
            "reference_segment": "forearm",
        },
        "fpmax": {
            "value": float(fpmax),
            "reference_value": FPMAX_REFERENCE,
            "change_percent": round(100.0 * (fpmax / FPMAX_REFERENCE - 1.0), 3),
            "at_bound": bool(min(abs(fpmax - b) for b in FPMAX_BOUNDS) < 1e-3),
        },
    }


def _apply(model: mujoco.MjModel, names: tuple[str, ...], shift: np.ndarray, fpmax: np.ndarray) -> None:
    for k, name in enumerate(names):
        a = model.actuator(name).id
        model.actuator_lengthrange[a] += shift[k]
        model.actuator_gainprm[a][7] = model.actuator_biasprm[a][7] = fpmax[k]


def fit_index(model: mujoco.MjModel, table: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Binder-Markey's fit of the four index muscles to Knutson's extrinsic MCP moment; updates *table*."""
    lengths, arms = geometry(model)
    target = knutson_extrinsic(MCP_DEG[None, :], WRIST_DEG[:, None])

    def error(shift: np.ndarray, fpmax: np.ndarray) -> float:
        return float(np.sum((target - model_torque(model, lengths, arms, shift, fpmax)) ** 2))  # Binder-Markey Eq. (2), N cm

    shift, fpmax = _fit(model, table, MUSCLES, error)
    before = target - model_torque(model, lengths, arms, np.zeros(len(MUSCLES)))
    after = target - model_torque(model, lengths, arms, shift, fpmax)
    for k, name in enumerate(MUSCLES):
        row = next(r for r in table["muscles"] if r["names"]["r"] == name)
        row.update(_parameter_entries(implied_slack(model, name), shift[k], fpmax[k], slack_window(model, table, name)))
    table["fit"] = {
        "grid": {"mcp_deg": [float(MCP_DEG[0]), float(MCP_DEG[-1]), 5.0], "wrist_deg": [-60.0, 60.0, 10.0]},
        "rmse_Ncm_before": float(np.sqrt(np.mean(before**2))),
        "rmse_Ncm_after": float(np.sqrt(np.mean(after**2))),
        "max_abs_err_Ncm_after": float(np.max(np.abs(after))),
        "target_peak_Ncm": float(np.max(np.abs(target))),
    }
    return shift, fpmax


def other_muscles(model: mujoco.MjModel, table: dict[str, Any]) -> tuple[str, ...]:
    """Muscles counted besides ``JOINT_MUSCLES``: those recorded at the fit, else every other actuator of *model*.

    Muscles added to the model after the fit (e.g. the thenar muscles) are left to the joint springs, which are
    fitted to the residual of all muscles.
    """
    recorded = table.get("joint_fit", {}).get("other_muscles")
    if recorded is not None:
        return tuple(recorded)
    return tuple(model.actuator(i).name for i in range(model.nu) if model.actuator(i).name not in JOINT_MUSCLES)


def joint_fit_terms(model: mujoco.MjModel, table: dict[str, Any]) -> list[dict[str, Any]]:
    """Per ``joint_targets`` row: geometry of ``JOINT_MUSCLES``, the other muscles' moment and the target (N m)."""
    terms = []
    others = other_muscles(model, table)
    for row in table["joint_targets"]["rows"]:
        postures, target = row_points(table, row)
        lengths, arms, rest = row_geometry(model, postures, row["joint"], JOINT_MUSCLES, others)
        terms.append({"row": row, "target": target, "lengths": lengths, "arms": arms, "rest": rest})
    return terms


def joint_moment(model: mujoco.MjModel, term: dict[str, Any], shift: np.ndarray, fpmax: np.ndarray | None = None) -> np.ndarray:
    """Passive moment (N m) of every muscle at the points of one fit term, ``JOINT_MUSCLES`` shifted by *shift*."""
    return term["rest"] + np.sum(term["arms"] * passive_force(model, JOINT_MUSCLES, term["lengths"], shift, fpmax), axis=-1)


def joint_error(
    model: mujoco.MjModel, terms: list[dict[str, Any]], shift: np.ndarray, fpmax: np.ndarray | None = None
) -> float:
    """Sum over rows of the mean one-sided squared error (N cm^2)."""
    total = 0.0
    for term in terms:
        excess, deficit = excess_and_deficit(100.0 * joint_moment(model, term, shift, fpmax), 100.0 * term["target"])
        total += float(np.mean(EXCESS_WEIGHT * excess**2 + deficit**2))
    return total


def fit_joints(model: mujoco.MjModel, table: dict[str, Any]) -> None:
    """Fit ``JOINT_MUSCLES`` to the total moments of ``joint_targets``; updates *table*."""
    table.setdefault("joint_fit", {}).pop("other_muscles", None)
    others = other_muscles(model, table)
    terms = joint_fit_terms(model, table)
    shift, fpmax = _fit(model, table, JOINT_MUSCLES, lambda s, f: joint_error(model, terms, s, f))
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
            **_parameter_entries(implied_slack(model, name), shift[k], fpmax[k], slack_window(model, table, name)),
        }
    table["muscles"] = [rows[n] for n in (*MUSCLES, *JOINT_MUSCLES)]
    zero = np.zeros(len(JOINT_MUSCLES))
    for term in terms:
        before, after = joint_moment(model, term, zero), joint_moment(model, term, shift, fpmax)
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
        "other_muscles": list(others),
        "excess_weight": EXCESS_WEIGHT,
        "max_change": MAX_CHANGE,
        "fpmax_bounds": list(FPMAX_BOUNDS),
        "fpmax_penalty": FPMAX_PENALTY,
        "cost": (
            "sum over rows of mean(excess_weight * excess^2 + deficit^2) (N cm^2) "
            "+ (1/100) sum |slack change / slack| + fpmax_penalty sum |ln(fpmax / reference)|"
        ),
        "error_before": joint_error(model, terms, zero),
        "error_after": joint_error(model, terms, shift, fpmax),
    }


@cache
def _validation() -> ModuleType:
    """``scripts/validate_hand_function.py`` (McFarland et al. 2023 protocols) as a module."""
    spec = importlib.util.spec_from_file_location("validate_hand_function", VALIDATION_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve their module
    spec.loader.exec_module(module)
    return module


def release_extension(table: dict[str, Any], fpmax: float) -> dict[str, float]:
    """MCP extension (deg) of each ``release_constraint`` digit over tenodesis trial B, its muscles at *fpmax*.

    Runs the importable model (its XML) with only the ``fpmax`` of the release muscles replaced.
    """
    spec = table["release_constraint"]
    v = _validation()
    protocol = v.load_table()

    def mutate(model: mujoco.MjModel) -> None:
        for name in spec["muscles"]:
            a = model.actuator(name).id
            model.actuator_gainprm[a][7] = model.actuator_biasprm[a][7] = fpmax

    trials = v.tenodesis(v.myohand(), protocol, mutate)
    net = v.tenodesis_metrics(trials, protocol)["B"]["net_deg"]
    return {str(d): float(net[f"mcp_flexion{d}"]) for d in spec["digits"]}


def release_bound(table: dict[str, Any]) -> float:
    """Smallest common ``fpmax`` (4 decimals) of the release muscles at which every release digit extends enough."""
    need = table["release_constraint"]["min_mcp_extension_deg"]

    def opens(f: float) -> bool:
        return min(release_extension(table, f).values()) >= need

    lo, hi = FPMAX_BOUNDS[0], FPMAX_REFERENCE
    if opens(lo):
        return lo
    if not opens(hi):
        raise ValueError(f"the digits do not open by {need} deg even at fpmax {hi}")
    while hi - lo > 1e-4:
        mid = float(np.sqrt(lo * hi)) if hi / lo > 1.01 else 0.5 * (lo + hi)
        lo, hi = (lo, mid) if opens(mid) else (mid, hi)
    return float(np.ceil(hi * 1e4) / 1e4)


def fit_release(model: mujoco.MjModel, table: dict[str, Any]) -> None:
    """Refit only the ``fpmax`` of the release muscles, at least ``release_bound``, by the joint loss; updates *table*.

    Slack lengths and every other muscle stay as tabled; *model* holds the tabled values.
    """
    spec = table["release_constraint"]
    names = tuple(spec["muscles"])
    idx = [JOINT_MUSCLES.index(n) for n in names]
    bound = release_bound(table)
    terms = joint_fit_terms(model, table)
    zero = np.zeros(len(JOINT_MUSCLES))
    base = np.array([model.actuator_gainprm[model.actuator(n).id][7] for n in JOINT_MUSCLES])

    def with_release(log_f: np.ndarray) -> np.ndarray:
        f = base.copy()
        f[idx] = FPMAX_REFERENCE * np.exp(log_f)
        return f

    def cost(log_f: np.ndarray) -> float:
        return joint_error(model, terms, zero, with_release(log_f)) + FPMAX_PENALTY * float(np.sum(np.abs(log_f)))

    lower = float(np.log(bound / FPMAX_REFERENCE))
    bounds = [(lower, float(np.log(FPMAX_BOUNDS[1] / FPMAX_REFERENCE)))] * len(names)
    x = _minimize(cost, [np.full(len(names), lower), np.full(len(names), lower + 1.0)], bounds)
    fitted = np.maximum(np.round(FPMAX_REFERENCE * np.exp(x), 4), bound)
    final = base.copy()
    final[idx] = fitted
    rows = {r["names"]["r"]: r for r in table["muscles"]}
    before = {}
    for name, value in zip(names, fitted):
        entry = rows[name]["fpmax"]
        before[name] = entry["value"]
        entry.update(
            value=float(value),
            change_percent=round(100.0 * (value / FPMAX_REFERENCE - 1.0), 3),
            at_bound=bool(abs(value - bound) < 1e-4 or abs(value - FPMAX_BOUNDS[1]) < 1e-3),
            lower_bound=bound,
            lower_bound_source="release_constraint",
        )
    for term in terms:
        t, row_fit = term["target"], term["row"]["fit"]
        after = joint_moment(model, term, zero, final)
        row_fit["max_excess_Nm_without_release"] = row_fit.get("max_excess_Nm_without_release", row_fit["max_excess_Nm_after"])
        row_fit["rmse_Nm_after"] = float(np.sqrt(np.mean((after - t) ** 2)))
        row_fit["max_excess_Nm_after"] = float(np.max(np.abs(excess_and_deficit(after, t)[0])))
    table["joint_fit"].setdefault("error_without_release", table["joint_fit"]["error_after"])
    table["joint_fit"]["error_after"] = joint_error(model, terms, zero, final)
    table["joint_fit"]["other_muscles"] = list(other_muscles(model, table))
    first = spec.get("result", {})  # a rerun keeps the values the first release step started from
    before = first.get("fpmax_before", before)
    spec["result"] = {
        "fpmax_lower_bound": bound,
        "fpmax_before": before,
        "mcp_extension_deg_before": first.get(
            "mcp_extension_deg_before", release_extension(table, float(np.mean(list(before.values()))))
        ),
        "mcp_extension_deg_at_bound": release_extension(table, bound),
    }


def _report(table: dict[str, Any]) -> None:
    for row in table["muscles"]:
        s, f = row["tendon_slack_m"], row["fpmax"]
        print(
            f"{row['muscle']:5s} slack {s['change_percent']:+7.2f} % (window {s['active_window_percent'][0]:+.1f}.."
            f"{s['active_window_percent'][1]:+.1f} %), fpmax {f['value']:.4f}{' (bound)' if f['at_bound'] else ''}"
        )
    for row in table["joint_targets"]["rows"]:
        f = row["fit"]
        print(
            f"{row['row']:20s} rmse {f['rmse_Nm_before']:.3f} -> {f['rmse_Nm_after']:.3f} Nm, "
            f"max excess {f['max_excess_Nm_before']:.3f} -> {f['max_excess_Nm_after']:.3f} Nm "
            f"(without release {f.get('max_excess_Nm_without_release', f['max_excess_Nm_after']):.3f}; target peak {f['target_peak_Nm']:.3f})"
        )
    if "result" in table.get("release_constraint", {}):
        print(json.dumps(table["release_constraint"]["result"], indent=1))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--xml-only", action="store_true", help="only write the JSON values into the XML")
    parser.add_argument("--release-only", action="store_true", help="only rerun the release step from the tabled values")
    args = parser.parse_args()
    model = myo_sim.load_spec("myoarm_r").compile()
    table = load_table()
    if args.xml_only:
        write_muscle_xml(fitted_lengthranges(model, table), fitted_fpmax(table))
        return
    if args.release_only:  # the XML holds the tabled values; only fpmax changes
        fit_release(model, table)
        DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
        unchanged = {
            r["names"]["r"]: [float(v) for v in model.actuator_lengthrange[model.actuator(r["names"]["r"]).id]]
            for r in table["muscles"]
        }
        write_muscle_xml(unchanged, fitted_fpmax(table))
        _report(table)
        return
    # Fit from the reference values, whatever the XML currently holds.
    for row in table["muscles"]:
        name = row["names"]["r"]
        a = model.actuator(name).id
        model.actuator_lengthrange[a] += row["tendon_slack_m"]["reference_value_m"] - implied_slack(model, name)
        model.actuator_gainprm[a][7] = model.actuator_biasprm[a][7] = FPMAX_REFERENCE
    shift, fpmax = fit_index(model, table)
    _apply(model, MUSCLES, shift, fpmax)  # the joint fit sees the fitted index muscles
    fit_joints(model, table)
    DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
    model = myo_sim.load_spec("myoarm_r").compile()
    write_muscle_xml(fitted_lengthranges(model, table), fitted_fpmax(table))
    # The release step simulates the written model, then refits the release muscles' fpmax.
    model = myo_sim.load_spec("myoarm_r").compile()
    fit_release(model, table)
    DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
    write_muscle_xml(fitted_lengthranges(model, table), fitted_fpmax(table))
    print(json.dumps(table["fit"], indent=1))
    _report(table)


if __name__ == "__main__":
    main()
