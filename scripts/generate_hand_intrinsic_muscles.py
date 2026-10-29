"""Write the hand intrinsic muscles of ``build/data/hand_intrinsic_muscles.json`` into the right-arm XML.

Each table row is one muscle (or muscle head): its sites, architecture and the moment arms it is fitted
to. This script writes

- the site positions (``pos`` of ``<name>-P<i>_r``) in ``myoarm_r_chain.xml``; the sites themselves are
  hand-written in their bodies;
- straight tendons through those sites in ``myoarm_r_tendon.xml`` and muscles in ``myoarm_r_muscle.xml``:
  rows without a ``block`` go to the script's main block, a row with a ``block`` label to its own block,
  which stands where the hand-written element it replaced stood (so actuator indices stay).

The length range is a rigid tendon of slack length ``(L_MT(q = 0) - L_fo cos(alpha)) / 1.033`` (Barry et
al. 2018, Eq. 1), so the muscle class's normalised fibre-length range spans the same range of ``L_fo``.

``--fit`` first refits the sites to the rows' target moment arms (``path_fit`` in the table; ``fit.reference``
targets are only reported), then the
passive-force scale of the muscles that cross the thumb to its measured passive moments (``passive_fit``),
and records both in the table. Rows of one ``fit.group`` are fitted together: the first row's targets apply to
the force-weighted mean of the group's moment arms, the other rows' targets to that row alone.

Run: ``uv run python scripts/generate_hand_intrinsic_muscles.py [--fit]``.
"""

from __future__ import annotations

import argparse
import json
import re
from functools import cache
from typing import Any

import mujoco
import numpy as np

from myo_sim.build.compose import RIGHT_ARM_CHAIN_XML, RIGHT_ARM_MUSCLES_XML, RIGHT_ARM_TENDONS_XML, build_spec
from myo_sim.build.hand_data import INTRINSIC_DATA_PATH, load_intrinsic_table, write_generated_block

SCRIPT = "generate_hand_intrinsic_muscles.py"
MUSCLE_CLASS = "myoarm_forearm_muscle"
DEG = np.pi / 180.0
H = 1e-5  # rad, central differences of the tendon length
PENALTY = 1000.0  # residual per unit of normalised fibre length outside the window
# Anatomical offsets (deg; flexion, abduction positive) spanning the measurement box of Smutz et al. 1998.
THUMB_BOX_DEG = {"cmc_flexion_r": (20.0, 0.0, -25.0), "cmc_abduction_r": (20.0, 0.0, -20.0), "mp_flexion_r": (-10.0, 0.0, 40.0)}
INDEX_BOX_DEG = {"mcp2_flexion_r": (0.0, 45.0, 90.0), "mcp2_abduction_r": (-15.0, 0.0, 15.0)}
LITTLE_BOX_DEG = {
    "mcp5_flexion_r": (0.0, 45.0, 90.0),
    "mcp5_abduction_r": (-15.0, 0.0, 15.0),
    "cmc5_flexion_r": (-10.3, 0.0, 33.0),
}
PROXIMAL_OF_MC5 = {"pisiform_r", "hamate_r", "capitate_r", "triquetrum_r", "lunate_r"}


def _cite(table: dict[str, Any], key: str) -> str:
    citation = table["sources"][key]["citation"]
    year = "".join(c for c in key if c.isdigit())
    return f"{citation.split()[0]} et al. {year}"


def site_name(row: dict[str, Any], index: int) -> str:
    """Right-side name of site *index* (0-based) of *row*."""
    return f"{row['name']}-P{index + 1}_r"


def rows_in_block(table: dict[str, Any], label: str | None) -> list[dict[str, Any]]:
    return [row for row in table["muscles"] if row.get("block") == label]


def block_labels(table: dict[str, Any]) -> list[str | None]:
    """The main block (None) and every labelled block, in table order."""
    return [None] + list(dict.fromkeys(row["block"] for row in table["muscles"] if row.get("block")))


# --- XML ------------------------------------------------------------------------------------------------


def _number(value: float) -> str:
    return f"{float(value) + 0.0:.6g}"


def chain_with_sites(text: str, table: dict[str, Any]) -> str:
    """*text* of ``myoarm_r_chain.xml`` with each tabled site at its ``pos``, rest unchanged."""
    for row in table["muscles"]:
        for i, site in enumerate(row["sites"]):
            name = site_name(row, i)
            tags = re.findall(rf'<site\b[^>]*\bname="{re.escape(name)}"[^>]*/>', text)
            if len(tags) != 1:
                raise ValueError(f"{RIGHT_ARM_CHAIN_XML.name}: expected one site {name}, found {len(tags)}")
            pos = " ".join(_number(v) for v in site["pos"])
            text = text.replace(tags[0], re.sub(r'\spos="[^"]*"', f' pos="{pos}"', tags[0]))
    return text


def tendon_block(table: dict[str, Any], label: str | None = None) -> str:
    """Generated block *label* of ``myoarm_r_tendon.xml``: one straight spatial tendon per row."""
    lines = []
    for row in rows_in_block(table, label):
        starts = sorted({_cite(table, s["start"]["source"]) for s in row["sites"] if s["start"]["source"] in table["sources"]})
        group = [r for r in table["muscles"] if r["fit"]["group"] == row["fit"]["group"]]
        targets = sorted({_cite(table, t["source"]) for r in group for t in r["fit"].get("targets", {}).values()})
        fitted = f", fitted to {' and '.join(targets)}" if targets else ""
        lines.append(f"        <!-- {row['name']} ({row['anatomy']}): sites from {' and '.join(starts)}{fitted} -->")
        lines.append(f'        <spatial name="{row["name"]}_r_tendon" class="{MUSCLE_CLASS}">')
        lines += [f'            <site site="{site_name(row, i)}"/>' for i in range(len(row["sites"]))]
        lines.append("        </spatial>")
    return "".join(f"{line}\n" for line in lines)


def architecture_lengthranges(table: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Length range per row from its straight site path at q = 0 (sites from ``myoarm_r_chain.xml``)."""
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_kinematics(model, data)
    lo, hi = model.actuator_gainprm[model.actuator(f"{table['template_muscle']}_r").id, :2]
    ranges = {}
    for row in table["muscles"]:
        points = [data.site_xpos[model.site(site_name(row, i)).id] for i in range(len(row["sites"]))]
        l_mt0 = float(sum(np.linalg.norm(b - a) for a, b in zip(points[:-1], points[1:])))
        l_fo = row["optimal_fiber_length_m"]["value"]
        l_ts = (l_mt0 - l_fo * np.cos(np.radians(row["pennation_deg"]["value"]))) / 1.033
        ranges[row["name"]] = (float(l_ts + lo * l_fo), float(l_ts + hi * l_fo))
    return ranges


def muscle_block(table: dict[str, Any], lengthranges: dict[str, tuple[float, float]], label: str | None = None) -> str:
    """Generated block *label* of ``myoarm_r_muscle.xml``: one muscle per row."""
    lines = []
    for row in rows_in_block(table, label):
        lines.append(
            f"        <!-- {row['name']}: force {row['force_N']['derivation']}; L_fo"
            f" {row['optimal_fiber_length_m']['value']} m, pennation {row['pennation_deg']['value']} deg"
            f" ({_cite(table, row['architecture_source'])}); lengthrange from the rigid-tendon slack length of"
            " Barry et al. 2018 Eq. 1 -->"
        )
        lo, hi = lengthranges[row["name"]]
        lines.append(
            f'        <muscle name="{row["name"]}_r" force="{row["force_N"]["value"]:g}" class="{MUSCLE_CLASS}"'
            f' tendon="{row["name"]}_r_tendon" lengthrange="{lo!r} {hi!r}"'
            + (f' fpmax="{row["fpmax"]["value"]:g}"' if "fpmax" in row else "")
            + "/>"
        )
    return "".join(f"{line}\n" for line in lines)


def write_xml(table: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Write the sites, then every tendon and muscle block (length ranges from the written sites)."""
    RIGHT_ARM_CHAIN_XML.write_text(chain_with_sites(RIGHT_ARM_CHAIN_XML.read_text(), table))
    ranges = architecture_lengthranges(table)
    for label in block_labels(table):
        write_generated_block(RIGHT_ARM_TENDONS_XML, SCRIPT, tendon_block(table, label), "</tendon>", label)
        write_generated_block(RIGHT_ARM_MUSCLES_XML, SCRIPT, muscle_block(table, ranges, label), "</actuator>", label)
    return ranges


# --- path fit -------------------------------------------------------------------------------------------


@cache
def _model() -> mujoco.MjModel:
    return build_spec("myoarm_r").compile()


class Poses:
    """Body positions and orientations of ``myoarm_r`` at a list of postures (joint name -> q, others 0)."""

    def __init__(self, postures: list[dict[str, float]]) -> None:
        model = _model()
        data = mujoco.MjData(model)
        pos, mat = [], []
        for posture in postures:
            data.qpos[:] = model.qpos0
            for joint, q in posture.items():
                data.qpos[model.joint(joint).qposadr[0]] = q
            mujoco.mj_kinematics(model, data)
            pos.append(data.xpos.copy())
            mat.append(data.xmat.copy().reshape(-1, 3, 3))
        self.pos, self.mat = np.array(pos), np.array(mat)

    def length(self, sites: list[tuple[str, np.ndarray]]) -> np.ndarray:
        """Length (m) of the straight path through *sites* (body name, local position) in every posture."""
        model = _model()
        world = []
        for body, local in sites:
            b = model.body(body).id
            world.append(self.pos[:, b] + np.einsum("nij,j->ni", self.mat[:, b], np.asarray(local)))
        return sum(np.linalg.norm(q - p, axis=1) for p, q in zip(world[:-1], world[1:]))


class Sweep:
    """dL/dq (mm) about *joint* at posture *base* with *joint* at each of *values*."""

    def __init__(self, base: dict[str, float], joint: str, values: np.ndarray) -> None:
        self.plus = Poses([base | {joint: float(v) + H} for v in values])
        self.minus = Poses([base | {joint: float(v) - H} for v in values])

    def arms(self, sites: list[tuple[str, np.ndarray]]) -> np.ndarray:
        return (self.plus.length(sites) - self.minus.length(sites)) / (2.0 * H) * 1e3


def thumb_neutral() -> dict[str, float]:
    """Smutz 1998 neutral: mid-point of the thumb CMC ranges, MP at 0."""
    model = _model()
    return {
        "cmc_flexion_r": float(np.mean(model.joint("cmc_flexion_r").range)),
        "cmc_abduction_r": float(np.mean(model.joint("cmc_abduction_r").range)),
        "mp_flexion_r": 0.0,
    }


def joint_q(joint: str, deg: float) -> float:
    """MyoHand angle of an anatomical angle *deg* (flexion, abduction positive); thumb: offset from neutral."""
    if joint not in THUMB_BOX_DEG:
        return deg * DEG
    sign = {"cmc_flexion_r": -1.0, "cmc_abduction_r": 1.0, "mp_flexion_r": -1.0}[joint]
    return thumb_neutral()[joint] + sign * deg * DEG


def crossed(row: dict[str, Any]) -> list[str]:
    """Joints of the box and range checks that *row* crosses (from the bodies of its sites)."""
    bodies = {s["body"] for s in row["sites"]}
    joints = []
    if bodies & {"firstmc_r", "proximal_thumb_r"}:
        joints += ["cmc_flexion_r", "cmc_abduction_r"] + (["mp_flexion_r"] if "proximal_thumb_r" in bodies else [])
    if "2proxph_r" in bodies:
        joints += ["mcp2_flexion_r", "mcp2_abduction_r"]
    if "5proxph_r" in bodies:
        joints += ["mcp5_flexion_r", "mcp5_abduction_r"]
    if bodies & {"fifthmc_r", "5proxph_r"} and bodies & PROXIMAL_OF_MC5:
        joints += ["cmc5_flexion_r"]
    return joints


def _grid(ranges: dict[str, Any]) -> list[dict[str, float]]:
    postures: list[dict[str, float]] = [{}]
    for joint, values in ranges.items():
        postures = [p | {joint: float(v)} for p in postures for v in values]
    return postures


def boxes(row: dict[str, Any]) -> tuple[Poses, Poses]:
    """Measurement box and full range of motion of the joints *row* crosses (``path_fit.measurement_box``)."""
    model = _model()
    joints = crossed(row)
    box = {j: [joint_q(j, d) for d in (THUMB_BOX_DEG | INDEX_BOX_DEG | LITTLE_BOX_DEG)[j]] for j in joints}
    full = {j: np.linspace(*model.joint(j).range, 5) for j in joints}
    return Poses(_grid(box)), Poses(_grid(full))


def target_sweep(joint: str, target: dict[str, Any]) -> tuple[Sweep, np.ndarray, np.ndarray, bool]:
    """Sweep, mean, SD and whether the model value is averaged over the sweep (``over``), for one target."""
    if "over" in target:  # index MCP2: averaged over the range (An et al. 1983)
        return (*average_sweep(joint), np.array([target["mean_mm"]]), np.array([target["sd_mm"]]), True)
    values = np.array([joint_q(joint, d) for d in target["posture_deg"]])
    return Sweep(thumb_neutral(), joint, values), np.atleast_1d(target["mean_mm"]), np.atleast_1d(target["sd_mm"]), False


def average_sweep(joint: str) -> tuple[Sweep]:
    """MCP2 flexion 0-90 deg or abduction -15 to 15 deg, the other joints at 0."""
    values = np.radians(np.linspace(0.0, 90.0, 10) if joint == "mcp2_flexion_r" else np.linspace(-15.0, 15.0, 7))
    return (Sweep({}, joint, values),)


def fit_group(table: dict[str, Any], group: str) -> None:
    """Fit the sites of the rows of *group* together and record positions and results in the table."""
    from scipy.optimize import least_squares

    cfg = table["path_fit"]
    floor = cfg["sd_floor_mm"]
    rows = [r for r in table["muscles"] if r["fit"]["group"] == group]
    forces = np.array([r["force_N"]["value"] for r in rows])
    weights = forces / forces.sum()
    sweeps = [(j, *target_sweep(j, t)) for j, t in rows[0]["fit"].get("targets", {}).items()]
    head_sweeps = [
        (k, j, *target_sweep(j, t)) for k, r in enumerate(rows[1:], 1) for j, t in r["fit"].get("targets", {}).items()
    ]
    names = [r["name"] for r in rows]
    order = [average_sweep(j)[0] for j in ("mcp2_flexion_r", "mcp2_abduction_r")] if {"DI1d", "DI1s"} <= set(names) else []
    windows = [boxes(r) for r in rows]
    lo_fl, hi_fl = cfg["fibre_length_window"]
    x0 = np.concatenate([np.ravel([s["start"]["pos"] for s in r["sites"]]) for r in rows])
    offsets = np.cumsum([0] + [3 * len(r["sites"]) for r in rows])
    rest = Poses([{}])

    def paths(x: np.ndarray) -> list[list[tuple[str, np.ndarray]]]:
        return [
            [(s["body"], x[offsets[k] + 3 * i : offsets[k] + 3 * i + 3]) for i, s in enumerate(r["sites"])]
            for k, r in enumerate(rows)
        ]

    def normalised(path: list, poses: Poses, row: dict[str, Any]) -> np.ndarray:
        l_fo = row["optimal_fiber_length_m"]["value"]
        l_ts = (rest.length(path)[0] - l_fo * np.cos(np.radians(row["pennation_deg"]["value"]))) / 1.033
        return (poses.length(path) - l_ts) / l_fo

    def model_values(p: list) -> list[np.ndarray]:
        out = []
        for _, sweep, _, _, over in sweeps:
            arms = sum(w * sweep.arms(path) for w, path in zip(weights, p))
            out.append(np.atleast_1d(arms.mean()) if over else arms)
        return out

    def head_values(p: list) -> list[np.ndarray]:
        return [np.atleast_1d(s.arms(p[k]).mean()) if over else s.arms(p[k]) for k, _, s, _, _, over in head_sweeps]

    def residuals(x: np.ndarray) -> np.ndarray:
        p = paths(x)
        res = [(v - m) / np.maximum(sd, floor) for v, (_, _, m, sd, _) in zip(model_values(p), sweeps)]
        res += [(v - m) / np.maximum(sd, floor) for v, (_, _, _, m, sd, _) in zip(head_values(p), head_sweeps)]
        if len(rows) > 1:  # every head acts in the direction of the group's clear targets (|mean| >= 2 mm)
            for _, sweep, mean, _, over in sweeps:
                for path in p:
                    arms = np.atleast_1d(sweep.arms(path).mean()) if over else sweep.arms(path)
                    clear = np.abs(mean) >= 2.0
                    res.append(np.where(clear, np.maximum(0.0, -np.sign(mean) * arms), 0.0))
        if order:  # Masquelet 1986: the deep head is the larger MCP2 flexor, the superficial head the larger abductor
            deep, superficial = p[names.index("DI1d")], p[names.index("DI1s")]
            res.append(max(0.0, order[0].arms(deep).mean() - order[0].arms(superficial).mean()))
            res.append(max(0.0, order[1].arms(superficial).mean() - order[1].arms(deep).mean()))
        res.append((x - x0) / cfg["regularisation_m"])
        for path, row, (box, full) in zip(p, rows, windows):
            inside = normalised(path, box, row)
            res.append(PENALTY * np.maximum(0.0, lo_fl - inside))
            res.append(PENALTY * np.maximum(0.0, inside - hi_fl))
            if cfg["full_range_max"] is not None:  # bounds the passive force at the ends of the joint ranges
                res.append(PENALTY * np.maximum(0.0, normalised(path, full, row) - cfg["full_range_max"]))
        return np.concatenate([np.atleast_1d(np.asarray(r, dtype=float)) for r in res])

    bound = cfg["bound_m"]
    x = np.round(least_squares(residuals, x0, bounds=(x0 - bound, x0 + bound), x_scale=1e-3).x, 6)
    p, p0 = paths(x), paths(x0)
    values = model_values(p)
    z = np.concatenate([np.zeros(0)] + [(v - m) / np.maximum(sd, floor) for v, (_, _, m, sd, _) in zip(values, sweeps)])
    for k, (row, path, (box, full)) in enumerate(zip(rows, p, windows)):
        for i, site in enumerate(row["sites"]):
            site["pos"] = [float(v) for v in x[offsets[k] + 3 * i : offsets[k] + 3 * i + 3]]
        inside, everywhere = normalised(path, box, row), normalised(path, full, row)
        result: dict[str, Any] = {
            "site_shift_mm": [
                round(float(np.linalg.norm(np.subtract(s["pos"], s["start"]["pos"])) * 1e3), 2) for s in row["sites"]
            ],
            "fibre_length_measurement_box": [round(float(inside.min()), 3), round(float(inside.max()), 3)],
            "fibre_length_full_range": [round(float(everywhere.min()), 3), round(float(everywhere.max()), 3)],
        }
        if k == 0 and sweeps:
            result["model_mm"] = {j: [round(float(v), 2) for v in value] for (j, *_), value in zip(sweeps, values)}
            result["start_mm"] = {j: [round(float(v), 2) for v in value] for (j, *_), value in zip(sweeps, model_values(p0))}
            result["rms_z"] = round(float(np.sqrt(np.mean(z**2))), 3)
            if len(rows) > 1:
                result["force_weights"] = {n: round(float(w), 3) for n, w in zip(names, weights)}
        if row["fit"].get("reference"):  # reported against the row's reference targets, not fitted to them
            result["reference_mm"] = {
                j: [round(float(a), 2) for a in target_sweep(j, t)[0].arms(path)] for j, t in row["fit"]["reference"].items()
            }
        own = [(j, v) for (kk, j, *_), v in zip(head_sweeps, head_values(p)) if kk == k]
        if own:
            result["model_mm"] = {j: [round(float(a), 2) for a in v] for j, v in own}
        if len(rows) > 1:  # this head alone on the group's targets
            result["head_mm"] = {
                j: [round(float(a), 2) for a in (np.atleast_1d(s.arms(path).mean()) if over else s.arms(path))]
                for j, s, _, _, over in sweeps
            }
        row["fit"]["result"] = result


def fit(table: dict[str, Any]) -> None:
    """Refit the sites of every fit group."""
    for group in dict.fromkeys(r["fit"]["group"] for r in table["muscles"]):
        fit_group(table, group)


def _passive_script() -> Any:
    import importlib.util
    from pathlib import Path

    path = Path(__file__).with_name("fit_passive_moments.py")
    spec = importlib.util.spec_from_file_location("fit_passive_moments", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fit_passive_scale(table: dict[str, Any]) -> None:
    """Fit the passive-force scale (``fpmax``) of the rows that cross the joints of ``passive_fit.rows``.

    MuJoCo's passive force is linear in ``fpmax``, so each joint moment is the other muscles' moment plus
    ``sum fpmax_i m_i(q)`` (only the muscles a row's source includes, if it names them). Loss as for the extrinsic muscles (``fit_muscle_passive.py``): per row of
    ``passive_fit.rows`` (passive_joint_moments.json) the mean of ``excess_weight * excess^2 + deficit^2``
    (N cm) over the row's fit range, plus ``penalty * |ln(fpmax / reference)|``; the joint springs are
    fitted afterwards to what remains (``fit_passive_moments.py``).
    """
    from scipy.optimize import minimize

    from myo_sim.build.hand_data import load_passive_moment_table

    cfg = table["passive_fit"]
    passive = _passive_script()
    rows = [r for r in load_passive_moment_table()["joints"] if r["joint"] in cfg["rows"]]
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    joints = {prow["names"]["r"] for prow in rows}
    fitted = [r for r in table["muscles"] if set(crossed(r)) & joints]
    ids = [model.actuator(f"{r['name']}_r").id for r in fitted]
    terms = []
    for prow in rows:
        joint = model.joint(prow["names"]["r"])
        span = prow["source_curve"].get("fit_range_deg")  # else the joint range
        lo, hi = np.radians(span) if span else model.jnt_range[joint.id]
        q = np.linspace(lo, hi, 25)
        target = passive.source_torque(prow["source_curve"], model.jnt_range[joint.id])(q) * 100.0
        unit, other = np.zeros((len(q), len(ids))), np.zeros(len(q))
        for n, value in enumerate(q):
            data.qpos[:] = model.qpos0
            data.qpos[joint.qposadr[0]] = value
            data.qvel[:] = data.act[:] = data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)
            moment = np.zeros((model.nu, model.nv))
            mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
            per = data.actuator_force * moment[:, joint.dofadr[0]] * 100.0
            unit[n] = per[ids] / model.actuator_gainprm[ids, 7]
            named = prow["source_curve"].get("subtract_model_muscles", {}).get("muscles")
            counted = [model.actuator(m).id for m in named] if named else range(model.nu)
            unit[n] *= np.array([float(i in counted) for i in ids])
            other[n] = sum(per[i] for i in counted if i not in ids)
        terms.append((prow["joint"], target, unit, other))
    reference, (f_lo, f_hi) = cfg["fpmax_reference"], cfg["fpmax_bounds"]

    def cost(log_f: np.ndarray) -> float:
        total = 0.0
        for _, target, unit, other in terms:
            moment = other + unit @ np.exp(log_f)
            inside = np.clip(moment, np.minimum(target, 0.0), np.maximum(target, 0.0))
            total += np.mean(cfg["excess_weight"] * (moment - inside) ** 2 + (target - inside) ** 2)
        return total + cfg["penalty"] * np.sum(np.abs(log_f - np.log(reference)))

    start = np.full(len(ids), np.log(reference))
    log_f = minimize(cost, start, method="L-BFGS-B", bounds=[(np.log(f_lo), np.log(f_hi))] * len(ids)).x
    for row, value in zip(fitted, np.exp(log_f)):
        row["fpmax"] = {"value": float(f"{value:.4g}"), "reference_value": reference, "method": "passive_fit"}
    cfg["result"] = {"cost_before": round(float(cost(start)), 3), "cost_after": round(float(cost(log_f)), 3)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="refit the sites to the tabled moment arms first")
    args = parser.parse_args()
    table = load_intrinsic_table()
    if args.fit:
        fit(table)
        for row in table["muscles"]:
            row.pop("fpmax", None)
        write_xml(table)  # the passive scale is fitted on the model with the fitted paths
        fit_passive_scale(table)
        INTRINSIC_DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    ranges = write_xml(table)
    for row in table["muscles"]:
        lo, hi = ranges[row["name"]]
        print(f"{row['name']}: lengthrange {lo:.5f} {hi:.5f}; {json.dumps(row['fit'].get('result', {}))}")


if __name__ == "__main__":
    main()
