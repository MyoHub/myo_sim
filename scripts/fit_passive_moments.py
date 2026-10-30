"""Fit the passive-moment springs of ``build/data/passive_joint_moments.json`` and write the XML fragments.

Each fitted row keeps its source curve (published model and parameters) under
``source_curve``. This script evaluates that curve in MyoHand joint coordinates
(internal passive moment, Nm, positive along the joint axis), fits one-sided
polynomial springs on each side of its equilibrium by non-negative least
squares, and writes the spring segments and the fit error back into the row.
It then rewrites its generated block in ``myoarm_r_tendon.xml`` (one fixed
tendon per spring segment of every applied row), the model's source of truth,
and the ``damping`` and ``range`` attributes of every joint whose row has a
``damping`` or ``range`` entry in ``myoarm_r_chain.xml``. The fit runs over the
joint ranges of the model as it is, so after a ``range`` change write the XML
first (``--xml-only``) and then refit.

Run: ``uv run python scripts/fit_passive_moments.py`` (rewrites the JSON, the
block and the joint attributes); ``--xml-only`` rewrites the XML from the JSON as is.
"""

from __future__ import annotations

import argparse
import json
import re
from typing import Any, Callable

import numpy as np

import myo_sim
from myo_sim.build.compose import RIGHT_ARM_CHAIN_XML, RIGHT_ARM_TENDONS_XML
from myo_sim.build.hand_data import (
    APPLIED_STATUSES,
    PASSIVE_DATA_PATH,
    fold_node_forces,
    passive_moment,
    passive_tendon_name,
    place_tendon_nodes,
    rows_with_status,
    spring_parameters,
    spring_segments,
    write_generated_block,
)

SCRIPT = "fit_passive_moments.py"

LIGAMENT_PREFIX = "lig_"  # passive ligament tendons (extensor mechanism ORL): part of the model's own passive moment
DEG = np.pi / 180.0


def knutson2000_intrinsic(p: dict[str, float], deg: np.ndarray) -> np.ndarray:
    """Knutson et al. 2000 Eq. (2): intrinsic MCP moment (N cm, +flexion) at flexion angle *deg*."""
    return p["A1"] * (np.exp(-p["k1"] * (deg - p["theta1"])) - 1.0) - p["A2"] * (np.exp(p["k2"] * (deg - p["theta2"])) - 1.0)


def kamper2002_static(p: dict[str, float], rad: np.ndarray) -> np.ndarray:
    """Kamper et al. 2002 Table 2: static passive torque (N m, +flexion), cubic in the flexion angle (rad).

    With ``hold_range_deg`` the torque is held at its values at the first and
    last measured operating points instead of extrapolating the cubic.
    """
    rad = np.asarray(rad, dtype=float)
    if "hold_range_deg" in p:
        rad = np.clip(rad, p["hold_range_deg"][0] * DEG, p["hold_range_deg"][1] * DEG)
    return p["c3"] * rad**3 + p["c2"] * rad**2 + p["c1"] * rad + p["c0"]


def linear_two_sided(p: dict[str, float], rad: np.ndarray) -> np.ndarray:
    """Direction-specific linear stiffness about neutral: internal moment (N m) at angle *rad*."""
    rad = np.asarray(rad, dtype=float)
    return np.where(rad > 0.0, -p["k_positive"] * rad, -p["k_negative"] * rad)


def ligament_moment(model: object, data: object, dof: int) -> float:
    """Passive moment (N m) at *dof* of the ligament tendons (``lig_``), from the forward-computed *data*."""
    import mujoco

    jac = np.zeros((model.ntendon, model.nv))
    mujoco.mju_sparse2dense(jac, data.ten_J, model.ten_J_rownnz, model.ten_J_rowadr, model.ten_J_colind)
    total = 0.0
    for t in range(model.ntendon):
        if not model.tendon(t).name.startswith(LIGAMENT_PREFIX):
            continue
        lo, hi = model.tendon_lengthspring[t]
        x = min(0.0, data.ten_length[t] - lo) + max(0.0, data.ten_length[t] - hi)
        k0, k1, k2 = model.tendon_stiffness[t] if np.ndim(model.tendon_stiffness[t]) else (model.tendon_stiffness[t], 0.0, 0.0)
        total += -(k0 * x + k1 * x * abs(x) + k2 * x**3) * jac[t, dof]
    return float(total)


def muscle_baseline(curve: dict[str, Any], joint: str, q: np.ndarray) -> np.ndarray:
    """Passive moment (N m) at *joint* of the model's own muscles (zero activation) and ligaments, other joints per ``posture_deg``.

    ``muscles`` (optional) restricts the sum to the muscles the source curve
    includes (no ligaments then); by default every muscle and ligament counts.
    """
    import mujoco

    base = curve["subtract_model_muscles"]
    model = myo_sim.load_spec(base["model"]).compile()
    data = mujoco.MjData(model)
    ids = [model.actuator(name).id for name in base["muscles"]] if "muscles" in base else None
    ligaments = "muscles" not in base  # a source that names its muscles excludes the ligaments too
    dof = model.joint(joint).dofadr[0]
    out = np.zeros_like(q)
    for i, value in enumerate(q):
        data.qpos[:] = model.qpos0
        for name, deg in base.get("posture_deg", {}).items():
            data.qpos[model.joint(name).qposadr[0]] = deg * DEG
        data.qpos[model.joint(joint).qposadr[0]] = value
        data.qvel[:] = 0.0
        data.act[:] = 0.0
        data.ctrl[:] = 0.0
        place_tendon_nodes(model, data)  # lumbrical nodes on their FDP ties
        mujoco.mj_forward(model, data)
        out[i] = ligament_moment(model, data, dof) if ligaments else 0.0
        if ids is None:
            out[i] += fold_node_forces(model, data, data.qfrc_actuator)[dof]
        else:
            moment = np.zeros((model.nu, model.nv))
            mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
            out[i] = float(data.actuator_force[ids] @ fold_node_forces(model, data, moment[ids])[:, dof])
    return out


def domalain2010_fe(p: dict[str, float], rel: np.ndarray) -> np.ndarray:
    """Domalain et al. 2010 Eq. (2) without the abduction term: external F-E moment (N cm) at phi - phi_max (deg)."""
    return p["a"] * np.exp(p["b"] * rel)


def domalain2010_aa(p: dict[str, float], rel: np.ndarray) -> np.ndarray:
    """Domalain et al. 2010 Eq. (3) without the flexion term: external A-A moment (N cm) at theta - theta_max (deg)."""
    return p["g"] + p["h"] * np.exp(p["i"] * rel) - p["j"] * np.exp(p["k"] * rel)


def source_torque(curve: dict[str, Any], joint_range: np.ndarray) -> Callable[[np.ndarray], np.ndarray]:
    """Internal passive moment (Nm) along the MyoHand joint axis as a function of q (rad)."""
    p, sign = curve["params"], curve["joint_sign"]  # sign: +1 when +q is the source's positive angle
    if curve["model"] == "linear_two_sided":
        return lambda q: sign * linear_two_sided(p, sign * np.asarray(q))
    if curve["model"] == "knutson2000_intrinsic":
        return lambda q: sign * knutson2000_intrinsic(p, sign * np.asarray(q) / DEG) / 100.0
    if curve["model"] == "kamper2002_static":
        return lambda q: sign * kamper2002_static(p, sign * np.asarray(q))
    if curve["model"] == "domalain2010_fe":
        # Aligned at the MyoHand extension limit (the source's phi_max); +q = extension here.
        q_ext = joint_range[1] if sign < 0 else joint_range[0]
        return lambda q: -sign * domalain2010_fe(p, sign * (np.asarray(q) - q_ext) / DEG) / 100.0
    if curve["model"] == "domalain2010_aa":
        # Aligned at the MyoHand abduction limit (the source's theta_max); external -> internal: minus.
        q_abd = joint_range[1] if sign > 0 else joint_range[0]
        return lambda q: -sign * domalain2010_aa(p, sign * (np.asarray(q) - q_abd) / DEG) / 100.0
    raise ValueError(f"unknown source model {curve['model']!r}")


def _fit_side(tau: Callable, q: np.ndarray, onset: float, sign: float, order: int = 3, signed: bool = False) -> list[float]:
    """[k0, k1, k2] of ``-sign (k0 x + k1 x^2 + k2 x^3)``, x = sign (q - onset) >= 0.

    Non-negative (convex, monotone) by default; *signed* allows concave curves
    (checked to stay restoring by :func:`fit_row`). *order* < 3 drops the
    highest terms (less growth when extrapolated).
    """
    from scipy.optimize import nnls

    x = sign * (q - onset)
    keep = x > 0
    if keep.sum() < 3:
        return [0.0, 0.0, 0.0]
    basis = np.stack([x[keep] ** (n + 1) for n in range(order)], axis=1)
    y = -sign * tau(q[keep])
    coef = np.linalg.lstsq(basis, y, rcond=None)[0] if signed else nnls(basis, y)[0]
    return [float(c) for c in coef] + [0.0] * (3 - order)


def _no_spring(row: dict[str, Any], curve: dict[str, Any]) -> None:
    """Record that the model's muscles already supply the whole source moment of *row*."""
    row.pop("positive", None)
    row.pop("negative", None)
    row["fit"] = {"spring": "none: the model's passive muscle moment already reaches the source curve over its range"}


def fit_row(row: dict[str, Any], joint_range: np.ndarray) -> None:
    """Fill ``positive`` / ``negative`` springs and ``fit`` of a row with a ``source_curve``.

    The springs are fitted to the residual: the source curve minus the model's
    own passive muscle moment (act = 0) at the source's posture, so the total
    (muscles + springs) approaches the source and no moment is counted twice.
    The non-negative fit cannot reduce a muscle excess; without a restoring
    residual the row gets no spring.
    """
    from scipy.optimize import brentq

    curve = row["source_curve"]
    raw = source = source_torque(curve, joint_range)
    curve.setdefault("subtract_model_muscles", {"model": "myoarm_r", "posture_deg": {}})
    if "subtract_model_muscles" in curve:
        # Source moment at its posture minus what the model's muscles already give there.
        joint = row["names"]["r"]
        grid = np.linspace(joint_range[0], joint_range[1], 200)
        baseline = muscle_baseline(curve, joint, grid)
        curve["subtract_model_muscles"]["baseline_Nm"] = {
            "q_rad": [float(grid[0]), float(grid[-1]), 200],
            "moment": [round(float(v), 6) for v in baseline],
        }

        def raw(qq: np.ndarray, _src: Callable = source, _g: np.ndarray = grid, _b: np.ndarray = baseline) -> np.ndarray:
            return _src(qq) - np.interp(qq, _g, _b)

    lo, hi = (np.array(curve.get("fit_range_deg", np.degrees(joint_range))) * DEG).tolist()
    if curve.get("hold_outside_measured", False):
        # Outside the measured span the target is held at its value at the span's
        # edge instead of extrapolating the exponential; fit over the whole range.
        def tau(qq: np.ndarray, _raw: Callable = raw, _lo: float = lo, _hi: float = hi) -> np.ndarray:
            return _raw(np.clip(qq, _lo, _hi))

        measured = (lo, hi)
        lo, hi = float(joint_range[0]), float(joint_range[1])
    else:
        tau, measured = raw, (lo, hi)
    q = np.linspace(lo, hi, 400)
    t = tau(q)
    # The equilibrium may lie outside the fitted range (e.g. beyond a joint limit):
    # look for it up to 1 rad further on either side.
    wide = np.linspace(lo - 1.0, hi + 1.0, 1200)
    locate = tau
    if curve.get("model") == "kamper2002_static" and "hold_range_deg" in curve["params"]:
        # The equilibrium comes from the cubic itself, even when it lies a few degrees past the held range.
        unheld = {**curve, "params": {k: v for k, v in curve["params"].items() if k != "hold_range_deg"}}
        unheld.pop("subtract_model_muscles", None)
        locate = source_torque(unheld, joint_range)
        if "subtract_model_muscles" in curve:
            locate = (lambda f, g: lambda qq: f(qq) - (g(qq) - tau(qq)))(locate, raw)  # same muscle baseline
    tw = locate(wide)
    # A stable equilibrium: the moment goes from + (below) to - (above).
    crossings = np.flatnonzero((tw[:-1] > 0) & (tw[1:] <= 0))
    if crossings.size:
        mid = 0.5 * (lo + hi)
        k = int(crossings[np.argmin(np.abs(wide[crossings] - mid))])
        onset_pos = onset_neg = brentq(locate, wide[k], wide[k + 1])
    else:  # one-sided: the spring starts where the residual is 1 % of its peak
        mag = np.abs(t)
        if not mag.max() > 0.0:
            _no_spring(row, curve)
            return
        side = 1.0 if t[np.argmax(mag)] < 0 else -1.0  # restoring moment opposes the side it acts on
        inside = q[mag >= 0.01 * mag.max()]
        onset_pos = onset_neg = float(inside.min() if side > 0 else inside.max())
    # Where the model's muscles already exceed the source the residual pushes away from
    # equilibrium; it stays in the fit, so a spring that would add to that excess pays for it.
    eq = onset_pos
    excess = np.maximum(tau(q) * np.sign(q - eq), 0.0)
    curve["subtract_model_muscles"]["model_excess_max_Nm"] = float(excess.max())
    if not np.any(tau(q) * np.sign(q - eq) < 0.0):
        _no_spring(row, curve)
        return
    for direction, sign, onset in (("positive", 1.0, onset_pos), ("negative", -1.0, onset_neg)):
        n = curve.get("segments", 1)
        n_segments = int(n[direction] if isinstance(n, dict) else n)
        o = curve.get("order", 3)
        order = int(o[direction] if isinstance(o, dict) else o)
        signed = bool(curve.get("signed", False))
        row.pop(direction, None)
        segments = []
        coef = _fit_side(tau, q, onset, sign, order, signed)
        if any(coef):
            segments.append({"onset": float(onset), "stiffness": coef})
        # Greedy stacking: each extra segment fits what the previous ones miss,
        # from the onset (on a grid over this side) that lowers the error most.
        side = q[sign * (q - onset) > 0]
        for _ in range(n_segments - 1):
            if not segments or side.size < 6:
                break
            row[direction] = segments

            def residual(qq: np.ndarray, _row: dict[str, Any] = row) -> np.ndarray:
                return tau(qq) - passive_moment(_row, qq)

            best = None
            for start in side[1:-3]:
                extra = _fit_side(residual, q, float(start), sign, order, signed)
                trial = {**row, direction: [*segments, {"onset": float(start), "stiffness": extra}]}
                e = float(np.mean((passive_moment(trial, side) - tau(side)) ** 2))
                if best is None or e < best[0]:
                    best = (e, {"onset": float(start), "stiffness": extra})
            if best is not None and any(best[1]["stiffness"]):
                segments.append(best[1])
        if segments:
            row[direction] = segments if len(segments) > 1 else segments[0]
        else:
            row.pop(direction, None)
    if "positive" not in row and "negative" not in row:
        _no_spring(row, curve)
        return
    full = np.linspace(joint_range[0], joint_range[1], 400)
    fitted = passive_moment(row, full)
    eq = row.get("positive", row.get("negative"))
    eq = (eq[0] if isinstance(eq, list) else eq)["onset"]
    if np.any(fitted * (full - eq) > 1e-4):  # 0.1 mN m
        if curve.get("signed"):  # a concave fit overshot: fall back to the monotone (non-negative) fit
            curve["signed"] = False
            curve["signed_fallback"] = "the signed fit was not restoring everywhere; non-negative coefficients used"
            return fit_row(row, joint_range)
        raise ValueError(f"{row['joint']}: fitted moment is not restoring somewhere in the joint range")
    inside = (q >= measured[0]) & (q <= measured[1])
    err = passive_moment(row, q) - t
    row["fit"] = {
        "range_rad": [lo, hi],
        "measured_range_rad": [float(measured[0]), float(measured[1])],
        "rmse_Nm": float(np.sqrt(np.mean(err[inside] ** 2))),
        "rmse_without_spring_Nm": float(np.sqrt(np.mean(t[inside] ** 2))),
        "max_abs_err_Nm": float(np.max(np.abs(err[inside]))),
        "residual_peak_Nm": float(np.max(np.abs(t[inside]))),
        "source_peak_Nm": float(np.max(np.abs(source(q[inside])))),
    }
    if not inside.all():
        row["fit"]["max_abs_err_outside_measured_Nm"] = float(np.max(np.abs(err[~inside])))
        row["fit"]["max_abs_moment_outside_measured_Nm"] = float(np.max(np.abs(passive_moment(row, q[~inside]))))


def _number(value: float) -> str:
    text = repr(float(value) + 0.0)  # + 0.0 turns -0.0 into 0.0
    return text[:-2] if text.endswith(".0") else text


def tendon_block(table: dict[str, Any]) -> str:
    """Generated block of ``myoarm_r_tendon.xml``: the springs of every applied row (right-side joint names)."""
    lines = []
    for row in rows_with_status(table, APPLIED_STATUSES):
        if not (spring_segments(row, "positive") or spring_segments(row, "negative")):
            continue
        source = table["sources"][row["source"]]
        cite = f"{source['citation'].split()[0]} et al. {''.join(c for c in row['source'] if c.isdigit())}"
        lines.append(f"        <!-- {row['joint']} ({row['status']}): {cite}, doi:{source['doi']} -->")
        for direction in ("positive", "negative"):
            for i, part in enumerate(spring_segments(row, direction)):
                springlength, stiffness = spring_parameters(direction, part["onset"], part["stiffness"])
                lines += [
                    f'        <fixed name="{passive_tendon_name(row["joint"], direction, i)}"'
                    f' springlength="{" ".join(map(_number, springlength))}" stiffness="{" ".join(map(_number, stiffness))}">',
                    f'            <joint joint="{row["names"]["r"]}" coef="1"/>',
                    "        </fixed>",
                ]
    return "".join(f"{line}\n" for line in lines)


def write_block(table: dict[str, Any]) -> None:
    """Rewrite the generated spring block of ``myoarm_r_tendon.xml`` from *table*."""
    write_generated_block(RIGHT_ARM_TENDONS_XML, SCRIPT, tendon_block(table), "</tendon>")


def chain_with_joint_attributes(text: str, table: dict[str, Any]) -> str:
    """*text* of ``myoarm_r_chain.xml`` with each tabled ``damping`` and ``range`` on its right-side joint, rest unchanged."""
    for row in table["joints"]:
        attributes = {}
        if "damping" in row:
            attributes["damping"] = _number(row["damping"]["value"])
        if "range" in row:
            attributes["range"] = " ".join(_number(round(np.radians(v), 6)) for v in row["range"]["value_deg"])
        if not attributes:
            continue
        pattern = re.compile(rf'<joint\b[^>]*\bname="{re.escape(row["names"]["r"])}"[^>]*/>')
        tags = pattern.findall(text)
        if len(tags) != 1:
            raise ValueError(f"myoarm_r_chain.xml: expected one joint {row['names']['r']}, found {len(tags)}")
        tag = tags[0]
        for key, value in attributes.items():
            if f' {key}="' in tag:
                tag = re.sub(rf'\s{key}="[^"]*"', f' {key}="{value}"', tag)
            else:
                tag = tag[:-2].rstrip() + f' {key}="{value}"/>'
        text = text.replace(tags[0], tag)
    return text


def write_joint_attributes(table: dict[str, Any]) -> None:
    """Write the tabled joint damping and ranges into ``myoarm_r_chain.xml``."""
    RIGHT_ARM_CHAIN_XML.write_text(chain_with_joint_attributes(RIGHT_ARM_CHAIN_XML.read_text(), table))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--xml-only", action="store_true", help="only rewrite the XML block from the JSON")
    args = parser.parse_args()
    table = json.loads(PASSIVE_DATA_PATH.read_text())
    if args.xml_only:
        write_block(table)
        write_joint_attributes(table)
        return
    model = myo_sim.load_spec("myoarm_r").compile()
    for row in table["joints"]:
        if "source_curve" not in row:
            continue
        joint_range = model.jnt_range[model.joint(row["names"]["r"]).id]
        fit_row(row, joint_range)
        f = row["fit"]
        if "spring" in f:
            print(f"{row['joint']:24s} no spring ({f['spring']})")
            continue
        out = (
            f", outside measured: max err {f['max_abs_err_outside_measured_Nm']:.3f}, max moment {f['max_abs_moment_outside_measured_Nm']:.3f} Nm"
            if "max_abs_err_outside_measured_Nm" in f
            else ""
        )
        print(
            f"{row['joint']:24s} rmse {f['rmse_Nm']:.4f} Nm, max {f['max_abs_err_Nm']:.4f} Nm (peak {f['source_peak_Nm']:.3f} Nm){out}"
        )
    PASSIVE_DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
    write_block(table)
    write_joint_attributes(table)


if __name__ == "__main__":
    main()
