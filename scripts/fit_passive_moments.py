"""Fit the passive-moment springs of ``build/data/passive_joint_moments.json`` to their source curves.

Each fitted row keeps its source curve (published model and parameters) under
``source_curve``. This script evaluates that curve in MyoHand joint coordinates
(internal passive moment, Nm, positive along the joint axis), fits one-sided
polynomial springs on each side of its equilibrium by non-negative least
squares, and writes the spring segments and the fit error back into the row.

Run: ``python scripts/fit_passive_moments.py`` (rewrites the JSON in place).
"""

from __future__ import annotations

import json
from typing import Any, Callable

import numpy as np
from scipy.optimize import brentq, nnls

import myo_sim
from myo_sim.build.passive import DATA_PATH, passive_moment

DEG = np.pi / 180.0


def knutson2000_intrinsic(p: dict[str, float], deg: np.ndarray) -> np.ndarray:
    """Knutson et al. 2000 Eq. (2): intrinsic MCP moment (N cm, +flexion) at flexion angle *deg*."""
    return p["A1"] * (np.exp(-p["k1"] * (deg - p["theta1"])) - 1.0) - p["A2"] * (np.exp(p["k2"] * (deg - p["theta2"])) - 1.0)


def domalain2010_fe(p: dict[str, float], rel: np.ndarray) -> np.ndarray:
    """Domalain et al. 2010 Eq. (2) without the abduction term: external F-E moment (N cm) at phi - phi_max (deg)."""
    return p["a"] * np.exp(p["b"] * rel)


def domalain2010_aa(p: dict[str, float], rel: np.ndarray) -> np.ndarray:
    """Domalain et al. 2010 Eq. (3) without the flexion term: external A-A moment (N cm) at theta - theta_max (deg)."""
    return p["g"] + p["h"] * np.exp(p["i"] * rel) - p["j"] * np.exp(p["k"] * rel)


def source_torque(curve: dict[str, Any], joint_range: np.ndarray) -> Callable[[np.ndarray], np.ndarray]:
    """Internal passive moment (Nm) along the MyoHand joint axis as a function of q (rad)."""
    p, sign = curve["params"], curve["joint_sign"]  # sign: +1 when +q is the source's positive angle
    if curve["model"] == "knutson2000_intrinsic":
        return lambda q: sign * knutson2000_intrinsic(p, sign * np.asarray(q) / DEG) / 100.0
    if curve["model"] == "domalain2010_fe":
        # Aligned at the MyoHand extension limit (the source's phi_max); +q = extension here.
        q_ext = joint_range[1] if sign < 0 else joint_range[0]
        return lambda q: -sign * domalain2010_fe(p, sign * (np.asarray(q) - q_ext) / DEG) / 100.0
    if curve["model"] == "domalain2010_aa":
        # Aligned at the MyoHand abduction limit (the source's theta_max); external -> internal: minus.
        q_abd = joint_range[1] if sign > 0 else joint_range[0]
        return lambda q: -sign * domalain2010_aa(p, sign * (np.asarray(q) - q_abd) / DEG) / 100.0
    raise ValueError(f"unknown source model {curve['model']!r}")


def _fit_side(tau: Callable, q: np.ndarray, onset: float, sign: float, order: int = 3) -> list[float]:
    """Non-negative [k0, k1, k2] of ``-sign (k0 x + k1 x^2 + k2 x^3)``, x = sign (q - onset) >= 0.

    *order* < 3 drops the highest terms (less growth when extrapolated).
    """
    x = sign * (q - onset)
    keep = x > 0
    if keep.sum() < 3:
        return [0.0, 0.0, 0.0]
    basis = np.stack([x[keep] ** (n + 1) for n in range(order)], axis=1)
    coef, _ = nnls(basis, -sign * tau(q[keep]))
    return [float(c) for c in coef] + [0.0] * (3 - order)


def fit_row(row: dict[str, Any], joint_range: np.ndarray) -> None:
    """Fill ``positive`` / ``negative`` springs and ``fit`` of a row with a ``source_curve``."""
    curve = row["source_curve"]
    raw = source_torque(curve, joint_range)
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
    if np.any(t > 0) and np.any(t < 0):
        k = int(np.flatnonzero(np.diff(np.sign(t)) != 0)[0])
        onset_pos = onset_neg = brentq(tau, q[k], q[k + 1])
    else:  # one-sided: the spring starts where the source moment is 1 % of its peak
        mag = np.abs(t)
        side = 1.0 if t[np.argmax(mag)] < 0 else -1.0  # restoring moment opposes the side it acts on
        inside = q[mag >= 0.01 * mag.max()]
        onset_pos = onset_neg = float(inside.min() if side > 0 else inside.max())
    for direction, sign, onset in (("positive", 1.0, onset_pos), ("negative", -1.0, onset_neg)):
        n = curve.get("segments", 1)
        n_segments = int(n[direction] if isinstance(n, dict) else n)
        o = curve.get("order", 3)
        order = int(o[direction] if isinstance(o, dict) else o)
        row.pop(direction, None)
        segments = []
        coef = _fit_side(tau, q, onset, sign, order)
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
                extra = _fit_side(residual, q, float(start), sign, order)
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
    inside = (q >= measured[0]) & (q <= measured[1])
    err = passive_moment(row, q) - t
    row["fit"] = {
        "range_rad": [lo, hi],
        "measured_range_rad": [float(measured[0]), float(measured[1])],
        "rmse_Nm": float(np.sqrt(np.mean(err[inside] ** 2))),
        "max_abs_err_Nm": float(np.max(np.abs(err[inside]))),
        "source_peak_Nm": float(np.max(np.abs(t[inside]))),
    }
    if not inside.all():
        row["fit"]["max_abs_err_outside_measured_Nm"] = float(np.max(np.abs(err[~inside])))
        row["fit"]["max_abs_moment_outside_measured_Nm"] = float(np.max(np.abs(passive_moment(row, q[~inside]))))


def main() -> None:
    table = json.loads(DATA_PATH.read_text())
    model = myo_sim.load_spec("myoarm_r").compile()
    for row in table["joints"]:
        if "source_curve" not in row:
            continue
        joint_range = model.jnt_range[model.joint(row["names"]["r"]).id]
        fit_row(row, joint_range)
        f = row["fit"]
        out = (
            f", outside measured: max err {f['max_abs_err_outside_measured_Nm']:.3f}, max moment {f['max_abs_moment_outside_measured_Nm']:.3f} Nm"
            if "max_abs_err_outside_measured_Nm" in f
            else ""
        )
        print(
            f"{row['joint']:24s} rmse {f['rmse_Nm']:.4f} Nm, max {f['max_abs_err_Nm']:.4f} Nm (peak {f['source_peak_Nm']:.3f} Nm){out}"
        )
    DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")


if __name__ == "__main__":
    main()
