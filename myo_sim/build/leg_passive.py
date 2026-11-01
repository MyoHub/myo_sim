"""Fit the leg muscles' tendon slack lengths and passive-force scales to measured passive joint moments.

Targets: Silder et al. (2007, J Biomech 40:2628-2635, doi:10.1016/j.jbiomech.2006.12.017) measured passive hip, knee and ankle
moments in 20 relaxed young adults over 15 trials stretching the uni- and bi-articular groups, and fitted an energy-conserving
exponential model of all three moments as functions of hip, knee and ankle angle (Table 4, Appendix). The model's passive
moments at zero activation are fitted to the mean model over postures covering those trials.

Method (as the hand extrinsics in Vittorio-Caggiano/myo_sim#2, after Binder-Markey & Murray 2017): MuJoCo's muscles have rigid
tendons, so a tendon slack change is a shift of ``actuator_lengthrange`` at constant ``L0``, and ``fpmax`` (``gainprm[7]``)
scales the passive force without changing the active force. Per muscle both are fitted with
- a one-sided loss: a moment beyond the measured total (which includes ligaments and other tissue), or opposite to it, costs
  ``EXCESS_WEIGHT`` times a deficit;
- penalties on the slack shift (in units of L0) and on ``|ln(fpmax / reference)|``; the shift is bounded by ``MAX_SHIFT`` L0;
- fascicle-length targets from ultrasound (Narici et al. 1996, medial gastrocnemius at rest), so the
  slack lengths keep the measured operating lengths;
- an active constraint: the active force-length factor at the angle of the group's maximal voluntary strength (Anderson,
  Madigan & Nussbaum 2007, J Biomech 40:3105-3113, doi:10.1016/j.jbiomech.2007.03.022, same test postures) stays at least
  ``MIN_FL``.
Passive moments are evaluated with MuJoCo's passive curve on precomputed path lengths and effective moment arms (dL/dtheta with the
joint-equality followers, e.g. knee translations and patella, resolved; they do not depend on the fitted parameters);
``tests/test_leg_passive.py`` checks them against ``mj_forward`` forces.

Silder Table 4 lives in ``myo_sim.build.references.SILDER2007``. Fit-time grids stay in this module; ``--write``
rewrites XML only.

Usage::

    uv run python scripts/fit_leg_passive.py          # fit and print
    uv run python scripts/fit_leg_passive.py --write  # rewrite range/lengthrange and fpmax (both sides)

Run after ``scripts/fit_leg_rigid_maps.py --write``: the shift is applied on top of the converted maps.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import mujoco
import numpy as np

from myo_sim.build.references import SILDER2007
from myo_sim.build.rigid_conversion import LEG_MUSCLE_XML, effective_moment_arms, sample_postures

T_MAX = 40.0  # N m; the exponential fits are not used beyond this
EXCESS_WEIGHT = 10.0
MAX_SHIFT = 0.15  # |slack shift| / L0
SHIFT_PENALTY = 20.0  # N m per L0 of shift
FPMAX_BOUNDS = (0.002, 2.6)
FPMAX_PENALTY = 1.0  # N m per unit |ln(fpmax / reference)|
MIN_FL = 0.5
RANGE_MARGIN = 0.05  # keep the optimal length at least this far inside the shifted operating range
JOINTS = ("hip_flexion_r", "knee_angle_r", "ankle_angle_r")
# Fit-time configuration. Table 4 itself is SILDER2007.
GROUPS = {
    "hamstrings": ["semimem", "semiten", "bflh"],
    "knee_flexors_uni": ["bfsh"],
    "rectus_femoris": ["recfem"],
    "knee_extensors": ["vaslat", "vasmed", "vasint"],
    "gastrocnemius": ["gasmed", "gaslat"],
    "plantarflexors_uni": ["soleus", "tibpost", "fdl", "fhl", "perlong", "perbrev"],
    "hip_flexors": ["iliacus", "psoas", "tfl", "sart"],
    "hip_extensors": ["glmax1", "glmax2", "glmax3", "addmagDist", "addmagIsch", "addmagMid", "addmagProx"],
}
SILDER_POSTURES_DEG = {
    "hip_knee_grid": {"hip": [-15, 0, 15, 30, 45, 60, 75, 85], "knee": [0, 10, 20, 35, 60, 90, 120], "ankle": -20},
    "ankle_sweep": {"hip": 15, "knee": [0, 30, 60, 90, 120], "ankle": [-20, -10, 0, 10, 20]},
}
ANDERSON_STRENGTH = {
    "hip_extension": {
        "joint": "hip_flexion_r",
        "posture_deg": {"knee_angle_r": 10},
        "peak_deg": 53.4,
        "muscles": ["glmax1", "glmax2", "glmax3", "addmagIsch", "semimem", "semiten", "bflh"],
    },
    "hip_flexion": {
        "joint": "hip_flexion_r",
        "posture_deg": {"knee_angle_r": 10},
        "peak_deg": -12.3,
        "muscles": ["iliacus", "psoas", "recfem", "tfl", "sart"],
    },
    "knee_extension": {
        "joint": "knee_angle_r",
        "posture_deg": {"hip_flexion_r": 70},
        "peak_deg": 64.9,
        "muscles": ["vaslat", "vasmed", "vasint", "recfem"],
    },
    "knee_flexion": {
        "joint": "knee_angle_r",
        "posture_deg": {"hip_flexion_r": 70},
        "peak_deg": 29.9,
        "muscles": ["semimem", "semiten", "bflh", "bfsh"],
    },
    "plantarflexion": {
        "joint": "ankle_angle_r",
        "posture_deg": {"hip_flexion_r": 80, "knee_angle_r": 50},
        "peak_deg": 23.4,
        "muscles": ["soleus"],
    },
}
NARICI_FASCICLE = {
    "weight_Nm_per_mm": 2.0,
    "rows": [
        {"muscle": "gasmed", "posture_deg": {"hip_flexion_r": 0, "knee_angle_r": 0, "ankle_angle_r": 0}, "mm": 57.0},
        {"muscle": "gasmed", "posture_deg": {"hip_flexion_r": 0, "knee_angle_r": 0, "ankle_angle_r": -20}, "mm": 50.8},
        {"muscle": "gasmed", "posture_deg": {"hip_flexion_r": 0, "knee_angle_r": 0, "ankle_angle_r": -40}, "mm": 41.7},
    ],
}


def load_passive_data() -> dict:
    return SILDER2007


def silder_moments(p: dict, hip: np.ndarray, knee: np.ndarray, ankle: np.ndarray) -> np.ndarray:
    """Silder et al. (2007) mean passive moments (N m) at hip, knee, ankle; angles in rad, flexion/dorsiflexion positive."""
    a = {k: np.radians(v["a_deg"]) for k, v in p.items()}

    def bi(name: str, prox: np.ndarray, dist: np.ndarray, sign: float) -> np.ndarray:
        bp, bd = p[name]["b_prox"], p[name]["b_dist"]
        return np.exp(sign * bp * (prox - (bd / bp) * dist - a[name]))

    # signs from the stretch directions: RF (hip extension, knee flexion), HAM (hip flexion, knee extension), GAS (knee
    # extension, ankle dorsiflexion); each term is 1 N m at its offset with the distal angle at zero (Appendix)
    rf, ham, gas = bi("RF", hip, knee, -1.0), bi("HAM", hip, knee, 1.0), bi("GAS", knee, ankle, -1.0)
    uni = {k: np.exp(p[k]["b"] * (th - a[k])) for k, th in (("HE", hip), ("KE", knee), ("PF", ankle))}
    uni.update({k: np.exp(-p[k]["b"] * (th - a[k])) for k, th in (("HF", hip), ("KF", knee))})
    m_hip = rf + uni["HF"] - ham - uni["HE"]  # Eq. A.5
    m_knee = (
        -(p["RF"]["b_dist"] / p["RF"]["b_prox"]) * rf
        - uni["KE"]
        + (p["HAM"]["b_dist"] / p["HAM"]["b_prox"]) * ham
        + uni["KF"]
        + gas
    )  # Eq. A.6
    m_ankle = -(p["GAS"]["b_dist"] / p["GAS"]["b_prox"]) * gas - uni["PF"]  # Eq. A.7
    return np.stack([m_hip, m_knee, m_ankle], axis=-1)


def passive_fl(ln: np.ndarray, lmax: np.ndarray, fpmax: np.ndarray) -> np.ndarray:
    """MuJoCo's passive force-length curve (``mju_muscleBias``) as a fraction of F0."""
    b = 0.5 * (1.0 + lmax)
    x1, x2 = (ln - 1.0) / (b - 1.0), (ln - b) / (b - 1.0)
    return np.where(ln <= 1.0, 0.0, np.where(ln <= b, fpmax * 0.5 * x1 * x1, fpmax * (0.5 + x2)))


def active_fl(ln: np.ndarray, lmin: np.ndarray, lmax: np.ndarray) -> np.ndarray:
    """MuJoCo's active force-length factor (``mju_muscleGain`` at zero velocity)."""
    a, b = 0.5 * (lmin + 1.0), 0.5 * (1.0 + lmax)
    return np.select(
        [ln <= lmin, ln <= a, ln <= 1.0, ln <= b, ln < lmax],
        [
            0.0 * ln,
            0.5 * ((ln - lmin) / (a - lmin)) ** 2,
            1.0 - 0.5 * ((1.0 - ln) / (1.0 - a)) ** 2,
            1.0 - 0.5 * ((ln - 1.0) / (b - 1.0)) ** 2,
            0.5 * ((lmax - ln) / (lmax - b)) ** 2,
        ],
        0.0,
    )


def silder_postures() -> np.ndarray:
    """(hip, knee, ankle) in rad covering the trials of Silder et al. (2007, Table 2)."""
    g, s = SILDER_POSTURES_DEG["hip_knee_grid"], SILDER_POSTURES_DEG["ankle_sweep"]
    out = [(h, k, g["ankle"]) for h in g["hip"] for k in g["knee"]]
    out += [(s["hip"], k, an) for k in s["knee"] for an in s["ankle"]]
    return np.radians(np.array(out, float))


def geometry(model: mujoco.MjModel, angles: np.ndarray, acts: list[int]) -> tuple[np.ndarray, np.ndarray]:
    """Actuator lengths (n, m) and effective moment arms dL/dtheta (n, m, 3) about the hip, knee and ankle coordinates."""
    lengths, arms = np.zeros((len(angles), len(acts))), np.zeros((len(angles), len(acts), 3))
    for i, q in enumerate(angles):
        lengths[i], arms[i] = effective_moment_arms(model, dict(zip(JOINTS, q)), list(JOINTS), acts)
    return lengths, arms


def model_passive(model: mujoco.MjModel, angles: np.ndarray, exclude: list[int] = ()) -> np.ndarray:
    """Passive (zero-activation) moments about hip, knee, ankle from all muscles except ``exclude``.

    Moments are actuator forces times effective moment arms (joint-equality followers included)."""
    muscles = [a for a in range(model.nu) if int(model.actuator_gaintype[a]) == int(mujoco.mjtGain.mjGAIN_MUSCLE)]
    acts = [a for a in muscles if a not in set(exclude)]
    data = mujoco.MjData(model)
    out = np.zeros((len(angles), 3))
    for i, q in enumerate(angles):
        _, arm = effective_moment_arms(model, dict(zip(JOINTS, q)), list(JOINTS), acts)
        data.qpos[:] = sample_postures(model, {j: (v, v) for j, v in zip(JOINTS, q)}, 1, 0)[0]
        data.act[:] = 0
        mujoco.mj_forward(model, data)
        out[i] = data.actuator_force[acts] @ arm  # actuator_force is minus the tension
    return out


class LegMuscles:
    """Fixed per-muscle data of the fitted leg muscles (right side) and the passive model on top of it."""

    def __init__(self, model: mujoco.MjModel, names: list[str]):
        self.names = names
        self.acts = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in names]
        g = model.actuator_gainprm[self.acts]
        self.r0, self.r1, self.f0, self.lmin, self.lmax, self.fpmax0 = g[:, 0], g[:, 1], g[:, 2], g[:, 4], g[:, 5], g[:, 7]
        self.lr = model.actuator_lengthrange[self.acts].copy()
        self.l0 = (self.lr[:, 1] - self.lr[:, 0]) / (self.r1 - self.r0)

    def ln(self, lengths: np.ndarray, shift: np.ndarray) -> np.ndarray:
        """Normalised fibre length with the slack length shifted by ``shift`` L0 (a longer tendon shortens the fibre)."""
        return self.r0 + (lengths - self.lr[:, 0]) / self.l0 - shift

    def passive_moments(self, lengths: np.ndarray, arms: np.ndarray, shift: np.ndarray, fpmax: np.ndarray) -> np.ndarray:
        force = self.f0 * passive_fl(self.ln(lengths, shift), self.lmax, fpmax)  # tension, N
        return -np.einsum("nm,nmj->nj", force, arms)  # qfrc = dL/dq * (-tension)


def fit_leg_passive(model: mujoco.MjModel) -> dict:
    """Fit slack shifts (L0) and fpmax per right-leg muscle; returns the result with errors before and after."""
    from scipy.optimize import least_squares

    ref = load_passive_data()
    names = [n + "_r" for group in GROUPS.values() for n in group]
    mus = LegMuscles(model, names)
    ang = silder_postures()
    target = silder_moments(ref["params"], ang[:, 0], ang[:, 1], ang[:, 2])
    ok = np.abs(target) < T_MAX
    lengths, arms = geometry(model, ang, mus.acts)
    # passive moments of the muscles that are not fitted, which the fitted ones add to
    fixed = model_passive(model, ang, exclude=mus.acts)
    # active constraint: each muscle at the angle of its group's maximal voluntary strength, in Anderson et al.'s posture
    s_post, s_owner = [], []
    for t in ANDERSON_STRENGTH.values():
        q = np.radians([t["posture_deg"].get(j, 0.0) for j in JOINTS])
        q[JOINTS.index(t["joint"])] = np.radians(t["peak_deg"])
        for n in t["muscles"]:
            if n + "_r" in names:
                s_post.append(q)
                s_owner.append(names.index(n + "_r"))
    s_owner = np.array(s_owner)
    s_len, _ = geometry(model, np.array(s_post), mus.acts)
    m = len(names)
    # fascicle-length targets (ultrasound): fibre length ln * L0 at the given postures
    ft = NARICI_FASCICLE
    f_idx = np.array([names.index(r["muscle"] + "_r") for r in ft["rows"]], int)
    f_mm = np.array([r["mm"] for r in ft["rows"]], float)
    f_len = (
        geometry(model, np.radians([[r["posture_deg"].get(j, 0.0) for j in JOINTS] for r in ft["rows"]]), mus.acts)[0]
        if ft["rows"]
        else np.zeros((0, m))
    )

    def fascicle_mm(shift: np.ndarray) -> np.ndarray:
        return 1e3 * (mus.ln(f_len, shift)[np.arange(len(f_idx)), f_idx] * mus.l0[f_idx])

    def unpack(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return x[:m], np.exp(x[m:])

    def strength_fl(shift: np.ndarray) -> np.ndarray:
        ln = mus.ln(s_len, shift)[np.arange(len(s_owner)), s_owner]
        return active_fl(ln, mus.lmin[s_owner], mus.lmax[s_owner])

    def residuals(x: np.ndarray) -> np.ndarray:
        shift, fp = unpack(x)
        err = (fixed + mus.passive_moments(lengths, arms, shift, fp) - target)[ok]
        w = np.where(np.sign(target[ok]) * err > 0, EXCESS_WEIGHT, 1.0)
        constraint = 1e3 * np.maximum(MIN_FL - strength_fl(shift), 0.0)
        fasc = ft["weight_Nm_per_mm"] * (fascicle_mm(shift) - f_mm)
        return np.concatenate(
            [err * np.sqrt(w), SHIFT_PENALTY * shift, FPMAX_PENALTY * np.log(fp / mus.fpmax0), constraint, fasc]
        )

    def errors(shift: np.ndarray, fp: np.ndarray) -> dict:
        e = np.where(ok, fixed + mus.passive_moments(lengths, arms, shift, fp) - target, np.nan)
        return {j: round(float(np.sqrt(np.nanmean(e[:, i] ** 2))), 2) for i, j in enumerate(("hip", "knee", "ankle"))}

    x0 = np.concatenate([np.zeros(m), np.log(mus.fpmax0)])
    # the shifted operating range must still contain the optimal length: range0 - shift < 1 < range1 - shift
    lo_s = np.maximum(-MAX_SHIFT, mus.r0 - 1.0 + RANGE_MARGIN)
    hi_s = np.minimum(MAX_SHIFT, mus.r1 - 1.0 - RANGE_MARGIN)
    none = lo_s >= hi_s  # the range does not contain the optimal length even unshifted: leave its slack length
    lo_s[none], hi_s[none] = -1e-9, 1e-9
    lo = np.concatenate([lo_s, np.full(m, np.log(FPMAX_BOUNDS[0]))])
    hi = np.concatenate([hi_s, np.full(m, np.log(FPMAX_BOUNDS[1]))])
    shift, fp = unpack(least_squares(residuals, np.clip(x0, lo + 1e-12, hi - 1e-12), bounds=(lo, hi)).x)
    return {
        "muscles": {
            n: {
                "slack_shift_L0": round(float(shift[i]), 4),
                "slack_shift_m": round(float(shift[i] * mus.l0[i]), 5),
                "fpmax": round(float(fp[i]), 4),
                "fpmax_before": round(float(mus.fpmax0[i]), 4),
            }
            for i, n in enumerate(names)
        },
        "fascicle_mm": {
            "target": f_mm.tolist(),
            "before": np.round(fascicle_mm(np.zeros(m)), 1).tolist(),
            "after": np.round(fascicle_mm(shift), 1).tolist(),
        },  # fmt: skip
        "rms_Nm_before": errors(np.zeros(m), mus.fpmax0),
        "rms_Nm_after": errors(shift, fp),
        "min_active_fl_at_strength_postures": {
            "before": round(float(strength_fl(np.zeros(m)).min()), 3),
            "after": round(float(strength_fl(shift).min()), 3),
        },  # fmt: skip
    }


def write_leg_passive(model: mujoco.MjModel, result: dict, xml_path: Path = LEG_MUSCLE_XML, margin: float = 0.02) -> int:
    """Apply the fitted slack shift and fpmax to each fitted muscle and its ``_l`` mirror.

    The shift keeps ``lengthrange`` and lowers ``range`` by the shift (the same linear map as a longer tendon); an anchor moves
    inward only where the shifted range would leave ``(lmin, lmax)``.
    """
    text = xml_path.read_text()
    n = 0
    for right, r in result["muscles"].items():
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, right)
        r0, r1 = model.actuator_gainprm[a, :2] - r["slack_shift_L0"]
        l0, l1 = model.actuator_lengthrange[a]
        lmin, lmax = model.actuator_gainprm[a, 4:6]
        slope = (r1 - r0) / (l1 - l0)
        if r0 < lmin + margin:
            l0, r0 = l0 + (lmin + margin - r0) / slope, lmin + margin
        if r1 > lmax - margin:
            l1, r1 = l1 - (r1 - lmax + margin) / slope, lmax - margin
        for name in (right, right[:-2] + "_l"):
            pat = re.compile(r'(<general[^>]*name="' + re.escape(name) + r'"[^>]*/>)')
            line = pat.search(text).group(1)
            for attr in ("gainprm", "biasprm"):
                vals = re.search(attr + r'="([^"]*)"', line).group(1).split()
                vals[0], vals[1], vals[7] = f"{r0:.6f}", f"{r1:.6f}", f"{r['fpmax']:.6g}"
                line = re.sub(attr + r'="[^"]*"', f'{attr}="{" ".join(vals)}"', line, count=1)
            line = re.sub(r'lengthrange="[^"]*"', f'lengthrange="{l0:.6f} {l1:.6f}"', line, count=1)
            text = pat.sub(lambda _m, line=line: line, text, count=1)
            n += 1
    xml_path.write_text(text)
    return n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="rewrite lengthrange and fpmax in myolegs_muscle.xml")
    args = ap.parse_args()
    import myo_sim

    model, _ = myo_sim.load("myolegs")
    result = fit_leg_passive(model)
    print(json.dumps({k: v for k, v in result.items() if k != "muscles"}))
    for n, r in result["muscles"].items():
        print(f"{n:14s} shift {r['slack_shift_L0']:+.3f} L0  fpmax {r['fpmax_before']:.3g} -> {r['fpmax']:.3g}")
    if args.write:
        print("rewrote", write_leg_passive(model, result), "actuators in", LEG_MUSCLE_XML)


if __name__ == "__main__":
    main()
