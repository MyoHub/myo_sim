"""Rigid-tendon (MuJoCo native) muscle maps derived directly from an OpenSim reference muscle.

MuJoCo's muscle maps the actuator length ``L`` linearly to a normalised fibre length,
``ln = range0 + (L - lengthrange0) * (range1 - range0) / (lengthrange1 - lengthrange0)``, with a rigid tendon and no pennation.
For an OpenSim muscle with optimal fibre length ``l0``, tendon slack length ``lts`` and pennation ``alpha0`` (constant width), the
rigid-tendon fibre length is ``sqrt(max(L - lts, 0)^2 + (l0 sin alpha0)^2) / l0``. ``fit_rigid_muscle_map`` fits MuJoCo's linear map to
it over sampled path lengths, so the converted muscle operates at the reference's fibre lengths without refitting ``l0`` to a
measured length range.

Cited table values live in ``myo_sim.build.references``.

Usage (development / release only; rewrites adopted ``range`` and ``lengthrange``)::

    uv run python scripts/fit_leg_rigid_maps.py --write
    uv run python scripts/fit_arm_rigid_maps.py --write
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import mujoco
import numpy as np

from myo_sim.build.references import MOBL_ARM, RAJAGOPAL2016_LEG

LEG_MUSCLE_XML = Path(__file__).parents[1] / "models" / "leg" / "assets" / "myolegs_muscle.xml"
MODELS = Path(__file__).parents[1] / "models"
REGIONS = {  # cited table, model, muscle XML, joints to sweep, and how the other side is derived
    "leg": {
        "reference": RAJAGOPAL2016_LEG,
        "model": "myolegs",
        "xml": LEG_MUSCLE_XML,
        "mirror": ("_r", "_l"),
        "joints": (
            "hip_flexion_r",
            "hip_adduction_r",
            "hip_rotation_r",
            "knee_angle_r",
            "ankle_angle_r",
            "subtalar_angle_r",
            "mtp_angle_r",
        ),
    },
    "arm": {
        "reference": MOBL_ARM,
        "model": "myoarm",
        "xml": MODELS / "arm" / "assets" / "myoarm_r_muscle.xml",
        "mirror": None,
        "joints": (
            "elv_angle_r",
            "shoulder_elv_r",
            "shoulder_rot_r",
            "elbow_flexion_r",
            "pro_sup_r",
            "deviation_r",
            "flexion_r",
        ),
    },  # left arm mirrored at compose time
}


def load_leg_reference() -> dict:
    return RAJAGOPAL2016_LEG


def adopted_names(ref: dict) -> list[str]:
    """Muscles whose published table values were written into the XML."""
    return [name for name, row in ref["muscles"].items() if row.get("status") == "adopted"]


def joint_ranges(model: mujoco.MjModel, names: tuple[str, ...]) -> dict[str, tuple[float, float]]:
    """Named joints' model ranges. Fit samples the model, not a stored grid."""
    out: dict[str, tuple[float, float]] = {}
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise KeyError(name)
        out[name] = (float(model.jnt_range[jid, 0]), float(model.jnt_range[jid, 1]))
    return out


def rigid_equivalent_fiber_length(path_length: np.ndarray, l0: float, lts: float, alpha0: float) -> np.ndarray:
    """Normalised fibre length of a rigid-tendon muscle with constant-width pennation."""
    along = np.maximum(np.asarray(path_length) - lts, 0.0)
    return np.sqrt(along**2 + (l0 * np.sin(alpha0)) ** 2) / l0


def fit_rigid_muscle_map(
    path_length: np.ndarray,
    l0: float,
    lts: float,
    alpha0: float,
    lmin: float,
    lmax: float,
    span_length: np.ndarray | None = None,
    margin: float = 0.02,
) -> tuple[float, float, float, float]:
    """Least-squares MuJoCo map ``(range0, range1, lengthrange0, lengthrange1)`` for the sampled path lengths.

    The map is fitted on ``path_length``; ``lengthrange`` spans ``span_length`` (default: the same samples), e.g. the fit
    sample plus single-joint sweeps over the full joint ranges. An anchor moves inward only where the fitted map would leave
    ``(lmin, lmax)`` there (a slack rigid tendon); MuJoCo extrapolates the same linear map beyond ``lengthrange``.
    """
    path_length = np.asarray(path_length, float)
    span = path_length if span_length is None else np.concatenate([path_length, np.asarray(span_length, float)])
    slope, icpt = np.polyfit(path_length, rigid_equivalent_fiber_length(path_length, l0, lts, alpha0), 1)
    lr0, lr1 = float(span.min()), float(span.max())
    lr0 = max(lr0, (lmin + margin - icpt) / slope)
    lr1 = min(lr1, (lmax - margin - icpt) / slope)
    return float(icpt + slope * lr0), float(icpt + slope * lr1), lr0, lr1


def _joint_equalities(model: mujoco.MjModel) -> list[tuple[int, int, np.ndarray]]:
    return [
        (int(model.eq_obj1id[e]), int(model.eq_obj2id[e]), model.eq_data[e, :5].copy())
        for e in range(model.neq)
        if int(model.eq_type[e]) == int(mujoco.mjtEq.mjEQ_JOINT) and model.eq_active0[e] and model.eq_obj2id[e] >= 0
    ]


def sample_postures(model: mujoco.MjModel, ranges: dict[str, tuple[float, float]], n: int, seed: int) -> np.ndarray:
    """``n`` qpos vectors with the named joints uniform in ``ranges`` and joint-equality followers resolved."""
    rng = np.random.default_rng(seed)
    eqs = _joint_equalities(model)
    adr = {name: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in ranges}
    out = np.tile(model.qpos0, (n, 1))
    for name, (lo, hi) in ranges.items():
        out[:, adr[name]] = rng.uniform(lo, hi, n)
    for _ in range(3):  # followers of followers
        for j1, j2, c in eqs:
            a1, a2 = model.jnt_qposadr[j1], model.jnt_qposadr[j2]
            x = out[:, a2] - model.qpos0[a2]
            out[:, a1] = model.qpos0[a1] + sum(c[k] * x**k for k in range(5))
    return out


def joint_sweep_postures(model: mujoco.MjModel, joints: list[str], n: int = 41) -> np.ndarray:
    """Each joint swept alone over its full model range (others at qpos0), joint-equality followers resolved."""
    out = []
    for name in joints:
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        lo, hi = model.jnt_range[j]
        for v in np.linspace(lo, hi, n):
            out.append(sample_postures(model, {name: (float(v), float(v))}, n=1, seed=0)[0])
    return np.array(out)


def actuator_lengths(model: mujoco.MjModel, postures: np.ndarray, actuators: list[int]) -> np.ndarray:
    data = mujoco.MjData(model)
    out = np.zeros((len(postures), len(actuators)))
    for k, q in enumerate(postures):
        data.qpos[:] = q
        mujoco.mj_fwdPosition(model, data)
        out[k] = data.actuator_length[actuators]
    return out


def effective_moment_arms(
    model: mujoco.MjModel, joints: dict[str, float], coords: list[str], acts: list[int], h: float = 1e-4
) -> tuple[np.ndarray, np.ndarray]:
    """Actuator lengths and dL/dtheta for the independent coordinates ``coords`` at the posture ``joints`` (rad).

    The derivative is taken with the joint-equality followers resolved (e.g. knee translations and patella), so it is the
    moment arm about the anatomical coordinate; ``data.actuator_moment`` at one DOF alone misses the followers' part.
    Returns lengths (len(acts),) and arms (len(acts), len(coords)); the moment of a tension T is ``-T * arm``.
    """

    def lengths(j: dict[str, float]) -> np.ndarray:
        return actuator_lengths(model, sample_postures(model, {k: (v, v) for k, v in j.items()}, 1, 0), acts)[0]

    arms = np.zeros((len(acts), len(coords)))
    for i, c in enumerate(coords):
        up, dn = dict(joints), dict(joints)
        up[c], dn[c] = joints.get(c, 0.0) + h, joints.get(c, 0.0) - h
        arms[:, i] = (lengths(up) - lengths(dn)) / (2 * h)
    return lengths(joints), arms


def mujoco_fiber_length(model: mujoco.MjModel, actuator: int, length: np.ndarray) -> np.ndarray:
    """MuJoCo's normalised fibre length for actuator lengths ``length``."""
    r0, r1 = model.actuator_gainprm[actuator, :2]
    l0, l1 = model.actuator_lengthrange[actuator]
    return r0 + (np.asarray(length) - l0) * (r1 - r0) / (l1 - l0)


def leg_rigid_maps(model: mujoco.MjModel, n: int = 400, seed: int = 123) -> dict[str, tuple[float, float, float, float]]:
    """Fitted maps for the adopted right-leg muscles (apply to both sides; the leg is left/right symmetric)."""
    return region_rigid_maps(model, "leg", n=n, seed=seed)


def region_rigid_maps(
    model: mujoco.MjModel, region: str, n: int = 400, seed: int = 123
) -> dict[str, tuple[float, float, float, float]]:
    """Fitted maps for a region's adopted muscles (``REGIONS``): random full-range postures plus single-joint sweeps."""
    ref = REGIONS[region]["reference"]
    ranges = joint_ranges(model, REGIONS[region]["joints"])
    postures = sample_postures(model, ranges, n, seed)
    names = adopted_names(ref)
    acts = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names]
    lengths = actuator_lengths(model, postures, acts)
    sweeps = actuator_lengths(model, joint_sweep_postures(model, list(ranges)), acts)
    maps = {}
    for j, (name, a) in enumerate(zip(names, acts)):
        p = ref["muscles"][name]
        lmin, lmax = (float(v) for v in model.actuator_gainprm[a, 4:6])
        maps[name] = fit_rigid_muscle_map(
            lengths[:, j],
            p["l0"],
            p["lts"],
            p["alpha"],
            lmin,
            lmax,
            span_length=sweeps[:, j],
        )
    return maps


def write_region_maps(maps: dict[str, tuple[float, float, float, float]], region: str) -> int:
    """Rewrite range and lengthrange of the region's muscles (and their mirrored names, if the XML holds both sides)."""
    cfg = REGIONS[region]
    names = {}
    for name, v in maps.items():
        names[name] = v
        if cfg["mirror"]:
            names[name[: -len(cfg["mirror"][0])] + cfg["mirror"][1]] = v
    return _write_maps(names, cfg["xml"])


def _write_maps(maps: dict[str, tuple[float, float, float, float]], xml_path: Path) -> int:
    text = xml_path.read_text()
    for name, (r0, r1, l0, l1) in maps.items():
        pat = re.compile(r'(<general[^>]*name="' + re.escape(name) + r'"[^>]*/>)')
        match = pat.search(text)
        if match is None:
            raise KeyError(name)
        line = match.group(1)
        for attr in ("gainprm", "biasprm"):
            line = re.sub(attr + r'="([^ "]+) ([^ "]+) ', lambda _m, a=attr: f'{a}="{r0:.6f} {r1:.6f} ', line, count=1)
        line = re.sub(r'lengthrange="[^"]*"', f'lengthrange="{l0:.6f} {l1:.6f}"', line, count=1)
        text = text[: match.start(1)] + line + text[match.end(1) :]
    xml_path.write_text(text)
    return len(maps)


def write_leg_maps(maps: dict[str, tuple[float, float, float, float]], xml_path: Path = LEG_MUSCLE_XML) -> int:
    """Rewrite range (gainprm/biasprm[0:2]) and lengthrange of the right muscles and their ``_l`` mirrors in place."""
    text = xml_path.read_text()
    n = 0
    for right, (r0, r1, l0, l1) in maps.items():
        for name in (right, right[:-2] + "_l"):
            pat = re.compile(r'(<general[^>]*name="' + re.escape(name) + r'"[^>]*/>)')
            match = pat.search(text)
            if match is None:
                raise KeyError(name)
            line = match.group(1)
            for attr in ("gainprm", "biasprm"):
                line = re.sub(attr + r'="([^ "]+) ([^ "]+) ', lambda _m: f'{attr}="{r0:.6f} {r1:.6f} ', line, count=1)
            line = re.sub(r'lengthrange="[^"]*"', f'lengthrange="{l0:.6f} {l1:.6f}"', line, count=1)
            text = text[: match.start(1)] + line + text[match.end(1) :]
            n += 1
    xml_path.write_text(text)
    return n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="rewrite the region's muscle XML (default: print the maps)")
    ap.add_argument("--region", choices=sorted(REGIONS), default="leg")
    args = ap.parse_args()
    import myo_sim

    model, _ = myo_sim.load(REGIONS[args.region]["model"])
    maps = region_rigid_maps(model, args.region)
    for name, (r0, r1, l0, l1) in maps.items():
        print(f"{name:14s} range {r0:.6f} {r1:.6f}  lengthrange {l0:.6f} {l1:.6f}")
    if args.write:
        print("rewrote", write_region_maps(maps, args.region), "actuators in", REGIONS[args.region]["xml"])


if __name__ == "__main__":
    main()
