"""Recalculate lengthrange + operating range from FL-curve bounds (lmin/lmax).

MuJoCo muscle ``gainprm`` / ``biasprm`` layout (relevant slots)::

    [0] range[0]  operating-range lower  (r0)  — maps to lengthrange[0]
    [1] range[1]  operating-range upper  (r1)  — maps to lengthrange[1]
    [2] force     peak active force
    [4] lmin      force-length curve lower
    [5] lmax      force-length curve upper

Normalized length at musculotendon length ``L``::

    L_norm = r0 + (L - LR0) / L0
    L0     = (LR1 - LR0) / (r1 - r0)

Given new FL bounds ``(lmin, lmax)`` and a measured path ``[L_path_lo, L_path_hi]``,
this utility places the operating band inside the FL curve and re-anchors
``lengthrange`` so the path spans that band (mid-ROM at L_norm=1 by default).

CLI::

    uv run python tests/recalc_from_fl_bounds.py --model myoarm_r
    uv run python tests/recalc_from_fl_bounds.py --model myoarm_r --muscle LAT1 --lmin 0.5 --lmax 1.6
    uv run python tests/recalc_from_fl_bounds.py --model myotorso --verify-only
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np

# tests/ is on pytest pythonpath; keep sibling imports working for CLI use.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from muscle_analysis_utils import (  # noqa: E402
    apply_eq_constraints,
    compute_moment_arm_curve,
    parse_model_joint_equalities,
)

import myo_sim  # noqa: E402

# Defaults: keep operating band inside FL with margin; mid-ROM at L_norm=1
DEFAULT_BAND_FRAC = 0.15  # fraction of (lmax-lmin) inset from each end
LR_MARGIN_M = 0.005
PATH_SAMPLES = 80


@dataclass
class MuscleParams:
    name: str
    lengthrange: tuple[float, float]
    r0: float
    r1: float
    lmin: float
    lmax: float
    fmax: float
    l0_implied: float

    @classmethod
    def from_model(cls, model: mujoco.MjModel, name: str) -> MuscleParams:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise ValueError(f"actuator {name!r} not found")
        prm = model.actuator_gainprm[aid]
        lr = model.actuator_lengthrange[aid]
        r0, r1 = float(prm[0]), float(prm[1])
        span = float(lr[1] - lr[0])
        l0 = span / (r1 - r0) if abs(r1 - r0) > 1e-12 else float("nan")
        return cls(
            name=name,
            lengthrange=(float(lr[0]), float(lr[1])),
            r0=r0,
            r1=r1,
            lmin=float(prm[4]),
            lmax=float(prm[5]),
            fmax=float(prm[2]),
            l0_implied=l0,
        )


@dataclass
class PathSample:
    joint: str
    mtu_min: float
    mtu_max: float


@dataclass
class RecalcProposal:
    name: str
    lmin: float
    lmax: float
    r0: float
    r1: float
    lengthrange_lo: float
    lengthrange_hi: float
    l0_implied: float
    path: PathSample
    current: MuscleParams


@dataclass
class Violation:
    name: str
    kind: str
    detail: str


def primary_joint(model, data, aid: int, eq_map) -> tuple[int, str]:
    tid = model.actuator_trnid[aid, 0]
    best = (-1, "", 0.0)
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        qs, ma = compute_moment_arm_curve(model, data, tid, j, n=40, eq_map=eq_map)
        if qs is None:
            continue
        score = float(np.mean(np.abs(ma)))
        if score > best[2]:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or f"j{j}"
            best = (j, name, score)
    return best[0], best[1]


def sample_path(model, data, eq_map, aid: int, jnt_id: int, n: int = PATH_SAMPLES) -> PathSample:
    qadr = model.jnt_qposadr[jnt_id]
    lo, hi = model.jnt_range[jnt_id]
    jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id) or f"j{jnt_id}"
    lengths = []
    for q in np.linspace(lo, hi, n):
        data.qpos[:] = 0.0
        data.qpos[qadr] = q
        apply_eq_constraints(data, model, eq_map)
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        lengths.append(float(data.actuator_length[aid]))
    lengths = np.asarray(lengths)
    return PathSample(joint=jname, mtu_min=float(lengths.min()), mtu_max=float(lengths.max()))


def operating_band_from_fl(lmin: float, lmax: float, band_frac: float = DEFAULT_BAND_FRAC) -> tuple[float, float]:
    """Place (r0, r1) inside [lmin, lmax], preferring to include 1.0 when possible."""
    if lmax <= lmin:
        raise ValueError(f"lmax ({lmax}) must be > lmin ({lmin})")
    span = lmax - lmin
    r0 = lmin + band_frac * span
    r1 = lmax - band_frac * span
    if r1 <= r0:
        r0, r1 = lmin + 0.05 * span, lmax - 0.05 * span
    # Prefer mid-ROM at L_norm=1: if 1 is inside (r0,r1) keep; else shift band to contain 1 if possible
    if r0 < 1.0 < r1:
        return r0, r1
    if lmin < 1.0 < lmax:
        half = 0.5 * (r1 - r0)
        r0 = max(lmin, 1.0 - half)
        r1 = min(lmax, 1.0 + half)
        if r1 <= r0:
            r0, r1 = lmin + band_frac * span, lmax - band_frac * span
    return float(r0), float(r1)


def lengthrange_from_path(
    path: PathSample,
    r0: float,
    r1: float,
    *,
    margin_m: float = LR_MARGIN_M,
) -> tuple[float, float, float, float, float]:
    """Anchor mid-path at L_norm=1; span path (+margin) over (r0,r1).

    Returns ``(LR0, LR1, L0, r0_out, r1_out)``.
    """
    mtu_lo = path.mtu_min - margin_m
    mtu_hi = path.mtu_max + margin_m
    mid = 0.5 * (path.mtu_min + path.mtu_max)
    path_span = mtu_hi - mtu_lo
    op_span = r1 - r0
    if op_span <= 1e-12:
        raise ValueError("operating range (r1-r0) too small")
    l0 = path_span / op_span
    lr0 = mid - (1.0 - r0) * l0
    lr1 = lr0 + path_span
    # Recompute r so mid stays at 1 after margin padding
    r0_out = 1.0 - (mid - lr0) / l0
    r1_out = r0_out + path_span / l0
    return float(lr0), float(lr1), float(l0), float(r0_out), float(r1_out)


def recalc_from_fl_bounds(
    path: PathSample,
    current: MuscleParams,
    lmin: float | None = None,
    lmax: float | None = None,
    band_frac: float = DEFAULT_BAND_FRAC,
) -> RecalcProposal:
    """Propose lengthrange + r0/r1 for target FL bounds (default: keep current lmin/lmax)."""
    lmin_u = current.lmin if lmin is None else float(lmin)
    lmax_u = current.lmax if lmax is None else float(lmax)
    r0_t, r1_t = operating_band_from_fl(lmin_u, lmax_u, band_frac)
    lr0, lr1, l0, r0, r1 = lengthrange_from_path(path, r0_t, r1_t)
    return RecalcProposal(
        name=current.name,
        lmin=lmin_u,
        lmax=lmax_u,
        r0=r0,
        r1=r1,
        lengthrange_lo=lr0,
        lengthrange_hi=lr1,
        l0_implied=l0,
        path=path,
        current=current,
    )


def apply_proposal_in_memory(model: mujoco.MjModel, proposal: RecalcProposal) -> None:
    """Patch compiled model actuator params (gainprm + biasprm + lengthrange)."""
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, proposal.name)
    model.actuator_lengthrange[aid, 0] = proposal.lengthrange_lo
    model.actuator_lengthrange[aid, 1] = proposal.lengthrange_hi
    for arr in (model.actuator_gainprm[aid], model.actuator_biasprm[aid]):
        arr[0] = proposal.r0
        arr[1] = proposal.r1
        arr[4] = proposal.lmin
        arr[5] = proposal.lmax


def verify_muscle(model, data, eq_map, name: str, *, path_n: int = PATH_SAMPLES) -> list[Violation]:
    """Return parameter / path consistency violations for one muscle."""
    cur = MuscleParams.from_model(model, name)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    violations: list[Violation] = []

    if cur.lmax <= cur.lmin:
        violations.append(Violation(name, "fl_bounds", f"lmax={cur.lmax} <= lmin={cur.lmin}"))
    if cur.r1 <= cur.r0:
        violations.append(Violation(name, "operating_range", f"r1={cur.r1} <= r0={cur.r0}"))

    # Operating band should lie within FL curve domain
    eps = 1e-6
    if cur.r0 < cur.lmin - eps or cur.r1 > cur.lmax + eps:
        violations.append(
            Violation(
                name,
                "operating_outside_fl",
                f"r=[{cur.r0:.4f},{cur.r1:.4f}] outside lmin/lmax=[{cur.lmin:.4f},{cur.lmax:.4f}]",
            )
        )

    jnt_id, jname = primary_joint(model, data, aid, eq_map)
    if jnt_id < 0:
        violations.append(Violation(name, "no_joint", "no hinge with moment arm"))
        return violations

    path = sample_path(model, data, eq_map, aid, jnt_id, n=path_n)
    lr0, lr1 = cur.lengthrange
    if path.mtu_min < lr0 - 1e-6 or path.mtu_max > lr1 + 1e-6:
        violations.append(
            Violation(
                name,
                "path_outside_lengthrange",
                f"path=[{path.mtu_min:.4f},{path.mtu_max:.4f}] m on {jname} outside lengthrange=[{lr0:.4f},{lr1:.4f}]",
            )
        )

    # Path-normalized lengths should fall in operating band (approx)
    if abs(cur.l0_implied) > 1e-9 and np.isfinite(cur.l0_implied):
        n_lo = cur.r0 + (path.mtu_min - lr0) / cur.l0_implied
        n_hi = cur.r0 + (path.mtu_max - lr0) / cur.l0_implied
        if n_hi < cur.lmin - 0.05 or n_lo > cur.lmax + 0.05:
            violations.append(
                Violation(
                    name,
                    "path_norm_outside_fl",
                    f"path L_norm≈[{n_lo:.3f},{n_hi:.3f}] vs FL [{cur.lmin:.3f},{cur.lmax:.3f}]",
                )
            )

    return violations


def verify_model(model_name: str) -> list[Violation]:
    model, data = myo_sim.load(model_name)
    eq_map = parse_model_joint_equalities(model)
    out: list[Violation] = []
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if not name:
            continue
        out.extend(verify_muscle(model, data, eq_map, name))
    return out


def fl_bound_diversity(model_name: str) -> dict:
    """Summarize uniqueness of (lmin, lmax) — highlights shared defaults."""
    model, _ = myo_sim.load(model_name)
    counts: dict[tuple[float, float], int] = {}
    for i in range(model.nu):
        prm = model.actuator_gainprm[i]
        key = (round(float(prm[4]), 6), round(float(prm[5]), 6))
        counts[key] = counts.get(key, 0) + 1
    return {
        "model": model_name,
        "n_actuators": model.nu,
        "n_unique_fl_bounds": len(counts),
        "counts": {f"{a},{b}": n for (a, b), n in sorted(counts.items(), key=lambda x: -x[1])},
    }


def propose_for_model(
    model_name: str,
    muscle: str | None = None,
    lmin: float | None = None,
    lmax: float | None = None,
    band_frac: float = DEFAULT_BAND_FRAC,
) -> list[RecalcProposal]:
    model, data = myo_sim.load(model_name)
    eq_map = parse_model_joint_equalities(model)
    names = [muscle] if muscle else [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    proposals = []
    for name in names:
        if not name:
            continue
        cur = MuscleParams.from_model(model, name)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        jnt_id, _ = primary_joint(model, data, aid, eq_map)
        if jnt_id < 0:
            continue
        path = sample_path(model, data, eq_map, aid, jnt_id)
        proposals.append(recalc_from_fl_bounds(path, cur, lmin=lmin, lmax=lmax, band_frac=band_frac))
    return proposals


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="myoarm_r", help="Registered myo_sim model name")
    ap.add_argument("--muscle", default=None, help="Single actuator; default=all")
    ap.add_argument("--lmin", type=float, default=None, help="Target FL lmin (default: keep current)")
    ap.add_argument("--lmax", type=float, default=None, help="Target FL lmax (default: keep current)")
    ap.add_argument("--band-frac", type=float, default=DEFAULT_BAND_FRAC)
    ap.add_argument("--verify-only", action="store_true", help="Only report violations")
    ap.add_argument("--json-out", type=Path, default=None)
    args = ap.parse_args()

    diversity = fl_bound_diversity(args.model)
    print(f"=== FL bound diversity ({args.model}) ===")
    print(f"  unique (lmin,lmax) pairs: {diversity['n_unique_fl_bounds']} / {diversity['n_actuators']}")
    for k, n in list(diversity["counts"].items())[:5]:
        print(f"    ({k}): {n}")

    violations = verify_model(args.model)
    by_kind: dict[str, int] = {}
    for v in violations:
        by_kind[v.kind] = by_kind.get(v.kind, 0) + 1
    print(f"\n=== Verify ({len(violations)} violations) ===")
    for kind, n in sorted(by_kind.items(), key=lambda x: -x[1]):
        print(f"  {kind}: {n}")
    for v in violations[:20]:
        print(f"  [{v.kind}] {v.name}: {v.detail}")
    if len(violations) > 20:
        print(f"  ... +{len(violations) - 20} more")

    payload: dict = {"diversity": diversity, "violations": [asdict(v) for v in violations]}

    if not args.verify_only:
        props = propose_for_model(args.model, args.muscle, args.lmin, args.lmax, args.band_frac)
        print(f"\n=== Recalc proposals ({len(props)}) ===")
        for p in props[:10]:
            c = p.current
            print(
                f"  {p.name:16s} FL[{p.lmin:.3f},{p.lmax:.3f}]  "
                f"r {c.r0:.3f},{c.r1:.3f}->{p.r0:.3f},{p.r1:.3f}  "
                f"LR {c.lengthrange[0]:.4f},{c.lengthrange[1]:.4f}"
                f"->{p.lengthrange_lo:.4f},{p.lengthrange_hi:.4f}  "
                f"L0 {c.l0_implied:.4f}->{p.l0_implied:.4f}"
            )
        if len(props) > 10:
            print(f"  ... +{len(props) - 10} more")
        payload["proposals"] = [
            {
                "name": p.name,
                "lmin": p.lmin,
                "lmax": p.lmax,
                "r0": p.r0,
                "r1": p.r1,
                "lengthrange": [p.lengthrange_lo, p.lengthrange_hi],
                "l0_implied": p.l0_implied,
                "current": asdict(p.current),
                "path": asdict(p.path),
            }
            for p in props
        ]

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2))
        print(f"\nwrote {args.json_out}")

    # Non-zero exit when verify finds issues (useful for CI)
    if args.verify_only and violations:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
