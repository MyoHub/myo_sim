"""Tendon paths on MuJoCo 4: side sites, via sites and how each path compares under the MuJoCo 3 rule.

MuJoCo 4 picks the side of a wrap by the winding of the polyline first endpoint -> side site -> second endpoint;
MuJoCo 3 used a half-plane test and the arc nearest the side site. This script holds a NumPy copy of wrap_circle
for both rules, so one process can score both; tests/test_side_sites.py ties it to the running engine.

    python scripts/migrate_side_sites.py select-sides FDS2_r_tendon/FDS_ellipsoid_wrap_r --also-mujoco3
    python scripts/migrate_side_sites.py fit-vias
    python scripts/migrate_side_sites.py write
    python scripts/migrate_side_sites.py record-jumps
    python scripts/migrate_side_sites.py dump out label      # under each MuJoCo
    python scripts/migrate_side_sites.py check out label3 label4
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np

WRAP_SITE, WRAP_SPHERE, WRAP_CYLINDER = 3, 4, 5
MJMINVAL = 1e-15
CHECK_MODELS = (
    "myoarm_r",
    "myoarms",
    "myofullbody",
    "myolegs",
    "myolegs26",
    "myolegs_abdomen",
    "myotorso",
    "myohands",
    "myohand_r",
)
LENGTH_TOL = 1e-6  # m
LIMIT_MARGIN = 0.13  # rad past the hinge limits, as in tests/test_tendon_path_continuity.py
# Posture sets: A in range on the joint couplings, B in range with every joint independent, C as A past the limits.
TIERS = {"A": (True, 0.0), "B": (False, 0.0), "C": (True, LIMIT_MARGIN)}


@dataclass(frozen=True)
class Wrap:
    """One geom wrap of a spatial tendon, with the two sites it lies between."""

    tendon: int
    tendon_name: str
    adr: int
    site0: int
    site1: int
    geom: int
    geom_name: str
    side: int
    side_name: str
    is_sphere: bool
    radius: float
    divisor: float


def model_wraps(model) -> list[Wrap]:
    """All sphere and cylinder wraps of a compiled model."""
    wraps = []
    for t in range(model.ntendon):
        start, n = model.tendon_adr[t], model.tendon_num[t]
        divisor = 1.0
        for a in range(start, start + n):
            kind = int(model.wrap_type[a])
            if kind == 2:
                divisor = float(model.wrap_prm[a])
            if kind not in (WRAP_SPHERE, WRAP_CYLINDER):
                continue
            g, s = int(model.wrap_objid[a]), int(round(model.wrap_prm[a]))
            wraps.append(
                Wrap(
                    t,
                    model.tendon(t).name,
                    a,
                    int(model.wrap_objid[a - 1]),
                    int(model.wrap_objid[a + 1]),
                    g,
                    model.geom(g).name,
                    s,
                    model.site(s).name if s >= 0 else "",
                    kind == WRAP_SPHERE,
                    float(model.geom_size[g][0]),
                    divisor,
                )
            )
    return wraps


def _is_intersect(p1, p2, p3, p4, strict: bool):
    """Vectorised is_intersect of engine_util_misc.c; strict is the MuJoCo 4 comparison."""
    u = p2 - p1
    v = p4 - p3
    det = v[..., 1] * u[..., 0] - v[..., 0] * u[..., 1]
    s = np.where(det > 0, 1.0, -1.0)
    w = p1 - p3
    d = p2 - p4
    a0 = (v[..., 0] * w[..., 1] - v[..., 1] * w[..., 0]) * s
    a1 = (v[..., 0] * d[..., 1] - v[..., 1] * d[..., 0]) * s
    b0 = (u[..., 0] * w[..., 1] - u[..., 1] * w[..., 0]) * s
    b1 = (u[..., 0] * d[..., 1] - u[..., 1] * d[..., 0]) * s
    if strict:
        hit = (a0 >= 0) & (a1 < 0) & (b0 >= 0) & (b1 < 0)
    else:
        hit = (a0 >= 0) & (a1 <= 0) & (b0 >= 0) & (b1 <= 0)
    return hit & (np.abs(det) >= MJMINVAL)


def _winding(p, q):
    return np.arctan2(p[..., 0] * q[..., 1] - p[..., 1] * q[..., 0], (p * q).sum(-1))


def wrap_circle(end0, end1, side, radius: float, rule: int):
    """``wrap_circle`` for arrays of 2D endpoints; rule is 3 or 4. Returns arc length (or -1) and tangent points.

    ``side`` is the unit-free 2D side direction at length ``radius``, or None for a wrap without side site.
    """
    sqlen0 = (end0 * end0).sum(-1)
    sqlen1 = (end1 * end1).sum(-1)
    sqrad = radius * radius
    dif = end1 - end0
    dd = (dif * dif).sum(-1)
    valid = (sqlen0 >= sqrad) & (sqlen1 >= sqrad) & (dd >= MJMINVAL)
    with np.errstate(divide="ignore", invalid="ignore"):
        a = np.clip(-(dif * end0).sum(-1) / dd, 0.0, 1.0)
    near = a[..., None] * dif + end0
    clear = (near * near).sum(-1) > sqrad

    wside = None
    if rule == 3:
        drop = clear if side is None else clear & ((side * near).sum(-1) >= 0)
    elif side is None:
        drop = clear
    else:
        wside = _winding(end0, side) + _winding(side, end1)
        drop = clear & (np.abs(wside - _winding(end0, end1)) < np.pi)

    with np.errstate(invalid="ignore"):
        sqrt0 = np.sqrt(np.maximum(sqlen0 - sqrad, 0.0))
        sqrt1 = np.sqrt(np.maximum(sqlen1 - sqrad, 0.0))
    sols = []
    goods = []
    for sgn in (1.0, -1.0):
        s0 = np.stack(
            [
                (end0[..., 0] * sqrad + sgn * radius * end0[..., 1] * sqrt0) / sqlen0,
                (end0[..., 1] * sqrad - sgn * radius * end0[..., 0] * sqrt0) / sqlen0,
            ],
            -1,
        )
        s1 = np.stack(
            [
                (end1[..., 0] * sqrad - sgn * radius * end1[..., 1] * sqrt1) / sqlen1,
                (end1[..., 1] * sqrad + sgn * radius * end1[..., 0] * sqrt1) / sqlen1,
            ],
            -1,
        )
        if rule == 3 and side is not None:
            mid = s0 + s1
            nrm = np.linalg.norm(mid, axis=-1, keepdims=True)
            good = ((mid / np.where(nrm < MJMINVAL, 1.0, nrm)) * side).sum(-1)
        else:
            dt = s0 - s1
            good = -(dt * dt).sum(-1)
        good = np.where(_is_intersect(end0, s0, end1, s1, strict=rule == 4), -10000.0, good)
        sols.append((s0, s1))
        goods.append(good)

    if rule == 4 and side is not None:
        pick1 = wside > 0
    else:
        pick1 = ~(goods[0] > goods[1])
    pnt0 = np.where(pick1[..., None], sols[1][0], sols[0][0])
    pnt1 = np.where(pick1[..., None], sols[1][1], sols[0][1])
    drop = drop | _is_intersect(end0, pnt0, end1, pnt1, strict=rule == 4)

    if rule == 4 and side is not None:
        arc = wside - _winding(end0, pnt0) - _winding(pnt1, end1)
        wlen = radius * np.abs(arc)
    else:
        cross = pnt0[..., 1] * pnt1[..., 0] - pnt0[..., 0] * pnt1[..., 1]
        angle = np.arctan2(np.abs(cross), (pnt0 * pnt1).sum(-1))
        flip = ((cross > 0) & pick1) | ((cross < 0) & ~pick1)
        if rule == 4:
            flip = flip & (angle > 1e-6)
        angle = np.where(flip, 2 * np.pi - angle, angle)
        wlen = radius * angle
    wlen = np.where(drop | ~valid, -1.0, wlen)
    return wlen, pnt0, pnt1


def plane_axes(p0, p1, is_sphere: bool):
    """The 2D frame of mju_wrap, for arrays of endpoints in the geom frame."""
    if not is_sphere:
        a0 = np.zeros_like(p0)
        a0[..., 0] = 1.0
        a1 = np.zeros_like(p0)
        a1[..., 1] = 1.0
        return a0, a1
    a0 = p0 / np.linalg.norm(p0, axis=-1, keepdims=True)
    normal = np.cross(p0, p1)
    nrm = np.linalg.norm(normal, axis=-1, keepdims=True)
    normal = normal / np.where(nrm < MJMINVAL, 1.0, nrm)
    a1 = np.cross(normal, a0)
    a1 = a1 / np.where(np.linalg.norm(a1, axis=-1, keepdims=True) < MJMINVAL, 1.0, np.linalg.norm(a1, axis=-1, keepdims=True))
    return a0, a1


def segment_length(p0, p1, side_pos, radius: float, is_sphere: bool, rule: int):
    """Length of the site-geom-site piece; returns length and whether it wraps. side_pos is None or (3,) or (..., 3)."""
    a0, a1 = plane_axes(p0, p1, is_sphere)
    end0 = np.stack([(p0 * a0).sum(-1), (p0 * a1).sum(-1)], -1)
    end1 = np.stack([(p1 * a0).sum(-1), (p1 * a1).sum(-1)], -1)
    side2 = None
    if side_pos is not None:
        s = np.broadcast_to(side_pos, p0.shape)
        side2 = np.stack([(s * a0).sum(-1), (s * a1).sum(-1)], -1)
        nrm = np.linalg.norm(side2, axis=-1, keepdims=True)
        # mju_normalize leaves a zero vector as (1, 0)
        unit = np.where(nrm < MJMINVAL, np.array([1.0, 0.0]), side2 / np.where(nrm < MJMINVAL, 1.0, nrm))
        side2 = unit * radius
    wlen, t0, t1 = wrap_circle(end0, end1, side2, radius, rule)
    res0 = t0[..., 0, None] * a0 + t0[..., 1, None] * a1
    res1 = t1[..., 0, None] * a0 + t1[..., 1, None] * a1
    wrapped = wlen >= 0
    if is_sphere:
        total = np.linalg.norm(p0 - res0, axis=-1) + wlen + np.linalg.norm(p1 - res1, axis=-1)
    else:
        l0 = np.linalg.norm((p0 - res0)[..., :2], axis=-1)
        l1 = np.linalg.norm((p1 - res1)[..., :2], axis=-1)
        den = l0 + wlen + l1
        with np.errstate(divide="ignore", invalid="ignore"):
            z0 = p0[..., 2] + (p1[..., 2] - p0[..., 2]) * l0 / den
            z1 = p0[..., 2] + (p1[..., 2] - p0[..., 2]) * (l0 + wlen) / den
        res0 = res0.copy()
        res1 = res1.copy()
        res0[..., 2] = z0
        res1[..., 2] = z1
        wl = np.sqrt(wlen * wlen + (z1 - z0) ** 2)
        total = np.linalg.norm(p0 - res0, axis=-1) + wl + np.linalg.norm(p1 - res1, axis=-1)
    straight = np.linalg.norm(p0 - p1, axis=-1)
    return np.where(wrapped, total, straight), wrapped


def rule_decides(wrap: Wrap, side: np.ndarray) -> bool:
    """Whether the side-site rule applies: an outside side site on the geom's body (inside wraps are the same in both)."""
    return wrap.side >= 0 and not np.isnan(side[0]) and np.linalg.norm(side) >= wrap.radius


def limited_joints(model) -> np.ndarray:
    """Limited hinge and slide joints."""
    kinds = (2, 3)
    return np.array([j for j in range(model.njnt) if model.jnt_limited[j] and int(model.jnt_type[j]) in kinds], dtype=int)


def apply_couplings(model, q: np.ndarray) -> None:
    """Move joint-equality followers onto their polynomial, in place, for an array of postures."""
    eqs = [e for e in range(model.neq) if model.eq_type[e] == 2 and model.eq_active0[e]]
    for _ in range(3):
        for e in eqs:
            j1, j2 = model.eq_obj1id[e], model.eq_obj2id[e]
            a1 = model.jnt_qposadr[j1]
            poly = model.eq_data[e][:5]
            if j2 < 0:
                q[..., a1] = model.qpos0[a1] + poly[0]
                continue
            a2 = model.jnt_qposadr[j2]
            x = q[..., a2] - model.qpos0[a2]
            q[..., a1] = model.qpos0[a1] + sum(c * x**i for i, c in enumerate(poly))


def sample_postures(model, n: int, seed: int, coupled: bool, margin: float = 0.0) -> np.ndarray:
    """Random in-range postures: half uniform, a quarter half at limits, a quarter all at limits."""
    rng = np.random.default_rng(seed)
    joints = limited_joints(model)
    adr = model.jnt_qposadr[joints]
    lo = model.jnt_range[joints, 0] - margin * (model.jnt_type[joints] == 3)
    hi = model.jnt_range[joints, 1] + margin * (model.jnt_type[joints] == 3)
    u = rng.random((n, len(joints)))
    corners = rng.integers(0, 2, (n, len(joints))).astype(float)
    n_half, n_quarter = n // 2, n // 4
    u[n_half : n_half + n_quarter] = np.where(
        rng.random((n_quarter, len(joints))) < 0.5, corners[n_half : n_half + n_quarter], u[n_half : n_half + n_quarter]
    )
    u[n_half + n_quarter :] = corners[n_half + n_quarter :]
    q = np.tile(model.qpos0, (n, 1))
    q[:, adr] = lo + u * (hi - lo)
    if coupled:
        apply_couplings(model, q)
    return q


def sweep_postures(model, steps: int, seed: int, coupled: bool, bases: int = 4, margin: float = 0.0):
    """Every limited joint swept alone, from qpos0 and from random postures. Returns q and a segment id per row."""
    joints = limited_joints(model)
    start = np.vstack([model.qpos0[None], sample_postures(model, bases, seed, coupled)])
    rows, ids = [], []
    for j in joints:
        adr = model.jnt_qposadr[j]
        follower = coupled and any(model.eq_type[e] == 2 and model.eq_obj1id[e] == j for e in range(model.neq))
        if follower:
            continue
        lo, hi = (
            model.jnt_range[j, 0] - margin * (model.jnt_type[j] == 3),
            model.jnt_range[j, 1] + margin * (model.jnt_type[j] == 3),
        )
        grid = np.linspace(lo, hi, steps)
        for b, base in enumerate(start):
            q = np.tile(base, (steps, 1))
            q[:, adr] = grid
            if coupled:
                apply_couplings(model, q)
            rows.append(q)
            ids.append(np.full(steps, j * 100 + b))
    return np.vstack(rows), np.concatenate(ids)


def forward_sites(model, qs: np.ndarray, wraps: list[Wrap]):
    """Endpoints of every wrap in its geom frame (n, wraps, 2, 3), and real tendon lengths (n, ntendon)."""
    import mujoco

    data = mujoco.MjData(model)
    ends = np.empty((len(qs), len(wraps), 2, 3))
    length = np.empty((len(qs), model.ntendon))
    s0 = np.array([w.site0 for w in wraps], dtype=int)
    s1 = np.array([w.site1 for w in wraps], dtype=int)
    g = np.array([w.geom for w in wraps], dtype=int)
    for i, q in enumerate(qs):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_tendon(model, data)
        length[i] = data.ten_length
        mats = data.geom_xmat[g].reshape(-1, 3, 3)
        ends[i, :, 0] = np.einsum("nji,nj->ni", mats, data.site_xpos[s0] - data.geom_xpos[g])
        ends[i, :, 1] = np.einsum("nji,nj->ni", mats, data.site_xpos[s1] - data.geom_xpos[g])
    return ends, length


def side_local(model, wraps: list[Wrap]) -> np.ndarray:
    """Side site in the geom frame; nan without a side site or with the site on another body (it moves)."""
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    out = np.full((len(wraps), 3), np.nan)
    for k, w in enumerate(wraps):
        if w.side >= 0:
            if model.site_bodyid[w.side] != model.geom_bodyid[w.geom]:
                continue
            out[k] = data.geom_xmat[w.geom].reshape(3, 3).T @ (data.site_xpos[w.side] - data.geom_xpos[w.geom])
    return out


@dataclass
class WrapSamples:
    """Per-posture quantities of one wrap that do not depend on the side site."""

    a0: np.ndarray
    a1: np.ndarray
    end0: np.ndarray
    end1: np.ndarray
    near: np.ndarray
    clear: np.ndarray
    mid: np.ndarray  # (n, 2, 2): normalised sum of the tangent points of each solution
    state_len3: np.ndarray  # (n, 3): length for no wrap, solution 0, solution 1 under rule 3
    state_len4: np.ndarray
    blocked3: np.ndarray  # (n, 2): tangent segments of the solution cross under rule 3
    blocked4: np.ndarray
    wind: np.ndarray
    valid: np.ndarray
    chord: np.ndarray  # (n, 2): squared distance between the tangent points of each solution


def precompute(p0, p1, radius: float, is_sphere: bool) -> WrapSamples:
    """Everything the two rules need per posture, so that candidate side sites cost little."""
    a0, a1 = plane_axes(p0, p1, is_sphere)
    end0 = np.stack([(p0 * a0).sum(-1), (p0 * a1).sum(-1)], -1)
    end1 = np.stack([(p1 * a0).sum(-1), (p1 * a1).sum(-1)], -1)
    sqlen0, sqlen1 = (end0 * end0).sum(-1), (end1 * end1).sum(-1)
    sqrad = radius * radius
    dif = end1 - end0
    dd = (dif * dif).sum(-1)
    valid = (sqlen0 >= sqrad) & (sqlen1 >= sqrad) & (dd >= MJMINVAL)
    with np.errstate(divide="ignore", invalid="ignore"):
        a = np.clip(-(dif * end0).sum(-1) / dd, 0.0, 1.0)
    near = a[..., None] * dif + end0
    clear = (near * near).sum(-1) > sqrad
    sqrt0 = np.sqrt(np.maximum(sqlen0 - sqrad, 0.0))
    sqrt1 = np.sqrt(np.maximum(sqlen1 - sqrad, 0.0))
    wind = _winding(end0, end1)
    n = len(p0)
    mid = np.empty((n, 2, 2))
    len3, len4 = np.empty((n, 3)), np.empty((n, 3))
    len3[:, 0] = len4[:, 0] = np.linalg.norm(p0 - p1, axis=-1)
    blocked3, blocked4 = np.empty((n, 2), bool), np.empty((n, 2), bool)
    chord = np.empty((n, 2))
    for i, sgn in enumerate((1.0, -1.0)):
        s0 = np.stack(
            [
                (end0[:, 0] * sqrad + sgn * radius * end0[:, 1] * sqrt0) / sqlen0,
                (end0[:, 1] * sqrad - sgn * radius * end0[:, 0] * sqrt0) / sqlen0,
            ],
            -1,
        )
        s1 = np.stack(
            [
                (end1[:, 0] * sqrad - sgn * radius * end1[:, 1] * sqrt1) / sqlen1,
                (end1[:, 1] * sqrad + sgn * radius * end1[:, 0] * sqrt1) / sqlen1,
            ],
            -1,
        )
        m = s0 + s1
        nrm = np.linalg.norm(m, axis=-1, keepdims=True)
        mid[:, i] = m / np.where(nrm < MJMINVAL, 1.0, nrm)
        chord[:, i] = ((s0 - s1) ** 2).sum(-1)
        blocked3[:, i] = _is_intersect(end0, s0, end1, s1, strict=False)
        blocked4[:, i] = _is_intersect(end0, s0, end1, s1, strict=True)
        cross = s0[:, 1] * s1[:, 0] - s0[:, 0] * s1[:, 1]
        angle = np.arctan2(np.abs(cross), (s0 * s1).sum(-1))
        flip = (cross > 0) if i == 1 else (cross < 0)
        arc3 = np.where(flip, 2 * np.pi - angle, angle)
        # wside is the representative of the end-to-end winding with the sign of the solution
        wside = np.where(wind > 0, wind, wind + 2 * np.pi) if i == 1 else np.where(wind < 0, wind, wind - 2 * np.pi)
        arc4 = np.abs(wside - _winding(end0, s0) - _winding(s1, end1))
        r0 = s0[:, 0, None] * a0 + s0[:, 1, None] * a1
        r1 = s1[:, 0, None] * a0 + s1[:, 1, None] * a1
        for out, arc in ((len3, arc3), (len4, arc4)):
            wl = radius * arc
            if is_sphere:
                out[:, i + 1] = np.linalg.norm(p0 - r0, axis=-1) + wl + np.linalg.norm(p1 - r1, axis=-1)
            else:
                l0 = np.linalg.norm((p0 - r0)[:, :2], axis=-1)
                l1 = np.linalg.norm((p1 - r1)[:, :2], axis=-1)
                den = l0 + wl + l1
                z0 = p0[:, 2] + (p1[:, 2] - p0[:, 2]) * l0 / den
                z1 = p0[:, 2] + (p1[:, 2] - p0[:, 2]) * (l0 + wl) / den
                c0, c1 = r0.copy(), r1.copy()
                c0[:, 2], c1[:, 2] = z0, z1
                out[:, i + 1] = (
                    np.linalg.norm(p0 - c0, axis=-1) + np.sqrt(wl * wl + (z1 - z0) ** 2) + np.linalg.norm(p1 - c1, axis=-1)
                )
    return WrapSamples(a0, a1, end0, end1, near, clear, mid, len3, len4, blocked3, blocked4, wind, valid, chord)


def states(ws: WrapSamples, side_dirs: np.ndarray, rule: int) -> np.ndarray:
    """Which path each candidate side direction (c, 3), or (c, n, 3), gives per posture: 0 no wrap, 1 and 2 the solutions; (c, n)."""
    if side_dirs.ndim == 2:
        sx = side_dirs @ ws.a0.T
        sy = side_dirs @ ws.a1.T
    else:
        sx = (side_dirs * ws.a0[None]).sum(-1)
        sy = (side_dirs * ws.a1[None]).sum(-1)
    if rule == 3:
        drop = ws.clear[None] & (sx * ws.near[:, 0] + sy * ws.near[:, 1] >= 0)
        g0 = np.where(ws.blocked3[:, 0], -10000.0, sx * ws.mid[:, 0, 0] + sy * ws.mid[:, 0, 1])
        g1 = np.where(ws.blocked3[:, 1], -10000.0, sx * ws.mid[:, 1, 0] + sy * ws.mid[:, 1, 1])
        pick1 = ~(g0 > g1)
        blocked = np.where(pick1, ws.blocked3[:, 1], ws.blocked3[:, 0])
    else:
        e0x, e0y = ws.end0[:, 0], ws.end0[:, 1]
        e1x, e1y = ws.end1[:, 0], ws.end1[:, 1]
        wside = np.arctan2(e0x * sy - e0y * sx, e0x * sx + e0y * sy) + np.arctan2(sx * e1y - sy * e1x, sx * e1x + sy * e1y)
        drop = ws.clear[None] & (np.abs(wside - ws.wind) < np.pi)
        pick1 = wside > 0
        blocked = np.where(pick1, ws.blocked4[:, 1], ws.blocked4[:, 0])
    state = pick1.astype(np.int8) + 1
    state[drop | blocked | ~ws.valid[None]] = 0
    return state


def states_no_side(ws: WrapSamples, rule: int) -> np.ndarray:
    """Path without a side site: the shorter arc if the segment crosses the geom; (n,). The same under both rules but for is_intersect."""
    blocked_all = ws.blocked3 if rule == 3 else ws.blocked4
    g0 = np.where(blocked_all[:, 0], -10000.0, -ws.chord[:, 0])
    g1 = np.where(blocked_all[:, 1], -10000.0, -ws.chord[:, 1])
    pick1 = ~(g0 > g1)
    blocked = np.where(pick1, blocked_all[:, 1], blocked_all[:, 0])
    state = pick1.astype(np.int8) + 1
    state[ws.clear | blocked | ~ws.valid] = 0
    return state


def deviation(ws: WrapSamples, side_dirs: np.ndarray, reference: np.ndarray, rule: int, chunk: int = 256) -> np.ndarray:
    """Length error against the reference per candidate and posture (c, n), float32."""
    table = ws.state_len3 if rule == 3 else ws.state_len4
    out = np.empty((len(side_dirs), len(reference)), np.float32)
    idx = np.arange(len(reference))
    for c in range(0, len(side_dirs), chunk):
        st = states(ws, side_dirs[c : c + chunk], rule)
        out[c : c + chunk] = np.abs(table[idx, st] - reference)
    return out


def direction_grid(is_sphere: bool, n_circle: int = 720, n_sphere: int = 4000) -> np.ndarray:
    """Candidate side directions in the geom frame: a circle for a cylinder, a Fibonacci sphere for a sphere."""
    if is_sphere:
        i = np.arange(n_sphere) + 0.5
        z = 1 - 2 * i / n_sphere
        th = np.pi * (1 + 5**0.5) * i
        r = np.sqrt(1 - z * z)
        return np.stack([r * np.cos(th), r * np.sin(th), z], 1)
    t = np.linspace(0, 2 * np.pi, n_circle, endpoint=False)
    return np.stack([np.cos(t), np.sin(t), np.zeros_like(t)], 1)


def wrap_geometry(model, qs: np.ndarray, wrap: Wrap, sides: list[tuple[str, np.ndarray]]):
    """Endpoints of a wrap and points fixed on bodies, all in the geom frame: (n, 2, 3) and one (n, 3) per side."""
    import mujoco

    data = mujoco.MjData(model)
    ends = np.empty((len(qs), 2, 3))
    local = [np.empty((len(qs), 3)) for _ in sides]
    bodies = [model.body(name).id for name, _ in sides]
    for i, q in enumerate(qs):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        rot, pos = data.geom_xmat[wrap.geom].reshape(3, 3), data.geom_xpos[wrap.geom]
        ends[i, 0] = rot.T @ (data.site_xpos[wrap.site0] - pos)
        ends[i, 1] = rot.T @ (data.site_xpos[wrap.site1] - pos)
        for out, b, (_, point) in zip(local, bodies, sides):
            out[i] = rot.T @ (data.xpos[b] + data.xmat[b].reshape(3, 3) @ point - pos)
    return ends, local


def rule_tendon_lengths(model, qs: np.ndarray, rule: int) -> tuple[np.ndarray, np.ndarray]:
    """Tendon lengths from the rule copy, and a mask of the tendons it covers (spatial, no inside wrap)."""
    wraps = model_wraps(model)
    side = side_local(model, wraps)
    ends, _ = forward_sites(model, qs, wraps)
    import mujoco

    data = mujoco.MjData(model)
    covered = np.array([int(model.wrap_type[model.tendon_adr[t]]) != 1 for t in range(model.ntendon)])
    for k, w in enumerate(wraps):
        if w.side >= 0 and not rule_decides(w, side[k]):
            covered[w.tendon] = False
    pieces = np.zeros((len(qs), model.ntendon))
    for k, w in enumerate(wraps):
        if not covered[w.tendon]:
            continue
        length, _ = segment_length(ends[:, k, 0], ends[:, k, 1], None if w.side < 0 else side[k], w.radius, w.is_sphere, rule)
        pieces[:, w.tendon] += length / w.divisor
    for i, q in enumerate(qs):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        for t in np.flatnonzero(covered):
            adr, num = model.tendon_adr[t], model.tendon_num[t]
            divisor = 1.0
            for a in range(adr, adr + num - 1):
                if int(model.wrap_type[a]) == 2:
                    divisor = float(model.wrap_prm[a])
                    continue
                if int(model.wrap_type[a + 1]) == 2 or int(model.wrap_type[a]) != WRAP_SITE:
                    continue
                if int(model.wrap_type[a + 1]) != WRAP_SITE:
                    continue
                s0, s1 = model.wrap_objid[a], model.wrap_objid[a + 1]
                pieces[i, t] += np.linalg.norm(data.site_xpos[s0] - data.site_xpos[s1]) / divisor
    return pieces, covered


def tier_postures(model, tier: str, n: int, steps: int, seed: int = 21) -> np.ndarray:
    """Random postures and single-joint sweeps of one posture set."""
    coupled, margin = TIERS[tier]
    return np.vstack(
        [sample_postures(model, n, seed, coupled, margin), sweep_postures(model, steps, seed + 1, coupled, margin=margin)[0]]
    )


def reference_lengths(ws: WrapSamples, side_dir: np.ndarray) -> np.ndarray:
    """The MuJoCo 3 length of the piece at the current side site."""
    state = states(ws, side_dir[None], 3)[0]
    return ws.state_len3[np.arange(len(state)), state]


def chain_between(model, body0: int, body1: int) -> list[int]:
    """Bodies on the kinematic path between two bodies."""

    def up(b: int) -> list[int]:
        path = [b]
        while b > 0:
            b = int(model.body_parentid[b])
            path.append(b)
        return path

    a, b = up(body0), up(body1)
    common = next(x for x in a if x in b)
    return a[: a.index(common) + 1] + b[: b.index(common)]


def off_body_sweep(name: str, wrap_key: str, body: str, joint: str, extra: float = 0.8) -> list[dict]:
    """Side sites on *body* for a wrap whose relative motion is one joint: on the MuJoCo 4 rule, the current path in range.

    Candidates are ranked by how far past the limits the path keeps its side.
    """
    import mujoco

    import myo_sim

    model = myo_sim.load_model(name)
    w = next(x for x in model_wraps(model) if f"{x.tendon_name}/{x.geom_name}" == wrap_key)
    b = model.body(body).id
    j = model.joint(joint)
    lo, hi = j.range
    grid = np.arange(lo - extra, hi + extra, 1e-3)
    data = mujoco.MjData(model)
    ends = np.empty((len(grid), 2, 3))
    rot_rel, pos_rel = np.empty((len(grid), 3, 3)), np.empty((len(grid), 3))
    side_now = np.empty((len(grid), 3))
    for i, x in enumerate(grid):
        data.qpos[:] = model.qpos0
        data.qpos[j.qposadr[0]] = x
        mujoco.mj_kinematics(model, data)
        rot, pos = data.geom_xmat[w.geom].reshape(3, 3), data.geom_xpos[w.geom]
        ends[i, 0] = rot.T @ (data.site_xpos[w.site0] - pos)
        ends[i, 1] = rot.T @ (data.site_xpos[w.site1] - pos)
        side_now[i] = rot.T @ (data.site_xpos[w.side] - pos)
        rot_rel[i] = rot.T @ data.xmat[b].reshape(3, 3)
        pos_rel[i] = rot.T @ (data.xpos[b] - pos)
    ws = precompute(ends[:, 0], ends[:, 1], w.radius, w.is_sphere)
    idx = np.arange(len(grid))
    ref = ws.state_len4[idx, states(ws, side_now[None], 4)[0]]
    data.qpos[:] = model.qpos0
    mujoco.mj_kinematics(model, data)
    rot0, pos0 = data.geom_xmat[w.geom].reshape(3, 3), data.geom_xpos[w.geom]
    rot_b, pos_b = data.xmat[b].reshape(3, 3), data.xpos[b]
    angles = np.radians(np.arange(0.0, 360.0, 1.0))
    i0, i1 = np.searchsorted(grid, lo), np.searchsorted(grid, hi)
    rows = []
    for rho in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0):
        for z in (0.0, 1.5, 2.0, 3.0, 4.0, -1.5, -2.0, -3.0, -4.0):
            plane = np.stack([rho * np.cos(angles), rho * np.sin(angles), z + 0 * angles], 1)
            points = ((pos0 + (plane * w.radius) @ rot0.T) - pos_b) @ rot_b
            loc = np.einsum("nij,cj->cni", rot_rel, points) + pos_rel[None]
            outside = np.linalg.norm(loc, axis=-1).min(1) > 1.5 * w.radius
            st = states(ws, loc, 4)
            exact = np.abs(ws.state_len4[idx, st] - ref) < LENGTH_TOL
            for c in np.flatnonzero(outside & exact[:, i0 : i1 + 1].all(1)):
                a, e = i0, i1
                while a > 0 and st[c, a - 1] == st[c, i0]:
                    a -= 1
                while e < len(grid) - 1 and st[c, e + 1] == st[c, i1]:
                    e += 1
                rows.append(
                    {
                        "pos": [round(float(v), 5) for v in points[c]],
                        "rho_radii": rho,
                        "z_radii": z,
                        "angle_deg": float(np.degrees(angles[c])),
                        "past_lower": round(float(lo - grid[a]), 3),
                        "past_upper": round(float(grid[e] - hi), 3),
                    }
                )
    rows.sort(key=lambda r: -min(r["past_lower"], r["past_upper"]))
    return rows


def _after_body_tag(lines: list[str], body: str) -> tuple[int, str]:
    """Line index just after a body's opening tag, outside any generated block, and the indent of its children."""
    import re

    anchor = next(i for i, x in enumerate(lines) if f'<body name="{body}"' in x)
    return anchor + 1, re.match(r"\s*", lines[anchor]).group(0) + "    "


def _edit_site(lines: list[str], site: dict) -> None:
    """Position of one moved side site, in place; a new parent body takes it just after its opening tag."""
    import re

    pos = 'pos="' + " ".join(f"{v:g}" for v in site["pos"]) + '"'
    found = [i for i, line in enumerate(lines) if f'<site name="{site["name"]}"' in line]
    if found and site["body"] == site.get("body_before", site["body"]):
        lines[found[0]] = re.sub(r'pos="[^"]*"', pos, lines[found[0]])
        return
    text = re.sub(r'pos="[^"]*"', pos, lines[found[0]]).strip() if found else f'<site name="{site["name"]}" {pos}/>'
    for i in reversed(found):
        del lines[i]
    k, indent = _after_body_tag(lines, site["body"])
    lines.insert(k, indent + text)


def write_chain() -> None:
    """Apply ``data/side_sites.json`` to the chains and tendon files: moved sites, removed side sites, geom radii."""
    import json
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    table = json.loads((root / "myo_sim/build/data/side_sites.json").read_text())
    models = root / "myo_sim/models"
    files = {
        "arm": (models / "arm/assets/myoarm_r_chain.xml", models / "arm/assets/myoarm_r_tendon.xml"),
        "torso": (models / "torso/assets/myotorso_chain.xml", models / "torso/assets/myotorso_tendon.xml"),
    }
    for part, (chain, tendons) in files.items():
        lines = chain.read_text().split("\n")
        text = tendons.read_text()
        for site in table["sites"]:
            if site.get("part", "arm") == part:
                _edit_site(lines, site)
        for removal in table.get("removed", []):
            if removal.get("part", "arm") != part:
                continue
            text = text.replace(f' sidesite="{removal["name"]}"', "")
            if f'"{removal["name"]}"' not in text:
                lines = [x for x in lines if f'<site name="{removal["name"]}"' not in x]
            if "radius_m" in removal:
                k = next(i for i, x in enumerate(lines) if f'name="{removal["geom"]}"' in x)
                size = re.search(r'size="([^"]*)"', lines[k]).group(1).split()
                size[0] = f"{removal['radius_m']:g}"
                lines[k] = re.sub(r'size="[^"]*"', 'size="' + " ".join(size) + '"', lines[k])
        for geom in table.get("geoms", []) if part == "arm" else []:
            lines = [x for x in lines if f'name="{geom["name"]}"' not in x]
            k = next(i for i, x in enumerate(lines) if f'name="{geom["copy_of"]}"' in x)
            pos = re.search(r'pos="([^"]*)"', lines[k]).group(1)
            size = f"{geom['radius_m']:g}"
            indent = re.match(r"\s*", lines[k]).group(0)
            lines.insert(
                k + 1, f'{indent}<geom class="myoarm_wrap" name="{geom["name"]}" type="sphere" pos="{pos}" size="{size}"/>'
            )
            for tendon in geom["tendons"]:
                start = text.index(f'name="{tendon}"')
                stop = text.index("</spatial>", start)
                text = (
                    text[:start]
                    + text[start:stop].replace(f'<geom geom="{geom["copy_of"]}"', f'<geom geom="{geom["name"]}"')
                    + text[stop:]
                )
        for via in table.get("vias", []):
            if via.get("part", "arm") != part:
                continue
            start = text.index(f'name="{via["tendon"]}"')
            stop = text.index("</spatial>", start)
            block = text[start:stop]
            # the site, in the wrap's place; a via replacing a site as well takes that entry out of the path
            block = re.sub(rf'<geom geom="{via["geom"]}"[^>]*?(/>|></geom>)', f'<site site="{via["name"]}"/>', block)
            for gone in via.get("drops_sites", []):
                block = re.sub(rf'\s*<site site="{gone}"/>', "", block)
            text = text[:start] + block + text[stop:]
            for name in (via["side"], via["geom"], *via.get("drops_sites", [])):
                if f'"{name}"' not in text:
                    lines = [x for x in lines if f'name="{name}"' not in x]
            if via.get("chain_block"):  # scripts/generate_extrinsic_muscles.py writes this site
                continue
            lines = [x for x in lines if f'<site name="{via["name"]}"' not in x]
            k, indent = _after_body_tag(lines, via["body"])
            pos = " ".join(f"{v:g}" for v in via["pos"])
            lines.insert(k, f'{indent}<site name="{via["name"]}" pos="{pos}"/>')
        chain.write_text("\n".join(lines))
        tendons.write_text(text)


def line_jumps(name: str, n: int, out_path: str, lines: int = 40, steps: int = 400) -> None:
    """Jumps of each differing wrap under both rules, on coupled in-range lines from a differing posture to mid-range.

    Rare differences hide from single-joint sweeps; a straight line in joint space through the coupling polynomials
    crosses them.
    """
    import json
    from pathlib import Path

    import myo_sim

    model = myo_sim.load_model(name)
    wraps = model_wraps(model)
    side = side_local(model, wraps)
    qs = sample_postures(model, n, 21, True)
    ends, _ = forward_sites(model, qs, wraps)
    joints = limited_joints(model)
    mid_q = model.qpos0.copy()[None]
    mid_q[0, model.jnt_qposadr[joints]] = model.jnt_range[joints].mean(1)
    apply_couplings(model, mid_q)
    t = np.linspace(0.0, 1.0, steps)[:, None]
    result = {}
    for k, w in enumerate(wraps):
        if not rule_decides(w, side[k]):
            continue
        args = (side[k], w.radius, w.is_sphere)
        l3, _ = segment_length(ends[:, k, 0], ends[:, k, 1], *args, 3)
        l4, _ = segment_length(ends[:, k, 0], ends[:, k, 1], *args, 4)
        bad = np.flatnonzero(np.abs(l3 - l4) > 1e-3)
        if len(bad) == 0:
            continue
        chosen = bad[np.random.default_rng(1).permutation(len(bad))[:lines]]
        worst = {3: 0.0, 4: 0.0}
        for i in chosen:
            path = qs[i][None] + t * (mid_q[0] - qs[i])[None]
            apply_couplings(model, path)
            e, _ = wrap_geometry(model, path, w, [])
            for rule in (3, 4):
                length, _ = segment_length(e[:, 0], e[:, 1], *args, rule)
                worst[rule] = max(worst[rule], float(np.abs(np.diff(length)).max()))
        key = f"{w.tendon_name}/{w.geom_name}"
        result[key] = {
            "differ_mm": round(float(np.abs(l3 - l4).max() * 1e3), 2),
            "share_percent": round(float((np.abs(l3 - l4) > 1e-3).mean() * 100), 3),
            "line_jump_mj3_mm": round(worst[3] * 1e3, 2),
            "line_jump_mj4_mm": round(worst[4] * 1e3, 2),
            "lines": len(chosen),
        }
        print(key, result[key], flush=True)
    Path(out_path).write_text(json.dumps(result, indent=1))


def dense_tendon_jacobian(model, data) -> np.ndarray:
    """Tendon Jacobian as a float32 array (ntendon, nv)."""
    import mujoco

    out = np.zeros((model.ntendon, model.nv))
    if data.ten_J.size == out.size:  # dense before the sparse tendon Jacobian
        return data.ten_J.reshape(out.shape).astype(np.float32)
    # the sparsity pattern moved from mjData to mjModel between MuJoCo versions
    holder = data if hasattr(data, "ten_J_rownnz") else model
    mujoco.mju_sparse2dense(out, data.ten_J, holder.ten_J_rownnz, holder.ten_J_rowadr, holder.ten_J_colind)
    return out.astype(np.float32)


def run_dump(out_dir: str, label: str, models: tuple[str, ...], n: int, steps: int, arm_postures: int) -> None:
    """Tendon lengths, moment arms and sweep jumps of the running MuJoCo, one file per model."""
    from pathlib import Path

    import mujoco

    import myo_sim

    folder = Path(out_dir)
    folder.mkdir(parents=True, exist_ok=True)
    for name in models:
        model = myo_sim.load_model(name)
        data = mujoco.MjData(model)
        spatial = np.array([int(model.wrap_type[model.tendon_adr[t]]) != 1 for t in range(model.ntendon)])
        result: dict[str, np.ndarray] = {
            "names": np.array([model.tendon(t).name for t in range(model.ntendon)]),
            "spatial": spatial,
        }
        for tier in TIERS:
            qs = tier_postures(model, tier, n, steps)
            length = np.empty((len(qs), model.ntendon))
            for i, q in enumerate(qs):
                data.qpos[:] = q
                mujoco.mj_kinematics(model, data)
                mujoco.mj_comPos(model, data)
                mujoco.mj_tendon(model, data)
                length[i] = data.ten_length
            result[f"length_{tier}"] = length
        arms = []
        for q in tier_postures(model, "A", arm_postures, 3)[:arm_postures]:
            data.qpos[:] = q
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            mujoco.mj_tendon(model, data)
            arms.append(dense_tendon_jacobian(model, data))
        result["arms"] = np.stack(arms)
        for coupled, tag in ((True, "coupled"), (False, "independent")):
            # the large models repeat the wraps of the smaller ones: fewer sweeps
            bases = 6 if model.njnt <= 60 else (1 if coupled else 0)
            qs, ids = sweep_postures(model, 1001, 31, coupled, bases=bases)
            length = np.empty((len(qs), model.ntendon))
            for i, q in enumerate(qs):
                data.qpos[:] = q
                mujoco.mj_kinematics(model, data)
                mujoco.mj_tendon(model, data)
                length[i] = data.ten_length
            step = np.abs(np.diff(length, axis=0))
            step[ids[1:] != ids[:-1]] = 0.0
            result[f"jump_{tag}"] = step.max(0)
        np.savez(folder / f"{label}_{name}.npz", version=mujoco.__version__, **result)
        print(label, name, mujoco.__version__, flush=True)


def run_check(folder: str, a: str, b: str, models: tuple[str, ...], limit_mm: float) -> None:
    """Largest length difference, share of postures and moment arm difference between two dumps."""
    from pathlib import Path

    for name in models:
        x, y = (np.load(Path(folder) / f"{label}_{name}.npz") for label in (a, b))
        names = x["names"]
        print(f"== {name}: {a} ({x['version']}) against {b} ({y['version']})")
        rows: dict[str, list] = {}
        for tier in TIERS:
            diff = np.abs(x[f"length_{tier}"] - y[f"length_{tier}"])
            worst, share = diff.max(0), (diff > limit_mm * 1e-3).mean(0)
            for t in np.flatnonzero(worst > limit_mm * 1e-3):
                rows.setdefault(str(names[t]), []).append(f"{tier} {worst[t] * 1e3:.2f} mm {share[t] * 100:.2f} %")
        arms = np.abs(x["arms"] - y["arms"]).max((0, 2))
        print(
            f"  tendons over {limit_mm} mm: {len(rows)}; moment arms: max {arms.max() * 1e3:.3f} mm, tendons over 0.1 mm {int((arms > 1e-4).sum())}"
        )
        for tendon, text in sorted(rows.items()):
            t = np.flatnonzero(names == tendon)[0]
            print(f"  {tendon:30s} {'; '.join(text)} | arm {arms[t] * 1e3:.2f} mm")


# Hinges with room to grow: searches keep these continuous past today's limits as well (rad).
EXTRA_MARGIN = {
    "flexion": 0.3,
    "deviation": 0.3,
    "mcp2_flexion": 0.55,
    "mcp3_flexion": 0.55,
    "mcp4_flexion": 0.55,
    "mcp5_flexion": 0.55,
}
JUMP_TOL = 0.3e-3  # m


def wrap_joints(model, wrap: Wrap) -> np.ndarray:
    """Limited joints that move the wrap's endpoints relative to its geom, followers of joint couplings included."""
    bodies = {int(model.site_bodyid[wrap.site0]), int(model.site_bodyid[wrap.site1]), int(model.geom_bodyid[wrap.geom])}
    chain: set[int] = set()
    ids = sorted(bodies)
    for a in ids:
        for b in ids:
            chain.update(chain_between(model, a, b))

    def depth(b: int) -> int:
        d = 0
        while b > 0:
            b = int(model.body_parentid[b])
            d += 1
        return d

    top = min(chain, key=depth)
    chain.discard(top)
    limited = set(limited_joints(model).tolist())
    return np.array([j for j in range(model.njnt) if int(model.jnt_bodyid[j]) in chain and j in limited], dtype=int)


def margin_of(model, joint: int, base: float) -> float:
    """How far past its limits a hinge is sampled."""
    name = model.joint(joint).name
    for token, extra in EXTRA_MARGIN.items():
        if name.startswith(token):
            return max(base, extra)
    return base if int(model.jnt_type[joint]) == 3 else 0.0


def continuity_postures(
    model,
    wrap: Wrap,
    past: bool,
    seed: int = 41,
    n_lines: int = 400,
    line_steps: int = 80,
    sweep_steps: int = 801,
    n_bases: int = 8,
):
    """Sequences of nearby postures for one wrap: sweeps of each joint that moves it, and straight lines in joint space."""
    rng = np.random.default_rng(seed)
    joints = wrap_joints(model, wrap)
    eqs = [e for e in range(model.neq) if int(model.eq_type[e]) == 2 and model.eq_active0[e] and model.eq_obj2id[e] >= 0]
    followers = {int(model.eq_obj1id[e]) for e in eqs}
    drivers = {int(model.eq_obj2id[e]) for e in eqs if int(model.eq_obj1id[e]) in set(joints.tolist())}
    drive = sorted({int(j) for j in joints if int(j) not in followers} | drivers)
    limits = {}
    for j in drive:
        extra = margin_of(model, j, LIMIT_MARGIN) if past else 0.0
        limits[j] = (model.jnt_range[j][0] - extra, model.jnt_range[j][1] + extra)

    def draw(n: int) -> np.ndarray:
        q = np.tile(model.qpos0, (n, 1))
        for j in drive:
            lo, hi = limits[j]
            u = rng.random(n)
            edge = rng.random(n) < 0.25
            u[edge] = rng.integers(0, 2, edge.sum())
            q[:, model.jnt_qposadr[j]] = lo + u * (hi - lo)
        return q

    rows, ids = [], []
    count = 0
    bases = np.vstack([model.qpos0[None], draw(n_bases - 1)])
    for j in drive:
        lo, hi = limits[j]
        grid = np.linspace(lo, hi, sweep_steps)
        for base in bases:
            q = np.tile(base, (sweep_steps, 1))
            q[:, model.jnt_qposadr[j]] = grid
            rows.append(q)
            ids.append(np.full(sweep_steps, count))
            count += 1
    t = np.linspace(0.0, 1.0, line_steps)[None, :, None]
    a, b = draw(n_lines), draw(n_lines)
    rows.append((a[:, None, :] * (1 - t) + b[:, None, :] * t).reshape(-1, model.nq))
    ids.append(np.repeat(np.arange(count, count + n_lines), line_steps))
    q = np.vstack(rows)
    apply_couplings(model, q)
    return q, np.concatenate(ids)


def state_jump(ws: WrapSamples, st: np.ndarray, rule: int, same: np.ndarray) -> np.ndarray:
    """Length discontinuity at each change of state between neighbouring samples, (..., n-1).

    Each state is compared at both samples and the smaller difference counts: a wrap that has just become invalid
    has a meaningless arc on one side of the change.
    """
    table = ws.state_len3 if rule == 3 else ws.state_len4
    n = st.shape[-1]
    idx = np.arange(n - 1)
    a, b = st[..., :-1], st[..., 1:]
    here = np.abs(table[idx, a] - table[idx, b])
    there = np.abs(table[idx + 1, a] - table[idx + 1, b])
    return np.where((a != b) & same, np.minimum(here, there), 0.0)


def side_search(name: str, wrap_key: str, n_circle: int = 720, n_sphere: int = 3000) -> dict:
    """Score side directions on the geom's body for one wrap: discontinuities under both rules, change from the current path."""
    import myo_sim

    model = myo_sim.load_model(name)
    wraps = model_wraps(model)
    side = side_local(model, wraps)
    k = next(i for i, w in enumerate(wraps) if f"{w.tendon_name}/{w.geom_name}" == wrap_key)
    w = wraps[k]
    dirs = direction_grid(w.is_sphere, n_circle, n_sphere)
    out: dict = {
        "wrap": wrap_key,
        "site": w.side_name,
        "radius": w.radius,
        "is_sphere": w.is_sphere,
        "dirs": dirs,
        "current": side[k],
    }
    for label, past in (("in", False), ("past", True)):
        qs, ids = continuity_postures(model, w, past)
        ends, _ = wrap_geometry(model, qs, w, [])
        ws = precompute(ends[:, 0], ends[:, 1], w.radius, w.is_sphere)
        same = ids[1:] == ids[:-1]
        now3 = states(ws, side[k][None], 3)[0]
        now4 = states(ws, side[k][None], 4)[0]
        idx = np.arange(len(qs))
        ref3 = ws.state_len3[idx, now3]
        out[f"{label}_now"] = {r: float(state_jump(ws, s[None], r, same).max()) for r, s in ((3, now3), (4, now4))}
        j3, j4 = np.zeros(len(dirs)), np.zeros(len(dirs))
        c3, c4, s3 = np.zeros(len(dirs)), np.zeros(len(dirs)), np.zeros(len(dirs))
        f4 = np.zeros(len(dirs), int)
        for c0 in range(0, len(dirs), 96):
            c = dirs[c0 : c0 + 96]
            st3, st4 = states(ws, c, 3), states(ws, c, 4)
            j3[c0 : c0 + 96] = state_jump(ws, st3, 3, same).max(1)
            j4[c0 : c0 + 96] = state_jump(ws, st4, 4, same).max(1)
            f4[c0 : c0 + 96] = ((st4[:, :-1] > 0) & (st4[:, 1:] > 0) & (st4[:, :-1] != st4[:, 1:]) & same).sum(1)
            l4 = ws.state_len4[idx, st4]
            l3 = ws.state_len3[idx, st3]
            c4[c0 : c0 + 96] = (np.abs(l4 - ref3) > 1e-3).mean(1)
            c3[c0 : c0 + 96] = (np.abs(l3 - ref3) > 1e-3).mean(1)
            s3[c0 : c0 + 96] = (np.abs(l3 - l4) > 1e-3).mean(1)
        out[f"{label}_flips4"] = f4
        out[f"{label}_j3"], out[f"{label}_j4"], out[f"{label}_change4"], out[f"{label}_change3"], out[f"{label}_rules"] = (
            j3,
            j4,
            c4,
            c3,
            s3,
        )
        out[f"{label}_postures"] = len(qs)
    return out


def fit_vias(selected: tuple[str, ...] = ()) -> None:
    """Fit the via sites of ``data/side_sites.json`` to their moment-arm targets and store the positions and residuals.

    The via must already be in the chain and tendon files (``write`` with any start position).
    """
    import json
    from pathlib import Path

    import mujoco
    from scipy.optimize import least_squares

    import myo_sim

    path = Path(__file__).resolve().parents[1] / "myo_sim/build/data/side_sites.json"
    table = json.loads(path.read_text())
    model = myo_sim.load_model("myoarm_r")
    data = mujoco.MjData(model)

    def arm(t: int, joint: str, deg: float, base: dict | None = None) -> float:
        adr = model.joint(joint).qposadr[0]
        lengths = []
        for h in (-1e-3, 1e-3):
            data.qpos[:] = model.qpos0
            for name, value in (base or {}).items():
                data.qpos[model.joint(name).qposadr[0]] = value
            data.qpos[adr] = np.radians(deg) + h
            mujoco.mj_kinematics(model, data)
            mujoco.mj_tendon(model, data)
            lengths.append(data.ten_length[t])
        return (lengths[1] - lengths[0]) / 2e-3 * 1e3

    for via in table["vias"]:
        if selected and via["name"] not in selected:
            continue
        site = model.site(via["name"]).id
        t = model.tendon(via["tendon"]).id

        def residuals(x: np.ndarray) -> np.ndarray:
            model.site_pos[site] = x
            base = via.get("base")
            out = [(arm(t, r["joint"], r["deg"], base) - r["arm_mm"]) / r["sd"] for r in via["targets"]]
            for r in via.get("mean_targets", []):
                grid = np.arange(r["range_deg"][0], r["range_deg"][1] + 1, 10.0)
                out.append((np.mean([arm(t, r["joint"], g) for g in grid]) - r["arm_mm"]) / r["sd"])
            for r in via.get("sign", []):
                grid = np.arange(r["range_deg"][0], r["range_deg"][1] + 1, 10.0)
                out += [max(0.0, r["margin_mm"] - r["sign"] * arm(t, r["joint"], g, base)) for g in grid]
            return np.array(out)

        start = np.array(via["pos"], float)
        bounds = (start - 0.02, start + 0.02)
        best = None
        rng = np.random.default_rng(3)
        for k in range(6):
            x0 = start if k == 0 else np.clip(start + rng.uniform(-0.01, 0.01, 3), *bounds)
            fit = least_squares(residuals, x0, bounds=bounds, diff_step=1e-3, x_scale=0.003, max_nfev=60)
            if best is None or fit.cost < best.cost:
                best = fit
        via["pos"] = [round(float(v), 6) for v in best.x]
        model.site_pos[site] = via["pos"]
        via["fit"] = {
            "rms_z": round(float(np.sqrt(np.mean(residuals(best.x) ** 2))), 3),
            "arms": {
                r["joint"] + "@" + str(r["deg"]): round(float(arm(t, r["joint"], r["deg"], via.get("base"))), 2)
                for r in via["targets"]
            },
            "curve_flexion_deg_mm": {str(g): round(float(arm(t, "flexion_r", g)), 2) for g in range(-90, 81, 30)},
            "curve_deviation_deg_mm": {str(g): round(float(arm(t, "deviation_r", g)), 2) for g in range(-30, 41, 10)},
        }
        print(via["name"], via["pos"], via["fit"], flush=True)
    path.write_text(json.dumps(table, indent=2) + "\n")


def record_jumps() -> None:
    """Store in ``data/side_sites.json`` every wrap whose length jumps on MuJoCo 4 by more than JUMP_TOL, with the MuJoCo 3 value."""
    import json
    from pathlib import Path

    import myo_sim

    path = Path(__file__).resolve().parents[1] / "myo_sim/build/data/side_sites.json"
    table = json.loads(path.read_text())
    table["jumps_mm"] = {}
    for name in ("myoarm_r", "myolegs", "myotorso"):
        model = myo_sim.load_model(name)
        wraps = model_wraps(model)
        side = side_local(model, wraps)
        out = table["jumps_mm"].setdefault(name, {})
        for k, w in enumerate(wraps):
            if w.side >= 0 and not np.isnan(side[k][0]) and np.linalg.norm(side[k]) < w.radius:
                continue
            row = {}
            for label, past in (("in", False), ("past", True)):
                qs, ids = continuity_postures(model, w, past)
                ends, _ = wrap_geometry(model, qs, w, [])
                ws = precompute(ends[:, 0], ends[:, 1], w.radius, w.is_sphere)
                same = ids[1:] == ids[:-1]
                if w.side < 0:
                    st3, st4 = states_no_side(ws, 3)[None], states_no_side(ws, 4)[None]
                elif np.isnan(side[k][0]):
                    body = model.body(model.site_bodyid[w.side]).name
                    moving = wrap_geometry(model, qs, w, [(body, np.array(model.site_pos[w.side]))])[1][0][None]
                    st3, st4 = states(ws, moving, 3), states(ws, moving, 4)
                else:
                    st3, st4 = states(ws, side[k][None], 3), states(ws, side[k][None], 4)
                row[label] = round(float(state_jump(ws, st4, 4, same).max() * 1e3), 2)
                row[label + "_mujoco3"] = round(float(state_jump(ws, st3, 3, same).max() * 1e3), 2)
            if max(row["in"], row["past"]) > JUMP_TOL * 1e3 or max(row["in_mujoco3"], row["past_mujoco3"]) > JUMP_TOL * 1e3:
                out[f"{w.tendon_name}/{w.geom_name}"] = row
                print(name, w.tendon_name, w.geom_name, row, flush=True)
    path.write_text(json.dumps(table, indent=2) + "\n")


def select_side_sites(wrap_keys: tuple[str, ...], also_mujoco3: bool, unchanged: bool) -> None:
    """Middle of the widest run of side directions without jump or change of side on MuJoCo 4, past the limits as well;
    optionally only those that also have none on MuJoCo 3, and that leave today's path as it is. Updates sites."""
    import json
    from pathlib import Path

    import mujoco

    import myo_sim

    path = Path(__file__).resolve().parents[1] / "myo_sim/build/data/side_sites.json"
    table = json.loads(path.read_text())
    model = myo_sim.load_model("myoarm_r")
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    wraps = model_wraps(model)
    for key in wrap_keys:
        w = next(x for x in wraps if f"{x.tendon_name}/{x.geom_name}" == key)
        r = side_search("myoarm_r", key, n_circle=720)
        good = (r["in_j4"] <= JUMP_TOL) & (r["past_j4"] <= JUMP_TOL) & (r["in_flips4"] == 0)
        width4 = float(good.sum() * 0.5)
        if also_mujoco3:
            good &= (r["in_j3"] <= JUMP_TOL) & (r["past_j3"] <= JUMP_TOL)
        if unchanged:
            good &= (r["in_change4"] < 1e-3) & (r["in_change3"] < 1e-3)
        doubled = np.r_[good, good]
        best_len, best_end, run = 0, 0, 0
        for i, g in enumerate(doubled):
            run = run + 1 if g else 0
            if best_len < run <= len(good):
                best_len, best_end = run, i
        direction = r["dirs"][((2 * best_end - best_len + 1) // 2) % len(good)]
        world = data.geom_xpos[w.geom] + data.geom_xmat[w.geom].reshape(3, 3) @ (direction * 2 * w.radius)
        body = int(model.site_bodyid[w.side])
        pos = data.xmat[body].reshape(3, 3).T @ (world - data.xpos[body])
        entry = next(e for e in table["sites"] if e["name"] == w.side_name)
        entry["pos"] = [round(float(v), 6) for v in pos]
        entry["plateau_width_deg"] = round(best_len * 0.5, 1)
        entry["plateau_width_mujoco4_deg"] = width4
        print(key, entry["pos"], best_len * 0.5, width4, flush=True)
    path.write_text(json.dumps(table, indent=2) + "\n")


def main() -> None:
    """Command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("write", help="apply data/side_sites.json to the chain and tendon files")
    fit = sub.add_parser("fit-vias", help="fit the via sites to their moment-arm targets")
    fit.add_argument("names", nargs="*")
    select = sub.add_parser("select-sides", help="middle of the jump-free side directions of a wrap")
    select.add_argument("wraps", nargs="+", help="tendon/geom")
    select.add_argument("--also-mujoco3", action="store_true")
    select.add_argument("--unchanged", action="store_true")
    sweep = sub.add_parser("off-body-sweep", help="side sites on another body for a wrap moved by one joint")
    sweep.add_argument("--model", default="myoarm_r")
    sweep.add_argument("--wrap", required=True)
    sweep.add_argument("--body", required=True)
    sweep.add_argument("--joint", required=True)
    sub.add_parser("record-jumps", help="store the wraps with a length jump in data/side_sites.json")
    lines = sub.add_parser("line-jumps", help="jumps of differing wraps on coupled lines")
    lines.add_argument("--model", default="myoarm_r")
    lines.add_argument("--out", required=True)
    lines.add_argument("--postures", type=int, default=20000)
    dump = sub.add_parser("dump", help="lengths, moment arms and sweep jumps of the running MuJoCo")
    dump.add_argument("folder")
    dump.add_argument("label")
    dump.add_argument("--models", nargs="*", default=list(CHECK_MODELS))
    dump.add_argument("--postures", type=int, default=12000)
    dump.add_argument("--steps", type=int, default=101)
    dump.add_argument("--arm-postures", type=int, default=800)
    check = sub.add_parser("check", help="compare two dumps")
    check.add_argument("folder")
    check.add_argument("a")
    check.add_argument("b")
    check.add_argument("--models", nargs="*", default=list(CHECK_MODELS))
    check.add_argument("--limit-mm", type=float, default=1.0)
    args = parser.parse_args()
    if args.command == "write":
        write_chain()
    elif args.command == "fit-vias":
        fit_vias(tuple(args.names))
    elif args.command == "select-sides":
        select_side_sites(tuple(args.wraps), args.also_mujoco3, args.unchanged)
    elif args.command == "off-body-sweep":
        rows = off_body_sweep(args.model, args.wrap, args.body, args.joint)
        print(len(rows), "candidates")
        for r in rows[:12]:
            print(r)
    elif args.command == "record-jumps":
        record_jumps()
    elif args.command == "line-jumps":
        line_jumps(args.model, args.postures, args.out)
    elif args.command == "dump":
        run_dump(args.folder, args.label, tuple(args.models), args.postures, args.steps, args.arm_postures)
    elif args.command == "check":
        run_check(args.folder, args.a, args.b, tuple(args.models), args.limit_mm)


if __name__ == "__main__":
    main()
