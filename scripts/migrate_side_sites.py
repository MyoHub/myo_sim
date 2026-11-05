"""Side sites that give the same tendon path under the MuJoCo 3 and the MuJoCo 4 wrap rule.

MuJoCo 4 picks the side of a wrap by the winding of the polyline first endpoint -> side site ->
second endpoint; MuJoCo 3 used a half-plane test and the arc nearest the side site. This script
holds a NumPy copy of ``wrap_circle`` for both rules, so one process can evaluate both. The real
engines check it: ``dump`` runs under each MuJoCo, ``check`` compares the dumps.

    python scripts/migrate_side_sites.py search --out result.json   # candidate sites per side site
    python scripts/migrate_side_sites.py write                      # apply data/side_sites.json to the chains
    python scripts/migrate_side_sites.py dump out.npz               # under each MuJoCo: lengths, arms, jumps
    python scripts/migrate_side_sites.py check a.npz b.npz          # compare two dumps
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np

WRAP_SITE, WRAP_SPHERE, WRAP_CYLINDER = 3, 4, 5
MJMINVAL = 1e-15
# myofullbody holds the same wraps; its 350 wraps exceed the memory budget of a scan and the dumps cover it.
SEARCH_MODELS = ("myoarm_r", "myolegs", "myolegs_abdomen", "myotorso")
CHECK_MODELS = ("myoarm_r", "myoarms", "myofullbody", "myolegs", "myolegs26", "myolegs_abdomen", "myotorso", "myohands", "myohand_r")
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
        lo, hi = model.jnt_range[j, 0] - margin * (model.jnt_type[j] == 3), model.jnt_range[j, 1] + margin * (model.jnt_type[j] == 3)
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
    return WrapSamples(a0, a1, end0, end1, near, clear, mid, len3, len4, blocked3, blocked4, wind, valid)


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


def scan_model(name: str, n: int, steps: int) -> dict[str, list[dict]]:
    """Wraps per side site whose current site differs between the rules in any posture set."""
    import myo_sim

    model = myo_sim.load_model(name)
    wraps = model_wraps(model)
    side = side_local(model, wraps)
    ends = {t: forward_sites(model, tier_postures(model, t, n, steps), wraps)[0] for t in TIERS}
    groups: dict[str, list[dict]] = {}
    for k, w in enumerate(wraps):
        if not rule_decides(w, side[k]):
            continue
        entry = {"model": name, "wrap": w, "side": side[k], "tiers": {}, "flagged": False}
        for t, e in ends.items():
            ws = precompute(e[:, k, 0], e[:, k, 1], w.radius, w.is_sphere)
            ref = reference_lengths(ws, side[k])
            dev = deviation(ws, side[k][None], ref, 4)[0]
            entry["tiers"][t] = {"ws": ws, "ref": ref, "dev": dev}
            entry["flagged"] |= bool(dev.max() > LENGTH_TOL)
        groups.setdefault(w.side_name, []).append(entry)
    return {site: g for site, g in groups.items() if any(e["flagged"] for e in g)}


def evaluate_group(group: list[dict], model, site_name: str) -> dict:
    """Candidate directions around the first geom of a side site's group, scored under both rules."""
    import mujoco

    first = group[0]["wrap"]
    is_sphere = first.is_sphere
    dirs = direction_grid(is_sphere)
    site_id = model.site(site_name).id
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    body = model.site_bodyid[site_id]
    body_rot = data.xmat[body].reshape(3, 3)
    body_pos = data.xpos[body]
    rot0 = data.geom_xmat[first.geom].reshape(3, 3)
    pos0 = data.geom_xpos[first.geom]
    radius = max(e["wrap"].radius for e in group)
    # two radii out, in the plane of a cylinder
    world = pos0 + (dirs * 2.0 * first.radius) @ rot0.T
    in_body = (world - body_pos) @ body_rot
    n = len(dirs)
    v3, v4a, v4rest = np.zeros(n, int), np.zeros(n, int), np.zeros(n, int)
    worst3, worst4 = np.zeros(n), np.zeros(n)
    outside = np.ones(n, bool)
    for e in group:
        w = e["wrap"]
        rot = data.geom_xmat[w.geom].reshape(3, 3)
        loc = (world - data.geom_xpos[w.geom]) @ rot
        outside &= np.linalg.norm(loc, axis=1) > 1.2 * w.radius
        for t, tier in e["tiers"].items():
            d3 = deviation(tier["ws"], loc, tier["ref"], 3)
            d4 = deviation(tier["ws"], loc, tier["ref"], 4)
            v3 += (d3 > LENGTH_TOL).sum(1)
            worst3 = np.maximum(worst3, d3.max(1))
            worst4 = np.maximum(worst4, d4.max(1))
            if t == "A":
                v4a += (d4 > LENGTH_TOL).sum(1)
            else:
                v4rest += (d4 > LENGTH_TOL).sum(1)
    feasible = outside & (v3 == 0) & (v4a == 0) & (v4rest == 0)
    tier_a_only = outside & (v3 == 0) & (v4a == 0)
    return {
        "dirs": dirs,
        "in_body": in_body,
        "v3": v3,
        "v4a": v4a,
        "v4rest": v4rest,
        "worst3": worst3,
        "worst4": worst4,
        "feasible": feasible,
        "tier_a_only": tier_a_only,
        "radius": radius,
    }


def widest_margin(dirs: np.ndarray, good: np.ndarray) -> int:
    """Index of the good direction farthest from any direction that is not good."""
    bad = dirs[~good]
    idx = np.where(good)[0]
    if len(bad) == 0:
        return int(idx[0])
    return int(idx[np.argmin((dirs[idx] @ bad.T).max(1))])


def group_summary(group: list[dict]) -> dict:
    """Largest difference and share of postures between the rules at the current site, per tier."""
    out = {}
    for t in TIERS:
        devs = np.concatenate([e["tiers"][t]["dev"] for e in group])
        out[t] = {"max_mm": round(float(devs.max() * 1e3), 3), "share_percent": round(float((devs > 1e-3).mean() * 100), 3)}
    return out


def run_search(n: int, steps: int, out_path: str, models: tuple[str, ...]) -> None:
    """Search every flagged side site and write the result as JSON."""
    import json
    from pathlib import Path

    import myo_sim

    result: dict[str, dict] = {}
    for name in models:
        groups = scan_model(name, n, steps)
        model = myo_sim.load_model(name)
        print(f"== {name}: {len(groups)} side sites differ", flush=True)
        for site, group in groups.items():
            ev = evaluate_group(group, model, site)
            summary = group_summary(group)
            entry = {
                "model": name,
                "body": model.body(model.site_bodyid[model.site(site).id]).name,
                "site_pos": [round(float(v), 6) for v in model.site_pos[model.site(site).id]],
                "users": sorted({f"{e['wrap'].tendon_name}/{e['wrap'].geom_name}" for e in group}),
                "difference_now": summary,
                "candidates": len(ev["dirs"]),
                "feasible_all_tiers": int(ev["feasible"].sum()),
                "feasible_tier_a": int(ev["tier_a_only"].sum()),
                "fewest_3x_violations": int(ev["v3"].min()),
            }
            # a site with no in-range difference stays unless a candidate also clears the other posture sets
            good = ev["feasible"] if ev["feasible"].any() or summary["A"]["max_mm"] == 0.0 else ev["tier_a_only"]
            if good.any():
                i = widest_margin(ev["dirs"], good)
                entry["pos"] = [round(float(v), 6) for v in ev["in_body"][i]]
                entry["left_v4_outside_tier_a"] = int(ev["v4rest"][i])
                entry["margin_deg"] = round(
                    float(np.degrees(np.arccos(np.clip((ev["dirs"][i] @ ev["dirs"][~good].T).max(), -1, 1)))), 1
                ) if (~good).any() else 180.0
            else:
                keep = ev["v3"] == 0
                if keep.any():
                    i = int(np.where(keep)[0][np.argmin(ev["v4a"][keep] + ev["v4rest"][keep])])
                else:
                    i = int(np.argmin(ev["v3"] * 1000 + ev["v4a"]))
                entry["best"] = {
                    "pos": [round(float(v), 6) for v in ev["in_body"][i]],
                    "v3": int(ev["v3"][i]),
                    "v4_tier_a": int(ev["v4a"][i]),
                    "v4_other": int(ev["v4rest"][i]),
                    "worst3_mm": round(float(ev["worst3"][i] * 1e3), 3),
                    "worst4_mm": round(float(ev["worst4"][i] * 1e3), 3),
                }
            key = site
            if key in result:
                result[key]["also_in"] = result[key].get("also_in", []) + [name]
                for t, v in summary.items():
                    if v["max_mm"] > result[key]["difference_now"][t]["max_mm"]:
                        result[key]["difference_now"][t] = v
                if "pos" in result[key] and "pos" not in entry:
                    result[key]["conflict"] = name
                continue
            result[key] = entry
            status = "MOVE" if "pos" in entry else "KEEP"
            print(
                f"{status} {site:46s} {summary['A']['max_mm']:8.2f} A {summary['B']['max_mm']:8.2f} B "
                f"{summary['C']['max_mm']:8.2f} C | feasible {entry['feasible_all_tiers']}/{entry['feasible_tier_a']} "
                f"of {entry['candidates']}",
                flush=True,
            )
        Path(out_path).write_text(json.dumps(result, indent=1))


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


def off_body_search(name: str, wrap_key: str, n: int, steps: int, distances: tuple[float, ...], n_dirs: int) -> list[dict]:
    """Side site on another body of the wrap's chain: violations under both rules in every posture set."""
    import mujoco
    import myo_sim

    model = myo_sim.load_model(name)
    wraps = model_wraps(model)
    side = side_local(model, wraps)
    k = next(i for i, w in enumerate(wraps) if f"{w.tendon_name}/{w.geom_name}" == wrap_key)
    w = wraps[k]
    geom_body = model.geom_bodyid[w.geom]
    chain = chain_between(model, model.site_bodyid[w.site0], model.site_bodyid[w.site1])
    bodies = [b for b in chain if b not in (geom_body, 0)]
    data = mujoco.MjData(model)
    sets = {}
    for tier in TIERS:
        qs = tier_postures(model, tier, n, steps)
        ends = np.empty((len(qs), 2, 3))
        rel = {b: (np.empty((len(qs), 3, 3)), np.empty((len(qs), 3))) for b in bodies}
        for i, q in enumerate(qs):
            data.qpos[:] = q
            mujoco.mj_kinematics(model, data)
            rot, pos = data.geom_xmat[w.geom].reshape(3, 3), data.geom_xpos[w.geom]
            ends[i, 0] = rot.T @ (data.site_xpos[w.site0] - pos)
            ends[i, 1] = rot.T @ (data.site_xpos[w.site1] - pos)
            for b in bodies:
                rel[b][0][i] = rot.T @ data.xmat[b].reshape(3, 3)
                rel[b][1][i] = rot.T @ (data.xpos[b] - pos)
        ws = precompute(ends[:, 0], ends[:, 1], w.radius, w.is_sphere)
        sets[tier] = (ws, reference_lengths(ws, side[k]), rel)
    data.qpos[:] = model.qpos0
    mujoco.mj_kinematics(model, data)
    rot0, pos0 = data.geom_xmat[w.geom].reshape(3, 3), data.geom_xpos[w.geom]
    dirs = direction_grid(True, n_sphere=n_dirs)
    out = []
    for b in bodies:
        rot_b, pos_b = data.xmat[b].reshape(3, 3), data.xpos[b]
        for dist in distances:
            points = ((pos0 + (dirs * w.radius * dist) @ rot0.T) - pos_b) @ rot_b
            v3, v4 = {t: np.zeros(n_dirs, int) for t in TIERS}, {t: np.zeros(n_dirs, int) for t in TIERS}
            outside = np.ones(n_dirs, bool)
            for tier, (ws, ref, rel) in sets.items():
                idx = np.arange(len(ref))
                for c0 in range(0, n_dirs, 64):
                    loc = np.einsum("nij,cj->cni", rel[b][0], points[c0 : c0 + 64]) + rel[b][1][None]
                    outside[c0 : c0 + 64] &= np.linalg.norm(loc, axis=-1).min(1) > 1.2 * w.radius
                    v3[tier][c0 : c0 + 64] += (np.abs(ws.state_len3[idx, states(ws, loc, 3)] - ref) > LENGTH_TOL).sum(1)
                    v4[tier][c0 : c0 + 64] += (np.abs(ws.state_len4[idx, states(ws, loc, 4)] - ref) > LENGTH_TOL).sum(1)
            in_range = sum(v3[t] + v4[t] for t in ("A", "B"))
            past = v3["C"] + v4["C"]
            good = outside & (in_range == 0)
            if good.any():
                good = good & (past == past[good].min())
                i = widest_margin(dirs, good)
            else:
                i = int(np.argmin(np.where(outside, in_range, 10**9)))
            out.append(
                {
                    "wrap": wrap_key,
                    "body": model.body(b).name,
                    "distance_radii": dist,
                    "feasible": int(good.sum()),
                    "pos": [round(float(x), 6) for x in points[i]],
                    "violations_3x": {t: int(v[i]) for t, v in v3.items()},
                    "violations_4": {t: int(v[i]) for t, v in v4.items()},
                    "postures": {t: len(sets[t][1]) for t in TIERS},
                }
            )
    return out


def off_body_sweep(name: str, wrap_key: str, body: str, joint: str, extra: float = 0.6) -> list[dict]:
    """Side sites on *body* for a wrap whose relative motion is one joint: equal to MuJoCo 3 in range under both rules.

    Candidates are ranked by how far past the limits the two rules still agree with each other.
    """
    import mujoco
    import myo_sim

    model = myo_sim.load_model(name)
    wraps = model_wraps(model)
    side = side_local(model, wraps)
    k = next(i for i, w in enumerate(wraps) if f"{w.tendon_name}/{w.geom_name}" == wrap_key)
    w = wraps[k]
    b = model.body(body).id
    j = model.joint(joint)
    lo, hi = j.range
    grid = np.arange(lo - extra, hi + extra, 1e-3)
    data = mujoco.MjData(model)
    ends = np.empty((len(grid), 2, 3))
    rot_rel, pos_rel = np.empty((len(grid), 3, 3)), np.empty((len(grid), 3))
    for i, x in enumerate(grid):
        data.qpos[:] = model.qpos0
        data.qpos[j.qposadr[0]] = x
        mujoco.mj_kinematics(model, data)
        rot, pos = data.geom_xmat[w.geom].reshape(3, 3), data.geom_xpos[w.geom]
        ends[i, 0] = rot.T @ (data.site_xpos[w.site0] - pos)
        ends[i, 1] = rot.T @ (data.site_xpos[w.site1] - pos)
        rot_rel[i] = rot.T @ data.xmat[b].reshape(3, 3)
        pos_rel[i] = rot.T @ (data.xpos[b] - pos)
    ws = precompute(ends[:, 0], ends[:, 1], w.radius, w.is_sphere)
    ref = reference_lengths(ws, side[k])
    data.qpos[:] = model.qpos0
    mujoco.mj_kinematics(model, data)
    rot0, pos0 = data.geom_xmat[w.geom].reshape(3, 3), data.geom_xpos[w.geom]
    rot_b, pos_b = data.xmat[b].reshape(3, 3), data.xpos[b]
    angles = np.radians(np.arange(0.0, 360.0, 1.0))
    i0, i1 = np.searchsorted(grid, lo), np.searchsorted(grid, hi)
    idx = np.arange(len(grid))
    rows = []
    for rho in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 6.0, 8.0, 12.0):
        for z in (0.0, 1.5, 2.0, 3.0, 4.0, -1.5, -2.0, -3.0, -4.0):
            plane = np.stack([rho * np.cos(angles), rho * np.sin(angles), z + 0 * angles], 1)
            points = ((pos0 + (plane * w.radius) @ rot0.T) - pos_b) @ rot_b
            loc = np.einsum("nij,cj->cni", rot_rel, points) + pos_rel[None]
            outside = np.linalg.norm(loc, axis=-1).min(1) > 1.5 * w.radius
            len3 = ws.state_len3[idx, states(ws, loc, 3)]
            len4 = ws.state_len4[idx, states(ws, loc, 4)]
            agree = np.abs(len3 - len4) < LENGTH_TOL
            exact = (np.abs(len3 - ref) < LENGTH_TOL) & agree
            for c in np.flatnonzero(outside & exact[:, i0 : i1 + 1].all(1)):
                a, e = i0, i1
                while a > 0 and agree[c, a - 1]:
                    a -= 1
                while e < len(grid) - 1 and agree[c, e + 1]:
                    e += 1
                rows.append(
                    {
                        "pos": [round(float(v), 5) for v in points[c]],
                        "rho_radii": rho,
                        "z_radii": z,
                        "angle_deg": float(np.degrees(angles[c])),
                        "agree_past_lower": round(float(lo - grid[a]), 3),
                        "agree_past_upper": round(float(grid[e] - hi), 3),
                    }
                )
    rows.sort(key=lambda r: -min(r["agree_past_lower"], r["agree_past_upper"]))
    return rows


def write_chain() -> None:
    """Apply ``data/side_sites.json`` to the right arm chain: position, and the parent body where it differs."""
    import json
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    table = json.loads((root / "myo_sim/build/data/side_sites.json").read_text())
    chain = root / "myo_sim/models/arm/assets/myoarm_r_chain.xml"
    lines = chain.read_text().split("\n")
    for site in table["sites"]:
        k = next(i for i, line in enumerate(lines) if f'<site name="{site["name"]}"' in line)
        line = re.sub(r'pos="[^"]*"', 'pos="' + " ".join(f"{v:g}" for v in site["pos"]) + '"', lines[k])
        del lines[k]
        anchor = next(i for i, x in enumerate(lines) if f'<body name="{site["body"]}"' in x)
        indent = re.match(r"\s*", lines[anchor]).group(0) + "    "
        # after the anchor body's last direct site, or its opening tag and inertial
        end = anchor + 1
        for i in range(anchor + 1, len(lines)):
            stripped = lines[i].strip()
            if stripped.startswith("<body") or stripped.startswith("</body"):
                break
            if stripped.startswith("<site"):
                end = i + 1
        lines.insert(end, indent + line.strip())
    chain.write_text("\n".join(lines))


def alternatives(name: str, n: int, sweep_steps: int, out_path: str) -> None:
    """For side sites with no site that keeps MuJoCo 3's path under both rules: the two sites worth comparing.

    ``unified``: same path on both versions at every in-range posture, with the fewest jumps along the sweeps and then
    the smallest change from MuJoCo 3. ``four_only``: under the MuJoCo 4 rule the closest to MuJoCo 3's path.
    """
    import json
    from pathlib import Path

    import myo_sim

    model = myo_sim.load_model(name)
    wraps = model_wraps(model)
    side = side_local(model, wraps)
    random_q = sample_postures(model, n, 21, True)
    sweep_q, ids = sweep_postures(model, sweep_steps, 22, True, bases=4)
    qs = np.vstack([random_q, sweep_q])
    ends, _ = forward_sites(model, qs, wraps)
    same_sweep = ids[1:] == ids[:-1]
    start = len(random_q)
    result = {}
    for k, w in enumerate(wraps):
        if not rule_decides(w, side[k]):
            continue
        ws = precompute(ends[:, k, 0], ends[:, k, 1], w.radius, w.is_sphere)
        ref = reference_lengths(ws, side[k])
        dev_now = deviation(ws, side[k][None], ref, 4)[0]
        if dev_now.max() < LENGTH_TOL:
            continue
        dirs = direction_grid(w.is_sphere, n_sphere=2000)
        idx = np.arange(len(ref))
        d3, d4, disagree, jumps = [np.empty((len(dirs), len(ref)), np.float32) for _ in range(2)] + [np.zeros(len(dirs), int), np.zeros(len(dirs), int)]
        worst_jump = np.zeros(len(dirs))
        for c0 in range(0, len(dirs), 128):
            c = dirs[c0 : c0 + 128]
            l3 = ws.state_len3[idx, states(ws, c, 3)]
            l4 = ws.state_len4[idx, states(ws, c, 4)]
            d3[c0 : c0 + 128] = np.abs(l3 - ref)
            d4[c0 : c0 + 128] = np.abs(l4 - ref)
            disagree[c0 : c0 + 128] = (np.abs(l3 - l4) > LENGTH_TOL).sum(1)
            steps = np.abs(np.diff(l4[:, start:], axis=1))[:, same_sweep]
            jumps[c0 : c0 + 128] = (steps > 2e-3).sum(1)
            worst_jump[c0 : c0 + 128] = steps.max(1)
        ref_steps = np.abs(np.diff(ref[start:]))[same_sweep]
        now_steps = np.abs(np.diff(ws.state_len4[idx, states(ws, side[k][None], 4)[0]][start:]))[same_sweep]
        order = np.lexsort(((d3 > LENGTH_TOL).sum(1), jumps, disagree))
        u = int(order[0])
        f = int(np.argmin((d4 > LENGTH_TOL).sum(1)))
        data = {
            "users": f"{w.tendon_name}/{w.geom_name}",
            "side": w.side_name,
            "postures": len(ref),
            "now": {
                "rules_differ_share_percent": round(float((dev_now > 1e-3).mean() * 100), 2),
                "rules_differ_max_mm": round(float(dev_now.max() * 1e3), 2),
                "mj3_jump_max_mm": round(float(ref_steps.max() * 1e3), 2),
                "mj3_jump_steps": int((ref_steps > 2e-3).sum()),
                "mj4_jump_max_mm": round(float(now_steps.max() * 1e3), 2),
                "mj4_jump_steps": int((now_steps > 2e-3).sum()),
            },
            "unified": {
                "direction_geom_frame": [round(float(v), 4) for v in dirs[u]],
                "rules_disagree_postures": int(disagree[u]),
                "jump_steps": int(jumps[u]),
                "jump_max_mm": round(float(worst_jump[u] * 1e3), 2),
                "changes_mj3_share_percent": round(float((d3[u] > 1e-3).mean() * 100), 2),
                "changes_mj3_max_mm": round(float(d3[u].max() * 1e3), 2),
            },
            "four_only": {
                "direction_geom_frame": [round(float(v), 4) for v in dirs[f]],
                "differs_from_mj3_share_percent": round(float((d4[f] > 1e-3).mean() * 100), 2),
                "differs_from_mj3_max_mm": round(float(d4[f].max() * 1e3), 2),
            },
        }
        result[data["users"]] = data
        print(json.dumps(data), flush=True)
        Path(out_path).write_text(json.dumps(result, indent=1))


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
        result: dict[str, np.ndarray] = {"names": np.array([model.tendon(t).name for t in range(model.ntendon)]), "spatial": spatial}
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
        print(f"  tendons over {limit_mm} mm: {len(rows)}; moment arms: max {arms.max() * 1e3:.3f} mm, tendons over 0.1 mm {int((arms > 1e-4).sum())}")
        for tendon, text in sorted(rows.items()):
            t = np.flatnonzero(names == tendon)[0]
            print(f"  {tendon:30s} {'; '.join(text)} | arm {arms[t] * 1e3:.2f} mm")


def main() -> None:
    """Command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    search = sub.add_parser("search")
    search.add_argument("--out", required=True)
    search.add_argument("--postures", type=int, default=12000)
    search.add_argument("--steps", type=int, default=101)
    search.add_argument("--models", nargs="*", default=list(SEARCH_MODELS))
    off = sub.add_parser("off-body")
    off.add_argument("--model", default="myoarm_r")
    off.add_argument("--wrap", nargs="+", required=True, help="tendon/geom")
    off.add_argument("--postures", type=int, default=6000)
    off.add_argument("--steps", type=int, default=41)
    off.add_argument("--dirs", type=int, default=1500)
    sweep = sub.add_parser("off-body-sweep")
    sweep.add_argument("--model", default="myoarm_r")
    sweep.add_argument("--wrap", required=True)
    sweep.add_argument("--body", required=True)
    sweep.add_argument("--joint", required=True)
    sub.add_parser("write")
    lj = sub.add_parser("line-jumps")
    lj.add_argument("--model", default="myoarm_r")
    lj.add_argument("--out", required=True)
    lj.add_argument("--postures", type=int, default=20000)
    alt = sub.add_parser("alternatives")
    alt.add_argument("--model", default="myoarm_r")
    alt.add_argument("--out", required=True)
    alt.add_argument("--postures", type=int, default=8000)
    alt.add_argument("--sweep-steps", type=int, default=501)
    dump = sub.add_parser("dump")
    dump.add_argument("folder")
    dump.add_argument("label")
    dump.add_argument("--models", nargs="*", default=list(CHECK_MODELS))
    dump.add_argument("--postures", type=int, default=12000)
    dump.add_argument("--steps", type=int, default=101)
    dump.add_argument("--arm-postures", type=int, default=800)
    check = sub.add_parser("check")
    check.add_argument("folder")
    check.add_argument("a")
    check.add_argument("b")
    check.add_argument("--models", nargs="*", default=list(CHECK_MODELS))
    check.add_argument("--limit-mm", type=float, default=1.0)
    args = parser.parse_args()
    if args.command == "write":
        write_chain()
    if args.command == "line-jumps":
        line_jumps(args.model, args.postures, args.out)
    if args.command == "alternatives":
        alternatives(args.model, args.postures, args.sweep_steps, args.out)
    if args.command == "dump":
        run_dump(args.folder, args.label, tuple(args.models), args.postures, args.steps, args.arm_postures)
    if args.command == "check":
        run_check(args.folder, args.a, args.b, tuple(args.models), args.limit_mm)
    if args.command == "off-body-sweep":
        rows = off_body_sweep(args.model, args.wrap, args.body, args.joint)
        print(len(rows), "candidates")
        for r in rows[:12]:
            print(r)
    if args.command == "search":
        run_search(args.postures, args.steps, args.out, tuple(args.models))
    elif args.command == "off-body":
        for key in args.wrap:
            rows = off_body_search(args.model, key, args.postures, args.steps, (1.5, 2.0, 3.0, 6.0, 12.0, 25.0, 60.0), args.dirs)
            rows.sort(key=lambda r: (-r["feasible"], sum(r["violations_3x"].values()) + sum(r["violations_4"].values())))
            for r in rows[:4]:
                print(r, flush=True)


if __name__ == "__main__":
    main()
