"""Batched replica of MuJoCo's tendon wrapping (``mju_wrap`` in ``engine_util_misc.c``) in torch.

Ported from muscle-supermimic (github.com/kaibohex/muscle-supermimic, Apache-2.0, commit 8a6244b,
``src/muscle_supermimic/tasks/tracking/mdp/muscle_geometry.py``) and changed to follow the C engine
instead of MuJoCo-Warp, for both side-site rules:

* ``"mujoco3"``: a side site picks the tangent pair whose mid-direction points at it, and the wrap is
  skipped when the straight segment clears the circle on the side site's side (MuJoCo 3.x);
* ``"mujoco4"``: the path is homotopic to the polyline end0 -> side -> end1; the winding of that polyline
  around the centre picks the orientation and the arc angle (MuJoCo 4.x).

Every branch of the C code is evaluated for every element and selected with ``torch.where``, with fixed
iteration counts, so the functions work under ``torch.compile`` and CUDA-graph capture.
"""

from __future__ import annotations

import math

import torch

from ._math import MINVAL, cross, dot, mtv, mv, norm, normalize, safe_sqrt

SIDE_RULES = ("mujoco3", "mujoco4")


def _det2(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """a[0] b[1] - a[1] b[0] for (..., 2) vectors."""
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _winding(p: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
    """Signed angle swept from p to q around the origin, in (-pi, pi] (MuJoCo 4 ``winding_angle``)."""
    return torch.atan2(_det2(p, q), dot(p, q))


def _intersect(p1: torch.Tensor, p2: torch.Tensor, p3: torch.Tensor, p4: torch.Tensor, rule: str) -> torch.Tensor:
    """Whether the 2D segments p1-p2 and p3-p4 intersect (``is_intersect``)."""
    u, v = p2 - p1, p4 - p3
    det = v[..., 1] * u[..., 0] - v[..., 0] * u[..., 1]
    ok = det.abs() >= MINVAL
    if rule == "mujoco3":
        sdet = torch.where(ok, det, torch.ones_like(det))
        w = p1 - p3
        a = (v[..., 0] * w[..., 1] - v[..., 1] * w[..., 0]) / sdet
        b = (u[..., 0] * w[..., 1] - u[..., 1] * w[..., 0]) / sdet
        return ok & (a >= 0) & (a <= 1) & (b >= 0) & (b <= 1)
    s = torch.where(det > 0, torch.ones_like(det), -torch.ones_like(det))
    w, d = p1 - p3, p2 - p4
    a0 = (v[..., 0] * w[..., 1] - v[..., 1] * w[..., 0]) * s
    a1 = (v[..., 0] * d[..., 1] - v[..., 1] * d[..., 0]) * s
    b0 = (u[..., 0] * w[..., 1] - u[..., 1] * w[..., 0]) * s
    b1 = (u[..., 0] * d[..., 1] - u[..., 1] * d[..., 0]) * s
    return ok & (a0 >= 0) & (a1 < 0) & (b0 >= 0) & (b1 < 0)


def _length_circle(p0: torch.Tensor, p1: torch.Tensor, ind: torch.Tensor, r: torch.Tensor, rule: str) -> torch.Tensor:
    """Arc length from p0 to p1 on the circle (``length_circle``); ``ind`` is the chosen solution (0/1)."""
    c = p0[..., 1] * p1[..., 0] - p0[..., 0] * p1[..., 1]
    # atan2(|cross|, dot) equals MuJoCo 3's acos of the normalised dot product, with finite gradients
    ang = torch.atan2(c.abs(), dot(p0, p1))
    flip = ((c > 0) & ind) | ((c < 0) & ~ind)
    if rule == "mujoco4":
        flip = flip & (ang > 1e-6)
    return r * torch.where(flip, 2 * math.pi - ang, ang)


def wrap_circle(
    e0: torch.Tensor, e1: torch.Tensor, side: torch.Tensor, has_side: torch.Tensor, r: torch.Tensor, rule: str
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """2D circle wrap of the segment e0-e1 around a circle of radius r at the origin (``wrap_circle``).

    Args:
        e0, e1: segment end points, (..., 2).
        side: side point projected and scaled to the circle, (..., 2); ignored where ``has_side`` is False.
        has_side: whether the wrap has a side site, (...).
        r: radius, (...).
        rule: side-site rule, ``"mujoco3"`` or ``"mujoco4"``.

    Returns:
        Arc length (-1 where there is no wrap) and the two tangent points, (..., 2) each.
    """
    sq0, sq1, sqr = dot(e0, e0), dot(e1, e1), r * r
    dif = e1 - e0
    dd = dot(dif, dif)
    nowrap = (sq0 < sqr) | (sq1 < sqr) | (r < MINVAL) | (dd < MINVAL)
    a = (-dot(dif, e0) / dd.clamp(min=MINVAL)).clamp(0.0, 1.0)
    near = a.unsqueeze(-1) * dif + e0
    clear = dot(near, near) > sqr
    if rule == "mujoco3":
        nowrap = nowrap | (clear & (~has_side | (dot(side, near) >= 0)))
    else:
        wside = _winding(e0, side) + _winding(side, e1)
        nowrap = nowrap | (clear & torch.where(has_side, (wside - _winding(e0, e1)).abs() < math.pi, torch.ones_like(clear)))
        ccw = wside > 0
    s0, s1 = safe_sqrt(sq0 - sqr), safe_sqrt(sq1 - sqr)
    q0, q1 = sq0.clamp(min=MINVAL), sq1.clamp(min=MINVAL)

    def sol(sg: float) -> tuple[torch.Tensor, torch.Tensor]:
        t0 = torch.stack(
            ((e0[..., 0] * sqr + sg * r * e0[..., 1] * s0) / q0, (e0[..., 1] * sqr - sg * r * e0[..., 0] * s0) / q0), -1
        )
        t1 = torch.stack(
            ((e1[..., 0] * sqr - sg * r * e1[..., 1] * s1) / q1, (e1[..., 1] * sqr + sg * r * e1[..., 0] * s1) / q1), -1
        )
        return t0, t1

    t00, t01 = sol(1.0)
    t10, t11 = sol(-1.0)
    short0, short1 = -dot(t00 - t01, t00 - t01), -dot(t10 - t11, t10 - t11)
    if rule == "mujoco3":
        good0 = torch.where(has_side, dot(normalize(t00 + t01), side), short0)
        good1 = torch.where(has_side, dot(normalize(t10 + t11), side), short1)
    else:
        good0, good1 = short0, short1
    good0 = torch.where(_intersect(e0, t00, e1, t01, rule), torch.full_like(good0, -10000.0), good0)
    good1 = torch.where(_intersect(e0, t10, e1, t11, rule), torch.full_like(good1, -10000.0), good1)
    second = ~(good0 > good1)
    if rule == "mujoco4":
        second = torch.where(has_side, ccw, second)
    p0 = torch.where(second.unsqueeze(-1), t10, t00)
    p1 = torch.where(second.unsqueeze(-1), t11, t01)
    nowrap = nowrap | _intersect(e0, p0, e1, p1, rule)
    length = _length_circle(p0, p1, second, r, rule)
    if rule == "mujoco4":
        arc = wside - _winding(e0, p0) - _winding(p1, e1)
        length = torch.where(has_side, r * arc.abs(), length)
    return torch.where(nowrap, -torch.ones_like(length), length), p0, p1


def wrap_inside(e0: torch.Tensor, e1: torch.Tensor, r: torch.Tensor, rule: str) -> tuple[torch.Tensor, torch.Tensor]:
    """2D inside wrap (``wrap_inside``): the single tangent point when the side site is inside the geom.

    Newton's method on the tangent condition ``asin(A z) + asin(B z) - 2 asin(z) + G = 0``, with MuJoCo's 20
    iterations, tolerance and fallbacks (the mean direction of the end points). MuJoCo 3 iterates on z from
    1 - 1e-7, MuJoCo 4 on phi = asin(z) from pi/2. The iterations run without gradient; the solution gets
    its gradient from the implicit function theorem (one Newton correction with a detached slope, which
    leaves its value unchanged), so the tangent point and everything computed from it are differentiable.

    Returns:
        0 where wrapped and -1 where not, (...), and the tangent point, (..., 2).
    """
    maxiter, tol = 20, 1e-6

    def asin(x: torch.Tensor) -> torch.Tensor:
        return torch.asin(x.clamp(-1.0, 1.0))

    def rsq(x: torch.Tensor) -> torch.Tensor:
        return 1.0 / (1 - x).clamp(min=0).sqrt().clamp(min=MINVAL)

    def z_of(x: torch.Tensor) -> torch.Tensor:
        return x if rule == "mujoco3" else torch.sin(x)

    def fun(x: torch.Tensor, A: torch.Tensor, B: torch.Tensor, G: torch.Tensor) -> torch.Tensor:
        z = z_of(x)
        return asin(A * z) + asin(B * z) - 2 * (asin(z) if rule == "mujoco3" else x) + G

    def slope(x: torch.Tensor, A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
        z = z_of(x)
        sz = z * z
        if rule == "mujoco3":
            return A * rsq(sz * A * A) + B * rsq(sz * B * B) - 2 * rsq(sz)
        return torch.cos(x) * (A * rsq(sz * A * A) + B * rsq(sz * B * B)) - 2

    def params(e0: torch.Tensor, e1: torch.Tensor) -> tuple[torch.Tensor, ...]:
        len0, len1 = norm(e0), norm(e1)
        dif = e1 - e0
        dd = dot(dif, dif)
        A, B = r / len0.clamp(min=MINVAL), r / len1.clamp(min=MINVAL)
        cosG = (len0 * len0 + len1 * len1 - dd) / (2 * len0 * len1).clamp(min=MINVAL)
        return len0, len1, dif, dd, A, B, cosG

    with torch.no_grad():
        len0, len1, dif, dd, A, B, cosG = params(e0, e1)
        nowrap = (len0 <= r) | (len1 <= r) | (r < MINVAL) | (len0 < MINVAL) | (len1 < MINVAL)
        a = -dot(dif, e0) / dd.clamp(min=MINVAL)
        nowrap = nowrap | ((dd > MINVAL) & (a > 0) & (a < 1) & (norm(e0 + a.unsqueeze(-1) * dif) <= r))
        nowrap = nowrap | (cosG < -1 + MINVAL)
        fail = cosG > 1 - MINVAL  # wrap at the default point
        G = torch.acos(cosG.clamp(-1.0, 1.0))
        x = torch.full_like(A, 1 - 1e-7 if rule == "mujoco3" else math.pi / 2)
        f = fun(x, A, B, G)
        fail = fail | (f > 0)
        updated = torch.zeros_like(fail)
        for _ in range(maxiter):
            run = ~fail & ~nowrap & (f.abs() > tol)
            df = slope(x, A, B)
            bad = run & (df > -MINVAL)
            x1 = x - f / torch.where(df.abs() < MINVAL, torch.full_like(df, -MINVAL), df)
            bad = bad | (run & (x1 > x))
            upd = run & ~bad
            f1 = fun(x1, A, B, G)
            x, f = torch.where(upd, x1, x), torch.where(upd, f1, f)
            fail = fail | bad | (upd & (f1 > tol))
            updated = upd
        # MuJoCo falls back to the default point when all iterations ran (iter >= maxiter)
        fail = fail | updated
        solved = ~fail & ~nowrap

    # differentiable solution; inputs replaced by safe values where it is not used
    _, _, _, _, A, B, cosG = params(e0, e1)
    half = torch.full_like(A, 0.5)
    A, B = torch.where(solved, A, half), torch.where(solved, B, half)
    G = torch.acos(torch.where(solved, cosG, half).clamp(-1.0, 1.0))
    x = torch.where(solved, x, torch.full_like(x, 0.5 if rule == "mujoco3" else 1.0))
    f = fun(x, A, B, G)
    x = x - (f - f.detach()) / slope(x, A, B).detach().clamp(max=-MINVAL)
    z = z_of(x)
    pos = _det2(e0, e1) > 0
    vec = normalize(torch.where(pos.unsqueeze(-1), e0, e1))
    phi = asin(z) if rule == "mujoco3" else x
    ang = phi - asin(torch.where(pos, A, B) * z)
    c, s = torch.cos(ang), torch.sin(ang)
    rot = r.unsqueeze(-1) * torch.stack((c * vec[..., 0] - s * vec[..., 1], s * vec[..., 0] + c * vec[..., 1]), -1)
    default = normalize(0.5 * (e0 + e1)) * r.unsqueeze(-1)
    pnt = torch.where(solved.unsqueeze(-1), rot, default)
    return torch.where(nowrap, -torch.ones_like(cosG), torch.zeros_like(cosG)), pnt


def wrap(
    x0: torch.Tensor,
    x1: torch.Tensor,
    gpos: torch.Tensor,
    gmat: torch.Tensor,
    radius: torch.Tensor,
    cylinder: torch.Tensor,
    side: torch.Tensor,
    has_side: torch.Tensor,
    rule: str,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """``mju_wrap`` for a batch of site-geom-site triples.

    Args:
        x0, x1: world positions of the two sites, (..., 3).
        gpos, gmat: world position (..., 3) and rotation (..., 3, 3) of the wrap geom.
        radius: geom radius (``geom_size[0]``), (...).
        cylinder: True for cylinder wraps, False for sphere wraps, (...).
        side: world position of the side site, (..., 3); ignored where ``has_side`` is False.
        has_side: whether the wrap has a side site, (...).
        rule: side-site rule, ``"mujoco3"`` or ``"mujoco4"``.

    Returns:
        Wrap length (-1 where there is no wrap), the two tangent points in world coordinates (..., 3),
        and whether the wrap is an inside wrap (side site within the geom).
    """
    if rule not in SIDE_RULES:
        raise ValueError(f"rule must be one of {SIDE_RULES}, got {rule!r}")
    p0, p1, sp = mtv(gmat, x0 - gpos), mtv(gmat, x1 - gpos), mtv(gmat, side - gpos)
    cyl = cylinder.unsqueeze(-1)
    # 2D frame: sphere - plane of (0, p0, p1); cylinder - local xy
    ax0 = normalize(p0)
    normal = cross(p0, p1)
    par = norm(normal) < MINVAL
    # (p0, p1) parallel: normal from a second axis with a zero at the largest component of ax0
    aa = ax0.abs()
    i1 = (aa[..., 1] > aa[..., 0]) & (aa[..., 1] > aa[..., 2])
    i2 = (aa[..., 2] > aa[..., 0]) & (aa[..., 2] > aa[..., 1])
    one, zero = torch.ones_like(aa[..., 0]), torch.zeros_like(aa[..., 0])
    alt = torch.stack((torch.where(i1 | i2, one, zero), torch.where(i1, zero, one), torch.where(i2, zero, one)), -1)
    normal = normalize(torch.where(par.unsqueeze(-1), cross(ax0, alt), normal))
    ax1 = normalize(cross(normal, ax0))
    ex, ey = torch.zeros_like(p0), torch.zeros_like(p0)
    ex[..., 0], ey[..., 1] = 1.0, 1.0
    ax0, ax1 = torch.where(cyl, ex, ax0), torch.where(cyl, ey, ax1)
    e0 = torch.stack((dot(p0, ax0), dot(p0, ax1)), -1)
    e1 = torch.stack((dot(p1, ax0), dot(p1, ax1)), -1)
    s2 = normalize(torch.stack((dot(sp, ax0), dot(sp, ax1)), -1)) * radius.unsqueeze(-1)
    inside = has_side & (norm(sp) < radius)
    wl_c, c0, c1 = wrap_circle(e0, e1, s2, has_side, radius, rule)
    # the inside solve is ill-conditioned near z = 1 (asin), so it always runs in float64 (as MuJoCo's C code)
    wl_i, ci = wrap_inside(e0.double(), e1.double(), radius.double(), rule)
    wl_i, ci = wl_i.to(e0.dtype), ci.to(e0.dtype)
    wl = torch.where(inside, wl_i, wl_c)
    tooclose = (norm(p0) < MINVAL) | (norm(p1) < MINVAL)
    wl = torch.where(tooclose, -torch.ones_like(wl), wl)
    ins = inside.unsqueeze(-1)
    res0 = ax0 * torch.where(ins, ci, c0)[..., :1] + ax1 * torch.where(ins, ci, c0)[..., 1:]
    res1 = ax0 * torch.where(ins, ci, c1)[..., :1] + ax1 * torch.where(ins, ci, c1)[..., 1:]
    # cylinder: heights of the tangent points by arc length, then the helix length
    L0 = norm(p0[..., :2] - res0[..., :2])
    L1 = norm(p1[..., :2] - res1[..., :2])
    tot = (L0 + wl + L1).clamp(min=MINVAL)
    z0 = p0[..., 2] + (p1[..., 2] - p0[..., 2]) * L0 / tot
    z1 = p0[..., 2] + (p1[..., 2] - p0[..., 2]) * (L0 + wl) / tot
    c = cylinder
    res0 = torch.cat((res0[..., :2], torch.where(c, z0, res0[..., 2]).unsqueeze(-1)), -1)
    res1 = torch.cat((res1[..., :2], torch.where(c, z1, res1[..., 2]).unsqueeze(-1)), -1)
    wl = torch.where(c & (wl >= 0), safe_sqrt(wl * wl + (z1 - z0) ** 2), wl)
    w0 = mv(gmat, res0) + gpos
    w1 = mv(gmat, res1) + gpos
    return wl, w0, w1, inside
