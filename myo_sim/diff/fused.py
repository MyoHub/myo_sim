"""Fused tendon lengths and Jacobians (moment arms) of spatial tendons in one Triton kernel.

The same computation as :class:`myo_sim.diff.tendon.Tendons` (``mj_tendon`` with ``mju_wrap``, both side-site
rules), written per element: one program per tendon and block of states walks the tendon's path segments,
computes each site-site segment and site-geom-site wrap (the inside wrap in float64, as MuJoCo's C code), and
accumulates the length and the Jacobian row in registers. A tendon's Jacobian entries are ``u . v0[k] +
omega[k] . (x_end x u)`` per straight piece and DOF ``k`` that moves exactly one of its end points (tangent points
fixed to the wrap geom), signed by the end and scaled by the piece's ``1/divisor`` and, optionally, a per-state
factor (a joint coupling's slope). Replaces some hundred small torch kernels by one; CUDA-graph capturable,
no host synchronisation.

The kernel itself has no gradient: :func:`tendon_length_jacobian` gives the values, and
:class:`FusedTendonFunction` adds the exact gradient by recomputing the torch replica in the backward pass.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import torch
import triton
import triton.language as tl
from triton.language.extra import libdevice

MINVAL = 1e-15  # mjMINVAL

if os.environ.get("TRITON_INTERPRET") == "1":  # CPU interpreter (tests): libdevice's math functions from numpy
    from triton.runtime.interpreter import TensorHandle

    def _np_fn(f):
        def g(*args, **_):
            t = next(a for a in args if isinstance(a, tl.tensor))
            h = [a.handle.data if isinstance(a, tl.tensor) else a for a in args]
            return tl.tensor(TensorHandle(f(*h).astype(h[0].dtype), t.dtype.scalar), t.type)

        return g

    libdevice.asin, libdevice.acos, libdevice.atan2 = _np_fn(np.arcsin), _np_fn(np.arccos), _np_fn(np.arctan2)


# ---------------------------------------------------------------------------------------------- helpers
@triton.jit
def _sqrt_safe(x):
    return tl.sqrt(tl.maximum(x, 1e-30))


@triton.jit
def _norm2(x, y):
    return _sqrt_safe(x * x + y * y)


@triton.jit
def _norm3(x, y, z):
    return _sqrt_safe(x * x + y * y + z * z)


@triton.jit
def _normalize2(x, y):
    """mju_normalize in 2D: the first basis vector below mjMINVAL."""
    n = _norm2(x, y)
    small = n < 1e-15
    d = tl.maximum(n, 1e-15)
    return tl.where(small, 1.0, x / d), tl.where(small, 0.0, y / d)


@triton.jit
def _normalize3(x, y, z):
    n = _norm3(x, y, z)
    small = n < 1e-15
    d = tl.maximum(n, 1e-15)
    return tl.where(small, 1.0, x / d), tl.where(small, 0.0, y / d), tl.where(small, 0.0, z / d)


@triton.jit
def _cross(ax, ay, az, bx, by, bz):
    return ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx


@triton.jit
def _winding(px, py, qx, qy):
    """Signed angle from p to q (MuJoCo 4 winding_angle)."""
    return libdevice.atan2(px * qy - py * qx, px * qx + py * qy)


@triton.jit
def _intersect(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, RULE4: tl.constexpr):
    """Whether the 2D segments p1-p2 and p3-p4 intersect (is_intersect)."""
    ux, uy = p2x - p1x, p2y - p1y
    vx, vy = p4x - p3x, p4y - p3y
    det = vy * ux - vx * uy
    ok = tl.abs(det) >= 1e-15
    wx, wy = p1x - p3x, p1y - p3y
    if RULE4:
        s = tl.where(det > 0, 1.0, -1.0)
        dx, dy = p2x - p4x, p2y - p4y
        a0 = (vx * wy - vy * wx) * s
        a1 = (vx * dy - vy * dx) * s
        b0 = (ux * wy - uy * wx) * s
        b1 = (ux * dy - uy * dx) * s
        res = ok & (a0 >= 0) & (a1 < 0) & (b0 >= 0) & (b1 < 0)
    else:
        sdet = tl.where(ok, det, 1.0)
        a = (vx * wy - vy * wx) / sdet
        b = (ux * wy - uy * wx) / sdet
        res = ok & (a >= 0) & (a <= 1) & (b >= 0) & (b <= 1)
    return res


@triton.jit
def _wrap_circle(e0x, e0y, e1x, e1y, sx, sy, has_side, r, RULE4: tl.constexpr):
    """wrap_circle: arc length (-1: no wrap) and the two tangent points."""
    sq0 = e0x * e0x + e0y * e0y
    sq1 = e1x * e1x + e1y * e1y
    sqr = r * r
    dx, dy = e1x - e0x, e1y - e0y
    dd = dx * dx + dy * dy
    nowrap = (sq0 < sqr) | (sq1 < sqr) | (r < 1e-15) | (dd < 1e-15)
    a = -(dx * e0x + dy * e0y) / tl.maximum(dd, 1e-15)
    a = tl.minimum(tl.maximum(a, 0.0), 1.0)
    nx, ny = a * dx + e0x, a * dy + e0y
    clear = nx * nx + ny * ny > sqr
    wside = sq0 * 0.0
    if RULE4:
        wside = _winding(e0x, e0y, sx, sy) + _winding(sx, sy, e1x, e1y)
        wdir = _winding(e0x, e0y, e1x, e1y)
        nowrap = nowrap | (clear & ((~has_side) | (tl.abs(wside - wdir) < 3.141592653589793)))
    else:
        nowrap = nowrap | (clear & ((~has_side) | (sx * nx + sy * ny >= 0)))
    s0 = _sqrt_safe(sq0 - sqr)
    s1 = _sqrt_safe(sq1 - sqr)
    q0 = tl.maximum(sq0, 1e-15)
    q1 = tl.maximum(sq1, 1e-15)
    # solution +1
    t00x = (e0x * sqr + r * e0y * s0) / q0
    t00y = (e0y * sqr - r * e0x * s0) / q0
    t01x = (e1x * sqr - r * e1y * s1) / q1
    t01y = (e1y * sqr + r * e1x * s1) / q1
    # solution -1
    t10x = (e0x * sqr - r * e0y * s0) / q0
    t10y = (e0y * sqr + r * e0x * s0) / q0
    t11x = (e1x * sqr + r * e1y * s1) / q1
    t11y = (e1y * sqr - r * e1x * s1) / q1
    g0 = -((t00x - t01x) * (t00x - t01x) + (t00y - t01y) * (t00y - t01y))
    g1 = -((t10x - t11x) * (t10x - t11x) + (t10y - t11y) * (t10y - t11y))
    if not RULE4:
        m0x, m0y = _normalize2(t00x + t01x, t00y + t01y)
        m1x, m1y = _normalize2(t10x + t11x, t10y + t11y)
        g0 = tl.where(has_side, m0x * sx + m0y * sy, g0)
        g1 = tl.where(has_side, m1x * sx + m1y * sy, g1)
    g0 = tl.where(_intersect(e0x, e0y, t00x, t00y, e1x, e1y, t01x, t01y, RULE4), -10000.0, g0)
    g1 = tl.where(_intersect(e0x, e0y, t10x, t10y, e1x, e1y, t11x, t11y, RULE4), -10000.0, g1)
    second = ~(g0 > g1)
    if RULE4:
        second = tl.where(has_side, wside > 0, second)
    p0x = tl.where(second, t10x, t00x)
    p0y = tl.where(second, t10y, t00y)
    p1x = tl.where(second, t11x, t01x)
    p1y = tl.where(second, t11y, t01y)
    nowrap = nowrap | _intersect(e0x, e0y, p0x, p0y, e1x, e1y, p1x, p1y, RULE4)
    # length_circle
    c = p0y * p1x - p0x * p1y
    ang = libdevice.atan2(tl.abs(c), p0x * p1x + p0y * p1y)
    flip = ((c > 0) & second) | ((c < 0) & (~second))
    if RULE4:
        flip = flip & (ang > 1e-6)
    length = r * tl.where(flip, 6.283185307179586 - ang, ang)
    if RULE4:
        arc = wside - _winding(e0x, e0y, p0x, p0y) - _winding(p1x, p1y, e1x, e1y)
        length = tl.where(has_side, r * tl.abs(arc), length)
    return tl.where(nowrap, -1.0, length), p0x, p0y, p1x, p1y


@triton.jit
def _asinc(x):
    return libdevice.asin(tl.minimum(tl.maximum(x, -1.0), 1.0))


@triton.jit
def _rsq(x):
    return 1.0 / tl.maximum(tl.sqrt(tl.maximum(1.0 - x, 0.0)), 1e-15)


@triton.jit
def _inside_fun(x, A, B, G, RULE4: tl.constexpr):
    if RULE4:
        z = tl.sin(x)
        f = _asinc(A * z) + _asinc(B * z) - 2 * x + G
    else:
        f = _asinc(A * x) + _asinc(B * x) - 2 * _asinc(x) + G
    return f


@triton.jit
def _inside_slope(x, A, B, RULE4: tl.constexpr):
    if RULE4:
        z = tl.sin(x)
        sz = z * z
        df = tl.cos(x) * (A * _rsq(sz * A * A) + B * _rsq(sz * B * B)) - 2
    else:
        sz = x * x
        df = A * _rsq(sz * A * A) + B * _rsq(sz * B * B) - 2 * _rsq(sz)
    return df


@triton.jit
def _wrap_inside(e0x, e0y, e1x, e1y, r, RULE4: tl.constexpr):
    """wrap_inside in the inputs' dtype (float64): 0 (wrapped) or -1, and the tangent point."""
    len0 = _norm2(e0x, e0y)
    len1 = _norm2(e1x, e1y)
    dx, dy = e1x - e0x, e1y - e0y
    dd = dx * dx + dy * dy
    A = r / tl.maximum(len0, 1e-15)
    B = r / tl.maximum(len1, 1e-15)
    cosG = (len0 * len0 + len1 * len1 - dd) / tl.maximum(2 * len0 * len1, 1e-15)
    nowrap = (len0 <= r) | (len1 <= r) | (r < 1e-15) | (len0 < 1e-15) | (len1 < 1e-15)
    a = -(dx * e0x + dy * e0y) / tl.maximum(dd, 1e-15)
    nowrap = nowrap | ((dd > 1e-15) & (a > 0) & (a < 1) & (_norm2(e0x + a * dx, e0y + a * dy) <= r))
    nowrap = nowrap | (cosG < -1 + 1e-15)
    fail = cosG > 1 - 1e-15
    G = libdevice.acos(tl.minimum(tl.maximum(cosG, -1.0), 1.0))
    if RULE4:
        x = A * 0.0 + 1.5707963267948966
    else:
        x = A * 0.0 + (1 - 1e-7)
    f = _inside_fun(x, A, B, G, RULE4)
    fail = fail | (f > 0)
    updated = fail & False
    # MuJoCo's 20 iterations, ended early once no element iterates (later iterations would change nothing)
    it = 0
    run = (~fail) & (~nowrap) & (tl.abs(f) > 1e-6)
    while (it < 20) & (tl.max(run.to(tl.int32), axis=0) > 0):
        df = _inside_slope(x, A, B, RULE4)
        bad = run & (df > -1e-15)
        x1 = x - f / tl.where(tl.abs(df) < 1e-15, -1e-15, df)
        bad = bad | (run & (x1 > x))
        upd = run & (~bad)
        f1 = _inside_fun(x1, A, B, G, RULE4)
        x = tl.where(upd, x1, x)
        f = tl.where(upd, f1, f)
        fail = fail | bad | (upd & (f1 > 1e-6))
        updated = upd
        it += 1
        run = (~fail) & (~nowrap) & (tl.abs(f) > 1e-6)
    # MuJoCo falls back to the default point when the last of all iterations still updated
    fail = fail | (updated & (it >= 20))
    solved = (~fail) & (~nowrap)
    if RULE4:
        z = tl.sin(x)
        phi = x
    else:
        z = x
        phi = _asinc(x)
    pos = e0x * e1y - e0y * e1x > 0
    vx, vy = _normalize2(tl.where(pos, e0x, e1x), tl.where(pos, e0y, e1y))
    ang = phi - _asinc(tl.where(pos, A, B) * z)
    c, s = tl.cos(ang), tl.sin(ang)
    hx, hy = _normalize2(0.5 * (e0x + e1x), 0.5 * (e0y + e1y))
    px = tl.where(solved, r * (c * vx - s * vy), hx * r)
    py = tl.where(solved, r * (s * vx + c * vy), hy * r)
    return tl.where(nowrap, -1.0, 0.0 * len0), px, py


@triton.jit
def _wrap(x0x, x0y, x0z, x1x, x1y, x1z, sdx, sdy, sdz, gx, gy, gz,
          g00, g01, g02, g10, g11, g12, g20, g21, g22, r, cyl, has_side, cand, RULE4: tl.constexpr):  # fmt: skip
    """mju_wrap of one site-geom-site triple: wrap length (-1: none) and the tangent points in world."""
    # into the geom frame: R^T (x - gpos)
    ax, ay, az = x0x - gx, x0y - gy, x0z - gz
    p0x, p0y, p0z = g00 * ax + g10 * ay + g20 * az, g01 * ax + g11 * ay + g21 * az, g02 * ax + g12 * ay + g22 * az
    ax, ay, az = x1x - gx, x1y - gy, x1z - gz
    p1x, p1y, p1z = g00 * ax + g10 * ay + g20 * az, g01 * ax + g11 * ay + g21 * az, g02 * ax + g12 * ay + g22 * az
    ax, ay, az = sdx - gx, sdy - gy, sdz - gz
    spx, spy, spz = g00 * ax + g10 * ay + g20 * az, g01 * ax + g11 * ay + g21 * az, g02 * ax + g12 * ay + g22 * az
    # 2D frame: sphere - plane of (0, p0, p1); cylinder - local xy
    a0x, a0y, a0z = _normalize3(p0x, p0y, p0z)
    nx, ny, nz = _cross(p0x, p0y, p0z, p1x, p1y, p1z)
    par = _norm3(nx, ny, nz) < 1e-15
    aa0, aa1, aa2 = tl.abs(a0x), tl.abs(a0y), tl.abs(a0z)
    i1 = (aa1 > aa0) & (aa1 > aa2)
    i2 = (aa2 > aa0) & (aa2 > aa1)
    alx = tl.where(i1 | i2, 1.0, 0.0)
    aly = tl.where(i1, 0.0, 1.0)
    alz = tl.where(i2, 0.0, 1.0)
    cx, cy, cz = _cross(a0x, a0y, a0z, alx, aly, alz)
    nx, ny, nz = _normalize3(tl.where(par, cx, nx), tl.where(par, cy, ny), tl.where(par, cz, nz))
    b0, b1, b2 = _cross(nx, ny, nz, a0x, a0y, a0z)
    a1x, a1y, a1z = _normalize3(b0, b1, b2)
    a0x = tl.where(cyl, 1.0, a0x)
    a0y = tl.where(cyl, 0.0, a0y)
    a0z = tl.where(cyl, 0.0, a0z)
    a1x = tl.where(cyl, 0.0, a1x)
    a1y = tl.where(cyl, 1.0, a1y)
    a1z = tl.where(cyl, 0.0, a1z)
    e0x, e0y = p0x * a0x + p0y * a0y + p0z * a0z, p0x * a1x + p0y * a1y + p0z * a1z
    e1x, e1y = p1x * a0x + p1y * a0y + p1z * a0z, p1x * a1x + p1y * a1y + p1z * a1z
    s2x, s2y = _normalize2(spx * a0x + spy * a0y + spz * a0z, spx * a1x + spy * a1y + spz * a1z)
    s2x, s2y = s2x * r, s2y * r
    wl, c0x, c0y, c1x, c1y = _wrap_circle(e0x, e0y, e1x, e1y, s2x, s2y, has_side, r, RULE4)
    inside = has_side & (_norm3(spx, spy, spz) < r)
    # the side site may lie inside the geom: MuJoCo's float64 solve, where any element of the block needs it
    if cand & (tl.max(inside.to(tl.int32), axis=0) > 0):
        wi, cix, ciy = _wrap_inside(e0x.to(tl.float64), e0y.to(tl.float64), e1x.to(tl.float64),
                                    e1y.to(tl.float64), r.to(tl.float64), RULE4)  # fmt: skip
        wl = tl.where(inside, wi.to(wl.dtype), wl)
        cix = cix.to(wl.dtype)
        ciy = ciy.to(wl.dtype)
        c0x = tl.where(inside, cix, c0x)
        c0y = tl.where(inside, ciy, c0y)
        c1x = tl.where(inside, cix, c1x)
        c1y = tl.where(inside, ciy, c1y)
    tooclose = (_norm3(p0x, p0y, p0z) < 1e-15) | (_norm3(p1x, p1y, p1z) < 1e-15)
    wl = tl.where(tooclose, -1.0, wl)
    r0x, r0y, r0z = a0x * c0x + a1x * c0y, a0y * c0x + a1y * c0y, a0z * c0x + a1z * c0y
    r1x, r1y, r1z = a0x * c1x + a1x * c1y, a0y * c1x + a1y * c1y, a0z * c1x + a1z * c1y
    # cylinder: heights of the tangent points by arc length, then the helix length
    L0 = _norm2(p0x - r0x, p0y - r0y)
    L1 = _norm2(p1x - r1x, p1y - r1y)
    tot = tl.maximum(L0 + wl + L1, 1e-15)
    z0 = p0z + (p1z - p0z) * L0 / tot
    z1 = p0z + (p1z - p0z) * (L0 + wl) / tot
    r0z = tl.where(cyl, z0, r0z)
    r1z = tl.where(cyl, z1, r1z)
    wl = tl.where(cyl & (wl >= 0), _sqrt_safe(wl * wl + (z1 - z0) * (z1 - z0)), wl)
    w0x = g00 * r0x + g01 * r0y + g02 * r0z + gx
    w0y = g10 * r0x + g11 * r0y + g12 * r0z + gy
    w0z = g20 * r0x + g21 * r0y + g22 * r0z + gz
    w1x = g00 * r1x + g01 * r1y + g02 * r1z + gx
    w1y = g10 * r1x + g11 * r1y + g12 * r1z + gy
    w1z = g20 * r1x + g21 * r1y + g22 * r1z + gz
    return wl, w0x, w0y, w0z, w1x, w1y, w1z


@triton.jit
def _site(xpos, xmat, sbp, sbm, bofs, m, site_body, site_pos, s):
    """World position of site s (body frame times the local offset)."""
    body = tl.load(site_body + s)
    p = xpos + bofs * sbp + body * 3
    R = xmat + bofs * sbm + body * 9
    dt = xpos.dtype.element_ty
    lx, ly, lz = tl.load(site_pos + 3 * s).to(dt), tl.load(site_pos + 3 * s + 1).to(dt), tl.load(site_pos + 3 * s + 2).to(dt)
    x = tl.load(p, mask=m, other=0.0) + tl.load(R, mask=m, other=0.0) * lx + tl.load(R + 1, mask=m, other=0.0) * ly + tl.load(R + 2, mask=m, other=0.0) * lz  # fmt: skip
    y = tl.load(p + 1, mask=m, other=0.0) + tl.load(R + 3, mask=m, other=0.0) * lx + tl.load(R + 4, mask=m, other=0.0) * ly + tl.load(R + 5, mask=m, other=0.0) * lz  # fmt: skip
    z = tl.load(p + 2, mask=m, other=0.0) + tl.load(R + 6, mask=m, other=0.0) * lx + tl.load(R + 7, mask=m, other=0.0) * ly + tl.load(R + 8, mask=m, other=0.0) * lz  # fmt: skip
    return x, y, z


@triton.jit
def _piece(acc, x1x, x1y, x1z, x2x, x2y, x2z, on, w, e_lo, e_hi, omega, v0, sbt, sdt, slope, sbs, bofs, m, cols,
           ent_dof, ent_end, ent_col, ent_cc):  # fmt: skip
    """Jacobian entries of one straight piece x1 -> x2 (where ``on``), added to the row tile."""
    dx, dy, dz = x2x - x1x, x2y - x1y, x2z - x1z
    n = _norm3(dx, dy, dz)
    ux, uy, uz = dx / n, dy / n, dz / n
    wt = tl.where(on, w, 0.0)
    for e in range(e_lo, e_hi):
        k = tl.load(ent_dof + e)
        end = tl.load(ent_end + e)
        col = tl.load(ent_col + e)
        cc = tl.load(ent_cc + e)
        o = omega + bofs * sbt + k * sdt
        v = v0 + bofs * sbt + k * sdt
        xe = tl.where(end == 1, x2x, x1x)
        ye = tl.where(end == 1, x2y, x1y)
        ze = tl.where(end == 1, x2z, x1z)
        cx, cy, cz = _cross(xe, ye, ze, ux, uy, uz)
        val = (ux * tl.load(v, mask=m, other=0.0) + uy * tl.load(v + 1, mask=m, other=0.0) + uz * tl.load(v + 2, mask=m, other=0.0)
               + cx * tl.load(o, mask=m, other=0.0) + cy * tl.load(o + 1, mask=m, other=0.0) + cz * tl.load(o + 2, mask=m, other=0.0))  # fmt: skip
        val = val * tl.where(end == 1, wt, -wt)
        if cc >= 0:
            val = val * tl.load(slope + bofs * sbs + cc, mask=m, other=0.0)
        acc += tl.where(cols[None, :] == col, val[:, None], 0.0)
    return acc, n


@triton.jit
def _tendon_kernel(
    xpos, xmat, omega, v0, slope, L_out, J_out,
    WL, B, sbp, sbm, sbt, sdt, sbs, nrow, nout, nseg,
    site_body, site_pos, geom_body, geom_pos, geom_R, geom_r,
    row_seg, seg_kind, seg_s0, seg_s1, seg_g, seg_side, seg_w, seg_flags, seg_ent,
    ent_dof, ent_end, ent_col, ent_cc, out_base, out_n,
    BS: tl.constexpr, JW: tl.constexpr, RULE4: tl.constexpr,
):  # fmt: skip
    pid_b = tl.program_id(0)
    row = tl.program_id(1)
    bofs = (pid_b * BS + tl.arange(0, BS)).to(tl.int64)
    m = bofs < B
    ones = m | (~m)
    cols = tl.arange(0, JW)
    fz = tl.load(xpos + bofs * sbp, mask=m, other=0.0) * 0.0
    acc = tl.zeros([BS, JW], dtype=fz.dtype)
    length = fz
    s_lo = tl.load(row_seg + row)
    s_hi = tl.load(row_seg + row + 1)
    for si in range(s_lo, s_hi):
        kind = tl.load(seg_kind + si)
        w = tl.load(seg_w + si).to(fz.dtype)  # float tables are float64, cast to the inputs' dtype
        x0x, x0y, x0z = _site(xpos, xmat, sbp, sbm, bofs, m, site_body, site_pos, tl.load(seg_s0 + si))
        x1x, x1y, x1z = _site(xpos, xmat, sbp, sbm, bofs, m, site_body, site_pos, tl.load(seg_s1 + si))
        e = seg_ent + 3 * si  # entry ranges of the pieces x0 -> t0, t1 -> x1, x0 -> x1: e[0..3]
        wrapped = fz > 1.0
        if kind == 1:
            g = tl.load(seg_g + si)
            flags = tl.load(seg_flags + si)
            side = tl.load(seg_side + si)
            gb = tl.load(geom_body + g)
            glx, gly, glz = (
                tl.load(geom_pos + 3 * g).to(fz.dtype),
                tl.load(geom_pos + 3 * g + 1).to(fz.dtype),
                tl.load(geom_pos + 3 * g + 2).to(fz.dtype),
            )
            p = xpos + bofs * sbp + gb * 3
            Rp = xmat + bofs * sbm + gb * 9
            b00, b01, b02 = (
                tl.load(Rp, mask=m, other=0.0),
                tl.load(Rp + 1, mask=m, other=0.0),
                tl.load(Rp + 2, mask=m, other=0.0),
            )
            b10, b11, b12 = (
                tl.load(Rp + 3, mask=m, other=0.0),
                tl.load(Rp + 4, mask=m, other=0.0),
                tl.load(Rp + 5, mask=m, other=0.0),
            )
            b20, b21, b22 = (
                tl.load(Rp + 6, mask=m, other=0.0),
                tl.load(Rp + 7, mask=m, other=0.0),
                tl.load(Rp + 8, mask=m, other=0.0),
            )
            gx = tl.load(p, mask=m, other=0.0) + b00 * glx + b01 * gly + b02 * glz
            gy = tl.load(p + 1, mask=m, other=0.0) + b10 * glx + b11 * gly + b12 * glz
            gz = tl.load(p + 2, mask=m, other=0.0) + b20 * glx + b21 * gly + b22 * glz
            # world rotation of the geom: body R times its local rotation
            q = geom_R + 9 * g
            l00, l01, l02 = tl.load(q).to(fz.dtype), tl.load(q + 1).to(fz.dtype), tl.load(q + 2).to(fz.dtype)
            l10, l11, l12 = tl.load(q + 3).to(fz.dtype), tl.load(q + 4).to(fz.dtype), tl.load(q + 5).to(fz.dtype)
            l20, l21, l22 = tl.load(q + 6).to(fz.dtype), tl.load(q + 7).to(fz.dtype), tl.load(q + 8).to(fz.dtype)
            g00, g01, g02 = (
                b00 * l00 + b01 * l10 + b02 * l20,
                b00 * l01 + b01 * l11 + b02 * l21,
                b00 * l02 + b01 * l12 + b02 * l22,
            )
            g10, g11, g12 = (
                b10 * l00 + b11 * l10 + b12 * l20,
                b10 * l01 + b11 * l11 + b12 * l21,
                b10 * l02 + b11 * l12 + b12 * l22,
            )
            g20, g21, g22 = (
                b20 * l00 + b21 * l10 + b22 * l20,
                b20 * l01 + b21 * l11 + b22 * l21,
                b20 * l02 + b21 * l12 + b22 * l22,
            )
            has_side = ones & (side >= 0)
            sdx, sdy, sdz = _site(xpos, xmat, sbp, sbm, bofs, m, site_body, site_pos, tl.maximum(side, 0))
            r = tl.load(geom_r + g).to(fz.dtype) + fz
            cyl = ones & ((flags & 1) != 0)
            wl, w0x, w0y, w0z, w1x, w1y, w1z = _wrap(x0x, x0y, x0z, x1x, x1y, x1z, sdx, sdy, sdz, gx, gy, gz,
                                                     g00, g01, g02, g10, g11, g12, g20, g21, g22, r, cyl, has_side,
                                                     (flags & 2) != 0, RULE4)  # fmt: skip
            wrapped = wl >= 0
            tl.store(WL + bofs * nseg + si, wl, mask=m)
            acc, na = _piece(acc, x0x, x0y, x0z, w0x, w0y, w0z, wrapped, w, tl.load(e), tl.load(e + 1), omega, v0,
                             sbt, sdt, slope, sbs, bofs, m, cols, ent_dof, ent_end, ent_col, ent_cc)  # fmt: skip
            acc, nb = _piece(acc, w1x, w1y, w1z, x1x, x1y, x1z, wrapped, w, tl.load(e + 1), tl.load(e + 2), omega, v0,
                             sbt, sdt, slope, sbs, bofs, m, cols, ent_dof, ent_end, ent_col, ent_cc)  # fmt: skip
            length += tl.where(wrapped, (na + wl + nb) * w, 0.0)
        acc, ns = _piece(acc, x0x, x0y, x0z, x1x, x1y, x1z, ~wrapped, w, tl.load(e + 2), tl.load(e + 3), omega, v0,
                         sbt, sdt, slope, sbs, bofs, m, cols, ent_dof, ent_end, ent_col, ent_cc)  # fmt: skip
        length += tl.where(wrapped, 0.0, ns * w)
    tl.store(L_out + bofs * nrow + row, length, mask=m)
    base = tl.load(out_base + row)
    n = tl.load(out_n + row)
    tl.store(J_out + bofs[:, None] * nout + base + cols[None, :], acc, mask=m[:, None] & (cols[None, :] < n))


# ---------------------------------------------------------------------------------------------- tables
@dataclass
class Segment:
    """One path segment of a tendon row: a site pair (``geom < 0``) or a site-geom-site wrap.

    Attributes:
        row: output row (tendon or muscle).
        s0, s1: site indices (into the site table) of the end points.
        geom: geom index (into the geom table), -1 for a pair.
        side: side site index, -1 without one.
        cylinder: cylinder wrap (else sphere).
        inside: the side site can be inside the geom (the float64 inside solve runs).
        weight: 1 / pulley divisor.
        entries: Jacobian entries of the pieces x0 -> t0, t1 -> x1 and x0 -> x1, each a list of
            ``(dof, end, column, coupling)``: end 1 if the DOF moves the piece's second end point, else 0;
            column within the row's outputs; coupling column of the per-state factor, or -1.
        tag: caller's index of the segment (e.g. its wrap triple), for :meth:`FusedTendons.wrap_lengths`.
    """

    row: int
    s0: int
    s1: int
    geom: int
    side: int
    cylinder: bool
    inside: bool
    weight: float
    entries: tuple[list, list, list]
    tag: int = -1


class FusedTendons:
    """Static tables of the fused kernel, on a device; float tables in float64 (cast to the inputs' dtype).

    Args:
        site_body, site_pos: frame index (into ``xpos``/``xmat``) and local position of each site.
        geom_body, geom_pos, geom_R, geom_radius: frame index, local position, local rotation (3 x 3) and radius
            of each wrap geom.
        segments: every path segment, any order.
        nrow: number of output rows.
        out_base, out_n: per row, the first output column and the number of columns (``J`` is (..., nout)).
        nout: number of output columns.
        device: torch device.
    """

    def __init__(self, site_body, site_pos, geom_body, geom_pos, geom_R, geom_radius, segments: list[Segment],
                 nrow: int, out_base, out_n, nout: int, device):  # fmt: skip
        segments = sorted(segments, key=lambda s: s.row)
        i32 = lambda x: torch.as_tensor(np.asarray(x, dtype=np.int32).reshape(-1), device=device)  # noqa: E731
        f = lambda x: torch.as_tensor(np.asarray(x, dtype=np.float64).reshape(-1), device=device)  # noqa: E731
        self.site_body, self.site_pos = i32(site_body), f(site_pos)
        self.geom_body, self.geom_pos = i32(geom_body), f(geom_pos)
        self.geom_R, self.geom_r = f(geom_R), f(geom_radius)
        counts = np.bincount(np.asarray([s.row for s in segments], dtype=np.int64), minlength=nrow)
        self.row_seg = i32(np.concatenate(([0], np.cumsum(counts))))
        self.seg_kind = i32([int(s.geom >= 0) for s in segments])
        self.seg_s0, self.seg_s1 = i32([s.s0 for s in segments]), i32([s.s1 for s in segments])
        self.seg_g = i32([max(s.geom, 0) for s in segments])
        self.seg_side = i32([s.side for s in segments])
        self.seg_w = f([s.weight for s in segments])
        self.seg_flags = i32([int(s.cylinder) | 2 * int(s.inside) for s in segments])
        sizes = [len(piece) for s in segments for piece in s.entries]
        self.seg_ent = i32(np.concatenate(([0], np.cumsum(sizes))))  # piece k of segment si: [3 si + k, 3 si + k + 1)
        e = np.array([x for s in segments for piece in s.entries for x in piece], dtype=np.int64).reshape(-1, 4)
        if not len(e):
            e = np.zeros((1, 4), dtype=np.int64)
        self.ent_dof, self.ent_end, self.ent_col, self.ent_cc = (i32(e[:, k]) for k in range(4))
        self.out_base, self.out_n = i32(out_base), i32(out_n)
        self.nrow, self.nout, self.nseg = nrow, nout, len(segments)
        self.jw = max(16, triton.next_power_of_2(int(max(out_n, default=1))))
        self.block = 64  # states per program (fastest of 16, 32, 64 on an RTX 5090)
        tags = np.array([s.tag for s in segments], dtype=np.int64)
        wrap_pos = np.flatnonzero(tags >= 0)
        # segment position of each tagged segment, in tag order
        self.tag_seg = torch.as_tensor(wrap_pos[np.argsort(tags[wrap_pos])], device=device)

    def wrap_lengths(self, WL: torch.Tensor) -> torch.Tensor:
        """Per tagged segment (tag order): the wrap length (-1: not wrapped), from the kernel's per-segment buffer."""
        return WL[..., self.tag_seg]

    def __call__(self, xpos: torch.Tensor, xmat: torch.Tensor, omega: torch.Tensor, v0: torch.Tensor,
                 slope: torch.Tensor | None = None, rule: str = "mujoco3") -> tuple[torch.Tensor, ...]:  # fmt: skip
        """Lengths (..., nrow), Jacobian (..., nout) and per-segment wrap lengths (..., nseg), without gradient.

        Args:
            xpos, xmat: frame positions (..., nframe, 3) and rotations (..., nframe, 3, 3).
            omega, v0: angular part and linear velocity at the origin of each DOF's motion, (..., ndof, 3); may be
                views of one (..., ndof, 6) tensor.
            slope: per-state factors of the coupled entries, (..., ncoupling).
            rule: side-site rule, ``"mujoco3"`` or ``"mujoco4"``.
        """
        return tendon_length_jacobian(self, xpos, xmat, omega, v0, slope, rule)


def tendon_length_jacobian(t: FusedTendons, xpos, xmat, omega, v0, slope=None,
                           rule: str = "mujoco3") -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:  # fmt: skip
    """The fused kernel (see :meth:`FusedTendons.__call__`)."""
    block = t.block
    lead = xpos.shape[:-2]
    B = int(np.prod(lead)) if lead else 1
    dt = xpos.dtype
    xp = xpos.reshape(B, -1).contiguous()
    xm = xmat.reshape(B, -1).contiguous()
    nd = omega.shape[-2]
    om, vv = omega.to(dt).reshape(B, nd, 3), v0.to(dt).reshape(B, nd, 3)
    if om.stride() != vv.stride() or om.stride(2) != 1:
        om, vv = om.contiguous(), vv.contiguous()
    sl = slope.to(dt).reshape(B, -1).contiguous() if slope is not None and slope.shape[-1] else xp
    L = torch.empty(B, t.nrow, device=xpos.device, dtype=dt)
    J = torch.empty(B, t.nout, device=xpos.device, dtype=dt)
    WL = torch.full((B, t.nseg), -1.0, device=xpos.device, dtype=dt)
    if B and t.nrow:
        grid = (triton.cdiv(B, block), t.nrow)
        _tendon_kernel[grid](
            xp, xm, om, vv, sl, L, J, WL, B, xp.stride(0), xm.stride(0), om.stride(0), om.stride(1), sl.stride(0),
            t.nrow, t.nout, t.nseg, t.site_body, t.site_pos, t.geom_body, t.geom_pos, t.geom_R, t.geom_r,
            t.row_seg, t.seg_kind, t.seg_s0, t.seg_s1, t.seg_g, t.seg_side, t.seg_w, t.seg_flags, t.seg_ent,
            t.ent_dof, t.ent_end, t.ent_col, t.ent_cc, t.out_base, t.out_n,
            BS=block, JW=t.jw, RULE4=rule == "mujoco4", num_warps=max(1, block // 32),
        )  # fmt: skip
    return L.reshape(lead + (t.nrow,)), J.reshape(lead + (t.nout,)), WL.reshape(lead + (t.nseg,))


class FusedTendonFunction(torch.autograd.Function):
    """The fused kernel's values with the gradient of a torch replica (recomputed in the backward pass).

    ``forward(t, reference, rule, xpos, xmat, omega, v0)``: ``reference(xpos, xmat, omega, v0) -> (L, J)`` is the
    differentiable torch computation the kernel replaces.
    """

    @staticmethod
    def forward(ctx, t, reference, rule, xpos, xmat, omega, v0):  # noqa: D102
        ctx.reference = reference
        ctx.save_for_backward(xpos, xmat, omega, v0)
        L, J, WL = tendon_length_jacobian(t, xpos, xmat, omega, v0, None, rule)
        ctx.mark_non_differentiable(WL)
        return L, J, WL

    @staticmethod
    def backward(ctx, gL, gJ, _gWL):  # noqa: D102
        inputs = [x.detach().requires_grad_(need) for x, need in zip(ctx.saved_tensors, ctx.needs_input_grad[3:])]
        with torch.enable_grad():
            L, J = ctx.reference(*inputs)
            want = [x for x in inputs if x.requires_grad]
            grads = iter(torch.autograd.grad((L, J), want, (gL, gJ), allow_unused=True))
        return (None, None, None) + tuple(next(grads) if x.requires_grad else None for x in inputs)
