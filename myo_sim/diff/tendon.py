"""Tendon lengths, Jacobians (moment arms) and velocities of a compiled MuJoCo model in torch (``mj_tendon``).

The path decomposition, the wrapped pieces and the Jacobian convention follow ``mj_tendon`` in
``engine_core_smooth.c``; the length and velocity bookkeeping of the straight and wrapped pieces is
adapted from muscle-supermimic (github.com/kaibohex/muscle-supermimic, Apache-2.0, commit 8a6244b,
``src/muscle_supermimic/tasks/tracking/mdp/muscle_geometry.py``).

Spatial tendons are split into pieces between consecutive path points: site-site segments, and for each
site-geom-site wrap the two tangent segments (wrapped) or the straight segment (not wrapped). Pulleys
divide the pieces after them by their divisor. Each piece contributes ``|x2 - x1| / divisor`` to the
length and ``u . (J(x2) - J(x1)) / divisor`` to the Jacobian, with the tangent points fixed to the wrap
geom's body (exact, since the length is stationary in them); arcs on the geom contribute no Jacobian.
Fixed tendons are linear in ``qpos``.

The Jacobian is returned in MuJoCo's sparse row layout (``ten_J_rownnz``, ``ten_J_rowadr``,
``ten_J_colind``), so ``ten_velocity = ten_J qvel`` is a gather and a scatter-add.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import torch

from ._math import cross, dot, norm
from .kinematics import Kinematics, KinematicsState, geom_tables, site_tables
from .wrap import wrap

# plain ints: MuJoCo 4's enums do not compare equal to numpy integers
_SITE, _PULLEY, _JOINT = int(mujoco.mjtWrap.mjWRAP_SITE), int(mujoco.mjtWrap.mjWRAP_PULLEY), int(mujoco.mjtWrap.mjWRAP_JOINT)
_SPHERE, _CYLINDER = int(mujoco.mjtWrap.mjWRAP_SPHERE), int(mujoco.mjtWrap.mjWRAP_CYLINDER)
_HINGE, _SLIDE = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)
_GSPHERE, _GCYLINDER = int(mujoco.mjtGeom.mjGEOM_SPHERE), int(mujoco.mjtGeom.mjGEOM_CYLINDER)


def body_dof_chain(m: mujoco.MjModel, b: int) -> set[int]:
    """DOFs that move body ``b``: the DOFs of the body and of all its ancestors."""
    dofs: set[int] = set()
    while b > 0:
        dofs.update(range(m.body_dofadr[b], m.body_dofadr[b] + m.body_dofnum[b]))
        b = int(m.body_parentid[b])
    return dofs


def sparse_layout(m: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Row layout of the tendon Jacobian: ``(rownnz, rowadr, colind)``.

    Read from the model (``ten_J_*``) when MuJoCo provides it, otherwise built the same way: every DOF
    that moves a body on the tendon's path (spatial) or the DOF of each joint (fixed), sorted.
    """
    if hasattr(m, "ten_J_rownnz"):
        return np.array(m.ten_J_rownnz), np.array(m.ten_J_rowadr), np.array(m.ten_J_colind)
    rows = []
    for t in range(m.ntendon):
        adr, num = m.tendon_adr[t], m.tendon_num[t]
        cols: set[int] = set()
        for k in range(adr, adr + num):
            typ, obj = int(m.wrap_type[k]), int(m.wrap_objid[k])
            if typ == _JOINT:
                cols.add(int(m.jnt_dofadr[obj]))
            elif typ == _SITE:
                cols |= body_dof_chain(m, int(m.site_bodyid[obj]))
            elif typ in (_SPHERE, _CYLINDER):
                cols |= body_dof_chain(m, int(m.geom_bodyid[obj]))
        rows.append(sorted(cols))
    rownnz = np.array([len(r) for r in rows], dtype=np.int64)
    rowadr = np.concatenate(([0], np.cumsum(rownnz)[:-1])).astype(np.int64) if len(rows) else np.zeros(0, np.int64)
    colind = np.array([c for r in rows for c in r], dtype=np.int64)
    return rownnz, rowadr, colind


@dataclass
class TendonState:
    """Tendon outputs.

    Attributes:
        length: tendon lengths, (..., ntendon) (``ten_length``).
        J: tendon Jacobian in sparse row layout, (..., nJ) (``ten_J``).
        velocity: tendon velocities, (..., ntendon) (``ten_velocity``), or None without ``qvel``.
        wrapped: whether each site-geom-site wrap is active, (..., nwrap_triples).
    """

    length: torch.Tensor
    J: torch.Tensor
    velocity: torch.Tensor | None
    wrapped: torch.Tensor


class Tendons:
    """Every tendon of a compiled model, spatial and fixed.

    Args:
        m: compiled model.
        kin: kinematics of the same model.
        side_rule: ``"mujoco3"`` or ``"mujoco4"`` (see :mod:`myo_sim.diff.wrap`).
    """

    def __init__(self, m: mujoco.MjModel, kin: Kinematics, side_rule: str):
        device, dtype = kin.device, kin.dtype
        self.kin, self.side_rule, self.ntendon, self.nv = kin, side_rule, m.ntendon, m.nv
        self.rownnz, self.rowadr, self.colind = sparse_layout(m)
        self.nJ = int(self.rownnz.sum())
        slot = {}  # (tendon, dof) -> position in the sparse values
        for t in range(m.ntendon):
            for k in range(self.rownnz[t]):
                slot[(t, int(self.colind[self.rowadr[t] + k]))] = int(self.rowadr[t] + k)
        chains = {b: body_dof_chain(m, b) for b in range(m.nbody)}

        sites: dict[int, int] = {}
        geoms: dict[int, int] = {}
        pairs = []  # (tendon, site0, site1, 1/divisor)
        triples = []  # (tendon, site0, geom, site1, side site or -1, cylinder, 1/divisor)
        fixed = []  # (tendon, qadr, dof, coef)
        for t in range(m.ntendon):
            adr, num = int(m.tendon_adr[t]), int(m.tendon_num[t])
            typ = [int(x) for x in m.wrap_type[adr : adr + num]]
            obj, prm = m.wrap_objid[adr : adr + num], m.wrap_prm[adr : adr + num]
            if typ[0] == _JOINT:
                for j in range(num):
                    jid = int(obj[j])
                    if int(m.jnt_type[jid]) not in (_HINGE, _SLIDE):
                        raise NotImplementedError(f"fixed tendon {m.tendon(t).name}: only hinge and slide joints")
                    fixed.append((t, int(m.jnt_qposadr[jid]), int(m.jnt_dofadr[jid]), float(prm[j])))
                continue
            inv_div, j = 1.0, 0
            while j < num - 1:
                t0, t1 = typ[j], typ[j + 1]
                if t0 == _PULLEY or t1 == _PULLEY:
                    if t0 == _PULLEY:
                        inv_div = 1.0 / float(prm[j])
                    j += 1
                    continue
                if t0 != _SITE:
                    raise NotImplementedError(f"tendon {m.tendon(t).name}: path element {j} is not a site")
                s0 = sites.setdefault(int(obj[j]), len(sites))
                if t1 in (_SPHERE, _CYLINDER):
                    side = int(round(float(prm[j + 1])))
                    gtype = int(m.geom_type[int(obj[j + 1])])
                    if gtype not in (_GSPHERE, _GCYLINDER):
                        raise NotImplementedError(f"tendon {m.tendon(t).name}: wrap geom type {gtype}")
                    triples.append(
                        (
                            t,
                            s0,
                            geoms.setdefault(int(obj[j + 1]), len(geoms)),
                            sites.setdefault(int(obj[j + 2]), len(sites)),
                            -1 if side < 0 else sites.setdefault(side, len(sites)),
                            t1 == _CYLINDER,
                            inv_div,
                        )
                    )
                    j += 2
                else:
                    pairs.append((t, s0, sites.setdefault(int(obj[j + 1]), len(sites)), inv_div))
                    j += 1
        site_ids, geom_ids = list(sites), list(geoms)
        self.site_ids, self.geom_ids = site_ids, geom_ids
        self.site_body, self.site_pos, _ = site_tables(m, site_ids, device, dtype)
        self.geom_body, self.geom_pos, self.geom_R = geom_tables(m, geom_ids, device, dtype)
        self.geom_radius = torch.as_tensor(m.geom_size[geom_ids, 0] if geom_ids else np.zeros(0), device=device, dtype=dtype)

        def it(x) -> torch.Tensor:
            return torch.as_tensor(np.asarray(x, dtype=np.int64).reshape(-1), device=device)

        def ft(x) -> torch.Tensor:
            return torch.as_tensor(np.asarray(x, dtype=np.float64).reshape(-1), device=device, dtype=dtype)

        ns, nt = len(site_ids), len(triples)
        self.ntriple = nt
        self.tri_ten, self.tri_s0, self.tri_g, self.tri_s1 = (it([x[k] for x in triples]) for k in range(4))
        side = np.array([x[4] for x in triples], dtype=np.int64)
        self.tri_has_side = torch.as_tensor(side >= 0, device=device)
        self.tri_side = it(np.maximum(side, 0))
        self.tri_cyl = torch.as_tensor(np.array([x[5] for x in triples], dtype=bool), device=device)
        self.tri_w = ft([x[6] for x in triples])

        # pieces: endpoints index into points = cat(sites, tangent points 0, tangent points 1);
        # kind 0 = always on, 1 = on when wrapped, 2 = on when not wrapped
        site_body = [int(m.site_bodyid[s]) for s in site_ids]
        geom_body = [int(m.geom_bodyid[g]) for g in geom_ids]
        pieces = []  # (tendon, point1, point2, body1, body2, 1/divisor, kind, triple)
        for t, a, b, w in pairs:
            pieces.append((t, a, b, site_body[a], site_body[b], w, 0, 0))
        for k, (t, a, g, b, _, _, w) in enumerate(triples):
            pieces.append((t, a, ns + k, site_body[a], geom_body[g], w, 1, k))
            pieces.append((t, ns + nt + k, b, geom_body[g], site_body[b], w, 1, k))
            pieces.append((t, a, b, site_body[a], site_body[b], w, 2, k))
        self.npiece = len(pieces)
        self.pc_ten, self.pc_p1, self.pc_p2 = (it([x[k] for x in pieces]) for k in range(3))
        self.pc_w = ft([x[5] for x in pieces])
        kind = np.array([x[6] for x in pieces], dtype=np.int64)
        self.pc_kind = it(kind)
        self.pc_tri = it([x[7] for x in pieces])

        # Jacobian entries: one per piece and DOF that moves exactly one of its end points
        ent = []  # (piece, dof, end (0: x1, 1: x2), sign, slot)
        for k, (t, _, _, b1, b2, _, _, _) in enumerate(pieces):
            c1, c2 = chains[b1], chains[b2]
            for dof in sorted(c1 ^ c2):
                if (t, dof) not in slot:
                    raise RuntimeError(f"tendon {m.tendon(t).name}: DOF {dof} missing from the sparse layout")
                end = 1 if dof in c2 else 0
                ent.append((k, dof, end, 1.0 if end else -1.0, slot[(t, dof)]))
        self.ent_piece, self.ent_dof, self.ent_end = (it([x[k] for x in ent]) for k in range(3))
        self.ent_sign = ft([x[3] for x in ent])
        self.ent_slot = it([x[4] for x in ent])

        # fixed tendons: constant Jacobian entries
        self.fx_ten, self.fx_qadr = it([x[0] for x in fixed]), it([x[1] for x in fixed])
        self.fx_coef = ft([x[3] for x in fixed])
        Jc = np.zeros(self.nJ)
        for t, _, dof, c in fixed:
            Jc[slot[(t, dof)]] += c
        self.J_const = ft(Jc)
        self.row_of_slot = it(np.repeat(np.arange(m.ntendon), self.rownnz))
        self.colind_t = it(self.colind)

    def forward(self, s: KinematicsState, qpos: torch.Tensor, qvel: torch.Tensor | None = None) -> TendonState:
        """Lengths, Jacobian and (with ``qvel``) velocities of every tendon."""
        lead = qpos.shape[:-1]
        X, _ = self.kin.frame_pose(s, self.site_body, self.site_pos)
        L = qpos.new_zeros(lead + (self.ntendon,))
        if self.ntriple:
            gpos, gR = self.kin.frame_pose(s, self.geom_body, self.geom_pos, self.geom_R)
            g = self.tri_g
            wl, T0, T1, _ = wrap(
                X[..., self.tri_s0, :],
                X[..., self.tri_s1, :],
                gpos[..., g, :],
                gR[..., g, :, :],
                self.geom_radius[g].expand(lead + (self.ntriple,)),
                self.tri_cyl.expand(lead + (self.ntriple,)),
                X[..., self.tri_side, :],
                self.tri_has_side.expand(lead + (self.ntriple,)),
                self.side_rule,
            )
            wrapped = wl >= 0
            points = torch.cat((X, T0, T1), -2)
            L = L.index_add(-1, self.tri_ten, torch.where(wrapped, wl, torch.zeros_like(wl)) * self.tri_w)
            on_w = wrapped[..., self.pc_tri]
            on = torch.where(self.pc_kind == 0, torch.ones_like(on_w), torch.where(self.pc_kind == 1, on_w, ~on_w))
        else:
            wrapped = qpos.new_zeros(lead + (0,), dtype=torch.bool)
            points = X
            on = torch.ones(lead + (self.npiece,), dtype=torch.bool, device=qpos.device)
        if self.fx_ten.numel():
            L = L.index_add(-1, self.fx_ten, qpos[..., self.fx_qadr] * self.fx_coef)
        J = self.J_const.expand(lead + (self.nJ,))
        if self.npiece:
            x1, x2 = points[..., self.pc_p1, :], points[..., self.pc_p2, :]
            d = x2 - x1
            n = norm(d)
            w = torch.where(on, self.pc_w, torch.zeros_like(n))
            L = L.index_add(-1, self.pc_ten, n * w)
            if self.ent_slot.numel():
                u = d / n.unsqueeze(-1)
                c = torch.cat((cross(x1, u), cross(x2, u)), -2)  # x_end x u, (..., 2 npiece, 3)
                omega, v0 = self.kin.dof_axes(s)
                e = self.ent_piece
                val = dot(u[..., e, :], v0[..., self.ent_dof, :]) + dot(
                    omega[..., self.ent_dof, :], c[..., self.ent_end * self.npiece + e, :]
                )
                J = J.index_add(-1, self.ent_slot, val * self.ent_sign * w[..., e])
        vel = None
        if qvel is not None:
            vel = qpos.new_zeros(lead + (self.ntendon,)).index_add(-1, self.row_of_slot, J * qvel[..., self.colind_t])
        return TendonState(length=L, J=J, velocity=vel, wrapped=wrapped)

    def dense_J(self, J: torch.Tensor) -> torch.Tensor:
        """Sparse tendon Jacobian (..., nJ) to dense (..., ntendon, nv) (``mju_sparse2dense``)."""
        lead = J.shape[:-1]
        flat = J.new_zeros(lead + (self.ntendon * self.nv,))
        return flat.index_add(-1, self.row_of_slot * self.nv + self.colind_t, J).reshape(lead + (self.ntendon, self.nv))
