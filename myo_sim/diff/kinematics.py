"""Differentiable forward kinematics of a compiled MuJoCo model in torch (``mj_kinematics`` and ``cdof``).

Adapted from muscle-supermimic (github.com/kaibohex/muscle-supermimic, Apache-2.0, commit 8a6244b,
``src/muscle_supermimic/tasks/tracking/mdp/kinematics.py``): generalised from the independent hinge and
slide coordinates of one model to the full ``qpos`` of any model (hinge, slide, ball and free joints),
and evaluated level by level of the body tree instead of body by body.

Each joint's local transform depends on its own ``qpos`` only, so all joints are evaluated at once; each
body's local transform (body offset, then its joints in order) is a fixed-length product over joint slots;
the world transforms then take one batched composition per tree level. Mocap bodies keep their model
pose (``body_pos``, ``body_quat``).
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import torch

from ._math import axis_angle_to_mat, cross, mm, mv, quat_to_mat

# plain ints: MuJoCo 4's enums do not compare equal to numpy integers
_FREE, _BALL, _SLIDE, _HINGE = (
    int(mujoco.mjtJoint.mjJNT_FREE),
    int(mujoco.mjtJoint.mjJNT_BALL),
    int(mujoco.mjtJoint.mjJNT_SLIDE),
    int(mujoco.mjtJoint.mjJNT_HINGE),
)


def _quat2mat(q: np.ndarray) -> np.ndarray:
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=np.float64))
    return R.reshape(3, 3)


@dataclass
class KinematicsState:
    """World poses of every body, and per-joint frames needed for the DOF axes.

    Attributes:
        xpos: body positions, (..., nbody, 3).
        xmat: body rotations, (..., nbody, 3, 3).
        jnt_xanchor: joint anchors, (..., njnt, 3) (``xanchor``; the body position for free joints).
        jnt_xaxis: joint axes in the world frame, (..., njnt, 3) (``xaxis``).
    """

    xpos: torch.Tensor
    xmat: torch.Tensor
    jnt_xanchor: torch.Tensor
    jnt_xaxis: torch.Tensor


class Kinematics:
    """Forward kinematics of every body of a compiled ``mujoco.MjModel`` from ``qpos``.

    Args:
        m: compiled model.
        device: torch device of the constant tables and outputs.
        dtype: floating point type, ``torch.float64`` or ``torch.float32``.
    """

    def __init__(self, m: mujoco.MjModel, device: torch.device | str | None = None, dtype: torch.dtype = torch.float64):
        self.nq, self.nv, self.nbody, self.njnt = m.nq, m.nv, m.nbody, m.njnt
        self.device, self.dtype = device, dtype

        def t(x, **kw) -> torch.Tensor:
            return torch.as_tensor(np.asarray(x), device=device, **kw)

        def f(x) -> torch.Tensor:
            return t(np.asarray(x, dtype=np.float64), dtype=dtype)

        def i(x) -> torch.Tensor:
            return t(np.asarray(x, dtype=np.int64).reshape(-1))

        jtype = [int(x) for x in m.jnt_type]
        groups = {k: [j for j in range(m.njnt) if jtype[j] == k] for k in (_HINGE, _SLIDE, _BALL, _FREE)}
        self.hinge, self.slide, self.ball, self.free = (i(groups[k]) for k in (_HINGE, _SLIDE, _BALL, _FREE))
        qadr = m.jnt_qposadr
        self.h_qadr, self.h_q0 = i(qadr[groups[_HINGE]]), f(m.qpos0[qadr[groups[_HINGE]]])
        self.h_axis, self.h_pos = f(m.jnt_axis[groups[_HINGE]].reshape(-1, 3)), f(m.jnt_pos[groups[_HINGE]].reshape(-1, 3))
        self.s_qadr, self.s_q0 = i(qadr[groups[_SLIDE]]), f(m.qpos0[qadr[groups[_SLIDE]]])
        self.s_axis = f(m.jnt_axis[groups[_SLIDE]].reshape(-1, 3))
        self.b_qadr = i(qadr[groups[_BALL]].reshape(-1, 1) + np.arange(4)).reshape(-1, 4)
        self.b_pos = f(m.jnt_pos[groups[_BALL]].reshape(-1, 3))
        self.f_qadr = i(qadr[groups[_FREE]].reshape(-1, 1) + np.arange(7)).reshape(-1, 7)
        # position of every joint in cat(hinge, slide, ball, free, identity)
        order = groups[_HINGE] + groups[_SLIDE] + groups[_BALL] + groups[_FREE]
        slot_of = {j: k for k, j in enumerate(order)}
        ident = len(order)

        # body local transforms: offset (identity for free bodies), then up to K joints
        nb = m.nbody - 1
        self.K = int(m.body_jntnum[1:].max()) if nb else 0
        jslot = np.full((nb, self.K), ident, dtype=np.int64)
        offR, offp = np.zeros((nb, 3, 3)), np.zeros((nb, 3))
        jnt_body_slot = np.zeros((m.njnt, 2), dtype=np.int64)  # (local body, slot) of every joint
        for b in range(1, m.nbody):
            adr, num = m.body_jntadr[b], m.body_jntnum[b]
            if num and jtype[adr] == _FREE:
                offR[b - 1] = np.eye(3)
            else:
                offR[b - 1], offp[b - 1] = _quat2mat(m.body_quat[b]), m.body_pos[b]
            for k in range(num):
                jslot[b - 1, k] = slot_of[adr + k]
                jnt_body_slot[adr + k] = (b - 1, k)
        self.jslot, self.offR, self.offp = i(jslot).reshape(nb, self.K), f(offR), f(offp)
        # joint frames before the joint: parent world frame composed with the body's partial product
        self.jnt_parent = i(m.body_parentid[m.jnt_bodyid])
        self.jnt_pre = i(jnt_body_slot[:, 1] * nb + jnt_body_slot[:, 0])
        self.jnt_axis, self.jnt_pos = f(m.jnt_axis.reshape(-1, 3)), f(m.jnt_pos.reshape(-1, 3))
        self.jnt_body = i(m.jnt_bodyid)
        self.jnt_is_free = t(np.array(jtype) == _FREE, dtype=torch.bool)

        # tree levels: bodies of each depth, their parents' positions in the previous level
        depth = np.zeros(m.nbody, dtype=np.int64)
        for b in range(1, m.nbody):
            depth[b] = depth[m.body_parentid[b]] + 1
        self.levels: list[tuple[torch.Tensor, torch.Tensor]] = []
        pos_in_level = {0: 0}
        flat_order = [0]
        for d in range(1, int(depth.max()) + 1 if nb else 1):
            bodies = [b for b in range(1, m.nbody) if depth[b] == d]
            for k, b in enumerate(bodies):
                pos_in_level[b] = k
            self.levels.append((i([b - 1 for b in bodies]), i([pos_in_level[m.body_parentid[b]] for b in bodies])))
            flat_order += bodies
        self.body_perm = i(np.argsort(np.asarray(flat_order)))

        # DOF axes: omega = W[widx], v0 = anchor[aidx] x omega + T[tidx]
        # W = cat(jnt_xaxis, body columns (b, k), 0); anchor = cat(jnt_xanchor, 0); T = cat(jnt_xaxis, eye, 0)
        widx, aidx, tidx = [], [], []
        nj, ncol = m.njnt, 3 * m.nbody
        for j in range(m.njnt):
            b, typ = m.jnt_bodyid[j], jtype[j]
            if typ == _HINGE:
                widx.append(j), aidx.append(j), tidx.append(nj + 3)
            elif typ == _SLIDE:
                widx.append(nj + ncol), aidx.append(nj), tidx.append(j)
            else:
                if typ == _FREE:
                    for k in range(3):
                        widx.append(nj + ncol), aidx.append(nj), tidx.append(nj + k)
                for k in range(3):
                    widx.append(nj + 3 * b + k), aidx.append(j), tidx.append(nj + 3)
        assert len(widx) == m.nv
        self.dof_widx, self.dof_aidx, self.dof_tidx = i(widx), i(aidx), i(tidx)
        self.eye = f(np.eye(3))

    def joint_transforms(self, qpos: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Local rotation and translation of every joint, in cat(hinge, slide, ball, free, identity) order."""
        lead = qpos.shape[:-1]
        Rs, ts = [], []
        if len(self.hinge):
            R = axis_angle_to_mat(self.h_axis, qpos[..., self.h_qadr] - self.h_q0)
            Rs.append(R), ts.append(self.h_pos - mv(R, self.h_pos))
        if len(self.slide):
            Rs.append(self.eye.expand(lead + (len(self.slide), 3, 3)))
            ts.append(self.s_axis * (qpos[..., self.s_qadr] - self.s_q0).unsqueeze(-1))
        if len(self.ball):
            R = quat_to_mat(qpos[..., self.b_qadr])
            Rs.append(R), ts.append(self.b_pos - mv(R, self.b_pos))
        if len(self.free):
            q = qpos[..., self.f_qadr]
            Rs.append(quat_to_mat(q[..., 3:])), ts.append(q[..., :3])
        Rs.append(self.eye.expand(lead + (1, 3, 3)))
        ts.append(qpos.new_zeros(lead + (1, 3)))
        return torch.cat(Rs, -3), torch.cat(ts, -2)

    def forward(self, qpos: torch.Tensor) -> KinematicsState:
        """World poses of every body and joint frame from ``qpos`` of shape (..., nq)."""
        lead = qpos.shape[:-1]
        JR, Jt = self.joint_transforms(qpos)
        R = self.offR.expand(lead + self.offR.shape)
        p = self.offp.expand(lead + self.offp.shape)
        preR, prep = [], []
        for k in range(self.K):
            preR.append(R), prep.append(p)
            idx = self.jslot[:, k]
            p = p + mv(R, Jt[..., idx, :])
            R = mm(R, JR[..., idx, :, :])
        # world transforms, one tree level at a time
        wR = [self.eye.expand(lead + (1, 3, 3))]
        wp = [qpos.new_zeros(lead + (1, 3))]
        for bodies, parents in self.levels:
            Rp, pp = wR[-1][..., parents, :, :], wp[-1][..., parents, :]
            wp.append(pp + mv(Rp, p[..., bodies, :]))
            wR.append(mm(Rp, R[..., bodies, :, :]))
        xmat = torch.cat(wR, -3)[..., self.body_perm, :, :]
        xpos = torch.cat(wp, -2)[..., self.body_perm, :]
        # joint anchors and axes from the frame just before each joint
        if self.K:
            preR_all = torch.cat(preR, -3)
            prep_all = torch.cat(prep, -2)
            Rpar, ppar = xmat[..., self.jnt_parent, :, :], xpos[..., self.jnt_parent, :]
            Rb = mm(Rpar, preR_all[..., self.jnt_pre, :, :])
            pb = ppar + mv(Rpar, prep_all[..., self.jnt_pre, :])
            xaxis = mv(Rb, self.jnt_axis)
            xanchor = pb + mv(Rb, self.jnt_pos)
            xanchor = torch.where(self.jnt_is_free.unsqueeze(-1), xpos[..., self.jnt_body, :], xanchor)
        else:
            xaxis = xanchor = qpos.new_zeros(lead + (0, 3))
        return KinematicsState(xpos=xpos, xmat=xmat, jnt_xanchor=xanchor, jnt_xaxis=xaxis)

    def dof_axes(self, s: KinematicsState) -> tuple[torch.Tensor, torch.Tensor]:
        """Motion axis of every DOF in the world frame: angular part and linear velocity at the world origin.

        The velocity of a point x of a body moved by DOF k at unit speed is ``v0[k] + omega[k] x x``; this
        is MuJoCo's ``cdof`` shifted from the subtree centre of mass to the world origin.

        Returns:
            omega and v0, (..., nv, 3) each.
        """
        lead = s.xpos.shape[:-2]
        zero = s.xpos.new_zeros(lead + (1, 3))
        cols = s.xmat.transpose(-1, -2).reshape(lead + (3 * self.nbody, 3))
        W = torch.cat((s.jnt_xaxis, cols, zero), -2)
        A = torch.cat((s.jnt_xanchor, zero), -2)
        T = torch.cat((s.jnt_xaxis, self.eye.expand(lead + (3, 3)), zero), -2)
        omega = W[..., self.dof_widx, :]
        v0 = cross(A[..., self.dof_aidx, :], omega) + T[..., self.dof_tidx, :]
        return omega, v0

    def frame_pose(
        self, s: KinematicsState, body: torch.Tensor, pos: torch.Tensor, rot: torch.Tensor | None = None
    ) -> tuple[torch.Tensor, torch.Tensor | None]:
        """World pose of frames fixed to bodies (``mj_local2Global``), e.g. sites or geoms.

        Args:
            s: kinematics state.
            body: body id of each frame, (n,).
            pos: local position of each frame, (n, 3).
            rot: optional local rotation of each frame, (n, 3, 3).

        Returns:
            World positions (..., n, 3) and, if ``rot`` is given, rotations (..., n, 3, 3).
        """
        R = s.xmat[..., body, :, :]
        x = s.xpos[..., body, :] + mv(R, pos)
        return x, (mm(R, rot) if rot is not None else None)


def site_tables(
    m: mujoco.MjModel, ids: list[int] | np.ndarray, device: torch.device | str | None, dtype: torch.dtype
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Body ids, local positions and local rotations of the given sites, as tensors."""
    ids = np.asarray(ids, dtype=np.int64).reshape(-1)
    R = np.array([_quat2mat(m.site_quat[s]) for s in ids]).reshape(-1, 3, 3)
    return (
        torch.as_tensor(m.site_bodyid[ids].astype(np.int64), device=device),
        torch.as_tensor(m.site_pos[ids].reshape(-1, 3), device=device, dtype=dtype),
        torch.as_tensor(R, device=device, dtype=dtype),
    )


def geom_tables(
    m: mujoco.MjModel, ids: list[int] | np.ndarray, device: torch.device | str | None, dtype: torch.dtype
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Body ids, local positions and local rotations of the given geoms, as tensors."""
    ids = np.asarray(ids, dtype=np.int64).reshape(-1)
    R = np.array([_quat2mat(m.geom_quat[g]) for g in ids]).reshape(-1, 3, 3)
    return (
        torch.as_tensor(m.geom_bodyid[ids].astype(np.int64), device=device),
        torch.as_tensor(m.geom_pos[ids].reshape(-1, 3), device=device, dtype=dtype),
        torch.as_tensor(R, device=device, dtype=dtype),
    )
