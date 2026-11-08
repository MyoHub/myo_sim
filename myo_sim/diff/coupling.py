"""Joint couplings (``mjEQ_JOINT`` equalities) and effective moment arms in torch.

A joint equality makes joint 1 a polynomial of joint 2,
``q1 = qpos0_1 + c0 + c1 d + c2 d^2 + c3 d^3 + c4 d^4`` with ``d = q2 - qpos0_2``, or holds joint 1 at
``qpos0_1 + c0`` when it names no joint 2. The joints that no active equality drives are the independent
coordinates; their DOFs, in model order, are the independent DOFs.

Effective moment arms convention: with the couplings satisfied, a muscle's moment arm about an independent
DOF j is the derivative of its length along j with every coupled joint following,
``r_eff[a, j] = r[a, j] + sum_c r[a, c] dq_c/dq_j``, where ``r`` is MuJoCo's ``actuator_moment``. It equals
the generalised force the muscle exerts on j per unit tension (virtual work), so
``qfrc_eff = r_eff^T force``. Held joints contribute nothing.
"""

from __future__ import annotations

import mujoco
import numpy as np
import torch

_HINGE, _SLIDE = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)


class JointCouplings:
    """The active joint equalities of a compiled model.

    Args:
        m: compiled model.
        device, dtype: of the constant tables and outputs.
    """

    def __init__(self, m: mujoco.MjModel, device=None, dtype: torch.dtype = torch.float64):
        coupled: dict[int, tuple[int, np.ndarray]] = {}
        for e in range(m.neq):
            if int(m.eq_type[e]) != int(mujoco.mjtEq.mjEQ_JOINT) or not m.eq_active0[e]:
                continue
            j1, j2 = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
            if j1 in coupled:
                raise NotImplementedError(f"joint {m.joint(j1).name} is driven by two equalities")
            coupled[j1] = (j2, m.eq_data[e, :5].astype(np.float64).copy())
        for j1, (j2, _) in coupled.items():
            if j2 in coupled:
                raise NotImplementedError(f"joint {m.joint(j1).name} is coupled to the coupled joint {m.joint(j2).name}")
            for j in (j1, j2):
                if j >= 0 and int(m.jnt_type[j]) not in (_HINGE, _SLIDE):
                    raise NotImplementedError(f"coupling of joint {m.joint(j).name}: only hinge and slide joints")
        self.coupled = coupled
        cdofs = {int(m.jnt_dofadr[j]) for j in coupled}
        cqpos = {int(m.jnt_qposadr[j]) for j in coupled}
        self.indep_dofs = [k for k in range(m.nv) if k not in cdofs]
        self.indep_qpos = [k for k in range(m.nq) if k not in cqpos]
        self.nv, self.nq, self.nind = m.nv, m.nq, len(self.indep_dofs)
        self.dof_names = [m.joint(int(m.dof_jntid[k])).name for k in self.indep_dofs]
        col_of_dof = {k: i for i, k in enumerate(self.indep_dofs)}
        col_of_qpos = {k: i for i, k in enumerate(self.indep_qpos)}

        def it(x) -> torch.Tensor:
            return torch.as_tensor(np.asarray(x, dtype=np.int64).reshape(-1), device=device)

        def ft(x) -> torch.Tensor:
            return torch.as_tensor(np.asarray(x, dtype=np.float64), device=device, dtype=dtype)

        order = sorted(coupled)
        self.nc = len(order)
        src = [coupled[j][0] for j in order]
        held = np.array([s < 0 for s in src], dtype=bool)
        self.c_held = torch.as_tensor(held, device=device)
        # source column in the independent coordinates (0 for held joints, masked)
        self.c_src_q = it([0 if s < 0 else col_of_qpos[int(m.jnt_qposadr[s])] for s in src])
        self.c_src_v = it([0 if s < 0 else col_of_dof[int(m.jnt_dofadr[s])] for s in src])
        self.c_src_q0 = ft([0.0 if s < 0 else m.qpos0[m.jnt_qposadr[s]] for s in src])
        self.c_src_qadr = it([0 if s < 0 else m.jnt_qposadr[s] for s in src])
        self.c_q0 = ft([m.qpos0[m.jnt_qposadr[j]] for j in order])
        coef = np.array([coupled[j][1] for j in order]).reshape(-1, 5)
        coef[held, 1:] = 0.0  # MuJoCo ignores the higher coefficients without joint 2
        self.c_coef = ft(coef)
        self.c_qadr, self.c_dof = it([m.jnt_qposadr[j] for j in order]), it([m.jnt_dofadr[j] for j in order])
        self.ind_qadr, self.ind_dof = it(self.indep_qpos), it(self.indep_dofs)
        # every DOF as (independent column, coupling index or -1)
        self.dof_col = it([col_of_dof.get(k, 0) for k in range(m.nv)])
        cidx = {int(m.jnt_dofadr[j]): i for i, j in enumerate(order)}
        self.dof_cpl = it([cidx.get(k, -1) for k in range(m.nv)])
        for k in range(m.nv):
            if k in cidx:
                self.dof_col[k] = self.c_src_v[cidx[k]]

    def _poly(self, d: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        c = self.c_coef
        val = c[:, 0] + d * (c[:, 1] + d * (c[:, 2] + d * (c[:, 3] + d * c[:, 4])))
        der = c[:, 1] + d * (2 * c[:, 2] + d * (3 * c[:, 3] + d * 4 * c[:, 4]))
        return val, der

    def slopes(self, qpos: torch.Tensor) -> torch.Tensor:
        """dq_c/dq_source of every coupled joint at ``qpos`` (..., nq), (..., ncoupled); 0 for held joints."""
        d = qpos[..., self.c_src_qadr] - self.c_src_q0
        _, der = self._poly(d)
        return torch.where(self.c_held, torch.zeros_like(der), der)

    def expand(self, qpos_ind: torch.Tensor, qvel_ind: torch.Tensor | None = None) -> tuple[torch.Tensor, torch.Tensor | None]:
        """Full ``qpos`` (..., nq) and ``qvel`` (..., nv) from the independent coordinates and velocities."""
        lead = qpos_ind.shape[:-1]
        d = qpos_ind[..., self.c_src_q] - self.c_src_q0
        val, der = self._poly(d)
        val = torch.where(self.c_held, self.c_coef[:, 0], val)
        qpos = qpos_ind.new_zeros(lead + (self.nq,))
        qpos = qpos.index_copy(-1, self.ind_qadr, qpos_ind).index_copy(-1, self.c_qadr, self.c_q0 + val)
        if qvel_ind is None:
            return qpos, None
        dq = torch.where(self.c_held, torch.zeros_like(der), der * qvel_ind[..., self.c_src_v])
        qvel = qvel_ind.new_zeros(lead + (self.nv,))
        qvel = qvel.index_copy(-1, self.ind_dof, qvel_ind).index_copy(-1, self.c_dof, dq)
        return qpos, qvel

    def effective(
        self, values: torch.Tensor, rows: torch.Tensor, cols: torch.Tensor, nrows: int, qpos: torch.Tensor
    ) -> torch.Tensor:
        """Map sparse per-DOF values (moment arms or generalised forces) to the independent DOFs.

        Args:
            values: (..., nnz) values at (rows, cols), e.g. ``actuator_moment``.
            rows, cols: row and DOF of each value, (nnz,).
            nrows: number of rows of the dense result.
            qpos: joint positions (..., nq), where the coupling slopes are evaluated.

        Returns:
            Dense (..., nrows, nind) values about the independent DOFs, in ``indep_dofs`` order.
        """
        lead = values.shape[:-1]
        cpl = self.dof_cpl[cols]
        if self.nc:
            s = self.slopes(qpos)[..., cpl.clamp(min=0)]
            factor = torch.where(cpl >= 0, s, torch.ones_like(s))
        else:
            factor = torch.ones_like(values)
        flat = values.new_zeros(lead + (nrows * self.nind,))
        flat = flat.index_add(-1, rows * self.nind + self.dof_col[cols], values * factor)
        return flat.reshape(lead + (nrows, self.nind))
