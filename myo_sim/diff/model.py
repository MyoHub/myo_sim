"""The differentiable muscle-tendon model: kinematics, tendons, actuators and couplings of one MjModel."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import torch

from .actuator import Actuators
from .coupling import JointCouplings
from .kinematics import Kinematics, KinematicsState
from .tendon import Tendons
from .wrap import SIDE_RULES


def default_side_rule() -> str:
    """Side-site rule of the installed MuJoCo: ``"mujoco4"`` from MuJoCo 4.0 on, ``"mujoco3"`` before."""
    return "mujoco4" if int(mujoco.__version__.split(".")[0]) >= 4 else "mujoco3"


@dataclass
class MuscleTendonState:
    """Outputs of :meth:`MuscleTendonModel.forward`; names follow ``mjData``.

    Attributes:
        kinematics: body poses and joint frames.
        ten_length, ten_J, ten_velocity: tendon lengths (..., ntendon), sparse Jacobian (..., nJ)
            and velocities (..., ntendon).
        actuator_length, actuator_moment, actuator_velocity, actuator_force: (..., nu), sparse moments
            (..., nM), (..., nu) and (..., nu).
        qfrc_per_actuator: generalised force of each actuator on each DOF, sparse like the moments (..., nM).
        qfrc_actuator: generalised actuator force per DOF, (..., nv).
        wrapped: whether each site-geom-site wrap is active, (..., ntriple), in ``tendons`` order
            (``tendons.tri_ten`` gives the tendon of each).
    """

    kinematics: KinematicsState
    ten_length: torch.Tensor
    ten_J: torch.Tensor
    ten_velocity: torch.Tensor | None
    actuator_length: torch.Tensor
    actuator_moment: torch.Tensor
    actuator_velocity: torch.Tensor | None
    actuator_force: torch.Tensor | None
    qfrc_per_actuator: torch.Tensor | None
    qfrc_actuator: torch.Tensor | None
    wrapped: torch.Tensor


class MuscleTendonModel:
    """Differentiable, batched replica of MuJoCo's tendons, muscles and activation dynamics.

    Built from a compiled ``mujoco.MjModel`` (any myo_sim model or composition, or any model within the
    supported feature set); every input has leading batch dimensions, e.g. ``qpos`` (..., nq).

    Args:
        m: compiled model.
        device: torch device of the tables and outputs.
        dtype: ``torch.float64`` (parity with MuJoCo) or ``torch.float32``.
        side_rule: ``"mujoco3"`` or ``"mujoco4"``; defaults to the installed MuJoCo's rule.

    Example:
        >>> model = MuscleTendonModel(myo_sim.load_model("myoarm_r"))
        >>> out = model.forward(qpos, qvel, act)
        >>> out.actuator_force, model.dense_moment(out.actuator_moment)
    """

    def __init__(
        self,
        m: mujoco.MjModel,
        device: torch.device | str | None = None,
        dtype: torch.dtype = torch.float64,
        side_rule: str | None = None,
    ):
        side_rule = side_rule or default_side_rule()
        if side_rule not in SIDE_RULES:
            raise ValueError(f"side_rule must be one of {SIDE_RULES}, got {side_rule!r}")
        if m.njnt and (m.jnt_actgravcomp.any() if hasattr(m, "jnt_actgravcomp") else False):
            raise NotImplementedError("actuator gravity compensation (jnt_actgravcomp)")
        self.side_rule, self.device, self.dtype = side_rule, device, dtype
        self.nq, self.nv, self.nu, self.na, self.ntendon = m.nq, m.nv, m.nu, m.na, m.ntendon
        self.kinematics = Kinematics(m, device, dtype)
        self.tendons = Tendons(m, self.kinematics, side_rule)
        self.actuators = Actuators(m, (self.tendons.rownnz, self.tendons.rowadr, self.tendons.colind), device, dtype)
        self.couplings = JointCouplings(m, device, dtype)
        self.actuator_names = [m.actuator(a).name for a in range(m.nu)]
        self.tendon_names = [m.tendon(t).name for t in range(m.ntendon)]
        # joint actuator force limits (hinge and slide joints), applied to qfrc_actuator like MuJoCo
        lim = [int(m.jnt_dofadr[j]) for j in range(m.njnt) if m.jnt_actfrclimited[j]]
        for j in range(m.njnt):
            if m.jnt_actfrclimited[j] and int(m.jnt_type[j]) not in (
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            ):
                raise NotImplementedError(f"actuator force limit on joint {m.joint(j).name}: only hinge and slide joints")
        rng = np.array([m.jnt_actfrcrange[m.dof_jntid[k]] for k in lim]).reshape(-1, 2)
        self.lim_dof = torch.as_tensor(np.asarray(lim, dtype=np.int64), device=device)
        self.lim_lo = torch.as_tensor(rng[:, 0], device=device, dtype=dtype)
        self.lim_hi = torch.as_tensor(rng[:, 1], device=device, dtype=dtype)

    def forward(
        self,
        qpos: torch.Tensor,
        qvel: torch.Tensor | None = None,
        act: torch.Tensor | None = None,
        ctrl: torch.Tensor | None = None,
    ) -> MuscleTendonState:
        """Tendon and actuator quantities at a batch of states, as ``mj_forward`` computes them.

        Args:
            qpos: joint positions, (..., nq).
            qvel: joint velocities, (..., nv); needed for velocities and forces.
            act: activations, (..., na); needed for forces of actuators with dynamics.
            ctrl: controls, (..., nu); needed for stateless actuators and ``actearly``.
        """
        s = self.kinematics.forward(qpos)
        ts = self.tendons.forward(s, qpos, qvel)
        a = self.actuators.forward(ts.length, ts.J, qpos, qvel, act, ctrl)
        per = total = None
        if a.force is not None:
            per, total = self.actuators.qfrc(a.moment, a.force)
            if self.lim_dof.numel():
                v = total[..., self.lim_dof]
                total = total.index_copy(-1, self.lim_dof, torch.minimum(torch.maximum(v, self.lim_lo), self.lim_hi))
        return MuscleTendonState(
            kinematics=s,
            ten_length=ts.length,
            ten_J=ts.J,
            ten_velocity=ts.velocity,
            actuator_length=a.length,
            actuator_moment=a.moment,
            actuator_velocity=a.velocity,
            actuator_force=a.force,
            qfrc_per_actuator=per,
            qfrc_actuator=total,
            wrapped=ts.wrapped,
        )

    __call__ = forward

    def act_dot(self, ctrl: torch.Tensor, act: torch.Tensor) -> torch.Tensor:
        """Activation derivatives (..., na) (``mj_fwdActuation``'s ``act_dot``)."""
        return self.actuators.act_dot(ctrl, act)

    def next_act(self, act: torch.Tensor, act_dot: torch.Tensor) -> torch.Tensor:
        """Activations after one timestep (``mj_nextActivation``)."""
        return self.actuators.next_act(act, act_dot)

    def step_act(self, act: torch.Tensor, ctrl: torch.Tensor, nsteps: int = 1) -> tuple[torch.Tensor, torch.Tensor]:
        """Activations after ``nsteps`` timesteps under constant controls, and their mean as seen by the forces."""
        return self.actuators.step(act, ctrl, nsteps)

    def dense_ten_J(self, ten_J: torch.Tensor) -> torch.Tensor:
        """Dense tendon Jacobian (..., ntendon, nv)."""
        return self.tendons.dense_J(ten_J)

    def dense_moment(self, moment: torch.Tensor) -> torch.Tensor:
        """Dense actuator moments (..., nu, nv)."""
        return self.actuators.dense_moment(moment)

    def dense_qfrc_per_actuator(self, qfrc_per_actuator: torch.Tensor) -> torch.Tensor:
        """Dense generalised force of each actuator on each DOF, (..., nu, nv)."""
        return self.actuators.dense_moment(qfrc_per_actuator)

    def effective_moment(self, moment: torch.Tensor, qpos: torch.Tensor) -> torch.Tensor:
        """Moment arms about the independent DOFs through the joint couplings, (..., nu, nind).

        See :mod:`myo_sim.diff.coupling` for the convention; columns follow ``couplings.indep_dofs``.
        """
        act = self.actuators
        return self.couplings.effective(moment, act.m_row, act.m_col, self.nu, qpos)

    def effective_qfrc(self, qfrc_per_actuator: torch.Tensor, qpos: torch.Tensor) -> torch.Tensor:
        """Generalised force of each actuator on each independent DOF, (..., nu, nind)."""
        act = self.actuators
        return self.couplings.effective(qfrc_per_actuator, act.m_row, act.m_col, self.nu, qpos)
