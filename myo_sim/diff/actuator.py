"""Actuator lengths, moments, forces and activation dynamics of a compiled MuJoCo model in torch.

Muscle curves (``mju_muscleGain``, ``mju_muscleBias``) and the activation dynamics (``mju_muscleDynamics``
with the ``tausmooth`` sigmoid) are ported from muscle-supermimic (github.com/kaibohex/muscle-supermimic,
Apache-2.0, commit 8a6244b, ``src/muscle_supermimic/tasks/tracking/mdp/muscle_geometry.py`` and
``muscle.py``) and generalised to per-actuator parameters, the ``force < 0`` scaling by ``acc0``, fixed
and affine gains and biases, the other activation dynamics, ``actearly`` and the control, activation and
force limits, following ``mj_fwdActuation`` and ``mj_nextActivation``.

Supported: tendon transmissions and joint transmissions on hinge and slide joints; gain types fixed,
affine and muscle; bias types none, affine and muscle; dynamics none, integrator, filter, filterexact
and muscle, with at most one activation per actuator.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import torch

from ._math import MINVAL

_TRN_JOINT, _TRN_TENDON = int(mujoco.mjtTrn.mjTRN_JOINT), int(mujoco.mjtTrn.mjTRN_TENDON)
_GAIN_FIXED, _GAIN_AFFINE, _GAIN_MUSCLE = (
    int(x) for x in (mujoco.mjtGain.mjGAIN_FIXED, mujoco.mjtGain.mjGAIN_AFFINE, mujoco.mjtGain.mjGAIN_MUSCLE)
)
_BIAS_NONE, _BIAS_AFFINE, _BIAS_MUSCLE = (
    int(x) for x in (mujoco.mjtBias.mjBIAS_NONE, mujoco.mjtBias.mjBIAS_AFFINE, mujoco.mjtBias.mjBIAS_MUSCLE)
)
_DYN = {
    k: int(getattr(mujoco.mjtDyn, k))
    for k in ("mjDYN_NONE", "mjDYN_INTEGRATOR", "mjDYN_FILTER", "mjDYN_FILTEREXACT", "mjDYN_MUSCLE")
}
_HINGE, _SLIDE = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)


def muscle_gain_length(L: torch.Tensor, lmin: torch.Tensor, lmax: torch.Tensor) -> torch.Tensor:
    """Active force-length curve (``mju_muscleGainLength``)."""
    a, b = 0.5 * (lmin + 1), 0.5 * (1 + lmax)
    x1 = (L - lmin) / (a - lmin).clamp(min=MINVAL)
    x2 = (1 - L) / (1 - a).clamp(min=MINVAL)
    x3 = (L - 1) / (b - 1).clamp(min=MINVAL)
    x4 = (lmax - L) / (lmax - b).clamp(min=MINVAL)
    out = torch.where(
        L <= a, 0.5 * x1 * x1, torch.where(L <= 1, 1 - 0.5 * x2 * x2, torch.where(L <= b, 1 - 0.5 * x3 * x3, 0.5 * x4 * x4))
    )
    return torch.where((L < lmin) | (L > lmax), torch.zeros_like(L), out)


def muscle_gain(len_: torch.Tensor, vel: torch.Tensor, lr0, lr1, F, prm) -> torch.Tensor:
    """``mju_muscleGain``: -F FL(L) FV(V); ``prm`` are the gainprm columns, ``F`` the scaled force."""
    L0 = (lr1 - lr0) / (prm[1] - prm[0]).clamp(min=MINVAL)
    L = prm[0] + (len_ - lr0) / L0.clamp(min=MINVAL)
    V = vel / (L0 * prm[6]).clamp(min=MINVAL)
    FL = muscle_gain_length(L, prm[4], prm[5])
    fvmax = prm[8]
    y = fvmax - 1
    FV = torch.where(V <= -1, torch.zeros_like(V),
                     torch.where(V <= 0, (V + 1) ** 2,
                                 torch.where(V <= y, fvmax - (y - V) ** 2 / y.clamp(min=MINVAL), fvmax.expand_as(V))))  # fmt: skip
    return -F * FL * FV


def muscle_bias(len_: torch.Tensor, lr0, lr1, F, prm) -> torch.Tensor:
    """``mju_muscleBias``: passive force, half-quadratic to (1 + lmax) / 2 and linear beyond."""
    L0 = (lr1 - lr0) / (prm[1] - prm[0]).clamp(min=MINVAL)
    L = prm[0] + (len_ - lr0) / L0.clamp(min=MINVAL)
    b = 0.5 * (1 + prm[5])
    x = (L - 1) / (b - 1).clamp(min=MINVAL)
    xb = (L - b) / (b - 1).clamp(min=MINVAL)
    P = torch.where(L <= 1, torch.zeros_like(L), torch.where(L <= b, 0.5 * x * x, 0.5 + xb))
    return -F * prm[7] * P


def sigmoid(x: torch.Tensor) -> torch.Tensor:
    """``mju_sigmoid``: quintic smoothstep on [0, 1], 0 below and 1 above."""
    x = x.clamp(0.0, 1.0)
    return x * x * x * (3 * x * (2 * x - 5) + 10)


@dataclass
class ActuatorState:
    """Actuator outputs.

    Attributes:
        length: actuator lengths, (..., nu) (``actuator_length``).
        moment: actuator moments in sparse row layout, (..., nM) (``actuator_moment``).
        velocity: actuator velocities, (..., nu), or None without ``qvel``.
        force: actuator forces, (..., nu), or None without activations and controls.
    """

    length: torch.Tensor
    moment: torch.Tensor
    velocity: torch.Tensor | None
    force: torch.Tensor | None


class Actuators:
    """Lengths, moments, forces and activation dynamics of every actuator.

    Args:
        m: compiled model.
        ten_layout: (rownnz, rowadr, colind) of the tendon Jacobian.
        device, dtype: of the constant tables and outputs.
    """

    def __init__(self, m: mujoco.MjModel, ten_layout: tuple[np.ndarray, np.ndarray, np.ndarray], device, dtype: torch.dtype):
        self.nu, self.na, self.nv = m.nu, m.na, m.nv
        self.timestep = float(m.opt.timestep)
        rownnz, rowadr, colind = ten_layout

        def ft(x) -> torch.Tensor:
            return torch.as_tensor(np.asarray(x, dtype=np.float64), device=device, dtype=dtype)

        def it(x) -> torch.Tensor:
            return torch.as_tensor(np.asarray(x, dtype=np.int64).reshape(-1), device=device)

        if m.nu and (m.actuator_actnum > 1).any():
            raise NotImplementedError("actuators with more than one activation")
        if hasattr(m, "actuator_outnum") and m.nu and (m.actuator_outnum != 1).any():
            raise NotImplementedError("actuators with several outputs")
        if m.ntendon and m.tendon_actfrclimited.any():
            raise NotImplementedError("tendon actuator force limits (tendon_actfrclimited)")
        if hasattr(m, "actuator_delay") and m.nu and (m.actuator_delay != 0).any():
            raise NotImplementedError("delayed actuators")
        # moment layout and gather sources: J_ext = cat(ten_J, 1)
        nJ = int(np.sum(rownnz))
        src, scale, mrow, mcol, lsrc = [], [], [], [], []
        for a in range(m.nu):
            trn, tid, gear = int(m.actuator_trntype[a]), int(m.actuator_trnid[a, 0]), float(m.actuator_gear[a, 0])
            if trn == _TRN_TENDON:
                for k in range(rownnz[tid]):
                    src.append(rowadr[tid] + k), scale.append(gear), mrow.append(a), mcol.append(colind[rowadr[tid] + k])
                lsrc.append(tid)
            elif trn == _TRN_JOINT and int(m.jnt_type[tid]) in (_HINGE, _SLIDE):
                src.append(nJ), scale.append(gear), mrow.append(a), mcol.append(m.jnt_dofadr[tid])
                lsrc.append(m.ntendon + m.jnt_qposadr[tid])
            else:
                raise NotImplementedError(f"actuator {m.actuator(a).name}: transmission type {trn}")
        self.m_src, self.m_scale = it(src), ft(scale)
        self.m_row, self.m_col = it(mrow), it(mcol)
        self.nM = len(src)
        self.gear = ft(m.actuator_gear[:, 0])
        self.len_src = it(lsrc)  # index into cat(ten_length, qpos)

        # gains and biases
        gt, bt = m.actuator_gaintype.astype(int), m.actuator_biastype.astype(int)
        for a in range(m.nu):
            if gt[a] not in (_GAIN_FIXED, _GAIN_AFFINE, _GAIN_MUSCLE):
                raise NotImplementedError(f"actuator {m.actuator(a).name}: gain type {gt[a]}")
            if bt[a] not in (_BIAS_NONE, _BIAS_AFFINE, _BIAS_MUSCLE):
                raise NotImplementedError(f"actuator {m.actuator(a).name}: bias type {bt[a]}")
        self.g_kind, self.b_kind = it(gt), it(bt)
        self.gprm = [ft(m.actuator_gainprm[:, k]) for k in range(9)]
        self.bprm = [ft(m.actuator_biasprm[:, k]) for k in range(9)]
        acc0 = np.maximum(MINVAL, m.actuator_acc0)
        self.gF = ft(np.where(m.actuator_gainprm[:, 2] < 0, m.actuator_gainprm[:, 3] / acc0, m.actuator_gainprm[:, 2]))
        self.bF = ft(np.where(m.actuator_biasprm[:, 2] < 0, m.actuator_biasprm[:, 3] / acc0, m.actuator_biasprm[:, 2]))
        self.lr0, self.lr1 = ft(m.actuator_lengthrange[:, 0]), ft(m.actuator_lengthrange[:, 1])

        # dynamics: act index of each stateful actuator, else -1
        dt = m.actuator_dyntype.astype(int)
        for a in range(m.nu):
            if dt[a] not in _DYN.values():
                raise NotImplementedError(f"actuator {m.actuator(a).name}: dynamics type {dt[a]}")
        self.d_kind = it(dt)
        self.has_act = torch.as_tensor(m.actuator_actnum > 0, device=device)
        self.act_adr = it(np.maximum(m.actuator_actadr, 0))
        self.stateful = it(np.flatnonzero(m.actuator_actnum > 0))
        self.dprm = [ft(m.actuator_dynprm[:, k]) for k in range(3)]
        self.actearly = torch.as_tensor(m.actuator_actearly.astype(bool), device=device)
        self.any_actearly = bool(m.actuator_actearly.any())
        self.any_stateless = bool((m.actuator_actnum == 0).any())
        clampctrl = not (m.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL))
        self.ctrl_lim = torch.as_tensor(m.actuator_ctrllimited.astype(bool) & clampctrl, device=device)
        self.ctrl_lo, self.ctrl_hi = ft(m.actuator_ctrlrange[:, 0]), ft(m.actuator_ctrlrange[:, 1])
        self.act_lim = torch.as_tensor(m.actuator_actlimited.astype(bool), device=device)
        self.act_lo, self.act_hi = ft(m.actuator_actrange[:, 0]), ft(m.actuator_actrange[:, 1])
        self.frc_lim = torch.as_tensor(m.actuator_forcelimited.astype(bool), device=device)
        self.frc_lo, self.frc_hi = ft(m.actuator_forcerange[:, 0]), ft(m.actuator_forcerange[:, 1])

    def _per_actuator_act(self, act: torch.Tensor) -> torch.Tensor:
        """Activation of each actuator, (..., nu) (0 for stateless actuators)."""
        a = act[..., self.act_adr]
        return torch.where(self.has_act, a, torch.zeros_like(a))

    def _to_act(self, x: torch.Tensor) -> torch.Tensor:
        """(..., nu) per-actuator values of the stateful actuators to (..., na)."""
        return x[..., self.stateful]

    def clamp_ctrl(self, ctrl: torch.Tensor) -> torch.Tensor:
        """Controls clamped to ``ctrlrange`` where ``ctrllimited`` (as ``mj_fwdActuation``)."""
        return torch.where(self.ctrl_lim, torch.minimum(torch.maximum(ctrl, self.ctrl_lo), self.ctrl_hi), ctrl)

    def act_dot(self, ctrl: torch.Tensor, act: torch.Tensor) -> torch.Tensor:
        """Time derivative of the activations, (..., na), from controls (..., nu) and activations (..., na)."""
        u = self.clamp_ctrl(ctrl)
        a = self._per_actuator_act(act)
        k = self.d_kind
        # muscle: Millard et al. (2013) time constants, optional smooth switch of width tausmooth
        ac = a.clamp(0.0, 1.0)
        tau_act = self.dprm[0] * (0.5 + 1.5 * ac)
        tau_deact = self.dprm[1] / (0.5 + 1.5 * ac)
        dctrl = u.clamp(0.0, 1.0) - a
        w = self.dprm[2]
        smooth = tau_deact + (tau_act - tau_deact) * sigmoid(dctrl / w.clamp(min=MINVAL) + 0.5)
        tau = torch.where(w < MINVAL, torch.where(dctrl > 0, tau_act, tau_deact), smooth)
        muscle = dctrl / tau.clamp(min=MINVAL)
        filt = (u - a) / self.dprm[0].clamp(min=MINVAL)
        out = torch.where(k == _DYN["mjDYN_MUSCLE"], muscle,
                          torch.where(k == _DYN["mjDYN_INTEGRATOR"], u, filt))  # fmt: skip
        return self._to_act(out)

    def next_act(self, act: torch.Tensor, act_dot: torch.Tensor) -> torch.Tensor:
        """Activations after one timestep (``mj_nextActivation``): Euler, exact for filterexact, then clamped."""
        a = act
        k = self.d_kind[self.stateful]
        tau = self.dprm[0][self.stateful].clamp(min=MINVAL)
        exact = a + act_dot * tau * (1 - torch.exp(-self.timestep / tau))
        nxt = torch.where(k == _DYN["mjDYN_FILTEREXACT"], exact, a + act_dot * self.timestep)
        lim = self.act_lim[self.stateful]
        lo, hi = self.act_lo[self.stateful], self.act_hi[self.stateful]
        return torch.where(lim, torch.minimum(torch.maximum(nxt, lo), hi), nxt)

    def step(self, act: torch.Tensor, ctrl: torch.Tensor, nsteps: int = 1) -> tuple[torch.Tensor, torch.Tensor]:
        """Integrate the activations over ``nsteps`` timesteps under constant controls.

        Returns:
            The activations after the last step, and their mean over the steps as seen by the forces
            (the activation before each step, or after it for ``actearly`` actuators).
        """
        total = torch.zeros_like(act)
        for _ in range(nsteps):
            nxt = self.next_act(act, self.act_dot(ctrl, act))
            used = torch.where(self.actearly[self.stateful], nxt, act) if self.any_actearly else act
            total = total + used
            act = nxt
        return act, total / nsteps

    def forward(
        self,
        ten_length: torch.Tensor,
        ten_J: torch.Tensor,
        qpos: torch.Tensor,
        qvel: torch.Tensor | None = None,
        act: torch.Tensor | None = None,
        ctrl: torch.Tensor | None = None,
    ) -> ActuatorState:
        """Actuator lengths, moments, velocities and forces.

        Args:
            ten_length, ten_J: tendon lengths (..., ntendon) and sparse Jacobian (..., nJ).
            qpos, qvel: joint positions (..., nq) and velocities (..., nv).
            act: activations (..., na), for actuators with dynamics.
            ctrl: controls (..., nu), for actuators without dynamics and for ``actearly``.

        Returns:
            The actuator state; ``force`` needs ``qvel`` and the activations or controls the model uses.
        """
        lead = qpos.shape[:-1]
        Jx = torch.cat((ten_J, ten_J.new_ones(lead + (1,))), -1)
        moment = Jx[..., self.m_src] * self.m_scale
        src = torch.cat((ten_length, qpos), -1)[..., self.len_src]
        length = src * self.gear
        if qvel is None:
            return ActuatorState(length=length, moment=moment, velocity=None, force=None)
        velocity = qvel.new_zeros(lead + (self.nu,)).index_add(-1, self.m_row, moment * qvel[..., self.m_col])
        need_act = self.na > 0
        need_ctrl = self.any_stateless or self.any_actearly
        if (need_act and act is None) or (need_ctrl and ctrl is None):
            return ActuatorState(length=length, moment=moment, velocity=velocity, force=None)
        # gain and bias
        gm = muscle_gain(length, velocity, self.lr0, self.lr1, self.gF, self.gprm)
        ga = self.gprm[0] + self.gprm[1] * length + self.gprm[2] * velocity
        gain = torch.where(
            self.g_kind == _GAIN_MUSCLE, gm, torch.where(self.g_kind == _GAIN_AFFINE, ga, self.gprm[0].expand_as(gm))
        )
        bm = muscle_bias(length, self.lr0, self.lr1, self.bF, self.bprm)
        ba = self.bprm[0] + self.bprm[1] * length + self.bprm[2] * velocity
        bias = torch.where(self.b_kind == _BIAS_MUSCLE, bm, torch.where(self.b_kind == _BIAS_AFFINE, ba, torch.zeros_like(bm)))
        # input: control for stateless actuators, activation (or next activation if actearly) otherwise
        inp = self.clamp_ctrl(ctrl) if ctrl is not None else torch.zeros_like(length)
        if need_act:
            a = act
            if self.any_actearly:
                a = torch.where(self.actearly[self.stateful], self.next_act(act, self.act_dot(ctrl, act)), act)
            per = torch.zeros_like(length).index_copy(-1, self.stateful, a)
            inp = torch.where(self.has_act, per, inp)
        force = gain * inp + bias
        force = torch.where(self.frc_lim, torch.minimum(torch.maximum(force, self.frc_lo), self.frc_hi), force)
        return ActuatorState(length=length, moment=moment, velocity=velocity, force=force)

    def qfrc(self, moment: torch.Tensor, force: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Generalised forces: per actuator and DOF in the sparse moment layout (..., nM), and per DOF (..., nv)."""
        per = moment * force[..., self.m_row]
        lead = force.shape[:-1]
        return per, force.new_zeros(lead + (self.nv,)).index_add(-1, self.m_col, per)

    def dense_moment(self, moment: torch.Tensor) -> torch.Tensor:
        """Sparse actuator moments (..., nM) to dense (..., nu, nv) (``mju_sparse2dense``)."""
        lead = moment.shape[:-1]
        flat = moment.new_zeros(lead + (self.nu * self.nv,))
        return flat.index_add(-1, self.m_row * self.nv + self.m_col, moment).reshape(lead + (self.nu, self.nv))
