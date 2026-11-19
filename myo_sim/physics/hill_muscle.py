# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Portable Hill-type muscle dynamics, force curves, and joint-torque MjSpec compiler.

Implements MuJoCo's exact ``mjDYN_MUSCLE`` (Millard activation ODE), ``mjGAIN_MUSCLE``
(active force-length-velocity bump curve), and ``mjBIAS_MUSCLE`` (passive force-length
curve) in both NumPy and PyTorch (selected automatically from the input array type,
with ``torch`` imported lazily via :mod:`myo_sim.physics._optional`).

Also provides :func:`strip_muscles_for_joint_torque_sim`, which transforms a
musculoskeletal ``MjSpec`` into a tendon-free joint-torque ``MjSpec`` preserving
rigid bodies, inertias, collision geoms, sites, and ``mjEQ_JOINT`` polynomial
couplings so rigid-body engines without spatial-tendon muscle kernels (such as
``genesis-world``) can load ``myo_sim`` skeletons and receive ``J_muscle.T @ f_muscle``
generalized torques externally.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from myo_sim.physics._optional import require

MJ_MINVAL = 1e-15


def _is_torch(x: Any) -> bool:
    return type(x).__module__.startswith("torch")


def _mj_bump(L: Any, a: Any, mid: Any, b: Any) -> Any:
    """Piecewise-quadratic bump spline matching MuJoCo's ``mju_bump``."""
    left_mid = 0.5 * (a + mid)
    right_mid = 0.5 * (mid + b)
    if _is_torch(L):
        torch = require("torch", "torch")
        out = torch.zeros_like(L)
        t1 = (L - a) / (left_mid - a).clamp(min=MJ_MINVAL)
        t2 = (mid - L) / (mid - left_mid).clamp(min=MJ_MINVAL)
        t3 = (L - mid) / (right_mid - mid).clamp(min=MJ_MINVAL)
        t4 = (b - L) / (b - right_mid).clamp(min=MJ_MINVAL)
        out = torch.where((L > a) & (L <= left_mid), 0.5 * t1 * t1, out)
        out = torch.where((L > left_mid) & (L <= mid), 1.0 - 0.5 * t2 * t2, out)
        out = torch.where((L > mid) & (L <= right_mid), 1.0 - 0.5 * t3 * t3, out)
        out = torch.where((L > right_mid) & (L < b), 0.5 * t4 * t4, out)
        return out

    out = np.zeros_like(L, dtype=np.float64)
    t1 = (L - a) / np.maximum(MJ_MINVAL, left_mid - a)
    t2 = (mid - L) / np.maximum(MJ_MINVAL, mid - left_mid)
    t3 = (L - mid) / np.maximum(MJ_MINVAL, right_mid - mid)
    t4 = (b - L) / np.maximum(MJ_MINVAL, b - right_mid)
    out = np.where((L > a) & (L <= left_mid), 0.5 * t1 * t1, out)
    out = np.where((L > left_mid) & (L <= mid), 1.0 - 0.5 * t2 * t2, out)
    out = np.where((L > mid) & (L <= right_mid), 1.0 - 0.5 * t3 * t3, out)
    out = np.where((L > right_mid) & (L < b), 0.5 * t4 * t4, out)
    return out


@dataclass(frozen=True)
class MuscleParams:
    """Extracted per-muscle parameters from a compiled ``mujoco.MjModel``."""

    tau_act: np.ndarray
    tau_deact: np.ndarray
    tausmooth: np.ndarray
    act_limited: np.ndarray
    act_range: np.ndarray
    length_range: np.ndarray
    acc0: np.ndarray
    gain_prm: np.ndarray
    bias_prm: np.ndarray

    @classmethod
    def from_model(cls, model: mujoco.MjModel) -> MuscleParams:
        """Extract Hill muscle parameters for all ``mjDYN_MUSCLE`` / ``mjGAIN_MUSCLE`` actuators."""
        mask = (model.actuator_dyntype == mujoco.mjtDyn.mjDYN_MUSCLE) | (
            model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE
        )
        if not np.all(mask):
            raise ValueError("MuscleParams.from_model requires all model actuators to be Hill muscles.")
        return cls(
            tau_act=model.actuator_dynprm[:, 0].astype(np.float64).copy(),
            tau_deact=model.actuator_dynprm[:, 1].astype(np.float64).copy(),
            tausmooth=model.actuator_dynprm[:, 2].astype(np.float64).copy(),
            act_limited=model.actuator_actlimited.astype(bool).copy(),
            act_range=model.actuator_actrange.astype(np.float64).copy(),
            length_range=model.actuator_lengthrange.astype(np.float64).copy(),
            acc0=model.actuator_acc0.astype(np.float64).copy(),
            gain_prm=model.actuator_gainprm[:, :9].astype(np.float64).copy(),
            bias_prm=model.actuator_biasprm[:, :9].astype(np.float64).copy(),
        )


def muscle_activation_dot(ctrl: Any, act: Any, params: MuscleParams) -> Any:
    """Time derivative ``da/dt`` matching MuJoCo's ``mju_muscleDynamics``."""
    if _is_torch(act):
        torch = require("torch", "torch")
        dev, dt = act.device, act.dtype
        u = torch.as_tensor(ctrl, device=dev, dtype=dt).clamp(0.0, 1.0)
        a_clamp = act.clamp(0.0, 1.0)
        tau_a = torch.as_tensor(params.tau_act, device=dev, dtype=dt) * (0.5 + 1.5 * a_clamp)
        tau_d = torch.as_tensor(params.tau_deact, device=dev, dtype=dt) / (0.5 + 1.5 * a_clamp)
        smooth = torch.as_tensor(params.tausmooth, device=dev, dtype=dt)
        dctrl = u - act
        hard = smooth < MJ_MINVAL
        x = (dctrl / smooth.clamp(min=MJ_MINVAL) + 0.5).clamp(0.0, 1.0)
        sig = x * x * x * (3.0 * x * (2.0 * x - 5.0) + 10.0)
        tau_smooth = tau_d + (tau_a - tau_d) * sig
        tau_hard = torch.where(dctrl > 0.0, tau_a, tau_d)
        tau = torch.where(hard, tau_hard, tau_smooth)
        return dctrl / tau.clamp(min=MJ_MINVAL)

    u_np = np.clip(np.asarray(ctrl, dtype=np.float64), 0.0, 1.0)
    a_np = np.asarray(act, dtype=np.float64)
    a_clamp = np.clip(a_np, 0.0, 1.0)
    tau_a = params.tau_act * (0.5 + 1.5 * a_clamp)
    tau_d = params.tau_deact / (0.5 + 1.5 * a_clamp)
    dctrl = u_np - a_np
    hard = params.tausmooth < MJ_MINVAL
    x = np.clip(dctrl / np.maximum(MJ_MINVAL, params.tausmooth) + 0.5, 0.0, 1.0)
    sig = x * x * x * (3.0 * x * (2.0 * x - 5.0) + 10.0)
    tau = np.where(hard, np.where(dctrl > 0.0, tau_a, tau_d), tau_d + (tau_a - tau_d) * sig)
    return dctrl / np.maximum(MJ_MINVAL, tau)


def step_muscle_activation(act: Any, ctrl: Any, params: MuscleParams, dt: float, substeps: int = 1) -> tuple[Any, Any]:
    """Advance ``act`` over ``substeps`` Euler steps; return ``(next_act, mean_act)``."""
    if _is_torch(act):
        torch = require("torch", "torch")
        dev, dtype = act.device, act.dtype
        lo = torch.where(
            torch.as_tensor(params.act_limited, device=dev),
            torch.as_tensor(params.act_range[:, 0], device=dev, dtype=dtype),
            torch.full((len(params.tau_act),), -torch.inf, device=dev, dtype=dtype),
        )
        hi = torch.where(
            torch.as_tensor(params.act_limited, device=dev),
            torch.as_tensor(params.act_range[:, 1], device=dev, dtype=dtype),
            torch.full((len(params.tau_act),), torch.inf, device=dev, dtype=dtype),
        )
        cur = act
        total = torch.zeros_like(cur)
        limited = bool(params.act_limited.any())
        for _ in range(substeps):
            total = total + cur
            cur = cur + dt * muscle_activation_dot(ctrl, cur, params)
            if limited:
                cur = torch.maximum(torch.minimum(cur, hi), lo)
        return cur, total / substeps

    cur_np = np.asarray(act, dtype=np.float64).copy()
    total_np = np.zeros_like(cur_np)
    lo_np = np.where(params.act_limited, params.act_range[:, 0], -np.inf)
    hi_np = np.where(params.act_limited, params.act_range[:, 1], np.inf)
    limited = bool(params.act_limited.any())
    for _ in range(substeps):
        total_np = total_np + cur_np
        cur_np = cur_np + dt * muscle_activation_dot(ctrl, cur_np, params)
        if limited:
            cur_np = np.clip(cur_np, lo_np, hi_np)
    return cur_np, total_np / substeps


def muscle_gain(length: Any, velocity: Any, params: MuscleParams) -> Any:
    """Active Hill gain ``-F0 * f_L(L) * f_V(V)`` matching MuJoCo's ``mju_muscleGain``."""
    prm = params.gain_prm
    lr = params.length_range
    L0_np = (lr[:, 1] - lr[:, 0]) / np.maximum(1e-10, prm[:, 1] - prm[:, 0])
    F0_np = np.where(prm[:, 2] < 0.0, prm[:, 3] / np.maximum(1e-10, params.acc0), prm[:, 2])

    if _is_torch(length):
        torch = require("torch", "torch")
        dev, dt = length.device, length.dtype
        L0 = torch.as_tensor(L0_np, device=dev, dtype=dt)
        F0 = torch.as_tensor(F0_np, device=dev, dtype=dt)
        r0 = torch.as_tensor(prm[:, 0], device=dev, dtype=dt)
        lr0 = torch.as_tensor(lr[:, 0], device=dev, dtype=dt)
        lmin = torch.as_tensor(prm[:, 4], device=dev, dtype=dt)
        lmax = torch.as_tensor(prm[:, 5], device=dev, dtype=dt)
        vmax = torch.as_tensor(prm[:, 6], device=dev, dtype=dt)
        fvmax = torch.as_tensor(prm[:, 8], device=dev, dtype=dt)

        L = r0 + (length - lr0) / L0.clamp(min=1e-10)
        V = velocity / (L0 * vmax).clamp(min=1e-10)
        fl = _mj_bump(L, lmin, torch.ones_like(lmin), lmax)
        y = fvmax - 1.0
        fv = torch.where(
            V <= -1.0,
            torch.zeros_like(V),
            torch.where(
                V <= 0.0,
                (V + 1.0) ** 2,
                torch.where(V <= y, fvmax - (y - V) ** 2 / y.clamp(min=MJ_MINVAL), fvmax),
            ),
        )
        return -F0 * fl * fv

    len_np = np.asarray(length, dtype=np.float64)
    vel_np = np.asarray(velocity, dtype=np.float64)
    L = prm[:, 0] + (len_np - lr[:, 0]) / np.maximum(1e-10, L0_np)
    V = vel_np / np.maximum(1e-10, L0_np * prm[:, 6])
    fl = _mj_bump(L, prm[:, 4], 1.0, prm[:, 5])
    fvmax = prm[:, 8]
    y = fvmax - 1.0
    fv = np.where(
        V <= -1.0,
        0.0,
        np.where(V <= 0.0, (V + 1.0) ** 2, np.where(V <= y, fvmax - (y - V) ** 2 / np.maximum(MJ_MINVAL, y), fvmax)),
    )
    return -F0_np * fl * fv


def muscle_bias(length: Any, params: MuscleParams) -> Any:
    """Passive Hill bias ``-F0 * f_P(L)`` matching MuJoCo's ``mju_muscleBias``."""
    prm = params.bias_prm
    lr = params.length_range
    L0_np = (lr[:, 1] - lr[:, 0]) / np.maximum(1e-10, prm[:, 1] - prm[:, 0])
    F0_np = np.where(prm[:, 2] < 0.0, prm[:, 3] / np.maximum(1e-10, params.acc0), prm[:, 2])
    b_np = 0.5 * (1.0 + prm[:, 5])
    denom_np = np.maximum(MJ_MINVAL, b_np - 1.0)

    if _is_torch(length):
        torch = require("torch", "torch")
        dev, dt = length.device, length.dtype
        L0 = torch.as_tensor(L0_np, device=dev, dtype=dt)
        F0 = torch.as_tensor(F0_np, device=dev, dtype=dt)
        r0 = torch.as_tensor(prm[:, 0], device=dev, dtype=dt)
        lr0 = torch.as_tensor(lr[:, 0], device=dev, dtype=dt)
        fpmax = torch.as_tensor(prm[:, 7], device=dev, dtype=dt)
        b = torch.as_tensor(b_np, device=dev, dtype=dt)
        denom = torch.as_tensor(denom_np, device=dev, dtype=dt)

        L = r0 + (length - lr0) / L0.clamp(min=1e-10)
        fp = fpmax * torch.where(
            L <= 1.0,
            torch.zeros_like(L),
            torch.where(L <= b, 0.5 * ((L - 1.0) / denom) ** 2, 0.5 + (L - b) / denom),
        )
        return -F0 * fp

    len_np = np.asarray(length, dtype=np.float64)
    L = prm[:, 0] + (len_np - lr[:, 0]) / np.maximum(1e-10, L0_np)
    fp = prm[:, 7] * np.where(
        L <= 1.0,
        0.0,
        np.where(L <= b_np, 0.5 * ((L - 1.0) / denom_np) ** 2, 0.5 + (L - b_np) / denom_np),
    )
    return -F0_np * fp


def muscle_actuator_force(length: Any, velocity: Any, act: Any, params: MuscleParams) -> Any:
    """Total scalar actuator force ``gain(L, V) * act + bias(L)`` (negative = pulling)."""
    return muscle_gain(length, velocity, params) * act + muscle_bias(length, params)


def dense_actuator_moment(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Return dense ``(nu, nv)`` moment-arm Jacobian across sparse and dense MuJoCo builds."""
    J = np.zeros((model.nu, model.nv), dtype=np.float64)
    if hasattr(data, "moment_rownnz"):
        mujoco.mju_sparse2dense(
            J,
            data.actuator_moment,
            data.moment_rownnz,
            data.moment_rowadr,
            data.moment_colind,
        )
    else:
        J[:] = data.actuator_moment.reshape(model.nu, model.nv)
    return J


class HillMuscleModel:
    """Stateless/stateful Hill muscle evaluator extracted from a ``mujoco.MjModel`` or ``MuscleParams``."""

    def __init__(
        self,
        model: mujoco.MjModel | None = None,
        *,
        params: MuscleParams | None = None,
        nv: int | None = None,
        dt: float = 0.002,
    ) -> None:
        if model is not None:
            self.params = MuscleParams.from_model(model)
            self.nu = int(model.nu)
            self.nv = int(model.nv)
            self.dt = float(model.opt.timestep)
        elif params is not None and nv is not None:
            self.params = params
            self.nu = len(params.tau_act)
            self.nv = int(nv)
            self.dt = float(dt)
        else:
            raise ValueError("Pass either a compiled mujoco.MjModel or both params= and nv=.")

    def generalized_forces(self, length: Any, velocity: Any, act: Any, moment_arm: Any) -> Any:
        """Compute ``qfrc_actuator = moment_arm.T @ force`` for single or batched inputs."""
        force = muscle_actuator_force(length, velocity, act, self.params)
        if _is_torch(force):
            torch = require("torch", "torch")
            J = torch.as_tensor(moment_arm, device=force.device, dtype=force.dtype)
            return torch.einsum("...uv,...u->...v", J, force)
        J_np = np.asarray(moment_arm, dtype=np.float64)
        return np.einsum("...uv,...u->...v", J_np, force)


def strip_muscles_for_joint_torque_sim(
    spec: mujoco.MjSpec,
    *,
    add_joint_motors: bool = True,
) -> mujoco.MjSpec:
    """Return a copy of ``spec`` with spatial tendons and muscles removed for rigid-body sims.

    Preserves all bodies, inertias, geoms, sites, contacts, and ``mjEQ_JOINT``
    quartic couplings while removing ``<tendon>`` and muscle ``<actuator>`` elements
    rejected by joint-only rigid parsers (such as ``genesis.utils.mjcf``). When
    ``add_joint_motors=True``, adds one unit-gear direct motor per independent 1-DOF
    joint so parsers expecting 1-to-1 joint actuation succeed cleanly.
    """
    out = spec.copy()
    probe = out.compile()
    coupled_dep_jnt_names: set[str] = set()
    for eq in out.equalities:
        if eq.type == mujoco.mjtEq.mjEQ_JOINT and eq.name1:
            coupled_dep_jnt_names.add(eq.name1)

    for act in list(out.actuators):
        out.delete(act)
    for ten in list(out.tendons):
        out.delete(ten)

    if add_joint_motors:
        for jid in range(probe.njnt):
            jtype = int(probe.jnt_type[jid])
            jname = probe.joint(jid).name
            if jtype not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                continue
            if not jname or jname in coupled_dep_jnt_names:
                continue
            motor = out.add_actuator(
                name=f"tau_{jname}",
                target=jname,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
            )
            motor.dyntype = mujoco.mjtDyn.mjDYN_NONE
            motor.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            motor.biastype = mujoco.mjtBias.mjBIAS_NONE
            motor.gainprm[0] = 1.0

    return out


__all__ = [
    "HillMuscleModel",
    "MuscleParams",
    "dense_actuator_moment",
    "muscle_activation_dot",
    "muscle_actuator_force",
    "muscle_bias",
    "muscle_gain",
    "step_muscle_activation",
    "strip_muscles_for_joint_torque_sim",
]
