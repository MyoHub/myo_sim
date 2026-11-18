# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Subject-specific muscle strength: native force transforms and MVC torque calibration.

Geometric scaling changes segments and muscle paths, not how strong the muscles
are. :func:`calibrate_muscle_strength` fits a strength factor per muscle so that
the model's maximal joint torques match measured maximal voluntary torques (e.g.
from a dynamometer), and :func:`scale_muscle_strength` applies the factors to a
compiled model (or :func:`scale_muscle_strength_spec` to an editable spec).
"""

from __future__ import annotations

import copy
from collections.abc import Sequence
from dataclasses import dataclass

import mujoco
import numpy as np

from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings


def peak_force(model: mujoco.MjModel, prm: np.ndarray | None = None) -> np.ndarray:
    """Return per-actuator peak-force parameters, resolving native automatic force.

    ``prm`` defaults to ``model.actuator_gainprm`` (active force); pass
    ``model.actuator_biasprm`` for passive force. A negative muscle force uses
    MuJoCo's ``scale / max(acc0, mjMINVAL)`` rule. Non-muscle entries retain
    column 2 unchanged, and the returned array owns its storage.
    """
    parameters = model.actuator_gainprm if prm is None else np.asarray(prm)
    if parameters.shape != model.actuator_gainprm.shape:
        raise ValueError("force parameters must have the actuator_gainprm shape")
    force = np.array(parameters[:, 2], dtype=np.float64)
    automatic = (force < 0) & (model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE)
    force[automatic] = parameters[automatic, 3] / np.maximum(model.actuator_acc0[automatic], mujoco.mjMINVAL)
    return force


def _muscles(model: mujoco.MjModel) -> np.ndarray:
    return model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE


def _scaled_forces(model: mujoco.MjModel, factors: float | np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    muscle = _muscles(model)
    passive = muscle & (model.actuator_biastype == mujoco.mjtBias.mjBIAS_MUSCLE)
    f = np.broadcast_to(np.asarray(factors, dtype=np.float64), (model.nu,))
    if not np.all(np.isfinite(f[muscle]) & (f[muscle] > 0)):
        raise ValueError("Strength factors must be positive and finite")
    with np.errstate(over="ignore", invalid="ignore"):
        gain = peak_force(model)[muscle] * f[muscle]
        bias = peak_force(model, model.actuator_biasprm)[passive] * f[passive]
    if not np.all(np.isfinite(gain)) or not np.all(np.isfinite(bias)):
        raise ValueError("Scaled peak forces must be finite")
    return muscle, passive, gain, bias


def scale_muscle_strength(model: mujoco.MjModel, factors: float | np.ndarray) -> None:
    """Multiply native muscle peak active and passive forces in place.

    Args:
        model: Compiled model. Non-muscle gains and non-muscle bias laws stay
            unchanged, as do geometry, activation dynamics and force clamps.
        factors: Positive finite scalar, or ``(nu,)`` per-actuator factors.
            Entries for non-muscle actuators are ignored.

    Automatic peak forces are resolved before writing explicit forces. A factor
    changes the force capacity, not a clamp or the legacy gain-only sarcopenia
    convention. Invalid factors raise before any force parameter is modified.
    """
    muscle, passive, gain, bias = _scaled_forces(model, factors)
    model.actuator_gainprm[muscle, 2] = gain
    model.actuator_biasprm[passive, 2] = bias


def scale_muscle_strength_spec(spec: mujoco.MjSpec, factors: float | np.ndarray) -> mujoco.MjSpec:
    """Write scaled explicit muscle forces to ``spec`` in place and return it.

    A copy is compiled once to resolve automatic force without freezing the
    caller's editable frames. Scalar/``(nu,)`` factors and validation match
    :func:`scale_muscle_strength`; rejected factors leave the source unchanged.
    """
    model = spec.copy().compile()
    muscle, passive, gain, bias = _scaled_forces(model, factors)
    for index, force in zip(np.flatnonzero(muscle), gain, strict=True):
        spec.actuators[int(index)].gainprm[2] = float(force)
    for index, force in zip(np.flatnonzero(passive), bias, strict=True):
        spec.actuators[int(index)].biasprm[2] = float(force)
    return spec


def strength_factor_from_body_mass(mass: float, reference_mass: float) -> float:
    """Strength factor of a subject relative to the model's reference subject.

    Total lower-limb muscle volume scales with height x mass (Handsfield et al.
    2014, J Biomech 47:631, R^2 = 0.92) and fibre length with height, so the
    cross-section, and with it peak force, scales with body mass. The intercept
    of the published regression is ignored; use measured torques
    (:func:`calibrate_muscle_strength`) where available.

    Args:
        mass: Subject body mass [kg].
        reference_mass: Body mass of the model's reference subject [kg].

    Returns:
        ``mass / reference_mass``.
    """
    if not (np.isfinite(mass) and np.isfinite(reference_mass) and mass > 0 and reference_mass > 0):
        raise ValueError("Masses must be positive and finite")
    return float(mass / reference_mass)


@dataclass(frozen=True)
class MaxTorqueMeasurement:
    """One measured maximal voluntary joint torque.

    Attributes:
        joint: Name of the measured joint (the master of any couplings).
        qpos: ``(nq,)`` posture of the trial; coupled joints are projected.
        torque: Gravity-corrected maximal torque [N m], positive.
        direction: ``+1`` for torque along the joint's positive axis, ``-1``
            for the opposite direction (e.g. extension of a flexion joint).
    """

    joint: str
    qpos: np.ndarray
    torque: float
    direction: float = 1.0


@dataclass(frozen=True)
class StrengthCalibration:
    """Fitted strength factors and the torques they reproduce.

    Attributes:
        factors: ``(nu,)`` strength factor per actuator (1 for non-muscles,
            ``prior`` for muscles no measurement involves).
        involved: ``(nu,)`` muscles whose force acts on some measurement.
        measured: ``(m,)`` measured torques [N m].
        nominal: ``(m,)`` maximal torques of the unscaled model [N m].
        predicted: ``(m,)`` maximal torques with ``factors`` applied [N m].
    """

    factors: np.ndarray
    involved: np.ndarray
    measured: np.ndarray
    nominal: np.ndarray
    predicted: np.ndarray


def _coupled_direction(model: mujoco.MjModel, couplings: JointCouplings, qpos: np.ndarray, joint: int) -> np.ndarray:
    """``(nv,)`` velocity of all dofs per unit velocity of ``joint``, couplings included."""
    h = 1e-6
    lo, hi = qpos.copy(), qpos.copy()
    lo[model.jnt_qposadr[joint]] -= h
    hi[model.jnt_qposadr[joint]] += h
    project_joint_couplings(couplings, lo)
    project_joint_couplings(couplings, hi)
    e = np.zeros(model.nv)
    mujoco.mj_differentiatePos(model, e, 2 * h, lo, hi)
    return e


def _validate_mvc_model(model: mujoco.MjModel) -> None:
    unsupported = [
        model.actuator(i).name for i in range(model.nu) if model.actuator_forcelimited[i] or model.actuator_actnum[i] > 1
    ] + [model.joint(int(j)).name for j in np.flatnonzero(model.jnt_actfrclimited)]
    if unsupported:
        raise ValueError(f"Force clamps and multi-state actuators are not supported for MVC: {unsupported}")
    if not model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL):
        restricted = (
            (model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE)
            & (model.actuator_actadr < 0)
            & (model.actuator_ctrllimited != 0)
            & ((model.actuator_ctrlrange[:, 0] > 0) | (model.actuator_ctrlrange[:, 1] < 1))
        )
        names = [model.actuator(int(i)).name for i in np.flatnonzero(restricted)]
        if names:
            raise ValueError(f"stateless muscle control clamps must include [0, 1] for MVC: {names}")
    if np.any(model.body_gravcomp != 0):
        raise ValueError("gravity compensation is not supported for gravity-corrected MVC")


def max_torque_terms(model: mujoco.MjModel, measurement: MaxTorqueMeasurement) -> tuple[np.ndarray, np.ndarray, float]:
    """Each actuator's active and passive torque, and the joints' passive torque, in a trial.

    Args:
        model: Compiled model.
        measurement: The trial (joint, posture, direction).

    Returns:
        ``(active, passive, joints)``: ``(nu,)`` torque of each actuator at full
        activation beyond its passive force, in the measured direction
        (antagonists negative); ``(nu,)`` torque of each actuator's passive
        force; and the torque of the joints' own springs [N m].

    Raises:
        ValueError: If force clamps, multi-state actuators, gravity compensation
            or stateless muscle control clamps excluding [0, 1] prevent the
            gravity-corrected, affine full-activation probe.
    """
    _validate_mvc_model(model)
    if np.any(model.actuator_actearly):
        model = copy.deepcopy(model)
        model.actuator_actearly[:] = 0
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, measurement.joint)
    # Plain ints: tuple membership of a NumPy scalar in MuJoCo enums is False on
    # some MuJoCo wheels, so compare integer values.
    if joint < 0 or int(model.jnt_type[joint]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
        raise ValueError(f"{measurement.joint!r} is not a hinge or slide joint of the model")
    couplings = JointCouplings.from_model(model)
    qpos = np.array(measurement.qpos, dtype=np.float64)
    project_joint_couplings(couplings, qpos)
    e = measurement.direction * _coupled_direction(model, couplings, qpos, joint)

    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_fwdPosition(model, data)
    mujoco.mj_fwdVelocity(model, data)
    stateful = model.actuator_actadr >= 0
    forces = []
    for level in (0.0, 1.0):
        data.act[model.actuator_actadr[stateful]] = level
        data.ctrl[~stateful] = level
        mujoco.mj_fwdActuation(model, data)
        forces.append(data.actuator_force.copy())
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
    arm = moment @ e  # generalized force along the trial direction per unit actuator force
    return arm * (forces[1] - forces[0]), arm * forces[0], float(e @ data.qfrc_passive)


def _bvls(a: np.ndarray, b: np.ndarray, lo: float, hi: float) -> np.ndarray:
    """Solve ``min_{lo <= x <= hi} ||a @ x - b||_2`` by Stark & Parker (1995) active-set BVLS."""
    n = a.shape[1]
    if n == 0:
        return np.empty(0, dtype=np.float64)
    if lo == hi:
        return np.full(n, lo, dtype=np.float64)
    x = np.full(n, lo, dtype=np.float64)
    state = np.full(n, -1, dtype=int)  # -1: lower bound, 0: free, +1: upper bound
    scale = max(1.0, float(np.linalg.norm(a, ord=np.inf)), float(np.linalg.norm(b, ord=np.inf)))
    tol = scale * np.finfo(np.float64).eps * 100.0
    excluded: set[int] = set()

    for _ in range(max(10, 4 * n)):
        grad = a.T @ (b - a @ x)
        violation = np.where(state == -1, grad, np.where(state == 1, -grad, -np.inf))
        for idx in excluded:
            violation[idx] = -np.inf
        candidate = int(np.argmax(violation))
        if violation[candidate] <= tol:
            if not excluded:
                break
            excluded.clear()
            continue
        state[candidate] = 0

        while np.any(state == 0):
            free = np.flatnonzero(state == 0)
            bound = np.flatnonzero(state != 0)
            rhs = b - a[:, bound] @ x[bound] if len(bound) else b
            z = np.linalg.lstsq(a[:, free], rhs, rcond=None)[0]
            if np.all((z > lo) & (z < hi)):
                x[free] = z
                excluded.clear()
                break
            step = z - x[free]
            alphas = np.ones(len(free), dtype=np.float64)
            low_hit = z <= lo
            high_hit = z >= hi
            alphas[low_hit] = np.where(step[low_hit] < 0, (lo - x[free[low_hit]]) / step[low_hit], 0.0)
            alphas[high_hit] = np.where(step[high_hit] > 0, (hi - x[free[high_hit]]) / step[high_hit], 0.0)
            alpha = float(np.clip(np.min(alphas), 0.0, 1.0))
            x[free] = np.clip(x[free] + alpha * step, lo, hi)
            if alpha == 0.0:
                excluded.add(candidate)
            for pos, idx in enumerate(free):
                if z[pos] <= lo and x[idx] <= lo + tol:
                    x[idx], state[idx] = lo, -1
                elif z[pos] >= hi and x[idx] >= hi - tol:
                    x[idx], state[idx] = hi, 1
    return x


def calibrate_muscle_strength(
    model: mujoco.MjModel,
    measurements: Sequence[MaxTorqueMeasurement],
    *,
    prior: float = 1.0,
    regularization: float = 1e-3,
    bounds: tuple[float, float] = (0.25, 4.0),
) -> StrengthCalibration:
    """Fit per-muscle strength factors to measured maximal torques.

    Minimises ``sum_j ((T_j(k) - T_meas_j) / T_meas_j)**2 + regularization *
    sum_i (k_i - prior)**2`` within ``bounds`` (bounded linear least squares);
    ``T_j(k)`` is the model's maximal torque of trial ``j`` with every muscle's
    peak force, active and passive, multiplied by ``k``, and non-muscle
    actuators held at zero control/activation. Their zero-input force and the
    joints' passive torque are fixed contributions. Net torques do not
    separate synergists: the small default regularization only decides how a
    trial's change is shared among them, without pulling the fit to the prior.

    Args:
        model: Compiled model (not modified).
        measurements: Trials; at least one.
        prior: Factor of muscles no trial involves, and the target of the
            regularization (e.g. :func:`strength_factor_from_body_mass`).
        regularization: Weight of the prior against the relative torque errors.
        bounds: Lower and upper bounds of each factor.

    Returns:
        The factors and the measured, nominal and predicted torques.

    Raises:
        ValueError: If there are no measurements, a torque is not positive,
            bounds/prior/regularization are invalid, or the model cannot support
            the steady probe in :func:`max_torque_terms`.
    """
    if not measurements or any(not np.isfinite(m.torque) or m.torque <= 0 for m in measurements):
        raise ValueError("Need measurements with positive torques")
    lo, hi = float(bounds[0]), float(bounds[1])
    if not (np.isfinite(lo) and np.isfinite(hi) and 0 < lo < hi):
        raise ValueError("bounds must be positive, finite and strictly ordered")
    if not (np.isfinite(prior) and prior > 0):
        raise ValueError("prior must be positive and finite")
    if not (np.isfinite(regularization) and regularization >= 0):
        raise ValueError("regularization must be non-negative and finite")

    terms = [max_torque_terms(model, m) for m in measurements]
    muscles = _muscles(model)
    # Torque per unit strength factor: agonists fully active plus every muscle's
    # passive force. Zero-input non-muscle force and joint springs stay fixed.
    per_unit = (np.maximum([a for a, _, _ in terms], 0.0) + [p for _, p, _ in terms]) * muscles
    fixed = np.array([j for _, _, j in terms]) + np.array([p for _, p, _ in terms])[:, ~muscles].sum(axis=1)
    measured = np.array([m.torque for m in measurements], dtype=np.float64)
    involved = muscles & (per_unit != 0).any(axis=0)

    n = int(involved.sum())
    factors = np.where(muscles, float(prior), 1.0)
    if n:
        a = np.vstack([per_unit[:, involved] / measured[:, None], np.sqrt(regularization) * np.eye(n)])
        b = np.r_[(measured - fixed) / measured, np.full(n, np.sqrt(regularization) * prior)]
        factors[involved] = _bvls(a, b, lo, hi)
    return StrengthCalibration(
        factors=factors,
        involved=involved,
        measured=measured,
        nominal=per_unit.sum(axis=1) + fixed,
        predicted=per_unit @ factors + fixed,
    )
