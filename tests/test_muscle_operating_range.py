"""Operating-range and moment-arm coupling gates for muscle actuators across composed models."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.compose import MODEL_REGISTRY
from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings

# Documented in docs/wiki/postures.md: LAT1 / LAT1_l sit at L_norm = -0.977 on dev.
LEGACY_OUTSIDE_OPERATING_RANGE: frozenset[str] = frozenset({"LAT1", "LAT1_l"})
# Upper-thoracic fascicles whose origin and insertion lie on the rigid thoracic cage.
LEGACY_RIGID_SPAN_ACTUATORS: frozenset[str] = frozenset({"LTpT_T1_r", "LTpT_T1_l"})

ROM_FRACTIONS: tuple[float, ...] = (0.15, 0.35, 0.50, 0.65, 0.85)
MIN_ACTIVE_FORCE_N: float = 1e-3
MIN_MOMENT_NORM: float = 1e-5


@dataclass(frozen=True)
class MuscleOperatingAudit:
    """Summary of muscle parameter, active-force, and moment-arm checks on one compiled model."""

    invalid_parameters: tuple[str, ...]
    inactive_at_reference: dict[str, tuple[float, float, float, float]]
    inactive_across_rom: tuple[str, ...]
    zero_moment_arm: tuple[str, ...]


def _normalised_fibre_lengths(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Normalised fibre length of every actuator in units of optimal fibre length."""
    low = model.actuator_gainprm[:, 0]
    high = model.actuator_gainprm[:, 1]
    span = np.where(np.abs(high - low) > 1e-12, high - low, 1.0)
    length0 = (model.actuator_lengthrange[:, 1] - model.actuator_lengthrange[:, 0]) / span
    tendon = model.actuator_lengthrange[:, 0] - low * length0
    return np.where(np.abs(length0) > 1e-12, (data.actuator_length - tendon) / length0, np.nan)


def _actuator_moment_norms(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Euclidean norm of each actuator's moment-arm row across dense and sparse MuJoCo layouts."""
    moment = np.asarray(data.actuator_moment, dtype=np.float64)
    if moment.ndim == 2:
        return np.linalg.norm(moment, axis=1)
    if not hasattr(data, "moment_rownnz"):
        return np.linalg.norm(moment.reshape(model.nu, model.nv), axis=1)
    norms = np.zeros(model.nu, dtype=np.float64)
    rownnz = data.moment_rownnz
    rowadr = data.moment_rowadr
    for idx in range(model.nu):
        nnz = int(rownnz[idx])
        if nnz > 0:
            adr = int(rowadr[idx])
            norms[idx] = float(np.linalg.norm(moment[adr : adr + nnz]))
    return norms


def _sample_postures(model: mujoco.MjModel) -> np.ndarray:
    """Build coupling-consistent reference and ROM sample postures ``(P, nq)``."""
    couplings = JointCouplings.from_model(model)
    if model.nkey > 0:
        q_ref = model.key_qpos[0].copy()
    else:
        q_ref = model.qpos0.copy()
        project_joint_couplings(couplings, q_ref)

    dependent_qpos = set(couplings.locked_qpos.tolist()) | set(couplings.dep_qpos.tolist())
    limited_joints = [
        jnt
        for jnt in range(model.njnt)
        if model.jnt_type[jnt] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
        and model.jnt_limited[jnt]
        and int(model.jnt_qposadr[jnt]) not in dependent_qpos
    ]

    poses = np.repeat(q_ref[None, :], 1 + len(ROM_FRACTIONS), axis=0)
    for row, frac in enumerate(ROM_FRACTIONS, start=1):
        for jnt in limited_joints:
            lo, hi = model.jnt_range[jnt]
            poses[row, model.jnt_qposadr[jnt]] = lo + frac * (hi - lo)
    project_joint_couplings(couplings, poses[1:])
    return poses


def audit_muscle_operating_range(model: mujoco.MjModel) -> MuscleOperatingAudit:
    """Evaluate parameter validity, active contractile force, and moment arms on *model*."""
    muscle_ids = np.flatnonzero(model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE)
    if len(muscle_ids) == 0:
        return MuscleOperatingAudit((), {}, (), ())

    invalid_params: list[str] = []
    for idx in muscle_ids:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(idx)) or f"actuator_{int(idx)}"
        lr_low, lr_high = model.actuator_lengthrange[idx]
        r_low, r_high = model.actuator_gainprm[idx, 0], model.actuator_gainprm[idx, 1]
        lmin, lmax = model.actuator_gainprm[idx, 4], model.actuator_gainprm[idx, 5]
        if not (
            np.isfinite(lr_low)
            and np.isfinite(lr_high)
            and lr_high > lr_low > 0.0
            and np.isfinite(r_low)
            and np.isfinite(r_high)
            and r_high > r_low > 0.0
            and np.isfinite(lmin)
            and np.isfinite(lmax)
            and lmax > lmin >= 0.0
        ):
            invalid_params.append(name)

    poses = _sample_postures(model)
    d_zero = mujoco.MjData(model)
    d_full = mujoco.MjData(model)
    active_forces = np.zeros((len(poses), model.nu), dtype=np.float64)
    norm_lengths = np.zeros((len(poses), model.nu), dtype=np.float64)
    moment_norms = np.zeros((len(poses), model.nu), dtype=np.float64)

    for p_idx, qpos in enumerate(poses):
        d_zero.qpos[:] = qpos
        d_full.qpos[:] = qpos
        d_zero.qvel[:] = 0.0
        d_full.qvel[:] = 0.0
        d_zero.act[:] = 0.0
        d_full.act[:] = 1.0
        mujoco.mj_forward(model, d_zero)
        mujoco.mj_forward(model, d_full)
        active_forces[p_idx] = -(d_full.actuator_force - d_zero.actuator_force)
        norm_lengths[p_idx] = _normalised_fibre_lengths(model, d_zero)
        moment_norms[p_idx] = _actuator_moment_norms(model, d_zero)

    peak_force = active_forces.max(axis=0)
    peak_moment = moment_norms.max(axis=0)
    lmin_all = model.actuator_gainprm[:, 4]
    lmax_all = model.actuator_gainprm[:, 5]

    inactive_ref: dict[str, tuple[float, float, float, float]] = {}
    inactive_rom: list[str] = []
    zero_moment: list[str] = []
    for idx in muscle_ids:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(idx)) or f"actuator_{int(idx)}"
        l_ref = float(norm_lengths[0, idx])
        f_ref = float(active_forces[0, idx])
        lmin, lmax = float(lmin_all[idx]), float(lmax_all[idx])
        if not (lmin < l_ref < lmax) or f_ref <= MIN_ACTIVE_FORCE_N:
            inactive_ref[name] = (round(l_ref, 4), round(f_ref, 4), round(lmin, 4), round(lmax, 4))
        if float(peak_force[idx]) <= MIN_ACTIVE_FORCE_N:
            inactive_rom.append(name)
        if float(peak_moment[idx]) <= MIN_MOMENT_NORM:
            zero_moment.append(name)

    return MuscleOperatingAudit(
        invalid_parameters=tuple(invalid_params),
        inactive_at_reference=inactive_ref,
        inactive_across_rom=tuple(inactive_rom),
        zero_moment_arm=tuple(zero_moment),
    )


@pytest.mark.parametrize("name", sorted(MODEL_REGISTRY))
def test_composed_model_muscle_operating_ranges(name: str) -> None:
    model = myo_sim.load_model(name)
    audit = audit_muscle_operating_range(model)

    assert not audit.invalid_parameters, f"{name}: invalid muscle parameters on {audit.invalid_parameters}"

    unexpected_ref = {
        act: details for act, details in audit.inactive_at_reference.items() if act not in LEGACY_OUTSIDE_OPERATING_RANGE
    }
    assert not unexpected_ref, (
        f"{name}: muscles outside active [lmin, lmax] or producing zero active force at reference pose "
        f"(L_norm, F_active_N, lmin, lmax): {unexpected_ref}"
    )

    unexpected_rom = sorted(set(audit.inactive_across_rom) - LEGACY_OUTSIDE_OPERATING_RANGE)
    assert not unexpected_rom, f"{name}: muscles with zero active force across entire sampled ROM: {unexpected_rom}"

    unexpected_zero_moment = sorted(set(audit.zero_moment_arm) - LEGACY_RIGID_SPAN_ACTUATORS)
    assert not unexpected_zero_moment, f"{name}: muscles with zero moment-arm coupling across ROM: {unexpected_zero_moment}"


def test_operating_range_audit_catches_lmax_saturation_and_zero_moment_arm() -> None:
    """Negative control: narrowing lmax below neutral length or zeroing gear must fail the audit."""
    baseline = myo_sim.load_model("myohand_r")
    clean_audit = audit_muscle_operating_range(baseline)
    assert not clean_audit.inactive_at_reference
    assert not clean_audit.zero_moment_arm

    saturated = myo_sim.load_model("myohand_r")
    fdp_id = mujoco.mj_name2id(saturated, mujoco.mjtObj.mjOBJ_ACTUATOR, "FDP3")
    if fdp_id < 0:
        fdp_id = 0
    fdp_name = mujoco.mj_id2name(saturated, mujoco.mjtObj.mjOBJ_ACTUATOR, fdp_id)
    d0 = mujoco.MjData(saturated)
    mujoco.mj_forward(saturated, d0)
    neutral_l_norm = float(_normalised_fibre_lengths(saturated, d0)[fdp_id])
    saturated.actuator_gainprm[fdp_id, 5] = neutral_l_norm - 0.05

    sat_audit = audit_muscle_operating_range(saturated)
    assert fdp_name in sat_audit.inactive_at_reference

    uncoupled = myo_sim.load_model("myohand_r")
    uncoupled.actuator_gear[fdp_id, :] = 0.0
    unc_audit = audit_muscle_operating_range(uncoupled)
    assert fdp_name in unc_audit.zero_moment_arm
