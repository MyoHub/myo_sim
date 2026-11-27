# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Passive joint damping, stiffness, and range-of-motion transforms.

Returns an independent ``MjModel`` copy with refreshed MuJoCo constants while
preserving muscle peak force capacity, segment inertia, and ``mjEQ_JOINT``
polynomial couplings.
"""

from __future__ import annotations

import copy
from collections.abc import Sequence

import mujoco
import numpy as np

from .joint_coupling import JointCouplings
from .muscle_strength import scale_muscle_strength

_ONE_DOF_TYPES = frozenset((int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)))


def _dependent_qpos_addrs(model: mujoco.MjModel) -> set[int]:
    couplings = JointCouplings.from_model(model)
    return set(np.asarray(couplings.locked_qpos, dtype=int).tolist()) | set(np.asarray(couplings.dep_qpos, dtype=int).tolist())


def _select_independent_joints(model: mujoco.MjModel, joints: Sequence[str] | None) -> np.ndarray:
    dep_qpos = _dependent_qpos_addrs(model)
    if joints is None:
        ids = [
            jid
            for jid in range(model.njnt)
            if int(model.jnt_type[jid]) in _ONE_DOF_TYPES and int(model.jnt_qposadr[jid]) not in dep_qpos
        ]
        if not ids:
            raise ValueError("model has no independent hinge or slide joints")
        return np.array(ids, dtype=int)

    names = list(joints)
    if not names or len(names) != len(set(names)):
        raise ValueError("joints must be a non-empty sequence of distinct joint names")
    ids = []
    for name in names:
        jid = model.joint(name).id
        if int(model.jnt_type[jid]) not in _ONE_DOF_TYPES:
            raise ValueError(f"joint {name!r} must be a 1-DOF hinge or slide joint")
        if int(model.jnt_qposadr[jid]) in dep_qpos:
            raise ValueError(f"joint {name!r} is coupled by an active mjEQ_JOINT constraint")
        ids.append(jid)
    return np.array(ids, dtype=int)


def _broadcast_nonneg(value: float | np.ndarray, count: int, label: str, *, strictly_positive: bool = False) -> np.ndarray:
    arr = np.broadcast_to(np.asarray(value, dtype=np.float64), (count,))
    valid = np.isfinite(arr) & (arr > 0 if strictly_positive else arr >= 0)
    if not np.all(valid):
        bound = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{label} factors must be {bound} and finite")
    return arr


def scale_joint_mechanics(
    model: mujoco.MjModel,
    *,
    damping: float | np.ndarray = 1.0,
    stiffness: float | np.ndarray = 1.0,
    range_scale: float | np.ndarray = 1.0,
    joints: Sequence[str] | None = None,
) -> mujoco.MjModel:
    """Return a copy of ``model`` with scaled passive joint mechanics.

    Args:
        model: Compiled model, left unchanged.
        damping: Non-negative finite scalar or ``(n_selected,)`` multiplier for
            ``dof_damping`` on the selected joints.
        stiffness: Non-negative finite scalar or ``(n_selected,)`` multiplier
            for ``jnt_stiffness`` on the selected joints.
        range_scale: Positive finite scalar or ``(n_selected,)`` multiplier
            scaling limited joint intervals ``[lo, hi]`` relative to
            ``clip(qpos0, lo, hi)`` so reference postures inside ``[lo, hi]``
            remain inside the scaled limits.
        joints: Optional sequence of distinct independent 1-DOF (``hinge`` or
            ``slide``) joint names. ``None`` selects every independent 1-DOF
            joint not driven by an active ``mjEQ_JOINT`` equality.

    Returns:
        Independent ``MjModel`` copy with refreshed ``mj_setConst`` constants
        and preserved automatic muscle peak forces, equality settings and
        model statistics.
    """
    jnt_ids = _select_independent_joints(model, joints)
    n_sel = len(jnt_ids)
    damp_f = _broadcast_nonneg(damping, n_sel, "damping")
    stiff_f = _broadcast_nonneg(stiffness, n_sel, "stiffness")
    rom_f = _broadcast_nonneg(range_scale, n_sel, "range_scale", strictly_positive=True)

    dof_ids = model.jnt_dofadr[jnt_ids]
    qpos_ids = model.jnt_qposadr[jnt_ids]
    limited = model.jnt_limited[jnt_ids].astype(bool)
    if np.any(rom_f != 1.0) and not np.any(limited):
        raise ValueError("range_scale != 1 requires at least one limited joint in selection")

    with np.errstate(over="ignore", invalid="ignore"):
        new_damping = model.dof_damping[dof_ids] * damp_f
        new_stiffness = model.jnt_stiffness[jnt_ids] * stiff_f
    if not np.isfinite(new_damping).all() or not np.isfinite(new_stiffness).all():
        raise ValueError("scaled damping and stiffness must be finite")

    new_ranges = model.jnt_range[jnt_ids].copy()
    if np.any(limited):
        lim_ranges = new_ranges[limited]
        lim_qpos0 = model.qpos0[qpos_ids[limited]]
        anchor = np.clip(lim_qpos0, lim_ranges[:, 0], lim_ranges[:, 1])
        scale = rom_f[limited][:, None]
        with np.errstate(over="ignore", invalid="ignore"):
            scaled_lim = anchor[:, None] + scale * (lim_ranges - anchor[:, None])
        spans = scaled_lim[:, 1] - scaled_lim[:, 0]
        if not np.isfinite(scaled_lim).all() or np.any(spans < mujoco.mjMINVAL):
            raise ValueError("scaled joint ranges must be finite and non-degenerate")
        new_ranges[limited] = scaled_lim

    out = copy.deepcopy(model)
    scale_muscle_strength(out, 1.0)
    out.dof_damping[dof_ids] = new_damping
    out.jnt_stiffness[jnt_ids] = new_stiffness
    out.jnt_range[jnt_ids] = new_ranges
    mujoco.mj_setConst(out, mujoco.MjData(out))
    # setConst also reanchors body connect constraints and resets statistics.
    # Joint mechanics change neither their geometry nor the model's inertia.
    out.eq_data[:] = model.eq_data
    for field in ("meanmass", "meaninertia", "meansize", "extent"):
        if hasattr(model.stat, field):
            setattr(out.stat, field, getattr(model.stat, field))
    out.stat.center[:] = model.stat.center
    return out
