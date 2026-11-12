# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Model-only scaling of native MuJoCo muscle strength.

Adapted from MyoSuite's muscle_conditions and muscle_strength helpers. Fitting
strength to measurements and choosing a subject's factors belong to callers.
"""

from __future__ import annotations

import mujoco
import numpy as np


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


def _scaled_forces(model: mujoco.MjModel, factors: float | np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    muscle = model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE
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
