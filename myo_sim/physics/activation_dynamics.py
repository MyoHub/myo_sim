# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Hill-type muscle activation and deactivation time-constant transforms.

MuJoCo's ``mjDYN_MUSCLE`` first-order activation ODE stores activation and
deactivation time constants ``(tau_act, tau_deact)`` in ``actuator_dynprm[:, :2]``
(seconds). Scaling them slows or accelerates excitation-contraction rise and
relaxation without altering steady-state active/passive force capacity, segment
geometry, or non-muscle actuators.
"""

from __future__ import annotations

import mujoco
import numpy as np


def _muscle_dynamics_mask(model: mujoco.MjModel) -> np.ndarray:
    return model.actuator_dyntype == mujoco.mjtDyn.mjDYN_MUSCLE


def activation_time_constants(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(tau_act, tau_deact)`` arrays of shape ``(nu,)`` in seconds.

    Non-``mjDYN_MUSCLE`` actuators retain their raw ``actuator_dynprm[:, 0]``
    and ``actuator_dynprm[:, 1]`` entries. Both returned arrays own their storage.
    """
    tau_act = np.array(model.actuator_dynprm[:, 0], dtype=np.float64)
    tau_deact = np.array(model.actuator_dynprm[:, 1], dtype=np.float64)
    return tau_act, tau_deact


def _scaled_taus(
    model: mujoco.MjModel,
    act_factors: float | np.ndarray,
    deact_factors: float | np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    muscle = _muscle_dynamics_mask(model)
    f_act = np.broadcast_to(np.asarray(act_factors, dtype=np.float64), (model.nu,))
    f_deact = f_act if deact_factors is None else np.broadcast_to(np.asarray(deact_factors, dtype=np.float64), (model.nu,))
    if not np.all(np.isfinite(f_act[muscle]) & (f_act[muscle] > 0)):
        raise ValueError("Activation time-constant factors must be positive and finite")
    if not np.all(np.isfinite(f_deact[muscle]) & (f_deact[muscle] > 0)):
        raise ValueError("Deactivation time-constant factors must be positive and finite")
    tau_act_0, tau_deact_0 = activation_time_constants(model)
    with np.errstate(over="ignore", invalid="ignore"):
        scaled_act = tau_act_0[muscle] * f_act[muscle]
        scaled_deact = tau_deact_0[muscle] * f_deact[muscle]
    if not np.all(np.isfinite(scaled_act) & (scaled_act >= mujoco.mjMINVAL)):
        raise ValueError("Scaled activation time constants must be positive and finite")
    if not np.all(np.isfinite(scaled_deact) & (scaled_deact >= mujoco.mjMINVAL)):
        raise ValueError("Scaled deactivation time constants must be positive and finite")
    return muscle, scaled_act, scaled_deact


def scale_activation_time_constants(
    model: mujoco.MjModel,
    act_factors: float | np.ndarray = 1.0,
    deact_factors: float | np.ndarray | None = None,
) -> None:
    """Multiply ``mjDYN_MUSCLE`` activation and deactivation time constants in place.

    Args:
        model: Compiled model. Peak active/passive forces, bias laws, geometry,
            clamps, stateless muscles, and non-muscle actuators stay unchanged.
        act_factors: Positive finite scalar or ``(nu,)`` per-actuator multiplier
            for ``tau_act`` (``actuator_dynprm[:, 0]``). Non-``mjDYN_MUSCLE``
            entries are ignored.
        deact_factors: Positive finite scalar or ``(nu,)`` multiplier for
            ``tau_deact`` (``actuator_dynprm[:, 1]``). Defaults to ``act_factors``
            when ``None``.

    Invalid factors raise ``ValueError`` before any parameter is modified.
    """
    muscle, scaled_act, scaled_deact = _scaled_taus(model, act_factors, deact_factors)
    model.actuator_dynprm[muscle, 0] = scaled_act
    model.actuator_dynprm[muscle, 1] = scaled_deact


def scale_activation_time_constants_spec(
    spec: mujoco.MjSpec,
    act_factors: float | np.ndarray = 1.0,
    deact_factors: float | np.ndarray | None = None,
) -> mujoco.MjSpec:
    """Write scaled ``mjDYN_MUSCLE`` time constants to ``spec`` in place and return it.

    A copy is compiled once to validate shapes and resolve inherited defaults
    without freezing the caller's editable frames; rejected factors leave
    ``spec`` unchanged.
    """
    model = spec.copy().compile()
    muscle, scaled_act, scaled_deact = _scaled_taus(model, act_factors, deact_factors)
    for index, tau_a, tau_d in zip(np.flatnonzero(muscle), scaled_act, scaled_deact, strict=True):
        actuator = spec.actuators[int(index)]
        actuator.dynprm[0] = float(tau_a)
        actuator.dynprm[1] = float(tau_d)
    return spec
