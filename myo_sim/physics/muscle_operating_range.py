# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Muscle operating-range transform: span and shift of ``actuator_lengthrange``."""

from __future__ import annotations

from collections.abc import Sequence

import mujoco
import numpy as np

from myo_sim.physics.muscle_strength import _selected_muscles


def scale_muscle_operating_range(
    model: mujoco.MjModel,
    span: float | np.ndarray = 1.0,
    *,
    shift: float | np.ndarray = 0.0,
    muscles: Sequence[str] | None = None,
) -> None:
    """Rescale and shift the operating length range of muscle actuators in place.

    A selected muscle's ``actuator_lengthrange`` ``[l0, l1]`` with midpoint ``m``
    and half-width ``h`` becomes ``[m + shift - span * h, m + shift + span * h]``.

    MuJoCo computes the optimal fibre length as ``L0 = (l1 - l0) / (range[1] -
    range[0])`` with ``range`` from ``gainprm[0:2]``, and normalises the length
    from ``l0``. ``span`` therefore scales the optimal fibre length about the
    midpoint, and ``shift`` moves the operating range along the actuator length
    (comparable to a tendon-slack change). Peak force, bias parameters, geometry
    and activation dynamics are untouched.

    Args:
        model: Compiled model.
        span: Positive finite scalar, or ``(nu,)`` per-actuator range multipliers.
        shift: Finite scalar, or ``(nu,)`` per-actuator midpoint shifts [m].
        muscles: Names of distinct muscle actuators (``mjGAIN_MUSCLE``) to
            transform; ``None`` selects every muscle. Other actuators stay
            bit-identical. Entries of ``span`` and ``shift`` for unselected
            actuators are ignored.

    Raises:
        ValueError: If a name is unknown, duplicated or not a muscle, a selected
            factor is invalid, a selected range is not increasing (``l1 > l0``) or
            a result is not finite. Nothing is modified in that case.
    """
    selected = _selected_muscles(model, muscles)
    span_all = np.broadcast_to(np.asarray(span, dtype=np.float64), (model.nu,))
    shift_all = np.broadcast_to(np.asarray(shift, dtype=np.float64), (model.nu,))
    span_sel, shift_sel = span_all[selected], shift_all[selected]
    if not np.all(np.isfinite(span_sel) & (span_sel > 0)):
        raise ValueError("span must be positive and finite")
    if not np.all(np.isfinite(shift_sel)):
        raise ValueError("shift must be finite")
    low, high = model.actuator_lengthrange[selected].T
    if not np.all(np.isfinite(low) & np.isfinite(high) & (high > low)):
        raise ValueError("selected muscles need a finite increasing lengthrange")
    with np.errstate(over="ignore", invalid="ignore"):
        mid, half = 0.5 * (low + high), 0.5 * (high - low)
        scaled = np.column_stack([mid + shift_sel - span_sel * half, mid + shift_sel + span_sel * half])
    if not np.all(np.isfinite(scaled)):
        raise ValueError("Transformed lengthrange must be finite")
    model.actuator_lengthrange[selected] = scaled
