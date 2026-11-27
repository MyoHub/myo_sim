# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Selected-body mass transforms without geometry or strength calibration.

Adapted from MyoSuite's subject-calibration helper. Choosing represented body
mass or a participant/reference ratio remains an explicit caller decision.
"""

from __future__ import annotations

import copy
from collections.abc import Sequence

import mujoco
import numpy as np

from .muscle_strength import scale_muscle_strength


def scale_body_mass(
    model: mujoco.MjModel,
    represented_mass: float | None = None,
    *,
    participant_mass: float | None = None,
    reference_mass: float | None = None,
    bodies: Sequence[str] | None = None,
) -> mujoco.MjModel:
    """Return a selected-body mass/inertia-scaled copy with unchanged strength.

    Args:
        model: Compiled model, unchanged. Segment geometry and COM stay fixed.
        represented_mass: Target sum [kg] of selected model bodies, explicitly
            not a participant's whole-body mass for a partial-body model.
        participant_mass: Participant whole-body mass [kg]. Requires explicit
            ``reference_mass``; selected bodies scale by their ratio.
        reference_mass: Whole-body mass [kg] of the model's reference participant.
        bodies: Distinct non-world body names. Only named bodies are selected,
            not their descendants. ``None`` selects all non-world bodies,
            including fixed scenery and objects; select anatomy explicitly to
            exclude those. The selection must have positive total mass.

    Returns:
        Independent model copy with refreshed MuJoCo constants. Automatic
        muscle forces are frozen explicitly before the mass change; active and
        passive force capacity, other actuators and force clamps stay unchanged.
        Equality settings and geometric statistics are preserved; default mass
        and inertia statistics follow the scaled model.
        Supply exactly one mass mode, using positive finite values.
    """
    ratio_mode = participant_mass is not None or reference_mass is not None
    if (represented_mass is None) == (not ratio_mode):
        raise ValueError("choose represented_mass or participant_mass with reference_mass")
    ids = np.arange(1, model.nbody) if bodies is None else np.array([model.body(name).id for name in bodies], dtype=int)
    if not len(ids) or 0 in ids or len(ids) != len(np.unique(ids)):
        raise ValueError("bodies must select distinct non-world bodies")
    selected_mass = float(model.body_mass[ids].sum())
    if not np.isfinite(selected_mass) or selected_mass <= 0:
        raise ValueError("selected bodies must have positive finite total mass")
    if ratio_mode:
        if participant_mass is None or reference_mass is None:
            raise ValueError("participant_mass requires explicit reference_mass")
        values = np.array([participant_mass, reference_mass], dtype=float)
    else:
        values = np.array([represented_mass, selected_mass], dtype=float)
    if not np.all(np.isfinite(values) & (values > 0)):
        raise ValueError("masses must be positive and finite")
    with np.errstate(over="ignore", under="ignore", divide="ignore"):
        ratio = float(values[0] / values[1])
    if not np.isfinite(ratio) or ratio <= 0:
        raise ValueError("mass ratio must be positive and finite")
    with np.errstate(over="ignore", invalid="ignore"):
        mass = model.body_mass[ids] * ratio
        inertia = model.body_inertia[ids] * ratio
    if not np.isfinite(mass).all() or not np.isfinite(inertia).all():
        raise ValueError("scaled mass and inertia must be finite")
    default_meanmass = bool(np.isclose(model.stat.meanmass, float(np.mean(model.body_mass[1:])), rtol=1e-12, atol=1e-12))
    has_meaninertia = hasattr(model.stat, "meaninertia")
    default_meaninertia = has_meaninertia and bool(
        np.isclose(model.stat.meaninertia, float(np.mean(model.dof_M0)) if model.nv else 1.0, rtol=1e-12, atol=1e-12)
    )
    out = copy.deepcopy(model)
    scale_muscle_strength(out, 1.0)
    out.body_mass[ids] = mass
    out.body_inertia[ids] = inertia
    mujoco.mj_setConst(out, mujoco.MjData(out))
    # Mass changes do not redefine body connect anchors or geometric statistics.
    out.eq_data[:] = model.eq_data
    out.stat.meansize = model.stat.meansize
    out.stat.extent = model.stat.extent
    out.stat.center[:] = model.stat.center
    out.stat.meanmass = float(np.mean(out.body_mass[1:])) if default_meanmass else model.stat.meanmass
    if has_meaninertia:
        out.stat.meaninertia = (
            (float(np.mean(out.dof_M0)) if out.nv else 1.0) if default_meaninertia else model.stat.meaninertia
        )
    return out
