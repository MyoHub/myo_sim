"""Arm tendon lengths must change continuously with the elbow angle, also just past its limits.

Soft joint limits let the elbow sit up to ~0.13 rad past its range under random activation. A
sidesite whose projection flips across the tendon line forces MuJoCo to wrap the long way round
the geom, and a wrap onset that sits on the limit makes the solver evaluate grazing segments over
and over; both showed up as BIClong length jumps of 15-20 mm and NaN lengths.
"""

import mujoco
import numpy as np
import pytest

from myo_sim import load_model

LIMIT_MARGIN = 0.13  # rad past each elbow limit, the deepest penetration seen in random-activation rollouts
ELBOW_STEP = 1e-3  # rad
PRO_SUP_SAMPLES = 9
MAX_STEP_CHANGE = 1e-3  # m per elbow step; smooth paths change by < 0.1 mm


@pytest.fixture(scope="module")
def arms():
    return load_model("myoarms")


@pytest.mark.parametrize("side", ["r", "l"])
def test_arm_tendon_lengths_continuous_over_elbow_sweep(arms, side):
    model = arms
    data = mujoco.MjData(model)
    elbow = model.joint(f"elbow_flexion_{side}")
    pro_sup = model.joint(f"pro_sup_{side}")
    tendons = [t for t in range(model.ntendon) if model.tendon(t).name.endswith("_l") == (side == "l")]
    angles = np.arange(elbow.range[0] - LIMIT_MARGIN, elbow.range[1] + LIMIT_MARGIN, ELBOW_STEP)

    worst = (0.0, "", 0.0, 0.0)
    for ps in np.linspace(*pro_sup.range, PRO_SUP_SAMPLES):
        lengths = np.empty((len(angles), len(tendons)))
        for i, q in enumerate(angles):
            data.qpos[:] = model.qpos0
            data.qpos[pro_sup.qposadr[0]] = ps
            data.qpos[elbow.qposadr[0]] = q
            mujoco.mj_kinematics(model, data)
            mujoco.mj_tendon(model, data)
            lengths[i] = data.ten_length[tendons]
        assert np.all(np.isfinite(lengths)), f"non-finite tendon length at pro_sup {ps:+.3f}"
        jumps = np.abs(np.diff(lengths, axis=0))
        i, k = np.unravel_index(np.argmax(jumps), jumps.shape)
        if jumps[i, k] > worst[0]:
            worst = (jumps[i, k], model.tendon(tendons[k]).name, angles[i], ps)

    step, name, q, ps = worst
    assert step < MAX_STEP_CHANGE, f"{name} jumps by {step * 1e3:.2f} mm at elbow {q:+.3f} rad, pro_sup {ps:+.3f} rad"
