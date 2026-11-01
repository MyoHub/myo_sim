"""Leg muscle maps reproduce the reference (Rajagopal et al. 2016) rigid-tendon fibre lengths.

For the muscles converted with ``myo_sim.build.rigid_conversion`` (``status: adopted`` in
``myo_sim.build.references.RAJAGOPAL2016_LEG``), MuJoCo's normalised fibre length over a full-range posture
sample must match the published rigid-tendon fibre length up to a constant slack shift from the passive calibration,
left and right maps must be identical, and the maps must stay inside the force-length bounds. The semimembranosus
check guards the regression where an oversized fit put it at ln ~1.9 standing (Rajagopal: ~0.95).
"""

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.rigid_conversion import (
    REGIONS,
    actuator_lengths,
    adopted_names,
    joint_ranges,
    joint_sweep_postures,
    load_leg_reference,
    mujoco_fiber_length,
    rigid_equivalent_fiber_length,
    sample_postures,
)

# RMSE of MuJoCo's linear map against the (slightly nonlinear, pennated) rigid-tendon reference after removing a
# constant slack shift. semimem is the worst fit (~0.12): its path spans ~2.3 optimal fibre lengths.
MAX_LN_RMSE = 0.15


@pytest.fixture(scope="module")
def leg():
    model, _ = myo_sim.load("myolegs")
    ref = load_leg_reference()
    postures = sample_postures(model, joint_ranges(model, REGIONS["leg"]["joints"]), n=200, seed=7)
    return model, ref, postures


def _act(model, name: str) -> int:
    a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert a >= 0, name
    return a


def test_adopted_maps_match_reference_fiber_length(leg):
    model, ref, postures = leg
    names = adopted_names(ref)
    acts = [_act(model, n) for n in names]
    lengths = actuator_lengths(model, postures, acts)
    failures = {}
    for j, (name, a) in enumerate(zip(names, acts)):
        p = ref["muscles"][name]
        target = rigid_equivalent_fiber_length(lengths[:, j], p["l0"], p["lts"], p["alpha"])
        mj = mujoco_fiber_length(model, a, lengths[:, j])
        offset = float(np.mean(mj - target))  # passive slack shift is a constant ln offset
        rmse = float(np.sqrt(np.mean((mj - target - offset) ** 2)))
        if not np.isfinite(rmse) or rmse > MAX_LN_RMSE:
            failures[name] = round(rmse, 3)
    assert not failures, failures


def test_adopted_maps_are_left_right_identical_and_inside_force_length_bounds(leg):
    model, ref, _ = leg
    for name in adopted_names(ref):
        r, left = _act(model, name), _act(model, name[:-2] + "_l")
        np.testing.assert_allclose(model.actuator_gainprm[r, :2], model.actuator_gainprm[left, :2], err_msg=name)
        np.testing.assert_allclose(model.actuator_lengthrange[r], model.actuator_lengthrange[left], err_msg=name)
        np.testing.assert_allclose(model.actuator_gainprm[r, :2], model.actuator_biasprm[r, :2], err_msg=name)
        r0, r1, lmin, lmax = model.actuator_gainprm[r, [0, 1, 4, 5]]
        assert lmin < r0 < 1.0 < r1 < lmax, (name, r0, r1, lmin, lmax)


def test_semimem_standing_fiber_length_near_reference(leg):
    model, ref, _ = leg
    data = mujoco.MjData(model)
    mujoco.mj_fwdPosition(model, data)  # qpos0: hip, knee and ankle at 0
    a = _act(model, "semimem_r")
    ln = float(mujoco_fiber_length(model, a, data.actuator_length[a]))
    assert 0.6 < ln < 1.3, ln  # Rajagopal2016 (OpenSim, passive): 0.95; the passive calibration shifts it by <= 0.15


def test_lengthrange_covers_single_joint_sweeps(leg):
    """lengthrange spans each joint swept alone over its range, except where an anchor was clipped to the FL bounds."""
    model, ref, _ = leg
    names = adopted_names(ref)
    sweeps = actuator_lengths(
        model, joint_sweep_postures(model, list(REGIONS["leg"]["joints"])), [_act(model, n) for n in names]
    )
    bad = {}
    for j, name in enumerate(names):
        a = _act(model, name)
        (r0, r1), (lr0, lr1), (lmin, lmax) = (
            model.actuator_gainprm[a, :2],
            model.actuator_lengthrange[a],
            model.actuator_gainprm[a, 4:6],
        )
        low_ok = sweeps[:, j].min() >= lr0 - 1e-6 or r0 <= lmin + 0.0201
        high_ok = sweeps[:, j].max() <= lr1 + 1e-6 or r1 >= lmax - 0.0201
        if not (low_ok and high_ok):
            bad[name] = (
                round(float(sweeps[:, j].min()), 4),
                round(float(lr0), 4),
                round(float(sweeps[:, j].max()), 4),
                round(float(lr1), 4),
            )
    assert not bad, bad
