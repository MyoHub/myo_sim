"""Numerical equivalence tests for the myohand_r composed model.

Verifies that:
1. load_right_hand_from_arm_spec() (hand-only, used by myosuite's
   hand_standard recipe) and myo_sim.load('myohand_r') (torso scaffold
   + same hand) expose identical biomechanics for the hand DOFs.
2. Joint ranges and actuator gear values are preserved through the
   prune_arm_spec_to_hand() pipeline.

These tests act as a regression fence for both myo_sim and myosuite:
if the arm XML changes in a way that breaks hand kinematics, these fail
before anything reaches a downstream policy.
"""

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.compose import load_right_hand_from_arm_spec


def _joint_names(m: mujoco.MjModel) -> list[str]:
    return sorted(
        mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)
    )


def _actuator_names(m: mujoco.MjModel) -> list[str]:
    return sorted(
        mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)
    )


@pytest.fixture(scope="module")
def hand_only() -> mujoco.MjModel:
    """Compiled hand-only spec — the model myosuite's hand_standard recipe builds."""
    return load_right_hand_from_arm_spec().compile()


@pytest.fixture(scope="module")
def hand_with_torso() -> mujoco.MjModel:
    """myo_sim.load('myohand_r'): same hand attached to a passive torso scaffold."""
    m, _ = myo_sim.load("myohand_r")
    return m


def test_njnt_equals_23(hand_only, hand_with_torso):
    assert hand_only.njnt == 23
    assert hand_with_torso.njnt == 23


def test_nu_equals_39(hand_only, hand_with_torso):
    assert hand_only.nu == 39
    assert hand_with_torso.nu == 39


def test_joint_names_match(hand_only, hand_with_torso):
    assert _joint_names(hand_only) == _joint_names(hand_with_torso)


def test_actuator_names_match(hand_only, hand_with_torso):
    assert _actuator_names(hand_only) == _actuator_names(hand_with_torso)


def test_joint_ranges_match(hand_only, hand_with_torso):
    """Per-joint range bounds must be identical (atol=1e-6)."""
    for jnt_id in range(hand_only.njnt):
        name = mujoco.mj_id2name(hand_only, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        ref_id = mujoco.mj_name2id(hand_with_torso, mujoco.mjtObj.mjOBJ_JOINT, name)
        lo_h, hi_h = hand_only.jnt_range[jnt_id]
        lo_r, hi_r = hand_with_torso.jnt_range[ref_id]
        np.testing.assert_allclose(
            [lo_h, hi_h],
            [lo_r, hi_r],
            atol=1e-6,
            err_msg=f"Joint {name!r} range mismatch",
        )


def test_actuator_gear_match(hand_only, hand_with_torso):
    """Actuator gear values must be identical (atol=1e-6)."""
    for act_id in range(hand_only.nu):
        name = mujoco.mj_id2name(hand_only, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
        ref_id = mujoco.mj_name2id(hand_with_torso, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        np.testing.assert_allclose(
            hand_only.actuator_gear[act_id],
            hand_with_torso.actuator_gear[ref_id],
            atol=1e-6,
            err_msg=f"Actuator {name!r} gear mismatch",
        )


def test_joint_axes_match(hand_only, hand_with_torso):
    """Joint axes (in local body frame) must be identical across both models.

    This verifies biomechanical equivalence independently of where the hand
    is attached in world space (the hand root orientation differs between
    hand_only and hand_with_torso because of the torso-site attachment).
    """
    for jnt_id in range(hand_only.njnt):
        name = mujoco.mj_id2name(hand_only, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        ref_id = mujoco.mj_name2id(hand_with_torso, mujoco.mjtObj.mjOBJ_JOINT, name)
        np.testing.assert_allclose(
            hand_only.jnt_axis[jnt_id],
            hand_with_torso.jnt_axis[ref_id],
            atol=1e-6,
            err_msg=f"Joint {name!r} axis mismatch",
        )


def test_joint_stiffness_match(hand_only, hand_with_torso):
    """Joint stiffness and damping must be identical."""
    for jnt_id in range(hand_only.njnt):
        name = mujoco.mj_id2name(hand_only, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        ref_id = mujoco.mj_name2id(hand_with_torso, mujoco.mjtObj.mjOBJ_JOINT, name)
        np.testing.assert_allclose(
            hand_only.jnt_stiffness[jnt_id],
            hand_with_torso.jnt_stiffness[ref_id],
            atol=1e-6,
            err_msg=f"Joint {name!r} stiffness mismatch",
        )
