# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Tests for passive joint damping, stiffness, and range-of-motion transforms."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.physics.joint_mechanics import scale_joint_mechanics
from myo_sim.physics.muscle_strength import peak_force


def _coupled_model(
    *,
    elbow_damping: float = 0.4,
    elbow_stiffness: float = 1.2,
    elbow_hi: float = 2.0,
) -> mujoco.MjModel:
    xml = f"""
    <mujoco>
      <compiler angle="radian"/>
      <worldbody>
        <site name="origin" pos="0 0 0.3"/>
        <body name="free_prop" pos="0.5 0 0.2">
          <freejoint name="prop_free"/>
          <geom size="0.03" mass="0.2"/>
        </body>
        <body name="upper" pos="0 0 0">
          <joint name="shoulder_ball" type="ball"/>
          <geom size="0.04" mass="1.5"/>
          <body name="forearm" pos="0.25 0 0">
            <joint name="elbow" type="hinge" axis="0 1 0" limited="true"
                   range="0 {elbow_hi}" damping="{elbow_damping}" stiffness="{elbow_stiffness}"/>
            <joint name="coupled_slave" type="hinge" axis="0 0 1" limited="true"
                   range="-0.8 0.8" damping="0.3" stiffness="0.5"/>
            <joint name="unlimited_twist" type="hinge" axis="1 0 0" damping="0.2" stiffness="0.4"/>
            <geom type="capsule" fromto="0 0 0 0.22 0 0" size="0.025" mass="1.0"/>
            <site name="insertion" pos="0.08 0 0.02"/>
          </body>
        </body>
      </worldbody>
      <equality>
        <joint joint1="coupled_slave" joint2="elbow" polycoef="0 0.35 0 0 0"/>
      </equality>
      <tendon>
        <spatial name="flex_ten"><site site="origin"/><site site="insertion"/></spatial>
      </tendon>
      <actuator>
        <muscle name="flexor" tendon="flex_ten" scale="220"/>
      </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def test_default_selection_skips_free_ball_and_coupled_joints_and_matches_native() -> None:
    original = _coupled_model()
    before_damp = original.dof_damping.copy()
    before_stiff = original.jnt_stiffness.copy()
    before_range = original.jnt_range.copy()
    before_force = peak_force(original)

    scaled = scale_joint_mechanics(original, damping=2.5, stiffness=1.5, range_scale=0.75)
    assert scaled is not original

    elbow_id = original.joint("elbow").id
    slave_id = original.joint("coupled_slave").id
    twist_id = original.joint("unlimited_twist").id

    assert scaled.dof_damping[original.jnt_dofadr[elbow_id]] == pytest.approx(1.0)
    assert scaled.dof_damping[original.jnt_dofadr[twist_id]] == pytest.approx(0.5)
    assert scaled.jnt_stiffness[elbow_id] == pytest.approx(1.8)
    assert scaled.jnt_stiffness[twist_id] == pytest.approx(0.6)
    np.testing.assert_allclose(scaled.jnt_range[elbow_id], [0.0, 1.5], rtol=1e-12)

    # Coupled slave joint and free/ball joints remain untouched.
    assert scaled.dof_damping[original.jnt_dofadr[slave_id]] == pytest.approx(0.3)
    assert scaled.jnt_stiffness[slave_id] == pytest.approx(0.5)
    np.testing.assert_array_equal(scaled.jnt_range[slave_id], original.jnt_range[slave_id])

    # Automatic muscle peak force and source model remain untouched.
    np.testing.assert_allclose(peak_force(scaled), before_force, rtol=1e-12)
    np.testing.assert_array_equal(original.dof_damping, before_damp)
    np.testing.assert_array_equal(original.jnt_stiffness, before_stiff)
    np.testing.assert_array_equal(original.jnt_range, before_range)

    # Subset joint selection matches native recompilation on elbow.
    elbow_only = scale_joint_mechanics(original, damping=2.5, stiffness=1.5, range_scale=0.75, joints=["elbow"])
    native = _coupled_model(elbow_damping=1.0, elbow_stiffness=1.8, elbow_hi=1.5)
    np.testing.assert_allclose(elbow_only.dof_damping, native.dof_damping, rtol=1e-12)
    np.testing.assert_allclose(elbow_only.jnt_stiffness, native.jnt_stiffness, rtol=1e-12)
    np.testing.assert_allclose(elbow_only.jnt_range, native.jnt_range, rtol=1e-12)


def test_qpos0_anchored_range_keeps_myoarm_reference_posture_within_limits() -> None:
    arm = myo_sim.load_model("myoarm_r")
    narrowed = scale_joint_mechanics(arm, damping=1.8, range_scale=0.7)
    elbow_id = arm.joint("elbow_flexion_r").id
    lo, hi = narrowed.jnt_range[elbow_id]
    q0 = narrowed.qpos0[narrowed.jnt_qposadr[elbow_id]]
    assert lo <= q0 <= hi
    assert (hi - lo) == pytest.approx(0.7 * (arm.jnt_range[elbow_id, 1] - arm.jnt_range[elbow_id, 0]))


def test_zero_damping_and_stiffness_are_supported() -> None:
    original = _coupled_model()
    frictionless = scale_joint_mechanics(original, damping=0.0, stiffness=0.0, joints=["elbow"])
    elbow_id = original.joint("elbow").id
    assert frictionless.dof_damping[original.jnt_dofadr[elbow_id]] == 0.0
    assert frictionless.jnt_stiffness[elbow_id] == 0.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"damping": -0.1},
        {"damping": np.nan},
        {"damping": np.inf},
        {"stiffness": -1.0},
        {"stiffness": np.nan},
        {"range_scale": 0.0},
        {"range_scale": -0.5},
        {"range_scale": np.nan},
        {"range_scale": np.inf},
        {"range_scale": 1e-20},
        {"joints": []},
        {"joints": ["elbow", "elbow"]},
        {"joints": ["coupled_slave"]},
        {"joints": ["shoulder_ball"]},
        {"joints": ["prop_free"]},
        {"joints": ["missing_joint"]},
        {"range_scale": 0.8, "joints": ["unlimited_twist"]},
        {"damping": np.array([1.0, 2.0, 3.0])},
    ],
)
def test_invalid_inputs_leave_source_model_unchanged(kwargs: dict) -> None:
    original = _coupled_model()
    before_damp = original.dof_damping.copy()
    before_stiff = original.jnt_stiffness.copy()
    before_range = original.jnt_range.copy()
    with pytest.raises((ValueError, KeyError)):
        scale_joint_mechanics(original, **kwargs)
    np.testing.assert_array_equal(original.dof_damping, before_damp)
    np.testing.assert_array_equal(original.jnt_stiffness, before_stiff)
    np.testing.assert_array_equal(original.jnt_range, before_range)
