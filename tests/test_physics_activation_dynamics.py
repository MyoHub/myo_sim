# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Tests for Hill-type muscle activation and deactivation time-constant transforms."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.physics.activation_dynamics import (
    activation_time_constants,
    scale_activation_time_constants,
    scale_activation_time_constants_spec,
)
from myo_sim.physics.muscle_strength import peak_force

_MIXED_XML = """
<mujoco>
  <option timestep="0.002"/>
  <default>
    <muscle timeconst="0.012 0.045"/>
  </default>
  <worldbody>
    <site name="origin" pos="0 0 0.25"/>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" range="-1.2 1.2" limited="true"/>
      <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.02" mass="1.5"/>
      <site name="ins_a" pos="0.08 0 0.02"/>
      <site name="ins_b" pos="0.10 0 -0.02"/>
      <site name="ins_stateless" pos="0.12 0 0.02"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="ten_a"><site site="origin"/><site site="ins_a"/></spatial>
    <spatial name="ten_b"><site site="origin"/><site site="ins_b"/></spatial>
    <spatial name="ten_stateless"><site site="origin"/><site site="ins_stateless"/></spatial>
  </tendon>
  <actuator>
    <muscle name="flexor" tendon="ten_a" force="180"/>
    <muscle name="extensor" tendon="ten_b" timeconst="0.015 0.055"/>
    <general name="stateless_muscle" tendon="ten_stateless" dyntype="none" gaintype="muscle" biastype="muscle"/>
    <motor name="assist_motor" joint="hinge" gear="8"/>
  </actuator>
</mujoco>
"""


def _spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string(_MIXED_XML)


def test_activation_time_constants_returns_owned_copies() -> None:
    model = _spec().compile()
    tau_act, tau_deact = activation_time_constants(model)
    assert tau_act.shape == (model.nu,)
    assert tau_deact.shape == (model.nu,)
    assert tau_act[0] == pytest.approx(0.012)
    assert tau_deact[0] == pytest.approx(0.045)
    assert tau_act[1] == pytest.approx(0.015)
    assert tau_deact[1] == pytest.approx(0.055)
    tau_act[:] = 99.0
    tau_deact[:] = 88.0
    assert model.actuator_dynprm[0, 0] == pytest.approx(0.012)
    assert model.actuator_dynprm[0, 1] == pytest.approx(0.045)


def test_scalar_and_vector_scaling_matches_spec_and_preserves_non_muscle_actuators() -> None:
    spec = _spec()
    model = spec.copy().compile()
    before_gain = peak_force(model)
    before_bias = peak_force(model, model.actuator_biasprm)
    before_non_muscle = model.actuator_dynprm[2:].copy()
    before_tausmooth = model.actuator_dynprm[:, 2:].copy()

    act_factors = np.array([1.5, 2.0, 999.0, -5.0])
    deact_factors = np.array([2.5, 0.8, 0.0, np.nan])
    scale_activation_time_constants(model, act_factors, deact_factors)

    returned = scale_activation_time_constants_spec(spec, act_factors, deact_factors)
    assert returned is spec
    from_spec = spec.compile()

    np.testing.assert_allclose(model.actuator_dynprm, from_spec.actuator_dynprm, rtol=1e-12, atol=1e-12)
    assert model.actuator_dynprm[0, 0] == pytest.approx(0.018)
    assert model.actuator_dynprm[0, 1] == pytest.approx(0.1125)
    assert model.actuator_dynprm[1, 0] == pytest.approx(0.030)
    assert model.actuator_dynprm[1, 1] == pytest.approx(0.044)
    np.testing.assert_array_equal(model.actuator_dynprm[2:], before_non_muscle)
    np.testing.assert_array_equal(model.actuator_dynprm[:, 2:], before_tausmooth)
    np.testing.assert_allclose(peak_force(model), before_gain, rtol=1e-12)
    np.testing.assert_allclose(peak_force(model, model.actuator_biasprm), before_bias, rtol=1e-12)


def test_omitted_deact_factors_scales_both_taus_together_on_registered_model() -> None:
    model = myo_sim.load_model("myoelbow")
    tau_act_0, tau_deact_0 = activation_time_constants(model)
    scale_activation_time_constants(model, 1.75)
    tau_act_1, tau_deact_1 = activation_time_constants(model)
    np.testing.assert_allclose(tau_act_1, 1.75 * tau_act_0, rtol=1e-12)
    np.testing.assert_allclose(tau_deact_1, 1.75 * tau_deact_0, rtol=1e-12)


def test_scaled_time_constants_change_transient_ode_but_preserve_steady_force() -> None:
    fast = _spec().compile()
    slow = _spec().compile()
    scale_activation_time_constants(slow, act_factors=2.5, deact_factors=3.0)

    d_fast = mujoco.MjData(fast)
    d_slow = mujoco.MjData(slow)
    d_fast.ctrl[:2] = 1.0
    d_slow.ctrl[:2] = 1.0
    for _ in range(10):
        mujoco.mj_step(fast, d_fast)
        mujoco.mj_step(slow, d_slow)
    assert np.all(d_slow.act[:2] < d_fast.act[:2])

    # Steady-state isometric force at identical activation and posture is unchanged.
    d_fast = mujoco.MjData(fast)
    d_slow = mujoco.MjData(slow)
    d_fast.act[:] = 0.85
    d_slow.act[:] = 0.85
    mujoco.mj_forward(fast, d_fast)
    mujoco.mj_forward(slow, d_slow)
    np.testing.assert_allclose(d_slow.actuator_force, d_fast.actuator_force, rtol=1e-12)

    # Relaxation from full activation with zero excitation is slower when tau_deact is larger.
    d_fast.ctrl[:] = 0.0
    d_slow.ctrl[:] = 0.0
    for _ in range(20):
        mujoco.mj_step(fast, d_fast)
        mujoco.mj_step(slow, d_slow)
    assert np.all(d_slow.act[:2] > d_fast.act[:2])


@pytest.mark.parametrize(
    ("act_factors", "deact_factors"),
    [
        (0.0, None),
        (-1.0, None),
        (np.nan, None),
        (np.inf, None),
        (1e-320, None),
        (1.0, 0.0),
        (1.0, -0.5),
        (1.0, np.nan),
        (1.0, np.inf),
        (np.array([1.0, 2.0]), None),
        (1.0, np.array([1.0, 2.0])),
    ],
)
def test_invalid_factors_leave_model_and_spec_unchanged(
    act_factors: float | np.ndarray,
    deact_factors: float | np.ndarray | None,
) -> None:
    spec = _spec()
    model = spec.copy().compile()
    before_model = model.actuator_dynprm.copy()
    before_spec = np.array([a.dynprm.copy() for a in spec.actuators])

    with pytest.raises(ValueError):
        scale_activation_time_constants(model, act_factors, deact_factors)
    np.testing.assert_array_equal(model.actuator_dynprm, before_model)

    with pytest.raises(ValueError):
        scale_activation_time_constants_spec(spec, act_factors, deact_factors)
    np.testing.assert_array_equal(np.array([a.dynprm.copy() for a in spec.actuators]), before_spec)


def test_overflow_factor_leaves_model_unchanged() -> None:
    model = _spec().compile()
    model.actuator_dynprm[0, 1] = 2.0
    before = model.actuator_dynprm.copy()
    with pytest.raises(ValueError):
        scale_activation_time_constants(model, 1.0, 1e308)
    np.testing.assert_array_equal(model.actuator_dynprm, before)
