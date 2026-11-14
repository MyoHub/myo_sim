# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Strength primitives reproduce native active and passive force scaling."""

import copy

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.physics.muscle_strength import peak_force, scale_muscle_strength, scale_muscle_strength_spec


def _spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string("""<mujoco><compiler angle="radian"/>
      <worldbody><body><joint name="elbow" range="0 3"/>
        <geom type="capsule" size=".04" fromto="0 0 0 1 0 0" mass="3"/>
      </body></worldbody><actuator>
        <muscle name="automatic" joint="elbow" gear="-1" lengthrange="-3 0" force="-1" scale="200"/>
        <muscle name="explicit" joint="elbow" lengthrange="0 3" force="80"/>
        <motor name="motor" joint="elbow" gear="3"/>
      </actuator></mujoco>""")


def _forces(model: mujoco.MjModel, angle: float, activation: float) -> np.ndarray:
    data = mujoco.MjData(model)
    data.qpos[:] = angle
    data.act[:] = activation
    data.ctrl[:] = activation
    mujoco.mj_forward(model, data)
    return data.actuator_force.copy()


def test_packaged_elbow_active_and_passive_forces_follow_individual_factors() -> None:
    original = myo_sim.load_model("myoelbow")
    scaled = copy.deepcopy(original)
    factors = np.random.default_rng(0).uniform(0.5, 2.0, original.nu)
    assert scale_muscle_strength(scaled, factors) is None
    for activation in (0.0, 0.4, 1.0):
        np.testing.assert_allclose(_forces(scaled, 0.3, activation), factors * _forces(original, 0.3, activation), rtol=1e-12)
    assert np.max(np.abs(_forces(original, 0.3, 0.0))) > 1


def test_automatic_gain_and_bias_and_spec_match_native_forces() -> None:
    spec = _spec()
    spec.actuator("automatic").biasprm[3] = 300  # Independent native passive force scale.
    original = spec.copy().compile()
    scaled = copy.deepcopy(original)
    factors = np.array([1.7, 0.6, np.nan])
    scale_muscle_strength(scaled, factors)
    assert scale_muscle_strength_spec(spec, factors) is spec
    from_spec = spec.compile()
    for angle in (0.1, 1.2, 2.8):
        for activation in (0.0, 1.0):
            expected = np.array([1.7, 0.6, 1.0]) * _forces(original, angle, activation)
            np.testing.assert_allclose(_forces(scaled, angle, activation), expected, rtol=1e-12, atol=1e-12)
            np.testing.assert_allclose(_forces(from_spec, angle, activation), expected, rtol=1e-12, atol=1e-12)
    np.testing.assert_array_equal(scaled.actuator_gainprm[-1], original.actuator_gainprm[-1])
    np.testing.assert_array_equal(scaled.actuator_biasprm[-1], original.actuator_biasprm[-1])
    assert scaled.actuator_biasprm[0, 2] == pytest.approx(1.5 * scaled.actuator_gainprm[0, 2])


def test_peak_force_resolves_auto_without_aliasing_or_changing_motor_parameters() -> None:
    model = _spec().compile()
    model.actuator_gainprm[-1, 2] = -7
    expected = model.actuator_gainprm.copy()
    resolved = peak_force(model)
    assert resolved[0] == pytest.approx(200 / max(mujoco.mjMINVAL, model.actuator_acc0[0]))
    assert resolved[1] == 80
    assert resolved[2] == -7
    resolved[:] = 0
    np.testing.assert_array_equal(model.actuator_gainprm, expected)


@pytest.mark.parametrize("bad", [0, -1, np.nan, np.inf, [1, 0, 1], [1, 2], 1e308])
def test_invalid_factors_are_transactional_for_compiled_and_spec(bad: float | list[float]) -> None:
    spec = _spec()
    model = spec.copy().compile()
    gain, bias = model.actuator_gainprm.copy(), model.actuator_biasprm.copy()
    text = spec.to_xml()
    with pytest.raises(ValueError):
        scale_muscle_strength(model, bad)
    with pytest.raises(ValueError):
        scale_muscle_strength_spec(spec, bad)
    np.testing.assert_array_equal(model.actuator_gainprm, gain)
    np.testing.assert_array_equal(model.actuator_biasprm, bias)
    assert spec.to_xml() == text


def test_non_muscle_bias_law_and_non_force_parameters_are_unchanged() -> None:
    model = _spec().compile()
    model.actuator_biastype[0] = mujoco.mjtBias.mjBIAS_AFFINE
    gain, bias = model.actuator_gainprm.copy(), model.actuator_biasprm.copy()
    scale_muscle_strength(model, 2.0)
    np.testing.assert_array_equal(model.actuator_biasprm[0], bias[0])
    np.testing.assert_array_equal(model.actuator_gainprm[:, [0, 1, 3, 4, 5, 6, 7, 8, 9]], gain[:, [0, 1, 3, 4, 5, 6, 7, 8, 9]])


def test_model_without_muscles_and_scalar_identity_are_supported() -> None:
    empty = mujoco.MjModel.from_xml_string("<mujoco/>")
    scale_muscle_strength(empty, 1.0)
    assert peak_force(empty).shape == (0,)
    original = _spec().compile()
    scaled = copy.deepcopy(original)
    scale_muscle_strength(scaled, 1.0)
    np.testing.assert_allclose(_forces(scaled, 0.1, 0.4), _forces(original, 0.1, 0.4), rtol=1e-12)
