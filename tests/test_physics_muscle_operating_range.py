# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Muscle operating-range transform follows MuJoCo's native muscle length normalisation."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from myo_sim.physics.muscle_operating_range import scale_muscle_operating_range

_XML = """<mujoco><compiler angle="radian"/>
  <worldbody><body><joint name="elbow" range="0 3"/>
    <geom type="capsule" size=".04" fromto="0 0 0 1 0 0" mass="3"/></body></worldbody>
  <actuator>
    <muscle name="a" joint="elbow" gear="-1" lengthrange="-3 -1" force="100"/>
    <muscle name="b" joint="elbow" lengthrange="0.5 2.5" force="80"/>
    <motor name="motor" joint="elbow" gear="3"/>
  </actuator></mujoco>"""


@pytest.fixture
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(_XML)


def _gain(model: mujoco.MjModel, actuator: int, length: float) -> float:
    return mujoco.mju_muscleGain(
        length, 0.0, model.actuator_lengthrange[actuator], model.actuator_acc0[actuator], model.actuator_gainprm[actuator, :9]
    )


def _optimal_length(model: mujoco.MjModel, actuator: int) -> float:
    low, high = model.actuator_lengthrange[actuator]
    return (high - low) / (model.actuator_gainprm[actuator, 1] - model.actuator_gainprm[actuator, 0])


def test_transform_is_exact_and_leaves_other_rows_unchanged(model: mujoco.MjModel) -> None:
    original = model.actuator_lengthrange.copy()
    gain, bias = model.actuator_gainprm.copy(), model.actuator_biasprm.copy()
    scale_muscle_operating_range(model, np.array([2.0, 0.5, 9.0]), shift=np.array([0.1, -0.2, np.nan]), muscles=["b"])
    np.testing.assert_array_equal(model.actuator_lengthrange[[0, 2]], original[[0, 2]])
    np.testing.assert_allclose(model.actuator_lengthrange[1], [1.5 - 0.2 - 0.5, 1.5 - 0.2 + 0.5], rtol=1e-12)
    np.testing.assert_array_equal(model.actuator_gainprm, gain)
    np.testing.assert_array_equal(model.actuator_biasprm, bias)
    scale_muscle_operating_range(model, 2.0, shift=0.05)
    np.testing.assert_array_equal(model.actuator_lengthrange[2], original[2])
    np.testing.assert_allclose(model.actuator_lengthrange[0], [-2 + 0.05 - 2, -2 + 0.05 + 2], rtol=1e-12)


def test_shift_moves_and_span_rescales_the_native_muscle_curve(model: mujoco.MjModel) -> None:
    original = mujoco.MjModel.from_xml_string(_XML)
    lengths = np.linspace(0.3, 2.7, 9)
    expected = [_gain(original, 1, length) for length in lengths]
    scale_muscle_operating_range(model, shift=0.15, muscles=["b"])
    np.testing.assert_allclose([_gain(model, 1, length + 0.15) for length in lengths], expected, rtol=1e-9)
    assert _optimal_length(model, 1) == pytest.approx(_optimal_length(original, 1))
    scale_muscle_operating_range(model, 1.3, muscles=["b"])
    assert _optimal_length(model, 1) == pytest.approx(1.3 * _optimal_length(original, 1))
    assert model.actuator_gainprm[1, 2] == original.actuator_gainprm[1, 2]


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"span": 0.0}, "span"),
        ({"span": np.nan}, "span"),
        ({"span": np.array([1.0, -1.0, 1.0])}, "span"),
        ({"span": np.ones(2)}, None),
        ({"shift": np.inf}, "shift"),
        ({"span": 1e308, "shift": 1e308}, "finite"),
        ({"muscles": ["missing"]}, "muscle"),
        ({"muscles": ["a", "a"]}, "muscle"),
        ({"muscles": ["motor"]}, "muscle"),
        ({"muscles": "a"}, "muscle"),
    ],
)
def test_invalid_input_leaves_the_model_unchanged(model: mujoco.MjModel, kwargs: dict, match: str) -> None:
    original = model.actuator_lengthrange.copy()
    with pytest.raises(ValueError, match=match):
        scale_muscle_operating_range(model, **kwargs)
    np.testing.assert_array_equal(model.actuator_lengthrange, original)


@pytest.mark.parametrize("lengthrange", [(1.0, 1.0), (2.0, 1.0), (0.0, np.inf)])
def test_degenerate_selected_range_is_rejected_but_unselected_is_ignored(model: mujoco.MjModel, lengthrange: tuple) -> None:
    model.actuator_lengthrange[0] = lengthrange
    original = model.actuator_lengthrange.copy()
    with pytest.raises(ValueError, match="increasing"):
        scale_muscle_operating_range(model)
    scale_muscle_operating_range(model, 2.0, muscles=["b"])
    np.testing.assert_array_equal(model.actuator_lengthrange[0], original[0])


def test_model_without_muscles_is_a_no_op() -> None:
    empty = mujoco.MjModel.from_xml_string("<mujoco/>")
    scale_muscle_operating_range(empty, 2.0, shift=1.0)
