# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Selected mass transforms match native recompilation and retain muscle force."""

import mujoco
import numpy as np
import pytest

from myo_sim.physics.body_mass import scale_body_mass


def _model(arm_mass: float = 3.0) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(f"""<mujoco><compiler angle="radian"/>
      <worldbody><body name="torso"><geom size=".1" mass="20"/>
        <body name="arm"><joint name="elbow" axis="0 1 0" range="0 3"/>
          <geom type="capsule" size=".04" fromto="0 0 0 1 0 0" mass="{arm_mass}"/>
          <body name="child" pos="1 0 0"><joint name="dependent" range="0 6"/>
            <geom size=".05" mass="1"/></body></body></body>
        <body name="object"><geom size=".1" mass="2"/></body>
        <body name="attachment"/>
      </worldbody><equality><joint joint1="dependent" joint2="elbow" polycoef="0 2 0 0 0"/></equality>
      <actuator><muscle name="automatic" joint="elbow" gear="-1" lengthrange="-3 0" force="-1"/>
        <muscle name="explicit" joint="elbow" lengthrange="0 3" force="80"/>
        <motor name="motor" joint="elbow" gear="3"/></actuator></mujoco>""")


def _data(model: mujoco.MjModel, activation: float) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[:] = [0.1, 0.2]
    data.act[:] = activation
    data.ctrl[:] = activation
    mujoco.mj_forward(model, data)
    return data


def test_partial_mass_modes_are_distinct_and_preserve_automatic_strength() -> None:
    original = _model()
    ratio = scale_body_mass(original, participant_mass=90, reference_mass=60, bodies=["arm"])
    represented = scale_body_mass(original, represented_mass=6, bodies=["arm"])
    assert ratio is not original
    assert ratio.body("arm").mass[0] == pytest.approx(4.5)
    assert represented.body("arm").mass[0] == pytest.approx(6)
    for scaled in (ratio, represented):
        for name in ("torso", "child", "object", "attachment"):
            np.testing.assert_array_equal(scaled.body(name).mass, original.body(name).mass)
            np.testing.assert_array_equal(scaled.body(name).inertia, original.body(name).inertia)
        for activation in (0.0, 0.4, 1.0):
            np.testing.assert_allclose(
                _data(scaled, activation).actuator_force, _data(original, activation).actuator_force, rtol=1e-12
            )
        assert scaled.actuator_gainprm[0, 2] > 0
        assert scaled.actuator_biasprm[0, 2] > 0
        for field in ("body_ipos", "body_iquat", "body_pos", "geom_size", "actuator_forcerange"):
            np.testing.assert_array_equal(getattr(scaled, field), getattr(original, field))
        np.testing.assert_array_equal(scaled.actuator_gainprm[-1], original.actuator_gainprm[-1])
        np.testing.assert_array_equal(scaled.actuator_biasprm[-1], original.actuator_biasprm[-1])


def test_refreshed_constants_match_native_recompilation_with_joint_coupling() -> None:
    original = _model()
    before = {
        name: getattr(original, name).copy()
        for name in ("body_mass", "body_inertia", "body_subtreemass", "actuator_acc0", "actuator_gainprm", "actuator_biasprm")
    }
    scaled = scale_body_mass(original, represented_mass=6, bodies=["arm"])
    native = _model(arm_mass=6)
    for field in ("body_mass", "body_inertia", "body_subtreemass", "body_invweight0", "actuator_acc0"):
        np.testing.assert_allclose(getattr(scaled, field), getattr(native, field), rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(_data(scaled, 0.4).qM, _data(native, 0.4).qM, rtol=1e-12)
    assert scaled.stat.meanmass == pytest.approx(native.stat.meanmass)
    assert scaled.stat.meaninertia == pytest.approx(native.stat.meaninertia)
    assert scaled.body_subtreemass[scaled.body("torso").id] == pytest.approx(27)
    assert scaled.body_subtreemass[scaled.body("arm").id] == pytest.approx(7)
    assert not np.array_equal(scaled.actuator_acc0, original.actuator_acc0)
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(original, name), value)


def test_default_selection_includes_fixed_bodies_and_empty_muscle_models() -> None:
    original = _model()
    scaled = scale_body_mass(original, represented_mass=52)
    np.testing.assert_allclose(scaled.body_mass, 2 * original.body_mass, rtol=1e-12)
    np.testing.assert_allclose(scaled.body_inertia, 2 * original.body_inertia, rtol=1e-12)
    no_muscle = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="a"><geom size=".1" mass="1"/></body></worldbody></mujoco>'
    )
    assert scale_body_mass(no_muscle, represented_mass=2).body("a").mass[0] == pytest.approx(2)


@pytest.mark.parametrize(
    "kwargs",
    [
        {},
        {"represented_mass": 6, "participant_mass": 90, "reference_mass": 60},
        {"participant_mass": 90},
        {"reference_mass": 60},
        {"represented_mass": 0},
        {"represented_mass": -1},
        {"represented_mass": np.nan},
        {"represented_mass": np.inf},
        {"participant_mass": 90, "reference_mass": 0},
        {"participant_mass": np.inf, "reference_mass": 60},
        {"represented_mass": 6, "bodies": []},
        {"represented_mass": 6, "bodies": ["world"]},
        {"represented_mass": 6, "bodies": ["arm", "arm"]},
        {"represented_mass": 6, "bodies": ["attachment"]},
        {"represented_mass": 6, "bodies": ["missing"]},
    ],
)
def test_invalid_mass_modes_and_body_selections_leave_source_unchanged(kwargs: dict) -> None:
    original = _model()
    before = original.body_mass.copy()
    gain = original.actuator_gainprm.copy()
    with pytest.raises((ValueError, KeyError)):
        scale_body_mass(original, **kwargs)
    np.testing.assert_array_equal(original.body_mass, before)
    np.testing.assert_array_equal(original.actuator_gainprm, gain)
