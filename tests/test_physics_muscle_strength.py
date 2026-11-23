# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Strength transforms and MVC torque calibration reproduce native MuJoCo forces."""

from __future__ import annotations

import copy

import mujoco
import numpy as np
import pytest
from scipy.optimize import lsq_linear

import myo_sim
from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings
from myo_sim.physics.muscle_strength import (
    MaxTorqueMeasurement,
    _bvls,
    calibrate_muscle_strength,
    max_joint_torque,
    max_torque_terms,
    peak_force,
    scale_muscle_strength,
    scale_muscle_strength_spec,
    strength_factor_from_body_mass,
)

_FLEXORS, _EXTENSORS = ("BIClong", "BICshort", "BRA"), ("TRIlong", "TRIlat", "TRImed")

_MVC_XML = """
<mujoco>
  <worldbody>
    <site name="origin" pos="0 0 .1"/>
    <body><joint name="hinge" axis="0 1 0" range="-1 1" stiffness="4"/>
      <geom type="capsule" fromto="0 0 0 .3 0 0" size=".02" mass="1"/>
      <site name="insertion" pos=".2 0 .05"/></body>
  </worldbody>
  <tendon><spatial name="path"><site site="origin"/><site site="insertion"/></spatial></tendon>
  <actuator>
    <muscle name="m" tendon="path" force="-1" scale="200"/>
    <motor name="motor" joint="hinge" gear="3"/>
  </actuator>
</mujoco>
"""


def _spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string("""<mujoco><compiler angle="radian"/>
      <worldbody><body><joint name="elbow" range="0 3"/>
        <geom type="capsule" size=".04" fromto="0 0 0 1 0 0" mass="3"/>
      </body></worldbody><actuator>
        <muscle name="automatic" joint="elbow" gear="-1" lengthrange="-3 0" force="-1" scale="200"/>
        <muscle name="explicit" joint="elbow" lengthrange="0 3" force="80"/>
        <motor name="motor" joint="elbow" gear="3"/>
      </actuator></mujoco>""")


def _forces(model: mujoco.MjModel, qpos: float | np.ndarray, activation: float) -> np.ndarray:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.act[:] = activation
    data.ctrl[:] = activation
    mujoco.mj_forward(model, data)
    return data.actuator_force.copy()


@pytest.fixture(scope="module")
def elbow() -> mujoco.MjModel:
    return myo_sim.load_model("myoelbow")


def test_packaged_elbow_active_and_passive_forces_follow_individual_factors(elbow: mujoco.MjModel) -> None:
    scaled = copy.deepcopy(elbow)
    factors = np.random.default_rng(0).uniform(0.5, 2.0, elbow.nu)
    assert scale_muscle_strength(scaled, factors) is None
    for activation in (0.0, 0.4, 1.0):
        np.testing.assert_allclose(_forces(scaled, 0.3, activation), factors * _forces(elbow, 0.3, activation), rtol=1e-12)
    assert np.max(np.abs(_forces(elbow, 0.3, 0.0))) > 1


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


def test_trial_torque_matches_mujoco(elbow: mujoco.MjModel) -> None:
    trial = MaxTorqueMeasurement("r_elbow_flex", np.array([1.57]), torque=50.0, direction=-1.0)
    active, passive, joints = max_torque_terms(elbow, trial)
    data = mujoco.MjData(elbow)
    data.qpos[:] = trial.qpos
    data.act[:] = active > 0
    mujoco.mj_forward(elbow, data)
    mujoco_torque = -(data.qfrc_actuator + data.qfrc_passive)[0]
    np.testing.assert_allclose(active[active > 0].sum() + passive.sum() + joints, mujoco_torque, rtol=1e-10)
    assert {elbow.actuator(i).name for i in np.flatnonzero(active > 0)} == set(_EXTENSORS)


def test_trial_torque_includes_joint_springs() -> None:
    model = mujoco.MjModel.from_xml_string(_MVC_XML)
    trial = MaxTorqueMeasurement("hinge", np.array([0.4]), torque=5.0)
    active, passive, joints = max_torque_terms(model, trial)
    assert joints == pytest.approx(-4 * 0.4)
    data = mujoco.MjData(model)
    data.qpos[:], data.act[:] = trial.qpos, (active > 0)[0]
    mujoco.mj_forward(model, data)
    mujoco_torque = data.actuator_force[0] * data.actuator_moment[0] + data.qfrc_passive[0]
    np.testing.assert_allclose(max(active[0], 0) + passive[0] + joints, mujoco_torque, rtol=1e-10)


def test_coupled_trial_torque_is_the_virtual_work_of_the_muscles() -> None:
    model = myo_sim.load_model("myoarm_r")
    couplings = JointCouplings.from_model(model)
    qpos = model.qpos0.copy()
    adr = model.joint("shoulder_elv_r").qposadr[0]
    qpos[adr] = 0.8
    active, _, _ = max_torque_terms(model, MaxTorqueMeasurement("shoulder_elv_r", qpos, torque=40.0))
    h = 1e-6
    length = []
    for sign in (-1, 1):
        q = qpos.copy()
        q[adr] += sign * h
        project_joint_couplings(couplings, q)
        data = mujoco.MjData(model)
        data.qpos[:] = q
        mujoco.mj_fwdPosition(model, data)
        length.append(data.actuator_length.copy())
    centre = qpos.copy()
    project_joint_couplings(couplings, centre)
    active_force = _forces(model, centre, 1.0) - _forces(model, centre, 0.0)
    virtual = active_force * (length[1] - length[0]) / (2 * h)
    np.testing.assert_allclose(active, virtual, rtol=1e-4, atol=1e-3)
    assert np.abs(active).max() > 10.0


def test_calibration_recovers_a_known_subject_and_matches_scipy(elbow: mujoco.MjModel) -> None:
    truth = np.array([1.3 if elbow.actuator(i).name in _FLEXORS else 0.8 for i in range(elbow.nu)])
    subject = copy.deepcopy(elbow)
    scale_muscle_strength(subject, truth)
    trials = []
    for angle in (0.5, 1.2, 1.9):
        for direction in (1.0, -1.0):
            probe = MaxTorqueMeasurement("r_elbow_flex", np.array([angle]), 1.0, direction)
            active, passive, joints = max_torque_terms(subject, probe)
            torque = active[active > 0].sum() + passive.sum() + joints
            trials.append(MaxTorqueMeasurement("r_elbow_flex", np.array([angle]), torque, direction))
    cal = calibrate_muscle_strength(elbow, trials)
    np.testing.assert_allclose(cal.predicted, cal.measured, rtol=5e-3)
    assert np.abs(cal.nominal / cal.measured - 1).max() > 0.15
    for group, value in ((_FLEXORS, 1.3), (_EXTENSORS, 0.8)):
        ids = [elbow.actuator(n).id for n in group]
        assert abs(cal.factors[ids].mean() - value) < 0.05


@pytest.mark.parametrize("direction", [1.0, -1.0])
def test_calibration_includes_zero_input_non_muscle_forces(direction: float) -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><compiler angle="radian"/><worldbody><body>'
        '<joint name="elbow" range="0 3" stiffness="2"/><geom size=".1" mass="1"/>'
        "</body></worldbody><actuator>"
        '<muscle name="agonist" joint="elbow" gear="-1" lengthrange="-3 0" force="100"/>'
        '<position name="brace" joint="elbow" kp="10"/>'
        "</actuator></mujoco>"
    )
    # Reverse the muscle without changing its operating length or the brace.
    if direction < 0:
        model.actuator_gear[0, 0] = 1.0
        model.actuator_lengthrange[0] = [0.0, 3.0]
    trial = MaxTorqueMeasurement("elbow", np.array([1.0]), 50.0, direction)
    original = copy.deepcopy(model)
    calibration = calibrate_muscle_strength(model, [trial], regularization=0.0)
    np.testing.assert_array_equal(model.actuator_gainprm, original.actuator_gainprm)
    np.testing.assert_array_equal(model.actuator_biasprm, original.actuator_biasprm)
    assert calibration.factors[1] == 1.0
    np.testing.assert_array_equal(calibration.involved, [True, False])
    for candidate, expected in ((original, calibration.nominal), (model, calibration.predicted)):
        if candidate is model:
            scale_muscle_strength(candidate, calibration.factors)
        data = mujoco.MjData(candidate)
        data.qpos[:] = trial.qpos
        data.act[:] = 1.0
        # Zero control still leaves the brace's -10 N m position bias.
        mujoco.mj_forward(candidate, data)
        native = direction * (data.qfrc_actuator[0] + data.qfrc_passive[0])
        np.testing.assert_allclose(expected, [native], rtol=1e-9)
    np.testing.assert_allclose(calibration.predicted, calibration.measured, rtol=1e-9)


def test_bvls_matches_scipy_lsq_linear_across_regimes() -> None:
    rng = np.random.default_rng(42)
    for m, n in ((8, 5), (3, 12), (10, 10)):
        a = rng.normal(size=(m, n))
        b = rng.normal(size=m) * 4.0
        for lo, hi in ((0.25, 4.0), (0.8, 1.2), (0.99, 1.01)):
            expected = lsq_linear(a, b, bounds=(lo, hi), method="bvls", tol=1e-14).x
            actual = _bvls(a, b, lo, hi)
            np.testing.assert_allclose(actual, expected, rtol=1e-10, atol=1e-10)
    assert np.allclose(_bvls(rng.normal(size=(4, 3)), np.ones(4), 1.0, 1.0), 1.0)
    # Collinear columns (rank-deficient, unregularized): objective value and KKT conditions match.
    col = rng.normal(size=(6, 1))
    a_rank1 = np.hstack([col, col, 2.0 * col])
    b_rank1 = (col[:, 0] * 5.0) + rng.normal(scale=0.1, size=6)
    expected = lsq_linear(a_rank1, b_rank1, bounds=(0.25, 4.0), method="bvls").x
    actual = _bvls(a_rank1, b_rank1, 0.25, 4.0)
    np.testing.assert_allclose(
        np.linalg.norm(a_rank1 @ actual - b_rank1), np.linalg.norm(a_rank1 @ expected - b_rank1), rtol=1e-12
    )


def test_muscles_no_trial_involves_keep_the_prior() -> None:
    model = myo_sim.load_model("myoarm_r")
    qpos = model.qpos0.copy()
    qpos[model.joint("elbow_flexion_r").qposadr[0]] = 1.57
    prior = strength_factor_from_body_mass(60.0, 75.0)
    cal = calibrate_muscle_strength(model, [MaxTorqueMeasurement("elbow_flexion_r", qpos, 45.0)], prior=prior)
    assert prior == pytest.approx(0.8)
    involved = {model.actuator(i).name for i in np.flatnonzero(cal.involved)}
    assert {"BIClong_r", "BRA_r"} <= involved or {"BIClong", "BRA"} <= involved
    assert not ({"DELT1_r", "DELT1"} & involved)
    np.testing.assert_allclose(cal.factors[~cal.involved], prior)
    np.testing.assert_allclose(cal.predicted, [45.0], rtol=0.05)


def test_rejects_invalid_trials_and_hyperparameters(elbow: mujoco.MjModel) -> None:
    trial = MaxTorqueMeasurement("r_elbow_flex", np.array([1.0]), 10.0)
    with pytest.raises(ValueError, match="positive torques"):
        calibrate_muscle_strength(elbow, [])
    with pytest.raises(ValueError, match="positive torques"):
        calibrate_muscle_strength(elbow, [MaxTorqueMeasurement("r_elbow_flex", np.array([1.0]), 0.0)])
    with pytest.raises(ValueError, match="not a hinge or slide"):
        max_torque_terms(elbow, MaxTorqueMeasurement("no_such_joint", np.array([1.0]), 10.0))
    with pytest.raises(ValueError, match="Masses must be positive"):
        strength_factor_from_body_mass(-1.0, 75.0)
    with pytest.raises(ValueError, match="bounds"):
        calibrate_muscle_strength(elbow, [trial], bounds=(2.0, 1.0))
    with pytest.raises(ValueError, match="bounds"):
        calibrate_muscle_strength(elbow, [trial], bounds=(1.0, 1.0))
    with pytest.raises(ValueError, match="prior"):
        calibrate_muscle_strength(elbow, [trial], prior=0.0)
    with pytest.raises(ValueError, match="regularization"):
        calibrate_muscle_strength(elbow, [trial], regularization=-1e-3)


def test_early_activation_terms_are_steady_without_modifying_the_model() -> None:
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><compiler angle="radian"/><worldbody><body>'
        '<joint name="elbow" range="0 3" stiffness="2"/>'
        '<geom size=".1" mass="1"/></body></worldbody><actuator>'
        '<muscle name="flexor" joint="elbow" gear="-1" lengthrange="-3 0" force="100"/></actuator></mujoco>'
    )
    trial = MaxTorqueMeasurement("elbow", np.array([1.0]), torque=40.0)
    expected = max_torque_terms(model, trial)
    assert expected[0][0] > 1.0
    model.actuator_actearly[0] = 1
    active, passive, joints = max_torque_terms(model, trial)
    for actual, nominal in zip((active, passive, joints), expected, strict=True):
        np.testing.assert_allclose(actual, nominal)
    assert model.actuator_actearly[0] == 1
    for activation in (0.5, 1.0):
        data = mujoco.MjData(model)
        data.qpos[:] = trial.qpos
        data.act[:] = activation
        data.ctrl[0] = activation
        mujoco.mj_forward(model, data)
        physical = data.actuator_force[0] * data.actuator_moment[0] + data.qfrc_passive[0]
        assert activation * active[0] + passive[0] + joints == pytest.approx(physical)


def test_early_activation_calibration_matches_steady_fit(elbow: mujoco.MjModel) -> None:
    model = copy.deepcopy(elbow)
    trials = [MaxTorqueMeasurement("r_elbow_flex", np.array([1.0]), 50.0)]
    expected = calibrate_muscle_strength(model, trials)
    model.actuator_actearly[:] = 1
    actual = calibrate_muscle_strength(model, trials)
    np.testing.assert_allclose(actual.factors, expected.factors)
    np.testing.assert_allclose(actual.predicted, expected.predicted)
    assert np.all(model.actuator_actearly == 1)


@pytest.mark.parametrize("unsupported", ["actuator_clamp", "joint_clamp", "multi_state"])
def test_mvc_rejects_unsupported_force_paths(unsupported: str) -> None:
    model = mujoco.MjModel.from_xml_string(_MVC_XML)
    if unsupported == "actuator_clamp":
        model.actuator_forcelimited[0] = True
        model.actuator_forcerange[0] = [-50.0, 0.0]
    elif unsupported == "joint_clamp":
        model.jnt_actfrclimited[0] = True
        model.jnt_actfrcrange[0] = [0.0, 20.0]
    else:
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><body><joint name="hinge"/><geom size=".1" mass="1"/></body></worldbody>'
            '<actuator><general joint="hinge" dyntype="user" actdim="2"/></actuator></mujoco>'
        )
    trial = MaxTorqueMeasurement("hinge", model.qpos0.copy(), 5.0)
    with pytest.raises(ValueError, match="Force clamps and multi-state"):
        max_torque_terms(model, trial)
    with pytest.raises(ValueError, match="Force clamps and multi-state"):
        calibrate_muscle_strength(model, [trial])


@pytest.mark.parametrize("route_to_actuation", [False, True])
def test_mvc_rejects_gravity_compensation(route_to_actuation: bool) -> None:
    model = mujoco.MjModel.from_xml_string(_MVC_XML)
    model.body_gravcomp[1] = 1
    model.jnt_actgravcomp[:] = route_to_actuation
    trial = MaxTorqueMeasurement("hinge", np.array([0.4]), 5.0)
    with pytest.raises(ValueError, match="gravity compensation"):
        max_torque_terms(model, trial)
    with pytest.raises(ValueError, match="gravity compensation"):
        calibrate_muscle_strength(model, [trial])


def _stateless_muscle() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(
        '<mujoco><compiler angle="radian"/><worldbody><body>'
        '<joint name="elbow" range="0 3"/><geom size=".1" mass="1"/>'
        '</body></worldbody><actuator><general name="flexor" joint="elbow" '
        'gear="-1" dyntype="none" gaintype="muscle" biastype="muscle" '
        'gainprm=".75 1.05 100 200 .5 1.6 1.5 1.3 1.2" '
        'biasprm=".75 1.05 100 200 .5 1.6 1.5 1.3 1.2" '
        'lengthrange="-3 0" ctrllimited="true" ctrlrange="0 1"/></actuator></mujoco>'
    )


@pytest.mark.parametrize("lower,upper", [(0.0, 0.5), (0.2, 1.0)])
def test_mvc_rejects_restricted_stateless_controls(lower: float, upper: float) -> None:
    model = _stateless_muscle()
    model.actuator_ctrlrange[0] = [lower, upper]
    trial = MaxTorqueMeasurement("elbow", np.array([1.0]), 40.0)
    data = mujoco.MjData(model)
    data.qpos[:] = trial.qpos
    data.ctrl[:] = 0.5
    mujoco.mj_forward(model, data)
    assert data.qfrc_actuator[0] == pytest.approx(49.0)
    with pytest.raises(ValueError, match="stateless muscle control clamps"):
        max_torque_terms(model, trial)
    with pytest.raises(ValueError, match="stateless muscle control clamps"):
        calibrate_muscle_strength(model, [trial])


@pytest.mark.parametrize("lower,upper,disable_clamp", [(0.0, 1.0, False), (-1.0, 2.0, False), (0.0, 0.5, True)])
def test_supported_stateless_terms_match_direct_actuation(lower: float, upper: float, disable_clamp: bool) -> None:
    model = _stateless_muscle()
    model.actuator_ctrlrange[0] = [lower, upper]
    if disable_clamp:
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL)
    trial = MaxTorqueMeasurement("elbow", np.array([1.0]), 40.0)
    active, passive, joints = max_torque_terms(model, trial)
    for activation in (0.5, 1.0):
        data = mujoco.MjData(model)
        data.qpos[:] = trial.qpos
        data.ctrl[:] = activation
        mujoco.mj_forward(model, data)
        assert activation * active[0] + passive[0] + joints == pytest.approx(data.qfrc_actuator[0])


def _arm(*, auto: bool = False, coupled: bool = False) -> mujoco.MjModel:
    child = '<body><joint name="dependent" axis="0 1 0" range="0 6"/><geom size=".05" mass="1"/></body>' if coupled else ""
    equality = '<equality><joint joint1="dependent" joint2="elbow" polycoef="0 2 0 0 0"/></equality>' if coupled else ""
    return mujoco.MjModel.from_xml_string(
        f"""<mujoco><compiler angle="radian"/><worldbody>
        <body name="torso"><geom size=".1" mass="20"/>
        <body name="arm"><joint name="elbow" axis="0 1 0" range="0 3" stiffness="2" springref="0"/>
        <geom type="capsule" fromto="0 0 0 1 0 0" size=".04" mass="3"/>{child}</body></body></worldbody>
        {equality}<actuator>
        <muscle name="flexor" joint="elbow" gear="-1" lengthrange="-3 0" force="{-1 if auto else 100}"/>
        <muscle name="extensor" joint="elbow" lengthrange="0 3" force="80"/>
        </actuator></mujoco>"""
    )


def _qpos(model: mujoco.MjModel, elbow: float) -> np.ndarray:
    q = model.qpos0.copy()
    q[0] = elbow
    return q


def _direct_torque(model: mujoco.MjModel, q: np.ndarray, activation: np.ndarray) -> float:
    data = mujoco.MjData(model)
    data.qpos[:] = q
    data.act[model.actuator_actadr] = activation
    data.ctrl[:] = activation
    mujoco.mj_forward(model, data)
    return float((data.qfrc_actuator + data.qfrc_passive)[0])


def test_selected_scaling_preserves_other_parameters_and_automatic_force() -> None:
    model = _arm(auto=True)
    gain, bias = model.actuator_gainprm.copy(), model.actuator_biasprm.copy()
    scale_muscle_strength(model, 2.0, muscles=["extensor"])
    np.testing.assert_array_equal(model.actuator_gainprm[0], gain[0])
    np.testing.assert_array_equal(model.actuator_biasprm[0], bias[0])
    assert model.actuator_gainprm[0, 2] < 0
    assert model.actuator_gainprm[1, 2] == 160
    assert model.actuator_biasprm[1, 2] == 160
    model.actuator_biastype[1] = mujoco.mjtBias.mjBIAS_NONE
    bias = model.actuator_biasprm.copy()
    scale_muscle_strength(model, 2.0, muscles=["extensor"])
    np.testing.assert_array_equal(model.actuator_biasprm, bias)
    assert model.actuator_gainprm[1, 2] == 320


def test_selected_scaling_matches_between_model_and_spec_and_validates_only_selected() -> None:
    spec = _spec()
    original = spec.copy().compile()
    scaled = copy.deepcopy(original)
    factors = np.array([np.nan, 1.5, np.nan])
    scale_muscle_strength(scaled, factors, muscles=["explicit"])
    scale_muscle_strength_spec(spec, factors, muscles=["explicit"])
    from_spec = spec.compile()
    for angle in (0.1, 1.2):
        expected = _forces(original, angle, 1.0) * np.array([1.0, 1.5, 1.0])
        np.testing.assert_allclose(_forces(scaled, angle, 1.0), expected, rtol=1e-12)
        np.testing.assert_allclose(_forces(from_spec, angle, 1.0), expected, rtol=1e-12)
    np.testing.assert_array_equal(from_spec.actuator_gainprm[[0, 2]], original.actuator_gainprm[[0, 2]])
    with pytest.raises(ValueError, match="positive and finite"):
        scale_muscle_strength(scaled, factors, muscles=["automatic"])


@pytest.mark.parametrize("muscles", [["missing"], ["explicit", "explicit"], ["motor"], "explicit"])
def test_invalid_muscle_selection_is_transactional(muscles: list[str] | str) -> None:
    spec = _spec()
    model = spec.copy().compile()
    gain, bias, text = model.actuator_gainprm.copy(), model.actuator_biasprm.copy(), spec.to_xml()
    with pytest.raises(ValueError, match="muscle"):
        scale_muscle_strength(model, 2.0, muscles=muscles)
    with pytest.raises(ValueError, match="muscle"):
        scale_muscle_strength_spec(spec, 2.0, muscles=muscles)
    np.testing.assert_array_equal(model.actuator_gainprm, gain)
    np.testing.assert_array_equal(model.actuator_biasprm, bias)
    assert spec.to_xml() == text


def test_max_joint_torque_caps_only_active_force() -> None:
    model = _arm()
    q = _qpos(model, 2.7)
    fresh, (name,) = max_joint_torque(model, q, "elbow")
    passive = max_joint_torque(model, q, "elbow", max_activation=np.zeros(2))[0]
    cap = np.ones(2)
    cap[model.actuator(name).id] = 0.5
    half, _ = max_joint_torque(model, q, "elbow", max_activation=cap)
    assert half == pytest.approx(passive + 0.5 * (fresh - passive))
    assert half == pytest.approx(_direct_torque(model, q, cap * (np.arange(2) == model.actuator(name).id)))
    assert fresh == pytest.approx(_direct_torque(model, q, np.arange(2) == model.actuator(name).id))
    assert passive == pytest.approx(_direct_torque(model, q, np.zeros(2)))
    (other,) = max_joint_torque(model, q, "elbow", -1.0)[1]
    assert other != name


def test_max_joint_torque_coupled_joint_uses_virtual_work_and_rejects_dependents() -> None:
    model = _arm(coupled=True)
    q = model.qpos0.copy()
    q[0] = 1.0
    model.jnt_stiffness[model.joint("dependent").id] = 0
    baseline = max_joint_torque(model, q, "elbow")[0]
    model.jnt_stiffness[model.joint("dependent").id] = 3
    # The dependent joint's spring torque -3*2 acts with dq_dependent/dq_elbow = 2.
    assert max_joint_torque(model, q, "elbow")[0] - baseline == pytest.approx(-12)
    with pytest.raises(ValueError, match="independent"):
        max_joint_torque(model, q, "dependent")


def test_max_joint_torque_rejects_invalid_input_and_is_steady_for_early_activation() -> None:
    model = _arm()
    q = _qpos(model, 1.0)
    nominal = max_joint_torque(model, q, "elbow")[0]
    for kwargs, match in [
        ({"direction": 0.5}, "direction"),
        ({"max_activation": np.array([1.1, 0.0])}, "max_activation"),
        ({"max_activation": np.array([np.nan, 0.0])}, "max_activation"),
        ({"max_activation": np.ones(3)}, "max_activation"),
    ]:
        with pytest.raises(ValueError, match=match):
            max_joint_torque(model, q, "elbow", **kwargs)
    for bad in (q[:0], np.array([np.nan])):
        with pytest.raises(ValueError, match="qpos"):
            max_joint_torque(model, bad, "elbow")
    with pytest.raises(ValueError, match="hinge or slide"):
        max_joint_torque(model, q, "missing")
    model.actuator_actearly[:] = 1
    assert max_joint_torque(model, q, "elbow")[0] == pytest.approx(nominal)
    assert np.all(model.actuator_actearly == 1)
    model.actuator_forcelimited[0] = True
    model.actuator_forcerange[0] = [-50.0, 0.0]
    with pytest.raises(ValueError, match="Force clamps"):
        max_joint_torque(model, q, "elbow")
