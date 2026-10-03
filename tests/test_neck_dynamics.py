"""Experimental fixture integrity, published reflex logic and numerical dynamics."""

import numpy as np
import pytest

from scripts.validate_neck_dynamics import DT, load_calibration, load_experiment, reflex_control, simulate


def test_experimental_marker_reference() -> None:
    reference = load_experiment()
    time = reference["time"]
    assert time.shape == (134,)
    assert time[0] == 0
    assert np.all(np.diff(time) > 0)
    for condition in ("supine", "prone"):
        for quantity in ("drop_m", "rotation_rad"):
            participants = reference[f"{condition}_{quantity}"]
            assert participants.shape == (134, 17)
            assert np.isfinite(participants).all()
            np.testing.assert_allclose(participants[0], 0, atol=3e-8)
            assert reference[f"{condition}_{quantity}_sd"][30:].max() > 0


def test_published_reflex_hysteresis_and_exclusions() -> None:
    # Table 5 distinguishes triggering, sustaining, relaxation and excluded muscles.
    strain = np.array([0.06, 0.02, 0.02, 0, -0.01, 0.06])
    previous = np.array([0, 1, 0, 1, 1, 0])
    eligible = np.array([True, True, True, True, True, False])
    np.testing.assert_array_equal(reflex_control(strain, previous, eligible), [1, 1, 0, 0, 0, 0])


@pytest.mark.parametrize("condition", ["supine", "prone"])
def test_dynamic_integration_and_fixed_torso_composition(condition: str) -> None:
    time = load_experiment()["time"]
    gain = load_calibration()["selected_gain"]
    standalone = simulate("myohead", condition, True, time, gain=gain)
    integrated = simulate("myofullbody_neck", condition, True, time, gain=gain)
    fine = simulate("myofullbody_neck", condition, True, time, DT / 2, gain=gain)
    for quantity, tolerance in (("drop_m", 0.001), ("rotation_rad", np.deg2rad(1))):
        np.testing.assert_allclose(standalone[quantity], integrated[quantity], atol=1e-8, rtol=0)
        np.testing.assert_allclose(integrated[quantity], fine[quantity], atol=tolerance, rtol=0)
    for result in (standalone, integrated, fine):
        assert result["max_coupling_residual_rad"] < 0.01
        assert result["first_stimulation_s"] >= 0.025
        assert 0 < result["peak_activation"] <= 1
        assert result["drop_m"][0] == 0


def test_source_fiber_observer_and_graded_feedback() -> None:
    import mujoco

    from scripts.validate_neck_dynamics import (
        controller_muscles,
        fiber_estimate,
        graded_control,
        load_physiology,
        release_model,
    )

    model = release_model("myohead", DT)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    properties, force_curve, strain_curve = load_physiology(model)
    relaxed = fiber_estimate(data.actuator_length, np.zeros(model.nu), properties, force_curve, strain_curve)
    loaded = fiber_estimate(data.actuator_length, properties[2] * 0.1, properties, force_curve, strain_curve)
    assert np.all(loaded < relaxed)  # Tendon elongation reduces CE stretch at fixed MT length.
    eligible = controller_muscles(model)
    assert eligible.sum() == 48
    for name in ("trap_acr", "levator_scap", "Geniohyoid", "SternoThyroid", "Omo_hyoid"):
        assert not eligible[model.actuator(f"myohead_{name}_r").id]
    assert eligible[model.actuator("myohead_Sterno_hyoid_r").id]
    np.testing.assert_allclose(
        graded_control(np.array([-0.01, 0, 0.01, 0.1]), np.ones(4) * 0.1, np.array([True, True, True, False]), 3),
        [0, 0, 0.3, 0],
    )


def test_passive_mechanics_are_restoring_and_dissipative() -> None:
    import mujoco

    from scripts.validate_neck import coupling_matrix, set_pose
    from scripts.validate_neck_dynamics import release_model

    model = release_model("myohead", DT)
    data = mujoco.MjData(model)
    matrix = coupling_matrix(model)
    # Both sagittal modes, simultaneously; source bushing mapping is exact here.
    pose = np.array([0.03, 0, 0, -0.02, 0, 0])
    set_pose(model, data, pose, matrix)
    assert np.all(model.jnt_stiffness > 0)
    assert data.qpos @ data.qfrc_passive < 0
    restoring = data.qfrc_passive.copy()
    data.qvel[:] = matrix @ np.array([0.1, 0, 0, -0.05, 0, 0])
    mujoco.mj_forward(model, data)
    assert data.qvel @ (data.qfrc_passive - restoring) < 0
