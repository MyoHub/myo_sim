"""Source, experimental-strength, symmetry and full-body neck integration gates."""

import mujoco
import numpy as np
import pytest

from myo_sim import load_model
from scripts.validate_neck import coupling_matrix, load_reference, neutral_strength, set_pose, source_agreement, tendon_jacobian


@pytest.mark.parametrize("name", ["myohead", "myofullbody_neck"])
def test_neck_matches_canonical_opensim_source(name: str) -> None:
    result = source_agreement(load_model(name))
    assert result["pass"], result


@pytest.mark.parametrize("name", ["myohead", "myofullbody_neck"])
def test_neutral_experimental_strength(name: str) -> None:
    for result in neutral_strength(load_model(name)):
        assert result["within_one_sd"], result
        assert result["equilibrium_residual_Nm"] < 1e-8, result


def test_neck_derivatives_and_bilateral_mirroring() -> None:
    model = load_model("myohead")
    data = mujoco.MjData(model)
    matrix = coupling_matrix(model)
    fixture = load_reference()
    # Interior multi-coordinate poses avoid testing only the neutral / limits.
    poses = fixture["poses"][::61] * 0.7
    reflection = np.array([1, -1, -1, 1, -1, -1])
    right = [a.id for a in (model.actuator(i) for i in range(model.nu)) if a.name.endswith("_r")]
    left = [model.actuator(model.actuator(i).name[:-2] + "_l").id for i in right]
    for pose in poses:
        set_pose(model, data, pose, matrix)
        lengths = data.actuator_length.copy()
        arms = -(tendon_jacobian(model, data) @ matrix)
        for k in range(6):
            delta = np.eye(6)[k] * 1e-6
            set_pose(model, data, pose + delta, matrix)
            high = data.ten_length.copy()
            set_pose(model, data, pose - delta, matrix)
            low = data.ten_length.copy()
            np.testing.assert_allclose(arms[:, k], -(high - low) / 2e-6, atol=1e-8, rtol=0)
        set_pose(model, data, pose * reflection, matrix)
        np.testing.assert_allclose(lengths[right], data.actuator_length[left], atol=1e-12, rtol=0)
    parameters = model.actuator_gainprm
    l0 = np.diff(model.actuator_lengthrange, axis=1)[:, 0] / (parameters[:, 1] - parameters[:, 0])
    slack = model.actuator_lengthrange[:, 0] - parameters[:, 0] * l0
    assert np.all(l0 > 0)
    assert np.all(slack >= 0), slack
    np.testing.assert_array_equal(parameters[right], parameters[left])


def test_neck_preserves_fullbody_landmarks_and_existing_actuators() -> None:
    base, extended = load_model("myofullbody"), load_model("myofullbody_neck")
    assert (base.njnt, base.nu) == (123, 416)
    assert (extended.njnt, extended.nu) == (147, 488)
    assert extended.neq - base.neq == 18
    base_data, extended_data = mujoco.MjData(base), mujoco.MjData(extended)
    mujoco.mj_forward(base, base_data)
    mujoco.mj_forward(extended, extended_data)
    np.testing.assert_allclose(base_data.xpos[base.body("head").id], extended_data.xpos[extended.body("head").id], atol=1e-12)
    for name in ("hat_jaw", "hat_skull", "hat_jaw_coll2", "hat_skull_coll", "hat_cervical_coll"):
        np.testing.assert_allclose(
            base_data.geom_xpos[base.geom(name).id], extended_data.geom_xpos[extended.geom(name).id], atol=1e-12
        )
    for i in range(base.nu):
        j = extended.actuator(base.actuator(i).name).id
        for attribute in ("actuator_gainprm", "actuator_biasprm", "actuator_dynprm", "actuator_lengthrange"):
            np.testing.assert_array_equal(getattr(base, attribute)[i], getattr(extended, attribute)[j])
    assert [base.sensor(i).name for i in range(base.nsensor)] == [extended.sensor(i).name for i in range(extended.nsensor)]
