"""Runtime neck composition and passive mechanics, without literature fixtures."""

import mujoco
import numpy as np
import pytest

from myo_sim import load_model


def _right_neck_actuator_ids(model: mujoco.MjModel, name: str) -> list[int]:
    if name == "myohead":
        return [i for i in range(model.nu) if model.actuator(i).name.endswith("_r")]
    base = load_model("myofullbody")
    base_names = {base.actuator(i).name for i in range(base.nu)}
    return [i for i in range(model.nu) if model.actuator(i).name.endswith("_r") and model.actuator(i).name not in base_names]


@pytest.mark.parametrize("name", ["myohead", "myofullbody_neck"])
def test_neck_passive_mechanics_and_muscle_parameters(name: str) -> None:
    model = load_model(name)
    data = mujoco.MjData(model)
    joints = [model.joint(i) for i in range(model.njnt) if model.joint(i).name.startswith("myohead_")]
    assert len(joints) == 24
    qpos = [joint.qposadr[0] for joint in joints]
    dofs = [joint.dofadr[0] for joint in joints]
    assert np.all(model.jnt_stiffness[[joint.id for joint in joints]] > 0)
    data.qpos[qpos] = 0.01
    mujoco.mj_forward(model, data)
    assert data.qpos[qpos] @ data.qfrc_passive[dofs] < 0
    restoring = data.qfrc_passive.copy()
    data.qvel[dofs] = 0.1
    mujoco.mj_forward(model, data)
    assert data.qvel[dofs] @ (data.qfrc_passive - restoring)[dofs] < 0
    right = _right_neck_actuator_ids(model, name)
    assert len(right) == 36
    left = [model.actuator(model.actuator(i).name[:-2] + "_l").id for i in right]
    np.testing.assert_array_equal(model.actuator_gainprm[right], model.actuator_gainprm[left])
    parameters = model.actuator_gainprm[right]
    lengthrange = model.actuator_lengthrange[right]
    l0 = np.diff(lengthrange, axis=1)[:, 0] / (parameters[:, 1] - parameters[:, 0])
    assert np.isfinite(l0).all() and np.all(l0 > 0)
    assert np.all(lengthrange[:, 0] - parameters[:, 0] * l0 >= 0)


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
