"""Runtime neck composition and passive mechanics, without literature fixtures."""

import mujoco
import numpy as np
import pytest

from myo_sim import load_model
from myo_sim.build.compose import MODEL_REGISTRY

# Only the full body carries the muscular neck; every part model keeps the rigid head.
NECK_MODELS = ("myofullbody",)


def _neck_joint_count(model: mujoco.MjModel) -> int:
    return sum(model.joint(i).name.startswith("myohead_") for i in range(model.njnt))


def _right_neck_actuator_ids(model: mujoco.MjModel) -> list[int]:
    # The neck is attached last, so its 72 actuators are the trailing block.
    return [i for i in range(model.nu - 72, model.nu) if model.actuator(i).name.endswith("_r")]


@pytest.mark.parametrize("name", ["myohead", "myofullbody"])
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
    right = _right_neck_actuator_ids(model)
    assert len(right) == 36
    left = [model.actuator(model.actuator(i).name[:-2] + "_l").id for i in right]
    np.testing.assert_array_equal(model.actuator_gainprm[right], model.actuator_gainprm[left])
    parameters = model.actuator_gainprm[right]
    lengthrange = model.actuator_lengthrange[right]
    l0 = np.diff(lengthrange, axis=1)[:, 0] / (parameters[:, 1] - parameters[:, 0])
    assert np.isfinite(l0).all() and np.all(l0 > 0)
    assert np.all(lengthrange[:, 0] - parameters[:, 0] * l0 >= 0)


def test_neck_is_part_of_myofullbody_only() -> None:
    for name in MODEL_REGISTRY:
        expected = 24 if name in NECK_MODELS + ("myohead",) else 0
        assert _neck_joint_count(load_model(name)) == expected, name
    full = load_model("myofullbody")
    assert (full.njnt, full.nu, full.neq) == (147, 488, 69)
    # Neck actuators are appended after the existing torso, arm and leg actuators.
    head = load_model("myohead")
    neck_names = {head.actuator(i).name for i in range(head.nu)}
    assert {full.actuator(i).name for i in range(full.nu - 72, full.nu)} == neck_names


def test_neck_preserves_head_landmarks_of_the_rigid_scaffold() -> None:
    """At qpos0 the articulated head sits exactly where the rigid scaffold's head does."""
    rigid, neck = load_model("myoarms"), load_model("myofullbody")
    rigid_data, neck_data = mujoco.MjData(rigid), mujoco.MjData(neck)
    mujoco.mj_forward(rigid, rigid_data)
    mujoco.mj_forward(neck, neck_data)
    rigid_root = rigid_data.xpos[rigid.body("Full Body").id]
    neck_root = neck_data.xpos[neck.body("Full Body").id]
    np.testing.assert_allclose(
        rigid_data.xpos[rigid.body("head").id] - rigid_root, neck_data.xpos[neck.body("head").id] - neck_root, atol=1e-12
    )
    for name in ("hat_jaw", "hat_skull", "hat_jaw_coll2", "hat_skull_coll", "hat_cervical_coll"):
        np.testing.assert_allclose(
            rigid_data.geom_xpos[rigid.geom(name).id] - rigid_root,
            neck_data.geom_xpos[neck.geom(name).id] - neck_root,
            atol=1e-12,
        )
