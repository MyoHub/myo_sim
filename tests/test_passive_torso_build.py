import mujoco

from myo_sim import load_model


def id_for(model, object_type, name: str) -> int:
    return mujoco.mj_name2id(model, object_type, name)


def test_myoarms_uses_passive_anatomical_torso_with_arm_controls_only():
    model = load_model("myoarms")

    assert id_for(model, mujoco.mjtObj.mjOBJ_GEOM, "torso_geom_13") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_GEOM, "Chest_ellipsoid_r") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "DELT1") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "DELT1_l") >= 0

    assert id_for(model, mujoco.mjtObj.mjOBJ_JOINT, "flex_extension") < 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_JOINT, "L4_L5_FE") < 0
    assert model.nu == 126


def test_myoarm_r_uses_passive_anatomical_torso_with_right_arm_only():
    model = load_model("myoarm_r")

    assert id_for(model, mujoco.mjtObj.mjOBJ_GEOM, "torso_geom_13") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "DELT1") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "DELT1_l") < 0

    assert id_for(model, mujoco.mjtObj.mjOBJ_JOINT, "flex_extension") < 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_JOINT, "L4_L5_FE") < 0
    assert model.nu == 63
