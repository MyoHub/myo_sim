import mujoco

from myo_sim.build.compose import build_model


def id_for(model, object_type, name: str) -> int:
    return mujoco.mj_name2id(model, object_type, name)


def test_myolegs_abdomen_builds_from_registered_spec():
    model = build_model("myolegs_abdomen")

    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "sacrum") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_GEOM, "pelvis_wrap") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "lumbar") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "iliacus_r") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "iliacus_l") >= 0
