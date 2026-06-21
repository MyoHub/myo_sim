import xml.etree.ElementTree as ET

import mujoco
import pytest

import myo_sim
from myo_sim.build.compose import build_model


def body_id(model, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def xml_body_names(path):
    return {body.get("name") for body in ET.parse(path).getroot().iter("body") if body.get("name")}


def test_chest_scaffold_is_torso_owned_not_arm_owned():
    arm_bodies = xml_body_names(myo_sim.MODELS_DIR / "arm" / "assets" / "myoarm_r_chain.xml")
    torso_bodies = xml_body_names(myo_sim.MODELS_DIR / "torso" / "assets" / "myotorso_chain.xml")

    assert "chest_r" not in arm_bodies
    assert "cervical_spine" not in arm_bodies
    assert "chest_r" in torso_bodies
    assert "cervical_spine" in torso_bodies


@pytest.mark.parametrize("model_name", ("myoarms", "myotorso_arms", "myofullbody"))
def test_composed_arm_models_do_not_mirror_torso_chest(model_name):
    model = build_model(model_name)

    assert body_id(model, "chest_r") >= 0
    assert body_id(model, "cervical_spine") >= 0
    assert body_id(model, "chest_l") < 0
