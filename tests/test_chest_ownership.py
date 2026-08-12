import xml.etree.ElementTree as ET

import mujoco
import pytest

import myo_sim
from myo_sim import load_spec


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
    model = load_spec(model_name).compile()

    assert body_id(model, "chest_r") >= 0
    assert body_id(model, "cervical_spine") >= 0
    assert body_id(model, "chest_l") < 0


@pytest.mark.parametrize("model_name", ("myoarms", "myotorso_arms", "myofullbody"))
def test_chest_scaffold_carries_no_phantom_mass(model_name):
    """chest_r holds only muscle-wrap ellipsoids, so it must stay effectively massless.

    Without an explicit inertial the wrap geoms were meshed at the default 1000 kg/m^3,
    giving chest_r ~12.87 kg of phantom mass.
    """
    model = load_spec(model_name).compile()

    assert model.body_mass[body_id(model, "chest_r")] < 0.01


def test_fullbody_total_mass_is_anthropometrically_consistent():
    """myofullbody should weigh ~84.3 kg, matching the MuscleMimic reference model."""
    model = load_spec("myofullbody").compile()

    assert model.body_mass.sum() == pytest.approx(84.3, abs=0.3)
