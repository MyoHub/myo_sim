from dataclasses import fields
from pathlib import Path

import mujoco

from myo_sim.build.compose import BuildStrategy, MODEL_REGISTRY, ModelRegistration, build_model


def test_model_registration_uses_explicit_build_strategy():
    field_names = {field.name for field in fields(ModelRegistration)}

    assert "build_strategy" in field_names
    assert "arms_body_only" not in field_names
    assert "right_hand_from_arm" not in field_names
    assert "both_hands_from_arm" not in field_names


def test_every_registered_model_has_build_strategy():
    expected = {
        "myotorso_arms": BuildStrategy.TORSO_ARMS,
        "myotorso": BuildStrategy.TORSO_BODY,
        "myoarms": BuildStrategy.ARMS_BODY,
        "myoarm_r": BuildStrategy.RIGHT_ARM_BODY,
        "myotorso_arm_r": BuildStrategy.TORSO_ARMS,
        "myohand_r": BuildStrategy.RIGHT_HAND,
        "myohands": BuildStrategy.BOTH_HANDS,
        "myofullbody": BuildStrategy.FULLBODY,
        "myolegs": BuildStrategy.LEGS_BODY,
        "myotorso_abdomen": BuildStrategy.TORSO_ABDOMEN,
        "myolegs_abdomen": BuildStrategy.LEGS_ABDOMEN,
    }

    assert {name: registration.build_strategy for name, registration in MODEL_REGISTRY.items()} == expected


def test_every_registered_model_includes_scene_floor():
    for name in MODEL_REGISTRY:
        model = build_model(name)

        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor") >= 0, name


def test_compose_does_not_depend_on_static_myotorso_xml():
    source = (Path(__file__).resolve().parents[1] / "myo_sim" / "build" / "compose.py").read_text()

    assert "myotorso.xml" not in source
    assert not (Path(__file__).resolve().parents[1] / "myo_sim" / "models" / "torso" / "myotorso.xml").exists()
