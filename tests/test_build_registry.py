from dataclasses import fields
from pathlib import Path

import mujoco

from myo_sim.build import compose
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


def test_generate_mjb_files_writes_base_model_outputs(tmp_path, monkeypatch):
    expected_targets = {
        "myoarms": Path("arm/myoarms.mjb"),
        "myotorso": Path("torso/myotorso.mjb"),
        "myolegs": Path("leg/myolegs.mjb"),
        "myofullbody": Path("myofullbody.mjb"),
    }
    build_calls = []
    save_calls = []

    class FakeModel:
        def __init__(self, model_name):
            self.model_name = model_name

    def fake_build_model(model_name):
        build_calls.append(model_name)
        return FakeModel(model_name)

    def fake_save_model(model, path):
        output_path = Path(path)
        save_calls.append((model.model_name, output_path))
        output_path.write_bytes(b"MJB")

    def fail_save_last_xml(path, model):
        raise AssertionError("generate_mjb_files should use mj_saveModel(), not mj_saveLastXML()")

    monkeypatch.setattr(compose, "build_model", fake_build_model)
    monkeypatch.setattr(compose.mujoco, "mj_saveModel", fake_save_model)
    monkeypatch.setattr(compose.mujoco, "mj_saveLastXML", fail_save_last_xml)

    output_paths = compose.generate_mjb_files(tmp_path)

    assert compose.GENERATE_MJB_TARGETS == expected_targets
    assert build_calls == list(expected_targets)
    assert output_paths == [tmp_path / rel_path for rel_path in expected_targets.values()]
    assert save_calls == list(zip(expected_targets, output_paths))
    assert all(path.read_bytes() == b"MJB" for path in output_paths)
