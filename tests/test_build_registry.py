from dataclasses import fields
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from myo_sim.build import compose
from myo_sim.build.compose import ALIASES, MODEL_REGISTRY, BuildStrategy, ModelRegistration, build_model


def test_aliases_resolve_to_registered_models():
    unknown = {target for target in ALIASES.values() if target not in MODEL_REGISTRY}

    assert not unknown, f"ALIASES point at names missing from MODEL_REGISTRY: {sorted(unknown)}"


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
        "myolegs26": BuildStrategy.LEGS26_BODY,
        "myotorso_abdomen": BuildStrategy.TORSO_ABDOMEN,
        "myolegs_abdomen": BuildStrategy.LEGS_ABDOMEN,
    }

    assert {name: registration.build_strategy for name, registration in MODEL_REGISTRY.items()} == expected


def test_myoarms_uses_default_root_position():
    assert MODEL_REGISTRY["myoarms"].root_pos == (0, 0, 1)


def test_every_registered_model_includes_scene_floor():
    for name in MODEL_REGISTRY:
        model = build_model(name)

        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor") >= 0, name


def test_compose_does_not_depend_on_static_myotorso_xml():
    source = (Path(__file__).resolve().parents[1] / "myo_sim" / "build" / "compose.py").read_text()

    assert 'Path("torso/myotorso.xml")' in source
    assert "myotorso.xml" not in source.replace('Path("torso/myotorso.xml")', "")


def test_generate_xml_files_writes_base_model_outputs(tmp_path, monkeypatch):
    expected_targets = {
        "myoarms": Path("arm/myoarms.xml"),
        "myotorso": Path("torso/myotorso.xml"),
        "myolegs": Path("leg/myolegs.xml"),
        "myolegs26": Path("leg/myolegs26.xml"),
        "myofullbody": Path("myofullbody.xml"),
    }
    spec_calls = []
    compile_calls = []

    class FakeSpec:
        def __init__(self, model_name):
            self.model_name = model_name
            self.compiler = type("Compiler", (), {"meshdir": "", "texturedir": ""})()
            self.ngeom = 0
            self.geom_contype = []
            self.geom_conaffinity = []

        def compile(self):
            compile_calls.append(self.model_name)
            return self

        def to_xml(self):
            assert self.compiler.meshdir == ""
            assert self.compiler.texturedir == ""
            return f'<mujoco model="{self.model_name}"/>\n'

    def fake_build_generated_model_spec(model_name):
        spec_calls.append(model_name)
        return FakeSpec(model_name)

    def fail_build_model(model_name):
        raise AssertionError(f"generate_xml_files should build specs directly, not compiled models: {model_name}")

    def fail_save_last_xml(path, model):
        raise AssertionError("generate_xml_files should use MjSpec.to_xml(), not mj_saveLastXML()")

    monkeypatch.setattr(compose, "build_generated_model_spec", fake_build_generated_model_spec, raising=False)
    monkeypatch.setattr(compose, "build_model", fail_build_model)
    monkeypatch.setattr(compose.mujoco, "mj_saveLastXML", fail_save_last_xml)

    output_paths = compose.generate_xml_files(tmp_path)

    assert compose.GENERATE_XML_TARGETS == expected_targets
    assert spec_calls == list(expected_targets)
    assert compile_calls == list(expected_targets)
    assert output_paths == [tmp_path / rel_path for rel_path in expected_targets.values()]
    for path, model_name in zip(output_paths, expected_targets):
        source = path.read_text()
        assert source.startswith(f'<mujoco model="{model_name}">')
        assert '<compiler meshdir="' in source
        assert 'texturedir="' in source


def test_sanitize_spec_xml_unwraps_nested_classless_defaults():
    xml = """<mujoco>
  <default>
    <geom contype="0"/>
    <default>
      <geom contype="0"/>
      <tendon width="0.001"/>
      <default class="arm">
        <joint limited="true"/>
      </default>
      <default class="arm">
        <joint limited="true"/>
      </default>
    </default>
  </default>
</mujoco>
"""

    sanitized = compose.sanitize_spec_xml(xml)

    assert "<default>\n    <geom" in sanitized
    assert sanitized.count("<geom") == 1
    assert sanitized.count("<tendon") == 1
    assert '<default class="arm">' in sanitized
    assert sanitized.count('class="arm"') == 1
    assert sanitized.count("<default>") == 1


def test_generated_xml_keeps_floor_collision_enabled(tmp_path):
    output_paths = compose.generate_xml_files(tmp_path)

    for output_path in output_paths:
        model = mujoco.MjModel.from_xml_path(str(output_path))
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")

        assert floor_id >= 0, output_path
        assert model.geom_contype[floor_id] == 1, output_path
        assert model.geom_conaffinity[floor_id] == 1, output_path


def test_myolegs26_knee_reset_uses_baked_tibia_offsets():
    model = build_model("myolegs26")
    knee_translation_joints = (
        "knee_r_translation1",
        "knee_r_translation2",
        "knee_l_translation1",
        "knee_l_translation2",
    )

    for joint_name in knee_translation_joints:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_adr = model.jnt_qposadr[joint_id]

        assert abs(model.qpos0[qpos_adr]) < 1e-12, joint_name
        assert model.jnt_range[joint_id][0] <= 0 <= model.jnt_range[joint_id][1], joint_name

    equality_root = ET.parse(compose.LEGS26_ASSETS_XML).getroot().find("equality")
    knee_polycoef_offsets = {
        joint.get("joint1"): float(joint.get("polycoef", "").split()[0])
        for joint in equality_root.iter("joint")
        if joint.get("joint1") in knee_translation_joints
    }

    assert knee_polycoef_offsets == {joint_name: 0.0 for joint_name in knee_translation_joints}


def test_myolegs26_loads_assembled_at_qpos0():
    model = build_model("myolegs26")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    eq_residual = max(
        (abs(data.efc_pos[i]) for i in range(data.nefc) if data.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY),
        default=0.0,
    )

    assert eq_residual < 1e-6


def test_generated_myofullbody_uses_musclemimic_scene(tmp_path):
    output_paths = compose.generate_xml_files(tmp_path)
    myofullbody_xml = output_paths[list(compose.GENERATE_XML_TARGETS).index("myofullbody")]
    source = myofullbody_xml.read_text()

    assert 'material name="MatPlane"' in source
    assert 'mesh name="meshscene"' not in source


def test_generated_xml_preserves_geom_collision_flags(tmp_path):
    output_paths = compose.generate_xml_files(tmp_path)

    for model_name, output_path in zip(compose.GENERATE_XML_TARGETS, output_paths):
        built_model = build_model(model_name)
        generated_model = mujoco.MjModel.from_xml_path(str(output_path))

        assert list(generated_model.geom_contype) == list(built_model.geom_contype), output_path
        assert list(generated_model.geom_conaffinity) == list(built_model.geom_conaffinity), output_path
