import mujoco
from pathlib import Path

from myo_sim.build.compose import build_model


ROOT = Path(__file__).resolve().parents[1]
TORSO_ASSETS = ROOT / "myo_sim" / "models" / "torso" / "assets"


def id_for(model, object_type, name: str) -> int:
    return mujoco.mj_name2id(model, object_type, name)


def joint_names(model) -> set[str]:
    return {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) for joint_id in range(model.njnt)}


def test_myotorso_abdomen_builds_from_registered_spec():
    model = build_model("myotorso_abdomen")

    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "sacrum") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "lumbar5") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_GEOM, "pelvis_wrap") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ercspn_r") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "intobl_l") >= 0


def test_myotorso_abdomen_locks_all_but_base_lumbar5_joints():
    model = build_model("myotorso_abdomen")

    assert joint_names(model) == {"flex_extension", "lat_bending", "axial_rotation"}


def test_myotorso_chain_contains_simple_abdomen_compatibility_points():
    model = build_model("myotorso_abdomen")
    required_sites = (
        "ercspn_r_ercspn_r-P1",
        "ercspn_l_ercspn_l-P1",
        "ercspn_r_ercspn_r-P2",
        "ercspn_l_ercspn_l-P2",
        "intobl_r_intobl_r-P1",
        "intobl_l_intobl_l-P1",
        "intobl_r_intobl_r-P2",
        "intobl_l_intobl_l-P2",
        "extobl_r_extobl_r-P1",
        "extobl_l_extobl_l-P1",
        "extobl_r_extobl_r-P2",
        "extobl_l_extobl_l-P2",
        "clavicle_l",
        "clavicle_r",
        "clavicle_spine",
        "thoracis_spine",
        "chest",
        "lumbar_spine",
    )

    for site_name in required_sites:
        assert id_for(model, mujoco.mjtObj.mjOBJ_SITE, site_name) >= 0


def test_myotorso_abdomen_uses_full_torso_assets_and_chain():
    assets_xml = TORSO_ASSETS / "myotorso_abdomen_assets.xml"
    chain_xml = TORSO_ASSETS / "myotorso_abdomen_chain.xml"
    tendon_xml = TORSO_ASSETS / "myotorso_abdomen_tendon.xml"
    muscle_xml = TORSO_ASSETS / "myotorso_abdomen_muscle.xml"

    assert not assets_xml.exists()
    assert not chain_xml.exists()
    assert tendon_xml.exists()
    assert muscle_xml.exists()


def test_myolegs_builds_with_passive_torso_scaffold():
    model = build_model("myolegs")

    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "Full Body") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "sacrum") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "iliacus_r") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "iliacus_l") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "IL_L1_r") == -1


def test_myolegs_abdomen_builds_from_registered_spec():
    model = build_model("myolegs_abdomen")

    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "sacrum") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_GEOM, "pelvis_wrap") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_BODY, "lumbar5") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "iliacus_r") >= 0
    assert id_for(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "iliacus_l") >= 0
