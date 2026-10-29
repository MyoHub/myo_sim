import mujoco
import pytest

from myo_sim import load_model, load_spec
from myo_sim.build.mass import audit_mass, subtree_mass

FULLBODY_MASS = 84.0
DE_LEVA_HEAD_NECK = 0.0694  # de Leva (1996) head-and-neck share of body mass
TORSO_MASS = 14.897  # OpenSim trunk minus the head it contained (#138)
# Scaffold part models keep the rigid head at the de Leva share of the 84.26 kg rigid-head body (#138).
RIGID_HEAD_MASS = 5.848
RIGID_HEAD_BODY_MASS = 84.26
# Torso models carry the same muscular neck as myofullbody.
NECK_PART_MODELS = ("myotorso", "myotorso_arms", "myotorso_arm_r")
RIGID_HEAD_MODELS = (
    "myotorso_abdomen",
    "myoarms",
    "myoarm_r",
    "myohands",
    "myohand_r",
    "myolegs",
    "myolegs26",
    "myolegs_abdomen",
)


@pytest.fixture(scope="module")
def fullbody_model():
    return load_spec("myofullbody").compile()


def test_total_mass_and_inertia_are_consistent(fullbody_model):
    report = audit_mass(fullbody_model)
    assert report["total_mass"] == pytest.approx(FULLBODY_MASS, abs=0.01)
    assert report["triangle_violations"] == []


def test_only_attachment_frames_are_massless(fullbody_model):
    massless = {
        mujoco.mj_id2name(fullbody_model, mujoco.mjtObj.mjOBJ_BODY, b)
        for b in range(1, fullbody_model.nbody)
        if fullbody_model.body_mass[b] == 0
    }
    assert massless == {
        "Full Body",
        "head_attach",
        "neck",
        "myohead_base",
        "Arm_attachment",
        "cervical_spine",
        "myoarm_r_root",
        "myoarm_l_root",
        "myolegs_root",
    }


def test_head_and_neck_carry_de_leva_fraction(fullbody_model):
    """Skull and cervical segments hold 6.94 % of body mass; the torso no longer contains the head."""
    share = subtree_mass(fullbody_model, "myohead_base") / fullbody_model.body_mass.sum()
    assert share == pytest.approx(DE_LEVA_HEAD_NECK, abs=1e-4)
    assert audit_mass(fullbody_model)["head_fraction"] == pytest.approx(share)
    assert fullbody_model.body_mass[fullbody_model.body("torso").id] == pytest.approx(TORSO_MASS)


@pytest.mark.parametrize("name", RIGID_HEAD_MODELS)
def test_part_models_keep_the_rigid_head(name):
    model = load_model(name)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "myohead_base") < 0
    head = model.body("head").id
    assert model.body_mass[head] == pytest.approx(RIGID_HEAD_MASS)
    assert model.body_mass[head] / RIGID_HEAD_BODY_MASS == pytest.approx(DE_LEVA_HEAD_NECK, abs=1e-4)
    assert model.body_mass[model.body("torso").id] == pytest.approx(TORSO_MASS)


@pytest.mark.parametrize("name", NECK_PART_MODELS)
def test_torso_models_carry_the_fullbody_head_and_neck(name, fullbody_model):
    model = load_model(name)
    assert subtree_mass(model, "myohead_base") == pytest.approx(subtree_mass(fullbody_model, "myohead_base"))
    assert model.body_mass[model.body("torso").id] == pytest.approx(TORSO_MASS)


def test_model_is_stable(fullbody_model):
    data = mujoco.MjData(fullbody_model)
    for _ in range(500):
        mujoco.mj_step(fullbody_model, data)
    assert abs(data.qpos).max() < 1e3
