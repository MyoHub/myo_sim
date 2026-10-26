import mujoco
import pytest

from myo_sim import load_spec
from myo_sim.build.mass import apply_head_mass_correction, audit_mass, subtree_mass

FULLBODY_MASS = 84.3
DE_LEVA_HEAD_NECK = (0.055, 0.085)


@pytest.fixture(scope="module")
def fullbody_model():
    return load_spec("myofullbody").compile()


def test_total_mass_and_inertia_are_consistent(fullbody_model):
    report = audit_mass(fullbody_model)
    assert report["total_mass"] == pytest.approx(FULLBODY_MASS, abs=0.1)
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
        "Arm_attachment",
        "cervical_spine",
        "myoarm_r_root",
        "myoarm_l_root",
        "myolegs_root",
    }


def test_published_head_is_light_by_default(fullbody_model):
    """Documents the current convention: the OpenSim torso already holds the head."""
    assert audit_mass(fullbody_model)["head_fraction"] < 0.03


def test_head_mass_correction_moves_mass_without_changing_total(fullbody_model):
    corrected = load_spec("myofullbody", head_mass_correction=True).compile()
    report = audit_mass(corrected)
    assert report["total_mass"] == pytest.approx(audit_mass(fullbody_model)["total_mass"], abs=1e-6)
    assert DE_LEVA_HEAD_NECK[0] < report["head_fraction"] < DE_LEVA_HEAD_NECK[1]
    assert report["triangle_violations"] == []
    assert subtree_mass(corrected, "humerus_r") == pytest.approx(subtree_mass(fullbody_model, "humerus_r"))
    assert subtree_mass(corrected, "femur_r") == pytest.approx(subtree_mass(fullbody_model, "femur_r"))


def test_corrected_model_is_stable():
    model = load_spec("myofullbody", head_mass_correction=True).compile()
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert abs(data.qpos).max() < 1e3


@pytest.mark.parametrize("model_name", ("myolegs", "myotorso_arms"))
def test_correction_keeps_total_on_partial_models(model_name):
    before = load_spec(model_name).compile().body_mass.sum()
    after = apply_head_mass_correction(load_spec(model_name)).compile().body_mass.sum()
    assert after == pytest.approx(before, abs=1e-6)
