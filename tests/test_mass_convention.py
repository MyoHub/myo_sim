import mujoco
import pytest

from myo_sim import load_spec
from myo_sim.build.mass import audit_mass

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


def test_head_carries_de_leva_fraction(fullbody_model):
    """The OpenSim torso mass was reduced so the head body holds about 6.9 % of body mass."""
    assert DE_LEVA_HEAD_NECK[0] < audit_mass(fullbody_model)["head_fraction"] < DE_LEVA_HEAD_NECK[1]


def test_model_is_stable(fullbody_model):
    data = mujoco.MjData(fullbody_model)
    for _ in range(500):
        mujoco.mj_step(fullbody_model, data)
    assert abs(data.qpos).max() < 1e3
