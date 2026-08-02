import mujoco
import pytest

import myo_sim
from myo_sim.build.elastic import DEFAULT_ACHILLES, build_elastic_spec, verify_see_kinematics


def test_no_muscles_reproduces_rigid_model():
    spec, meta = build_elastic_spec(lambda: myo_sim.load_spec("myolegs"), muscles=[])
    model = spec.compile()
    rigid, _ = myo_sim.load("myolegs")

    assert model.nu == rigid.nu
    assert model.nq == rigid.nq
    assert model.nv == rigid.nv
    assert meta["applied"] == {}


def test_achilles_see_adds_expected_dofs():
    rigid, _ = myo_sim.load("myolegs")
    spec, meta = build_elastic_spec(lambda: myo_sim.load_spec("myolegs"), muscles=list(DEFAULT_ACHILLES.values()))
    model = spec.compile()

    assert model.nq - rigid.nq == len(DEFAULT_ACHILLES)
    assert model.nv - rigid.nv == len(DEFAULT_ACHILLES)
    assert set(meta["applied"]) == set(DEFAULT_ACHILLES)
    for name in DEFAULT_ACHILLES:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) >= 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_tendon_q") >= 0


@pytest.mark.parametrize("name", list(DEFAULT_ACHILLES))
def test_achilles_see_kinematics(name):
    spec, _ = build_elastic_spec(lambda: myo_sim.load_spec("myolegs"), muscles=list(DEFAULT_ACHILLES.values()))
    model = spec.compile()

    dl_dq = verify_see_kinematics(model, name)
    assert dl_dq == pytest.approx(-1.0, abs=0.03)
