"""The ``standing`` keyframe: a physically clean start that changes nothing else in the model."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.compose import MODEL_REGISTRY, build_spec
from myo_sim.build.keyframes import STANDING_KEYFRAME_PATH

MODELS = ("myofullbody", "myolegs", "myolegs_abdomen")
MAX_FOOT_FORCE_N = 150.0
MAX_DEPTH_M = 1e-3
# Peak joint speed over 50 ms at zero activation, per model. Passive muscle force sets it: see the wiki page.
MAX_SPEED_50MS = {"myofullbody": 5.5, "myolegs": 3.0, "myolegs_abdomen": 3.0}
MAX_SPEED_50MS_MUSCLES_OFF = 2.0
MIN_QPOS0_SPEED_50MS = 9.0
MAX_DEPTH_50MS_M = {"myofullbody": 1e-3, "myolegs": 2e-3, "myolegs_abdomen": 2e-3}
# No tendon or equality refers to the toes; the hand joints are held by the passive tendons and couplings.
DELETED_JOINT_TOKENS = ("mtp_angle_",)


def _script():
    path = Path(__file__).resolve().parents[1] / "scripts" / "generate_standing_keyframes.py"
    spec = importlib.util.spec_from_file_location("generate_standing_keyframes", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SCRIPT = _script()


def _state(model, qpos):
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    return data


def _standing_qpos(model):
    return model.key_qpos[model.key("standing").id].copy()


def _joint_values(model, qpos, prefix=""):
    out = {}
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE and name.startswith(prefix):
            out[name[len(prefix) :]] = float(qpos[model.jnt_qposadr[joint]])
    return out


def test_table_is_what_the_script_writes():
    written = json.loads(STANDING_KEYFRAME_PATH.read_text())
    table = SCRIPT.build_table()
    assert table["provenance"] == written["provenance"]
    assert table["models"].keys() == written["models"].keys()
    for name, entry in table["models"].items():
        assert entry["joints"].keys() == written["models"][name]["joints"].keys()
        assert entry["root_z"] == pytest.approx(written["models"][name]["root_z"], abs=1e-7)
        for key in ("root_y_offset", "root_quat"):
            assert entry.get(key, 0.0) == pytest.approx(written["models"][name].get(key, 0.0), abs=1e-7), key
        for joint, value in entry["joints"].items():
            assert value == pytest.approx(written["models"][name]["joints"][joint], abs=1e-7), joint


@pytest.mark.parametrize("name", MODELS)
def test_keyframe_changes_no_other_model_array(name):
    with_key = myo_sim.load_model(name)
    without = build_spec(name).compile()
    assert with_key.nkey == 1 and without.nkey == 0
    assert mujoco.mj_id2name(with_key, mujoco.mjtObj.mjOBJ_KEY, 0) == "standing"
    for field in ("qpos0", "qpos_spring", "body_mass", "jnt_range", "eq_data", "actuator_gainprm", "tendon_lengthspring"):
        assert np.array_equal(getattr(with_key, field), getattr(without, field)), field


@pytest.mark.parametrize("name", sorted(set(MODEL_REGISTRY) - set(MODELS)))
def test_other_models_have_no_keyframe(name):
    assert myo_sim.load_model(name).nkey == 0


@pytest.mark.parametrize("name", MODELS)
def test_couplings_hold(name):
    model = myo_sim.load_model(name)
    assert SCRIPT.coupling_residual(model, _standing_qpos(model)) < 1e-7
    assert SCRIPT.coupling_residual(model, model.qpos0) > 0.05, "negative control: qpos0 violates the knee couplings"


@pytest.mark.parametrize("name", MODELS)
def test_only_the_feet_touch_at_t0(name):
    model = myo_sim.load_model(name)
    contacts, weight = SCRIPT.t0_contacts(model, _state(model, _standing_qpos(model)))
    assert contacts, "the feet should be on the floor"
    assert all("foot" in geom1 + geom2 for geom1, geom2, _, _ in contacts), contacts
    assert all(force < MAX_FOOT_FORCE_N for _, _, force, _ in contacts), contacts
    assert sum(force for _, _, force, _ in contacts) < 1.1 * weight
    assert all(dist > -MAX_DEPTH_M for _, _, _, dist in contacts), contacts


@pytest.mark.parametrize("side", ("r", "l"))
def test_thumb_starts_at_its_passive_equilibrium(side):
    model = myo_sim.load_model("myofullbody")
    data = mujoco.MjData(model)
    assert np.abs(SCRIPT.thumb_moments(model, data, _standing_qpos(model), side)).max() < 1e-5
    assert np.abs(SCRIPT.thumb_moments(model, data, model.qpos0, side)).max() > 0.5, "negative control"


def test_trunk_hinge_stands_over_the_hip_axis():
    model = myo_sim.load_model("myolegs_abdomen")
    qpos = _standing_qpos(model)
    data = _state(model, qpos)
    hinge, hip = data.joint(SCRIPT.TRUNK_HINGE).xanchor, data.joint("hip_flexion_r").xanchor
    assert hinge[1] == pytest.approx(hip[1], abs=1e-6)
    upright = _state(model, model.qpos0)
    assert np.linalg.norm(data.body("torso").xmat - upright.body("torso").xmat) < 1e-6, "the trunk stays upright"
    assert np.linalg.norm(data.body("tibia_r").xmat - upright.body("tibia_r").xmat) < 0.02, "the legs stay upright"
    assert data.joint(SCRIPT.TRUNK_HINGE).xanchor[1] != pytest.approx(upright.joint(SCRIPT.TRUNK_HINGE).xanchor[1], abs=0.01), (
        "negative control: without the tilt the hinge is behind the hips"
    )


def test_qpos0_negative_control_for_contacts():
    model = myo_sim.load_model("myofullbody")
    contacts, _ = SCRIPT.t0_contacts(model, _state(model, model.qpos0))
    assert max(force for _, _, force, _ in contacts) > 500.0
    assert min(dist for _, _, _, dist in contacts) < -0.01


def test_arms_clear_the_thorax():
    model = myo_sim.load_model("myofullbody")
    data = _state(model, _standing_qpos(model))
    thorax = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "thorax_coll1")
    for side in ("r", "l"):
        humerus = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"humerus_coll_{side}")
        assert mujoco.mj_geomDistance(model, data, thorax, humerus, 0.2, np.zeros(6)) > 0.005


@pytest.mark.parametrize("name", MODELS)
def test_first_50_ms_stay_slow(name):
    model = myo_sim.load_model(name)
    standing = _standing_qpos(model)
    peak, joint, deepest = SCRIPT.rollout_extremes(model, standing)
    assert peak < MAX_SPEED_50MS[name], joint
    assert deepest > -MAX_DEPTH_50MS_M[name]
    assert SCRIPT.rollout_extremes(model, model.qpos0)[0] > MIN_QPOS0_SPEED_50MS, "negative control"
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    assert SCRIPT.rollout_extremes(model, standing)[0] < MAX_SPEED_50MS_MUSCLES_OFF


@pytest.mark.parametrize("name", MODELS)
def test_muscle_lengths_stay_valid(name):
    model = myo_sim.load_model(name)
    at_qpos0 = SCRIPT.normalised_lengths(model, _state(model, model.qpos0))
    standing = SCRIPT.normalised_lengths(model, _state(model, _standing_qpos(model)))
    low, high = model.actuator_gainprm[:, 4], model.actuator_gainprm[:, 5]
    assert ((standing < low) | (standing > high)).sum() <= ((at_qpos0 < low) | (at_qpos0 > high)).sum()
    assert standing.max() <= at_qpos0.max()


def _delete_joints(spec) -> int:
    joints = [j for j in spec.joints if j.name.startswith(DELETED_JOINT_TOKENS)]
    assert joints
    for joint in joints:
        spec.delete(joint)
    return len(joints)


def test_spec_stays_editable_by_default():
    spec = myo_sim.load_spec("myofullbody")
    assert len(spec.keys) == 0
    _delete_joints(spec)
    assert spec.compile().nkey == 0


def test_deleting_a_joint_from_a_spec_with_the_keyframe_fails():
    """Why ``load_spec`` does not add the keyframe by default."""
    spec = myo_sim.load_spec("myofullbody", build_kwargs={"standing_keyframe": True})
    _delete_joints(spec)
    with pytest.raises(ValueError, match="standing"):
        spec.compile()


def test_add_after_deleting_joints_matches_by_name():
    reference = myo_sim.load_model("myofullbody")
    spec = myo_sim.load_spec("myofullbody")
    deleted = _delete_joints(spec)
    myo_sim.add_standing_keyframe(spec, "myofullbody")
    model = spec.compile()
    want = _joint_values(reference, _standing_qpos(reference))
    got = _joint_values(model, model.key_qpos[0])
    assert len(got) == len(want) - deleted
    assert all(got[name] == pytest.approx(want[name]) for name in got)
    assert model.key_qpos[0][2] == pytest.approx(reference.key_qpos[0][2])


def test_add_after_adding_a_joint_matches_by_name():
    reference = myo_sim.load_model("myofullbody")
    spec = myo_sim.load_spec("myofullbody")
    body = spec.body("femur_r").add_body(name="extra")
    body.add_joint(name="extra_hinge", axis=[1, 0, 0], type=mujoco.mjtJoint.mjJNT_HINGE, range=[-1, 1])
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.01, 0, 0], mass=0.01)
    myo_sim.add_standing_keyframe(spec, "myofullbody")
    model = spec.compile()
    want = _joint_values(reference, _standing_qpos(reference))
    got = _joint_values(model, model.key_qpos[0])
    assert got.pop("extra_hinge") == 0.0
    assert all(got[name] == pytest.approx(want[name]) for name in want)


def test_adding_twice_keeps_one_keyframe():
    spec = myo_sim.load_spec("myofullbody", build_kwargs={"standing_keyframe": True})
    myo_sim.add_standing_keyframe(spec, "myofullbody")
    assert spec.compile().nkey == 1


def test_attach_to_an_empty_spec():
    reference = myo_sim.load_model("myofullbody")
    parent = mujoco.MjSpec()
    child = myo_sim.load_spec("myofullbody", build_kwargs={"standing_keyframe": True})
    parent.attach(child, prefix="a_", frame=parent.worldbody.add_frame())
    model = parent.compile()
    assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0) == "a_standing"
    assert np.allclose(model.key_qpos[0], reference.key_qpos[0])


def test_two_body_attach_keeps_one_keyframe_per_body():
    reference = myo_sim.load_model("myofullbody")
    parent = mujoco.MjSpec()
    for prefix, offset in (("a_", 0.0), ("b_", 2.0)):
        frame = parent.worldbody.add_frame()
        frame.pos = [0.0, offset, 0.0]
        parent.attach(myo_sim.load_spec("myofullbody", build_kwargs={"standing_keyframe": True}), prefix=prefix, frame=frame)
    model = parent.compile()
    assert model.nkey == 2
    want = _joint_values(reference, reference.key_qpos[0])
    for index, (prefix, other) in enumerate((("a_", "b_"), ("b_", "a_"))):
        assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, index) == f"{prefix}standing"
        got = _joint_values(model, model.key_qpos[index], prefix)
        assert all(got[name] == pytest.approx(value) for name, value in want.items())
        idle = _joint_values(model, model.key_qpos[index], other)
        rest = _joint_values(model, model.qpos0, other)
        assert all(idle[name] == pytest.approx(rest[name]) for name in idle), "the other body stays at qpos0"


def test_attach_without_the_keyframe_stays_at_no_keyframe():
    parent = mujoco.MjSpec()
    for prefix in ("a_", "b_"):
        parent.attach(myo_sim.load_spec("myofullbody"), prefix=prefix, frame=parent.worldbody.add_frame())
    assert parent.compile().nkey == 0


@pytest.mark.parametrize("name", sorted(myo_sim.FRAGMENT_SPEC_BUILDERS))
def test_part_specs_have_no_keyframe(name):
    assert myo_sim.FRAGMENT_SPEC_BUILDERS[name]().compile().nkey == 0


def test_unknown_model_raises():
    with pytest.raises(KeyError):
        build_spec("myoarms", build_kwargs={"standing_keyframe": True})
