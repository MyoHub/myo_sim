"""The posture keyframes: physically clean starts that change nothing else in the model."""

from __future__ import annotations

import copy
import importlib.util
import sys
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.compose import MODEL_REGISTRY, build_spec
from myo_sim.build.keyframes import add_posture_keyframe, add_posture_keyframes, load_posture_table, posture_qpos

TABLE = load_posture_table()
MODELS = tuple(TABLE["models"])
FREE_STANDING = tuple(name for name, postures in TABLE["models"].items() if "standing" in postures)
ALL_POSTURES = [(model, posture) for model, postures in TABLE["models"].items() for posture in postures]
FEET_POSTURES = [(model, posture) for model, posture in ALL_POSTURES if TABLE["postures"][posture]["feet_support"]]
WITH_ARMS = [
    (model, posture)
    for model, posture in ALL_POSTURES
    if model in ("myofullbody", "myoarm_r", "myoarms", "myotorso_arm_r", "myotorso_arms")
]
EDITABLE = {"myofullbody": "femur_r", "myoarms": "humerus_r", "myohands": "radius_r"}
BILATERAL = ("myofullbody", "myoarms", "myotorso_arms", "myohands")
FINGER_TOKENS = ("cmc_", "mcp", "pm", "md")

# Peak joint speed over 50 ms at zero activation. Passive muscle force sets it: see the wiki page.
# The mesh feet of myolegs26 sink further and rock on their toes, more so on MuJoCo older than 3.17.
MAX_SPEED_50MS = {"free": 6.0, "mesh_feet": 6.0, "arm": 2.5, "hand": 2.0}
MAX_SPEED_50MS_MUSCLES_OFF = {"free": 4.0, "mesh_feet": 6.5, "arm": 2.0, "hand": 1.0}
MAX_DEPTH_50MS_M = {"free": 3e-3, "mesh_feet": 12e-3, "arm": 1e-3, "hand": 1e-3}
MIN_QPOS0_SPEED_50MS = 9.0
MIN_THORAX_CLEARANCE_M = 5e-3
MIN_BALANCE_MARGIN_M = 0.02
MAX_MIRROR_ERROR_M = 0.02


def _script():
    path = Path(__file__).resolve().parents[1] / "scripts" / "generate_postures.py"
    spec = importlib.util.spec_from_file_location("generate_postures", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses look their module up there
    spec.loader.exec_module(module)
    return module


SCRIPT = _script()


@lru_cache(maxsize=None)
def _model(name):
    return myo_sim.load_model(name)


def _family(name):
    if name == "myolegs26":
        return "mesh_feet"
    if name in FREE_STANDING:
        return "free"
    return "hand" if "hand" in name else "arm"


def _key_qpos(name, posture):
    model = _model(name)
    return model.key_qpos[model.key(posture).id].copy()


def _state(model, qpos):
    return SCRIPT.state_at(model, qpos)


def _joint_values(model, qpos, prefix=""):
    out = {}
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE and name.startswith(prefix):
            out[name[len(prefix) :]] = float(qpos[model.jnt_qposadr[joint]])
    return out


def _key_names(model):
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key) for key in range(model.nkey)]


def table_differences(written, generated, tolerance=1e-7):
    """Where a written table differs from what the script generates."""
    differences = []
    if written["provenance"] != generated["provenance"]:
        differences.append("provenance")
    if written["postures"] != generated["postures"]:
        differences.append("postures")
    if list(written["models"]) != list(generated["models"]):
        differences.append("models")
    for name, postures in generated["models"].items():
        if list(postures) != list(written["models"].get(name, {})):
            differences.append(f"{name}: posture list")
            continue
        for posture, entry in postures.items():
            stored = written["models"][name][posture]
            if entry["joints"].keys() != stored["joints"].keys() or entry.keys() != stored.keys():
                differences.append(f"{name}/{posture}: keys")
                continue
            for key in entry.keys() - {"joints"}:
                if abs(entry[key] - stored[key]) > tolerance:
                    differences.append(f"{name}/{posture}: {key}")
            for joint, value in entry["joints"].items():
                if abs(value - stored["joints"][joint]) > tolerance:
                    differences.append(f"{name}/{posture}: {joint}")
    return differences


@pytest.fixture(scope="module")
def generated():
    return SCRIPT.build_table()


def test_table_is_what_the_script_writes(generated):
    assert table_differences(TABLE, generated) == []


@pytest.mark.parametrize(
    "joint_path",
    [("myofullbody", "ready", "joints", "knee_angle_r"), ("myohand_r", "relaxed", "joints", "md3_flexion")],
)
def test_comparison_with_the_script_catches_an_edited_value(generated, joint_path):
    edited = copy.deepcopy(TABLE)
    model, posture, _, joint = joint_path
    joints = edited["models"][model][posture]["joints"]
    joints[next(name for name in joints if name.startswith(joint))] += 1e-3
    assert table_differences(edited, generated)


def test_comparison_with_the_script_catches_a_moved_root(generated):
    edited = copy.deepcopy(TABLE)
    edited["models"]["myolegs"]["squat"]["root_z"] += 1e-3
    assert "myolegs/squat: root_z" in table_differences(edited, generated)


@pytest.mark.parametrize("posture", sorted(TABLE["postures"]))
def test_every_posture_has_provenance_and_a_wiki_entry(posture):
    entry = TABLE["postures"][posture]
    assert entry["description"] and entry["source"]
    wiki = Path(__file__).resolve().parents[1] / "docs" / "wiki" / "postures.md"
    assert f"`{posture}`" in wiki.read_text()


def test_listing():
    assert myo_sim.list_postures("myofullbody")[0] == "standing"
    assert myo_sim.list_postures("myoarm") == myo_sim.list_postures("myoarm_r")
    assert myo_sim.list_postures("hand") == myo_sim.list_postures("myohand_r")
    assert myo_sim.list_postures("myotorso") == []
    assert myo_sim.list_postures("no_such_model") == []


@pytest.mark.parametrize("name", FREE_STANDING)
def test_standing_comes_first(name):
    assert _key_names(_model(name))[0] == "standing"
    assert np.array_equal(_model(name).key_qpos[0], _key_qpos(name, "standing"))


@pytest.mark.parametrize("name", MODELS)
def test_load_includes_all_postures_and_changes_no_other_model_array(name):
    with_keys = _model(name)
    without = build_spec(name).compile()
    assert _key_names(with_keys) == myo_sim.list_postures(name)
    assert without.nkey == 0
    for field in ("qpos0", "qpos_spring", "body_mass", "jnt_range", "eq_data", "actuator_gainprm", "tendon_lengthspring"):
        assert np.array_equal(getattr(with_keys, field), getattr(without, field)), field


@pytest.mark.parametrize("alias, name", [("hand", "myohand_r"), ("myohand", "myohand_r"), ("myoarm", "myoarm_r")])
def test_aliases_load_the_postures(alias, name):
    assert _key_names(myo_sim.load_model(alias)) == myo_sim.list_postures(name)


@pytest.mark.parametrize("name", sorted(set(MODEL_REGISTRY) - set(MODELS)))
def test_other_models_have_no_keyframe(name):
    assert myo_sim.load_model(name).nkey == 0


@pytest.mark.parametrize("name, posture", ALL_POSTURES)
def test_keyframe_is_the_solved_pose(name, posture):
    model = build_spec(name).compile()
    solved, _ = SCRIPT.solve_pose(model, name, _declaration(name, posture))
    assert np.allclose(_key_qpos(name, posture), solved, atol=2e-7)


def _declaration(name, posture):
    return next(p for p in SCRIPT.MODELS[name] if p.name == posture)


@pytest.mark.parametrize("name, posture", ALL_POSTURES)
def test_physical_checks(name, posture):
    model = _model(name)
    assert SCRIPT.posture_failures(model, _key_qpos(name, posture), TABLE["postures"][posture]["feet_support"]) == []


def _mutated_qpos(name, posture, change):
    """``qpos`` from a table entry after *change* has edited a copy of it."""
    entry = copy.deepcopy(TABLE["models"][name][posture])
    change(entry)
    return posture_qpos(_model(name), entry)


def _add_to_joint(entry, joint, amount):
    entry["joints"][joint] = entry["joints"].get(joint, 0.0) + amount


FOOT_MUTATIONS = {
    "root_up": lambda entry: entry.update(root_z=entry["root_z"] + 0.02),
    "root_down": lambda entry: entry.update(root_z=entry["root_z"] - 0.03),
    "ankle": lambda entry: _add_to_joint(entry, "ankle_angle_r", 0.1),
    "ankle_left": lambda entry: _add_to_joint(entry, "ankle_angle_l", -0.1),
    "pelvis_pitch": lambda entry: entry.update(root_pitch=entry.get("root_pitch", 0.0) + 0.1),
    "knee": lambda entry: _add_to_joint(entry, "knee_angle_r", 0.2),
}


@pytest.mark.parametrize("mutation", sorted(FOOT_MUTATIONS))
@pytest.mark.parametrize("name, posture", FEET_POSTURES)
def test_checks_fail_on_a_mutated_foot_entry(name, posture, mutation):
    qpos = _mutated_qpos(name, posture, FOOT_MUTATIONS[mutation])
    assert SCRIPT.posture_failures(_model(name), qpos, True)


@pytest.mark.parametrize("name, posture", ALL_POSTURES)
def test_checks_fail_on_a_joint_out_of_range(name, posture):
    model = _model(name)
    joint = next(j for j in range(model.njnt) if model.jnt_limited[j] and model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE)
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    qpos = _mutated_qpos(
        name, posture, lambda entry: entry["joints"].update({joint_name: float(model.jnt_range[joint][1]) + 0.1})
    )
    failures = SCRIPT.posture_failures(model, qpos, TABLE["postures"][posture]["feet_support"])
    assert any("outside" in failure for failure in failures), failures


@pytest.mark.parametrize("name", [n for n in MODELS if "arm" in n])
def test_checks_fail_when_the_arm_is_back_inside_the_thorax(name):
    posture = myo_sim.list_postures(name)[0]
    model = _model(name)
    entry = copy.deepcopy(TABLE["models"][name][posture])
    entry["joints"] = {}
    failures = SCRIPT.posture_failures(model, posture_qpos(model, entry), False)
    assert any("overlap" in failure for failure in failures), failures


@pytest.mark.parametrize("name", FREE_STANDING)
def test_qpos0_is_a_negative_control(name):
    model = _model(name)
    failures = SCRIPT.posture_failures(model, model.qpos0, True)
    assert failures and any("off the floor" in f or "overlap" in f for f in failures)
    if name != "myolegs26":
        assert SCRIPT.coupling_residual(model, model.qpos0) > 0.05


@pytest.mark.parametrize("name, posture", FEET_POSTURES)
def test_only_the_feet_touch_at_t0(name, posture):
    model = _model(name)
    data = _state(model, _key_qpos(name, posture))
    feet = {g for side in SCRIPT.SIDES for g in SCRIPT._foot_collision_geoms(model, side)}
    assert data.ncon > 0
    for index in range(data.ncon):
        assert set(data.contact[index].geom.tolist()) & feet


@pytest.mark.parametrize("name, posture", WITH_ARMS)
def test_arms_clear_the_thorax(name, posture):
    model = _model(name)
    data = _state(model, _key_qpos(name, posture))
    thorax = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "thorax_coll1")
    for side in ("r", "l"):
        humerus = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"humerus_coll_{side}")
        if humerus >= 0:
            distance = mujoco.mj_geomDistance(model, data, thorax, humerus, 0.2, np.zeros(6))
            assert distance > MIN_THORAX_CLEARANCE_M, f"{side}: {distance}"


@pytest.mark.parametrize("name, posture", ALL_POSTURES)
def test_first_50_ms_stay_slow(name, posture):
    model = _model(name)
    qpos = _key_qpos(name, posture)
    peak, joint, deepest = SCRIPT.rollout_extremes(model, qpos)
    assert peak < MAX_SPEED_50MS[_family(name)], joint
    assert deepest > -MAX_DEPTH_50MS_M[_family(name)]
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    try:
        assert SCRIPT.rollout_extremes(model, qpos)[0] < MAX_SPEED_50MS_MUSCLES_OFF[_family(name)]
    finally:
        model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)


@pytest.mark.parametrize("name", [n for n in FREE_STANDING if n != "myolegs26"])
def test_qpos0_is_fast_in_the_first_50_ms(name):
    assert SCRIPT.rollout_extremes(_model(name), _model(name).qpos0)[0] > MIN_QPOS0_SPEED_50MS


@pytest.mark.parametrize("name, posture", ALL_POSTURES)
def test_muscle_lengths_stay_valid(name, posture):
    model = _model(name)
    low, high = model.actuator_gainprm[:, 4], model.actuator_gainprm[:, 5]

    def outside(qpos):
        lengths = SCRIPT.normalised_lengths(model, _state(model, qpos))
        indices = np.flatnonzero((lengths < low) | (lengths > high))
        return {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(i)) for i in indices}

    # RI5 runs short with a flexed little finger: see the wiki page.
    new = {muscle for muscle in outside(_key_qpos(name, posture)) - outside(model.qpos0) if not muscle.startswith("RI5")}
    assert not new


@pytest.mark.parametrize("name", FREE_STANDING)
def test_balance_is_what_the_wiki_says(name):
    model = _model(name)
    for posture in myo_sim.list_postures(name):
        margin = SCRIPT.support_margin(model, _state(model, _key_qpos(name, posture)))
        assert margin < 0.0 if posture == "seat_off" else margin > MIN_BALANCE_MARGIN_M, posture


@pytest.mark.parametrize("name", ["myoarm_r", "myoarms", "myohand_r", "myohands"])
def test_rest_postures_hold_at_zero_activation(name):
    model = _model(name)
    assert SCRIPT.collapse_time(model, model.key_qpos[0], horizon=1.0) == (None, "")


@pytest.mark.parametrize("name", FREE_STANDING)
def test_no_free_standing_posture_holds_at_zero_activation(name):
    model = _model(name)
    for key in range(model.nkey):
        assert SCRIPT.collapse_time(model, model.key_qpos[key], horizon=1.0)[0] is not None


def test_relaxed_fingers_are_flexed_towards_the_palm():
    model = _model("myohand_r")
    relaxed = _state(model, _key_qpos("myohand_r", "relaxed"))
    rest = _state(model, model.qpos0)
    for digit in range(2, 6):
        tip = f"distph{digit}_r"
        at = np.linalg.norm(relaxed.body(tip).xpos - relaxed.body("lunate_r").xpos)
        assert at < np.linalg.norm(rest.body(tip).xpos - rest.body("lunate_r").xpos), tip


@pytest.mark.parametrize("name", BILATERAL)
def test_left_and_right_mirror(name):
    model = _model(name)
    for posture in myo_sim.list_postures(name):
        if posture == "staggered":
            continue
        data = _state(model, _key_qpos(name, posture))
        for body in ("distph3", "distph5", "distal_thumb"):
            right, left = data.body(f"{body}_r").xpos, data.body(f"{body}_l").xpos
            centre = data.qpos[0] if SCRIPT._has_joint(model, "root") else 0.0
            assert abs((right[0] - centre) + (left[0] - centre)) < MAX_MIRROR_ERROR_M, (posture, body)
            assert abs(right[1] - left[1]) < MAX_MIRROR_ERROR_M, (posture, body)
            assert abs(right[2] - left[2]) < MAX_MIRROR_ERROR_M, (posture, body)


def _delete_finger_joints(spec) -> int:
    fingers = [j for j in spec.joints if j.name.startswith(FINGER_TOKENS)]
    assert fingers
    for joint in fingers:
        spec.delete(joint)
    return len(fingers)


@pytest.mark.parametrize("name", EDITABLE)
def test_spec_stays_editable_by_default(name):
    spec = myo_sim.load_spec(name)
    assert len(spec.keys) == 0
    _delete_finger_joints(spec)
    assert spec.compile().nkey == 0


@pytest.mark.parametrize("name", EDITABLE)
def test_deleting_a_joint_from_a_spec_with_keyframes_fails(name):
    """Why ``load_spec`` does not add the keyframes by default."""
    spec = myo_sim.load_spec(name, build_kwargs={"postures": True})
    _delete_finger_joints(spec)
    with pytest.raises(ValueError, match="invalid qpos size"):
        spec.compile()


@pytest.mark.parametrize("name", EDITABLE)
def test_add_after_deleting_joints_matches_by_name(name):
    reference = _model(name)
    spec = myo_sim.load_spec(name)
    deleted = _delete_finger_joints(spec)
    keys = myo_sim.add_posture_keyframes(spec, name)
    model = spec.compile()
    assert model.nkey == len(keys) == reference.nkey
    for key in range(model.nkey):
        want = _joint_values(reference, reference.key_qpos[key])
        got = _joint_values(model, model.key_qpos[key])
        assert len(got) == len(want) - deleted
        assert all(got[joint] == pytest.approx(want[joint]) for joint in got)


@pytest.mark.parametrize("name, parent", EDITABLE.items())
def test_add_after_adding_a_joint_matches_by_name(name, parent):
    reference = _model(name)
    spec = myo_sim.load_spec(name)
    body = spec.body(parent).add_body(name="extra")
    body.add_joint(name="extra_hinge", axis=[1, 0, 0], type=mujoco.mjtJoint.mjJNT_HINGE, range=[-1, 1])
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.01, 0, 0], mass=0.01)
    myo_sim.add_posture_keyframes(spec, name)
    model = spec.compile()
    for key in range(model.nkey):
        want = _joint_values(reference, reference.key_qpos[key])
        got = _joint_values(model, model.key_qpos[key])
        assert got.pop("extra_hinge") == 0.0
        assert all(got[joint] == pytest.approx(want[joint]) for joint in want)


def test_root_pitch_survives_a_deleted_joint():
    spec = myo_sim.load_spec("myofullbody")
    _delete_finger_joints(spec)
    add_posture_keyframe(spec, "myofullbody", "squat")
    model = spec.compile()
    assert np.allclose(model.key_qpos[0][:7], _key_qpos("myofullbody", "squat")[:7])
    assert abs(model.key_qpos[0][4]) > 0.1


def test_adding_twice_keeps_one_keyframe_each():
    spec = myo_sim.load_spec("myofullbody", build_kwargs={"postures": True})
    myo_sim.add_posture_keyframes(spec, "myofullbody")
    assert _key_names(spec.compile()) == myo_sim.list_postures("myofullbody")


def test_replacing_a_keyframe_moves_it_to_the_end():
    spec = myo_sim.load_spec("myoarms", build_kwargs={"postures": True})
    add_posture_keyframe(spec, "myoarms", "arms_at_side")
    assert _key_names(spec.compile()) == ["forearm_raised", "reach", "arms_at_side"]


def test_subset_keeps_the_requested_order():
    spec = myo_sim.load_spec("myolegs")
    add_posture_keyframes(spec, "myolegs", ["squat", "standing"])
    model = spec.compile()
    assert _key_names(model) == ["squat", "standing"]
    assert np.array_equal(model.key_qpos[1], _key_qpos("myolegs", "standing"))


def test_build_kwarg_with_a_list():
    model = myo_sim.load_spec("myolegs", build_kwargs={"postures": ["ready"]}).compile()
    assert _key_names(model) == ["ready"]


def test_old_build_kwarg_and_wrapper_still_add_standing():
    spec = myo_sim.load_spec("myofullbody", build_kwargs={"standing_keyframe": True})
    assert _key_names(spec.compile()) == ["standing"]
    edited = myo_sim.load_spec("myofullbody")
    myo_sim.add_standing_keyframe(edited)
    model = edited.compile()
    assert _key_names(model) == ["standing"]
    assert np.array_equal(model.key_qpos[0], _key_qpos("myofullbody", "standing"))


@pytest.mark.parametrize("name", EDITABLE)
def test_attach_to_an_empty_spec(name):
    reference = _model(name)
    parent = mujoco.MjSpec()
    child = myo_sim.load_spec(name, build_kwargs={"postures": True})
    parent.attach(child, prefix="a_", frame=parent.worldbody.add_frame())
    model = parent.compile()
    assert _key_names(model) == [f"a_{posture}" for posture in myo_sim.list_postures(name)]
    assert np.allclose(model.key_qpos, reference.key_qpos)


@pytest.mark.parametrize("name", EDITABLE)
def test_two_body_attach_keeps_all_postures_per_body(name):
    reference = _model(name)
    parent = mujoco.MjSpec()
    for prefix, offset in (("a_", 0.0), ("b_", 2.0)):
        frame = parent.worldbody.add_frame()
        frame.pos = [0.0, offset, 0.0]
        parent.attach(myo_sim.load_spec(name, build_kwargs={"postures": True}), prefix=prefix, frame=frame)
    model = parent.compile()
    postures = myo_sim.list_postures(name)
    assert model.nkey == 2 * len(postures)
    names = _key_names(model)
    for prefix, other in (("a_", "b_"), ("b_", "a_")):
        for posture in postures:
            key = names.index(f"{prefix}{posture}")
            want = _joint_values(reference, reference.key_qpos[reference.key(posture).id])
            got = _joint_values(model, model.key_qpos[key], prefix)
            assert all(got[joint] == pytest.approx(value) for joint, value in want.items())
            idle = _joint_values(model, model.key_qpos[key], other)
            rest = _joint_values(model, model.qpos0, other)
            assert all(idle[joint] == pytest.approx(rest[joint]) for joint in idle), "the other body stays at qpos0"


def test_attach_without_keyframes_stays_at_no_keyframe():
    parent = mujoco.MjSpec()
    for prefix in ("a_", "b_"):
        parent.attach(myo_sim.load_spec("myofullbody"), prefix=prefix, frame=parent.worldbody.add_frame())
    assert parent.compile().nkey == 0


@pytest.mark.parametrize("name", sorted(myo_sim.FRAGMENT_SPEC_BUILDERS))
def test_part_specs_have_no_keyframe(name):
    assert myo_sim.FRAGMENT_SPEC_BUILDERS[name]().compile().nkey == 0


def test_unknown_posture_or_model_raises():
    with pytest.raises(KeyError):
        build_spec("myoarms", build_kwargs={"standing_keyframe": True})
    with pytest.raises(KeyError):
        add_posture_keyframe(myo_sim.load_spec("myolegs"), "myolegs", "handstand")
    with pytest.raises(KeyError):
        add_posture_keyframe(myo_sim.load_spec("myolegs"), "no_such_model", "standing")
