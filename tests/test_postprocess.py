"""Generic post-processing functions (model policy 4): myo_sim.disable_collisions()."""

import re

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.compose import COARSE_COLLISION_DISABLED_GEOM_BASENAMES, disable_finger_collision

FINGER_BODIES = r"(\d?proxph\d?|midph\d|distph\d|.*_thumb)_[rl]"


def _geom_names(model: mujoco.MjModel) -> list[str]:
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "" for i in range(model.ngeom)]


def _collidable(model: mujoco.MjModel) -> np.ndarray:
    return (model.geom_contype != 0) | (model.geom_conaffinity != 0)


def _pair_names(model: mujoco.MjModel) -> set[frozenset[str]]:
    names = _geom_names(model)
    return {frozenset((names[model.pair_geom1[i]], names[model.pair_geom2[i]])) for i in range(model.npair)}


def test_disable_all_collisions() -> None:
    model = myo_sim.disable_collisions(myo_sim.load_spec("myohands")).compile()
    assert not _collidable(model).any()
    assert model.npair == 0
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    assert data.ncon == 0


def test_disable_by_body_pattern_keeps_the_rest() -> None:
    reference = myo_sim.load_model("myohands")
    model = myo_sim.disable_collisions(myo_sim.load_spec("myohands"), bodies=FINGER_BODIES).compile()
    finger_bodies = {i for i in range(model.nbody) if re.fullmatch(FINGER_BODIES, model.body(i).name)}
    assert len(finger_bodies) == 2 * 14  # per hand: 4 fingers x 3 phalanges + 2 thumb segments
    in_fingers = np.isin(model.geom_bodyid, list(finger_bodies))
    assert in_fingers.any()
    assert not _collidable(model)[in_fingers].any()
    # Everything else, palm skin included, is untouched.
    assert np.array_equal(_collidable(model)[~in_fingers], _collidable(reference)[~in_fingers])
    assert _collidable(model)[model.geom("2mcskin_coll_r").id]


def test_disable_by_geom_pattern_removes_its_pairs() -> None:
    reference = myo_sim.load_model("myoarms")
    pattern = r"(humerus|ulna|radius)_coll(_\d)?_[rl]"
    model = myo_sim.disable_collisions(myo_sim.load_spec("myoarms"), geoms=pattern).compile()
    names = _geom_names(model)
    selected = np.array([bool(re.fullmatch(pattern, name)) for name in names])
    assert selected.sum() == 10
    assert not _collidable(model)[selected].any()
    assert np.array_equal(_collidable(model)[~selected], _collidable(reference)[~selected])
    kept = {pair for pair in _pair_names(reference) if not any(re.fullmatch(pattern, n) for n in pair)}
    assert _pair_names(reference) != kept, "myoarms has forearm pairs to remove"
    assert _pair_names(model) == kept


def test_pattern_list_and_union() -> None:
    model = myo_sim.disable_collisions(
        myo_sim.load_spec("myohand_r"), bodies=["distph2_r", "distph3_r"], geoms="1mcskin_r"
    ).compile()
    for name in ("distph2_coll_r", "distph3_coll_2_r", "1mcskin_r"):
        assert not _collidable(model)[model.geom(name).id], name
    assert _collidable(model)[model.geom("distph4_coll_r").id]


@pytest.mark.parametrize("name", ["myohand_r", "myohands", "myoarms"])
def test_disable_finger_collision_unchanged(name: str) -> None:
    """The baoding helper still disables exactly its fixed geom list and the pairs that reference it."""
    reference = myo_sim.load_model(name)
    model = disable_finger_collision(myo_sim.load_spec(name)).compile()
    listed = {base + side for base in COARSE_COLLISION_DISABLED_GEOM_BASENAMES for side in ("_r", "_l")}
    selected = np.array([name in listed for name in _geom_names(model)])
    assert selected.any()
    assert not _collidable(model)[selected].any()
    assert np.array_equal(_collidable(model)[~selected], _collidable(reference)[~selected])
    assert _pair_names(model) == {pair for pair in _pair_names(reference) if not pair & listed}
