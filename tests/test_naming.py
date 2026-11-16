"""Side naming: every right-arm name carries ``_r``; the left arm mirrors it; arm and hand models share names.

Convention (``docs/wiki/model-authoring.md``): joints, actuators, geoms, sites, bodies and cameras end in
``_r`` in the right-arm source, muscle tendons are named after their actuator (``OP_r`` -> ``OP_r_tendon``),
and the left arm is mirrored from them (``_l``, ``_l_tendon``).
"""

from __future__ import annotations

import mujoco
import pytest

import myo_sim
from myo_sim.build.compose import build_mirrored_left_arm_spec, build_right_arm_spec
from myo_sim.build.utils import MirrorRules, mirror_name

# Names that are not side-specific: the fragment's root and world, and shared materials and textures.
UNSIDED = {"world", "myoarm_r_root", "geom", "MatSkin", "texgeom"}
OBJECTS = {
    "joint": mujoco.mjtObj.mjOBJ_JOINT,
    "actuator": mujoco.mjtObj.mjOBJ_ACTUATOR,
    "tendon": mujoco.mjtObj.mjOBJ_TENDON,
    "geom": mujoco.mjtObj.mjOBJ_GEOM,
    "site": mujoco.mjtObj.mjOBJ_SITE,
    "body": mujoco.mjtObj.mjOBJ_BODY,
    "camera": mujoco.mjtObj.mjOBJ_CAMERA,
}


def _sided(name: str, side: str) -> bool:
    return name.endswith(f"_{side}") or name.endswith(f"_{side}_tendon")


def _names(model: mujoco.MjModel, obj: mujoco.mjtObj) -> list[str]:
    count = {
        mujoco.mjtObj.mjOBJ_JOINT: model.njnt,
        mujoco.mjtObj.mjOBJ_ACTUATOR: model.nu,
        mujoco.mjtObj.mjOBJ_TENDON: model.ntendon,
        mujoco.mjtObj.mjOBJ_GEOM: model.ngeom,
        mujoco.mjtObj.mjOBJ_SITE: model.nsite,
        mujoco.mjtObj.mjOBJ_BODY: model.nbody,
        mujoco.mjtObj.mjOBJ_CAMERA: model.ncam,
    }[obj]
    return [n for n in (mujoco.mj_id2name(model, obj, i) for i in range(count)) if n]


def test_every_right_arm_name_carries_the_side() -> None:
    spec = build_right_arm_spec()
    collections = {
        "joint": spec.joints,
        "actuator": spec.actuators,
        "tendon": spec.tendons,
        "geom": spec.geoms,
        "site": spec.sites,
        "body": spec.bodies,
        "camera": spec.cameras,
        "equality": spec.equalities,
        "mesh": spec.meshes,
    }
    for kind, elements in collections.items():
        unsided = [e.name for e in elements if e.name and e.name not in UNSIDED and not _sided(e.name, "r")]
        assert not unsided, f"{kind}: {unsided}"


def test_muscle_tendons_are_named_after_their_actuator() -> None:
    model = myo_sim.load_model("myoarms")
    for a in range(model.nu):
        if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_TENDON:
            assert model.tendon(model.actuator_trnid[a, 0]).name == f"{model.actuator(a).name}_tendon"


@pytest.mark.parametrize("kind", ["joints", "actuators", "tendons", "geoms", "sites", "bodies", "cameras", "equalities"])
def test_left_arm_mirrors_every_right_name(kind: str) -> None:
    right_spec, left_spec = build_right_arm_spec(), build_mirrored_left_arm_spec()  # keep the specs alive
    right = {e.name for e in getattr(right_spec, kind) if e.name and e.name not in UNSIDED}
    left = {e.name for e in getattr(left_spec, kind) if e.name and e.name not in {"world", "myoarm_l_root"}}
    assert {mirror_name(n, MirrorRules()) for n in right} == left
    assert all(_sided(n, "l") for n in left)


@pytest.mark.parametrize("hand,arm", [("myohand_r", "myoarm_r"), ("myohands", "myoarms")])
def test_hand_and_arm_models_share_names(hand: str, arm: str) -> None:
    """The same muscle, tendon, joint and geom has the same name in the arm and the hand model."""
    hand_model, arm_model = myo_sim.load_model(hand), myo_sim.load_model(arm)
    for kind in ("joint", "actuator", "tendon", "geom", "camera"):
        missing = set(_names(hand_model, OBJECTS[kind])) - set(_names(arm_model, OBJECTS[kind]))
        assert not missing, f"{kind}: {sorted(missing)}"
