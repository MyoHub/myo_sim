"""MjSpec.attach()-based composition of the myo_sim musculoskeletal models.

This is the production build pipeline: ``build_spec(name)`` is the public entry
point and every composed model in ``MODEL_REGISTRY`` is compiled through it.

Run from the repository root:

    uv run python -m myo_sim.build.compose
    uv run python -m myo_sim.build.compose --model myotorso_arm_r
    uv run python -m myo_sim.build.compose --view

The model registry below controls how each composed model is built. The default
`myotorso_arms` model loads the right arm and mirrors it in memory to create the
left arm before attaching both specs to the torso.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import mujoco

from myo_sim import MODELS_DIR

try:
    from .hand import prune_arm_spec_to_hand
    from .utils import (
        MirrorRules,
        add_contact_pairs,
        add_sensors,
        build_child_xml_from_components,
        build_mirrored_child_xml,
        find_body,
        find_site,
    )
except ImportError:
    from hand import prune_arm_spec_to_hand
    from utils import (
        MirrorRules,
        add_contact_pairs,
        add_sensors,
        build_child_xml_from_components,
        build_mirrored_child_xml,
        find_body,
        find_site,
    )


ROOT = MODELS_DIR
GENERATE_XML_TARGETS: dict[str, Path] = {
    "myoarms": Path("arm/myoarms.xml"),
    "myotorso": Path("torso/myotorso.xml"),
    "myolegs": Path("leg/myolegs.xml"),
    "myolegs26": Path("leg/myolegs26.xml"),
    "myofullbody": Path("myofullbody.xml"),
}

TORSO_ABDOMEN_TENDONS_XML = ROOT / "torso" / "assets" / "myotorso_abdomen_tendon.xml"
TORSO_ABDOMEN_MUSCLES_XML = ROOT / "torso" / "assets" / "myotorso_abdomen_muscle.xml"
TORSO_ASSETS_XML = ROOT / "torso" / "assets" / "myotorso_assets.xml"
TORSO_TENDONS_XML = ROOT / "torso" / "assets" / "myotorso_tendon.xml"
TORSO_MUSCLES_XML = ROOT / "torso" / "assets" / "myotorso_muscle.xml"
TORSO_CHAIN_XML = ROOT / "torso" / "assets" / "myotorso_chain.xml"
HEAD_SIMPLE_ASSETS_XML = ROOT / "head" / "assets" / "myohead_simple_assets.xml"
SCENE_XML = ROOT / "scene" / "myosuite_scene.xml"
MUSCLEMIMIC_SCENE_XML = ROOT / "scene" / "myosuite_scene_musclemimic.xml"
TORSO_ABDOMEN_UNLOCKED_JOINTS = {"flex_extension", "lat_bending", "axial_rotation"}
ARM_CONTACTS_XML = ROOT / "contacts" / "myoarm_contacts.xml"
HAND_CONTACTS_XML = ROOT / "contacts" / "myohand_contacts.xml"
LEG_CONTACTS_XML = ROOT / "contacts" / "myolegs_contacts.xml"
FULLBODY_CONTACTS_XML = ROOT / "contacts" / "myofullbody_contacts.xml"
FULLBODY_SENSORS_XML = ROOT / "sensors" / "myofullbody_sensors.xml"
LEGS_SENSORS_XML = ROOT / "sensors" / "myolegs_sensors.xml"
LEGS26_CONTACTS_XML = ROOT / "contacts" / "myolegs26_contacts.xml"
LEGS26_SENSORS_XML = ROOT / "sensors" / "myolegs26_sensors.xml"
RIGHT_ARM_ASSETS_XML = ROOT / "arm" / "assets" / "myoarm_r_assets.xml"
RIGHT_ARM_TENDONS_XML = ROOT / "arm" / "assets" / "myoarm_r_tendon.xml"
RIGHT_ARM_MUSCLES_XML = ROOT / "arm" / "assets" / "myoarm_r_muscle.xml"
RIGHT_ARM_CHAIN_XML = ROOT / "arm" / "assets" / "myoarm_r_chain.xml"
LEGS_ASSETS_XML = ROOT / "leg" / "assets" / "myolegs_assets.xml"
LEGS_TENDONS_XML = ROOT / "leg" / "assets" / "myolegs_tendon.xml"
LEGS_MUSCLES_XML = ROOT / "leg" / "assets" / "myolegs_muscle.xml"
LEGS_CHAIN_XML = ROOT / "leg" / "assets" / "myolegs_chain.xml"
LEGS26_ASSETS_XML = ROOT / "leg" / "assets" / "myolegs26_assets.xml"
LEGS26_TENDONS_XML = ROOT / "leg" / "assets" / "myolegs26_tendon.xml"
LEGS26_MUSCLES_XML = ROOT / "leg" / "assets" / "myolegs26_muscle.xml"
LEGS26_CHAIN_XML = ROOT / "leg" / "assets" / "myolegs26_chain.xml"

TORSO_ROOT_BODY = "Torso"
RIGHT_ARM_ATTACH_SITE = "arm_attach_r"
LEFT_ARM_ATTACH_SITE = "arm_attach_l"
TORSO_ABDOMEN_ROOT_QUAT = (0.707388, 0, 0, -0.706825)

LEFT_ARM_STRATEGY_MIRROR_RIGHT = "mirror_right_to_left"
LEFT_ARM_STRATEGY_NONE = "none"


class BuildStrategy(str, Enum):
    TORSO_BODY = "torso_body"
    TORSO_ARMS = "torso_arms"
    ARMS_BODY = "arms_body"
    RIGHT_ARM_BODY = "right_arm_body"
    RIGHT_HAND = "right_hand"
    BOTH_HANDS = "both_hands"
    FULLBODY = "fullbody"
    LEGS_BODY = "legs_body"
    LEGS26_BODY = "legs26_body"
    TORSO_ABDOMEN = "torso_abdomen"
    LEGS_ABDOMEN = "legs_abdomen"


@dataclass(frozen=True)
class ModelRegistration:
    # TODO: too specific fields. The concept of a left arm is not always relevant, and should be handled
    #       by the builder of the relevant subspec. Similarly, not all models should  have fields associated with
    #       contacts specific to the arm, or the body. Each subspec/submodel should have a relevant and specific config,
    #       with generalist fields, and perhaps some specific ones from a childclass of ModelRegistration.
    name: str
    build_strategy: BuildStrategy
    left_arm_strategy: str
    description: str
    include_left_arm_contacts: bool
    include_arm_contacts: bool = True
    include_legs: bool = False
    include_fullbody_contacts: bool = False
    add_root_freejoint: bool = False
    root_pos: tuple[float, float, float] = (0, 0, 1)
    mirror_rules: MirrorRules = MirrorRules()


MODEL_REGISTRY = {
    "myotorso": ModelRegistration(
        name="myotorso",
        build_strategy=BuildStrategy.TORSO_BODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Torso scaffold with torso muscles",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
    ),
    "myotorso_abdomen": ModelRegistration(
        name="myotorso_abdomen",
        build_strategy=BuildStrategy.TORSO_ABDOMEN,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Simple abdomen scaffold",
        include_left_arm_contacts=False,
    ),
    "myotorso_arms": ModelRegistration(
        name="myotorso_arms",
        build_strategy=BuildStrategy.TORSO_ARMS,
        left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
        description="Torso + right arm + mirrored-right left arm",
        include_left_arm_contacts=True,
    ),
    "myotorso_arm_r": ModelRegistration(
        name="myotorso_arm_r",
        build_strategy=BuildStrategy.TORSO_ARMS,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Torso + right arm only",
        include_left_arm_contacts=False,
    ),
    "myoarms": ModelRegistration(
        name="myoarms",
        build_strategy=BuildStrategy.ARMS_BODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
        description="Passive anatomical torso scaffold + mirrored arms",
        include_left_arm_contacts=True,
    ),
    "myoarm_r": ModelRegistration(
        name="myoarm_r",
        build_strategy=BuildStrategy.RIGHT_ARM_BODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + right arm",
        include_left_arm_contacts=False,
    ),
    "myohands": ModelRegistration(
        name="myohands",
        build_strategy=BuildStrategy.BOTH_HANDS,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + right hand + mirrored-left hand from pruned arms",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
    ),
    "myohand_r": ModelRegistration(
        name="myohand_r",
        build_strategy=BuildStrategy.RIGHT_HAND,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + right hand derived from pruned right arm",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
    ),
    "myolegs": ModelRegistration(
        name="myolegs",
        build_strategy=BuildStrategy.LEGS_BODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + legs",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
        include_legs=True,
    ),
    "myolegs26": ModelRegistration(
        name="myolegs26",
        build_strategy=BuildStrategy.LEGS26_BODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + 26-muscle legs",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
        include_legs=False,
    ),
    "myolegs_abdomen": ModelRegistration(
        name="myolegs_abdomen",
        build_strategy=BuildStrategy.LEGS_ABDOMEN,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Simple abdomen scaffold plus legs",
        include_left_arm_contacts=False,
    ),
    "myofullbody": ModelRegistration(
        name="myofullbody",
        build_strategy=BuildStrategy.FULLBODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
        description="Full body: torso + mirrored arms + legs",
        include_left_arm_contacts=True,
        include_legs=True,
        include_fullbody_contacts=True,
        add_root_freejoint=True,
        root_pos=(-0.025, 0.1, 1),
    ),
}

# Legacy names that resolve to a MODEL_REGISTRY entry. This is the single source
# of truth for load()-level aliases; every value must be a key of MODEL_REGISTRY.
ALIASES: dict[str, str] = {
    "hand": "myohand_r",
    "myohand": "myohand_r",
    "myoarm": "myoarm_r",
}


def pair_is_supported(pair: dict, include_left_arm_contacts: bool) -> bool:
    if include_left_arm_contacts:
        return True
    return not (pair.get("geom1", "").endswith("_l") or pair.get("geom2", "").endswith("_l"))


def load_torso_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    """Build the single-torso model from component XMLs."""
    scene_xml = MUSCLEMIMIC_SCENE_XML if registration.build_strategy == BuildStrategy.FULLBODY else SCENE_XML
    torso_xml = build_child_xml_from_components(
        model_name="myotorso_attach",
        compiler_meshdir=ROOT,
        assets_xml=TORSO_ASSETS_XML,
        tendons_xml=TORSO_TENDONS_XML,
        muscles_xml=TORSO_MUSCLES_XML,
        chain_xml=TORSO_CHAIN_XML,
        root_body_name=TORSO_ROOT_BODY,
        root_site_name="torso_root_attach",
        extra_assets_xmls=(HEAD_SIMPLE_ASSETS_XML,),
        scene_xmls=(scene_xml,),
    )
    torso = mujoco.MjSpec.from_string(torso_xml)
    torso.compiler.balanceinertia = True

    root_body = find_body(torso, TORSO_ROOT_BODY)
    root_body.name = "Full Body"
    root_body.pos = registration.root_pos
    if registration.add_root_freejoint:
        root_body.add_freejoint(name="root")

    if registration.include_arm_contacts:
        add_contact_pairs(
            torso,
            ARM_CONTACTS_XML,
            include_pair=lambda pair: pair_is_supported(pair, registration.include_left_arm_contacts),
        )
    if registration.include_legs:
        add_contact_pairs(torso, LEG_CONTACTS_XML)
    if registration.include_fullbody_contacts:
        add_contact_pairs(torso, FULLBODY_CONTACTS_XML)
    return torso


def build_torso_body_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    return load_torso_spec(registration)


def make_torso_passive(torso: mujoco.MjSpec) -> None:
    """Remove torso dynamics so the anatomical torso acts as a fixed scaffold."""
    for equality in list(torso.equalities):
        torso.delete(equality)
    for actuator in list(torso.actuators):
        torso.delete(actuator)
    for tendon in list(torso.tendons):
        torso.delete(tendon)
    for joint in list(torso.joints):
        torso.delete(joint)


def load_passive_torso_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_torso_spec(registration)
    make_torso_passive(torso)
    return torso


def load_right_arm_spec() -> mujoco.MjSpec:
    right_arm_xml = build_child_xml_from_components(
        model_name="myoarm_r_attach",
        compiler_meshdir=ROOT,
        assets_xml=RIGHT_ARM_ASSETS_XML,
        tendons_xml=RIGHT_ARM_TENDONS_XML,
        muscles_xml=RIGHT_ARM_MUSCLES_XML,
        chain_xml=RIGHT_ARM_CHAIN_XML,
        root_body_name="myoarm_r_root",
        root_site_name="arm_root_attach_r",
    )
    right_arm = mujoco.MjSpec.from_string(right_arm_xml)
    right_arm.compiler.balanceinertia = True
    return right_arm


def build_mirrored_left_arm_xml(mirror_rules: MirrorRules) -> str:
    """Build a left-arm MJCF child by mirroring the right-arm XML in memory."""
    return build_mirrored_child_xml(
        model_name="myoarm_l_mirrored_from_r",
        compiler_meshdir=ROOT,
        source_assets_xml=RIGHT_ARM_ASSETS_XML,
        source_tendons_xml=RIGHT_ARM_TENDONS_XML,
        source_muscles_xml=RIGHT_ARM_MUSCLES_XML,
        source_chain_xml=RIGHT_ARM_CHAIN_XML,
        root_body_name="myoarm_l_root",
        root_site_name="arm_root_attach_l",
        rules=mirror_rules,
    )


def load_mirrored_left_arm_spec(mirror_rules: MirrorRules) -> mujoco.MjSpec:
    left_arm = mujoco.MjSpec.from_string(build_mirrored_left_arm_xml(mirror_rules))
    left_arm.compiler.balanceinertia = True
    return left_arm


def load_legs_spec() -> mujoco.MjSpec:
    legs_xml = build_child_xml_from_components(
        model_name="myolegs_attach",
        compiler_meshdir=ROOT,
        assets_xml=LEGS_ASSETS_XML,
        tendons_xml=LEGS_TENDONS_XML,
        muscles_xml=LEGS_MUSCLES_XML,
        chain_xml=LEGS_CHAIN_XML,
        root_body_name="myolegs_root",
        root_site_name="legs_root_attach",
    )
    legs = mujoco.MjSpec.from_string(legs_xml)
    legs.compiler.balanceinertia = True
    add_sensors(legs, LEGS_SENSORS_XML)
    return legs


def load_legs26_spec() -> mujoco.MjSpec:
    legs_xml = build_child_xml_from_components(
        model_name="myolegs26_attach",
        compiler_meshdir=ROOT,
        assets_xml=LEGS26_ASSETS_XML,
        tendons_xml=LEGS26_TENDONS_XML,
        muscles_xml=LEGS26_MUSCLES_XML,
        chain_xml=LEGS26_CHAIN_XML,
        root_body_name="myolegs26_root",
        root_site_name="legs26_root_attach",
    )
    legs = mujoco.MjSpec.from_string(legs_xml)
    legs.compiler.balanceinertia = True
    add_contact_pairs(legs, LEGS26_CONTACTS_XML)
    add_sensors(legs, LEGS26_SENSORS_XML)
    return legs


def load_torso_abdomen_spec() -> mujoco.MjSpec:
    abdomen_xml = build_child_xml_from_components(
        model_name="myotorso_abdomen_attach",
        compiler_meshdir=ROOT,
        assets_xml=TORSO_ASSETS_XML,
        tendons_xml=TORSO_ABDOMEN_TENDONS_XML,
        muscles_xml=TORSO_ABDOMEN_MUSCLES_XML,
        chain_xml=TORSO_CHAIN_XML,
        root_body_name="root",
        root_site_name="torso_abdomen_root_attach",
        extra_assets_xmls=(HEAD_SIMPLE_ASSETS_XML,),
        scene_xmls=(SCENE_XML,),
    )
    abdomen = mujoco.MjSpec.from_string(abdomen_xml)
    abdomen.compiler.balanceinertia = True
    lock_torso_abdomen_joints(abdomen)
    return abdomen


def lock_torso_abdomen_joints(abdomen: mujoco.MjSpec) -> None:
    """Keep only the base L5 torso joints movable for the simple abdomen scaffold."""
    for equality in list(abdomen.equalities):
        abdomen.delete(equality)
    for joint in list(abdomen.joints):
        if joint.name not in TORSO_ABDOMEN_UNLOCKED_JOINTS:
            abdomen.delete(joint)


def build_arms_body_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_passive_torso_spec(registration)
    torso.attach(load_right_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    torso.attach(
        load_mirrored_left_arm_spec(registration.mirror_rules),
        prefix="",
        suffix="",
        site=find_site(torso, LEFT_ARM_ATTACH_SITE),
    )

    return torso


def build_right_arm_body_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_passive_torso_spec(registration)
    torso.attach(load_right_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))

    return torso


def load_right_hand_from_arm_spec() -> mujoco.MjSpec:
    hand = load_right_arm_spec()
    hand.modelname = "myohand_r_from_myoarm_r"
    hand.compiler.balanceinertia = True
    prune_arm_spec_to_hand(hand, "r")
    return hand


def load_left_hand_from_arm_spec() -> mujoco.MjSpec:
    hand = load_mirrored_left_arm_spec(MirrorRules())
    hand.modelname = "myohand_l_from_mirrored_myoarm_r"
    prune_arm_spec_to_hand(hand, "l")
    return hand


def build_right_hand_from_arm_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_passive_torso_spec(registration)
    torso.attach(load_right_hand_from_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    add_contact_pairs(
        torso,
        HAND_CONTACTS_XML,
        include_pair=lambda pair: pair_is_supported(pair, include_left_arm_contacts=False),
    )
    return torso


def build_both_hands_from_arm_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_passive_torso_spec(registration)
    torso.attach(load_right_hand_from_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    torso.attach(load_left_hand_from_arm_spec(), prefix="", suffix="", site=find_site(torso, LEFT_ARM_ATTACH_SITE))
    add_contact_pairs(torso, HAND_CONTACTS_XML)
    return torso


def load_left_arm_spec(registration: ModelRegistration) -> mujoco.MjSpec | None:
    if registration.left_arm_strategy == LEFT_ARM_STRATEGY_MIRROR_RIGHT:
        return load_mirrored_left_arm_spec(registration.mirror_rules)
    if registration.left_arm_strategy == LEFT_ARM_STRATEGY_NONE:
        return None
    raise ValueError(f"Unknown left arm strategy for {registration.name}: {registration.left_arm_strategy}")


def build_torso_arms_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_torso_spec(registration)
    right_arm = load_right_arm_spec()
    left_arm = load_left_arm_spec(registration)

    torso.attach(right_arm, prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    if left_arm is not None:
        torso.attach(left_arm, prefix="", suffix="", site=find_site(torso, LEFT_ARM_ATTACH_SITE))

    return torso


def build_fullbody_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_torso_spec(registration)
    torso.attach(load_right_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    left_arm = load_left_arm_spec(registration)
    if left_arm is not None:
        torso.attach(left_arm, prefix="", suffix="", site=find_site(torso, LEFT_ARM_ATTACH_SITE))

    add_sensors(torso, FULLBODY_SENSORS_XML)

    full_body = find_body(torso, "Full Body")
    legs_frame = full_body.add_frame(name="legs_attach")
    torso.attach(load_legs_spec(), prefix="", suffix="", frame=legs_frame)

    return torso


def build_legs_body_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_passive_torso_spec(registration)
    root_body = find_body(torso, "Full Body")
    root_body.add_freejoint(name="root")
    legs_frame = root_body.add_frame(name="legs_attach")
    torso.attach(load_legs_spec(), prefix="", suffix="", frame=legs_frame)

    return torso


def build_legs26_body_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_passive_torso_spec(registration)
    root_body = find_body(torso, "Full Body")
    root_body.add_freejoint(name="root")
    legs_frame = root_body.add_frame(name="legs_attach")
    torso.attach(load_legs26_spec(), prefix="", suffix="", frame=legs_frame)
    return torso


def build_torso_abdomen_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    abdomen = load_torso_abdomen_spec()
    root_body = find_body(abdomen, "root")
    root_body.pos = registration.root_pos
    root_body.quat = TORSO_ABDOMEN_ROOT_QUAT
    return abdomen


def build_legs_abdomen_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    abdomen = load_torso_abdomen_spec()

    root_body = find_body(abdomen, "root")
    root_body.pos = registration.root_pos
    root_body.quat = TORSO_ABDOMEN_ROOT_QUAT
    root_body.add_freejoint(name="root")
    legs_frame = root_body.add_frame(name="legs_attach")
    abdomen.attach(load_legs_spec(), prefix="", suffix="", frame=legs_frame)

    return abdomen


SPEC_BUILDERS = {
    BuildStrategy.TORSO_BODY: build_torso_body_spec,
    BuildStrategy.TORSO_ARMS: build_torso_arms_spec,
    BuildStrategy.ARMS_BODY: build_arms_body_spec,
    BuildStrategy.RIGHT_ARM_BODY: build_right_arm_body_spec,
    BuildStrategy.RIGHT_HAND: build_right_hand_from_arm_spec,
    BuildStrategy.BOTH_HANDS: build_both_hands_from_arm_spec,
    BuildStrategy.FULLBODY: build_fullbody_spec,
    BuildStrategy.LEGS_BODY: build_legs_body_spec,
    BuildStrategy.LEGS26_BODY: build_legs26_body_spec,
    BuildStrategy.TORSO_ABDOMEN: build_torso_abdomen_spec,
    BuildStrategy.LEGS_ABDOMEN: build_legs_abdomen_spec,
}


def build_spec(model_name: str) -> mujoco.MjSpec:
    try:
        registration = MODEL_REGISTRY[model_name]
    except KeyError as exc:
        available = ", ".join(sorted(MODEL_REGISTRY))
        raise ValueError(f"Unknown model selection: {model_name}. Available: {available}") from exc
    return SPEC_BUILDERS[registration.build_strategy](registration)


def unwrap_nested_classless_defaults(element: ET.Element) -> None:
    for child in list(element):
        unwrap_nested_classless_defaults(child)
        if element.tag == "default" and child.tag == "default" and child.get("class") is None:
            index = list(element).index(child)
            element.remove(child)
            for grandchild in list(child):
                element.insert(index, grandchild)
                index += 1


def dedupe_default_element_children(element: ET.Element) -> None:
    for child in list(element):
        dedupe_default_element_children(child)

    if element.tag != "default":
        return

    seen: dict[str, ET.Element] = {}
    for child in list(element):
        if child.tag == "default":
            continue
        existing = seen.get(child.tag)
        if existing is None:
            seen[child.tag] = child
            continue
        existing.attrib.update(child.attrib)
        element.remove(child)


def drop_duplicate_default_classes(element: ET.Element, seen: set[str] | None = None) -> None:
    if seen is None:
        seen = set()

    for child in list(element):
        if child.tag == "default" and child.get("class") is not None:
            class_name = child.get("class", "")
            if class_name in seen:
                element.remove(child)
                continue
            seen.add(class_name)
        drop_duplicate_default_classes(child, seen)


def enable_floor_collision(root: ET.Element) -> None:
    for geom in root.iter("geom"):
        if geom.get("name") == "floor":
            geom.set("contype", "1")
            geom.set("conaffinity", "1")


def iter_worldbody_geoms(root: ET.Element) -> list[ET.Element]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        return []
    return list(worldbody.iter("geom"))


def apply_compiled_geom_collision_flags(root: ET.Element, model: mujoco.MjModel) -> None:
    """Preserve collision flags that can be lost through exported default inheritance."""
    geoms = iter_worldbody_geoms(root)
    if len(geoms) != model.ngeom:
        raise ValueError(f"Generated XML has {len(geoms)} geoms, but compiled model has {model.ngeom}")

    for geom_id, geom in enumerate(geoms):
        geom.set("contype", str(int(model.geom_contype[geom_id])))
        geom.set("conaffinity", str(int(model.geom_conaffinity[geom_id])))


def sanitize_spec_xml(xml: str, asset_dir: str | None = None, model: mujoco.MjModel | None = None) -> str:
    root = ET.fromstring(xml)
    if asset_dir is not None:
        compiler = root.find("compiler")
        if compiler is None:
            compiler = ET.Element("compiler")
            root.insert(0, compiler)
        compiler.set("meshdir", asset_dir)
        compiler.set("texturedir", asset_dir)
    unwrap_nested_classless_defaults(root)
    dedupe_default_element_children(root)
    drop_duplicate_default_classes(root)
    enable_floor_collision(root)
    if model is not None:
        apply_compiled_geom_collision_flags(root, model)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def write_spec_xml(spec: mujoco.MjSpec, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    model = spec.compile()
    asset_dir = os.path.relpath(ROOT, output_path.parent)
    output_path.write_text(sanitize_spec_xml(spec.to_xml(), asset_dir=asset_dir, model=model))


def generate_xml_files(output_root: Path = ROOT) -> list[Path]:
    """Generate compiled base XML files for the primary composed models."""
    output_paths: list[Path] = []
    for model_name, rel_path in GENERATE_XML_TARGETS.items():
        output_path = output_root / rel_path
        spec = build_spec(model_name)
        write_spec_xml(spec, output_path)
        output_paths.append(output_path)
    return output_paths


def view_model(model: mujoco.MjModel) -> None:
    import mujoco.viewer

    data = mujoco.MjData(model)
    try:
        viewer_context = mujoco.viewer.launch_passive(model, data)
    except RuntimeError as exc:
        if sys.platform == "darwin" and "mjpython" in str(exc):
            script = Path(__file__).name
            raise SystemExit(
                f"MuJoCo passive viewer requires mjpython on macOS.\nRun: mjpython -m myo_sim.build.{Path(script).stem} --view"
            ) from exc
        raise

    with viewer_context as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        choices=tuple(sorted(MODEL_REGISTRY)),
        default="myotorso_arms",
        help="Which registered MjSpec-composed model to compile",
    )
    parser.add_argument(
        "--generate",
        action="store_true",
        help="Generate compiled XML files for myoarms, myotorso, myolegs, and myofullbody",
    )
    parser.add_argument("--view", action="store_true", help="Open MuJoCo viewer")
    args = parser.parse_args()

    if args.generate:
        for output_path in generate_xml_files():
            print(f"generated: {output_path}")
        return

    model = build_spec(args.model).compile()
    registration = MODEL_REGISTRY[args.model]
    print(f"compiled {args.model}: nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}")
    print(f"description: {registration.description}")

    if args.view:
        view_model(model)


if __name__ == "__main__":
    main()
