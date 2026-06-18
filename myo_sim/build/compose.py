"""MjSpec.attach() prototype for composing torso + arm models.

Run from the repository root:

    python -m myo_sim.build.compose
    python -m myo_sim.build.compose --model myotorso_arm_r
    python -m myo_sim.build.compose --view

The model registry below controls how each composed model is built. The default
`myotorso_arms` model loads the right arm and mirrors it in memory to create the
left arm before attaching both specs to the torso.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
import argparse
import sys
import time

import mujoco
from myo_sim import MODELS_DIR

try:
    from .hand import disable_fingers_on_arm_spec, prune_arm_spec_to_hand
    from .utils import (
        MirrorRules,
        add_contact_pairs,
        build_child_xml_from_components,
        build_mirrored_child_xml,
        find_body,
        find_site,
    )
except ImportError:
    from hand import disable_fingers_on_arm_spec, prune_arm_spec_to_hand
    from utils import (
        MirrorRules,
        add_contact_pairs,
        build_child_xml_from_components,
        build_mirrored_child_xml,
        find_body,
        find_site,
    )


ROOT = MODELS_DIR

TORSO_ABDOMEN_TENDONS_XML = ROOT / "torso" / "assets" / "myotorso_abdomen_tendon.xml"
TORSO_ABDOMEN_MUSCLES_XML = ROOT / "torso" / "assets" / "myotorso_abdomen_muscle.xml"
TORSO_ASSETS_XML = ROOT / "torso" / "assets" / "myotorso_assets.xml"
TORSO_TENDONS_XML = ROOT / "torso" / "assets" / "myotorso_tendon.xml"
TORSO_MUSCLES_XML = ROOT / "torso" / "assets" / "myotorso_muscle.xml"
TORSO_TORQUE_XML = ROOT / "torso" / "assets" / "myotorso_torque.xml"
TORSO_CHAIN_XML = ROOT / "torso" / "assets" / "myotorso_chain.xml"
HEAD_SIMPLE_ASSETS_XML = ROOT / "head" / "assets" / "myohead_simple_assets.xml"
SCENE_XML = ROOT / "scene" / "myosuite_scene.xml"
TORSO_ABDOMEN_UNLOCKED_JOINTS = {"flex_extension", "lat_bending", "axial_rotation"}
ARM_CONTACTS_XML = ROOT / "contacts" / "myoarm_contacts.xml"
HAND_CONTACTS_XML = ROOT / "contacts" / "myohand_contacts.xml"
LEG_CONTACTS_XML = ROOT / "contacts" / "myolegs_contacts.xml"
FULLBODY_CONTACTS_XML = ROOT / "contacts" / "myofullbody_contacts.xml"
RIGHT_ARM_ASSETS_XML = ROOT / "arm" / "assets" / "myoarm_r_assets.xml"
RIGHT_ARM_TENDONS_XML = ROOT / "arm" / "assets" / "myoarm_r_tendons.xml"
RIGHT_ARM_MUSCLES_XML = ROOT / "arm" / "assets" / "myoarm_r_muscles.xml"
RIGHT_ARM_TORQUE_XML = ROOT / "arm" / "assets" / "myoarm_r_torque.xml"
RIGHT_ARM_CHAIN_XML = ROOT / "arm" / "assets" / "myoarm_r_chain.xml"
LEGS_ASSETS_XML = ROOT / "leg" / "assets" / "myolegs_assets.xml"
LEGS_TENDONS_XML = ROOT / "leg" / "assets" / "myolegs_tendon.xml"
LEGS_MUSCLES_XML = ROOT / "leg" / "assets" / "myolegs_muscle.xml"
LEGS_TORQUE_XML = ROOT / "leg" / "assets" / "myolegs_torque.xml"
LEGS_CHAIN_XML = ROOT / "leg" / "assets" / "myolegs_chain.xml"

ACTUATION_MUSCLE = "muscle"
ACTUATION_TORQUE = "torque"
PART_TORSO = "torso"
PART_ARMS = "arms"
PART_LEGS = "legs"
DEFAULT_ACTUATION = {PART_TORSO: ACTUATION_MUSCLE, PART_ARMS: ACTUATION_MUSCLE, PART_LEGS: ACTUATION_MUSCLE}


def resolve_actuation(actuation: str | dict[str, str] | None, registration: "ModelRegistration") -> dict[str, str]:
    """Resolve a per-part actuation mode mapping for a build.

    `actuation` may be `None` (use the registration default for every part),
    a single string applied uniformly to every part, or a dict keyed by
    `PART_TORSO`/`PART_ARMS`/`PART_LEGS` overriding individual parts.
    """
    resolved = dict(registration.default_actuation)
    if actuation is None:
        return resolved
    if isinstance(actuation, str):
        return {part: actuation for part in resolved}
    resolved.update(actuation)
    return resolved

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
    TORSO_ABDOMEN = "torso_abdomen"
    LEGS_ABDOMEN = "legs_abdomen"


@dataclass(frozen=True)
class ModelRegistration:
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
    default_actuation: dict[str, str] = field(default_factory=lambda: dict(DEFAULT_ACTUATION))


MODEL_REGISTRY = {
    "myotorso": ModelRegistration(
        name="myotorso",
        build_strategy=BuildStrategy.TORSO_BODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Torso scaffold with torso muscles",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
    ),
    "myotorso_arms": ModelRegistration(
        name="myotorso_arms",
        build_strategy=BuildStrategy.TORSO_ARMS,
        left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
        description="Torso + right arm + mirrored-right left arm",
        include_left_arm_contacts=True,
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
    "myotorso_arm_r": ModelRegistration(
        name="myotorso_arm_r",
        build_strategy=BuildStrategy.TORSO_ARMS,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Torso + right arm only",
        include_left_arm_contacts=False,
    ),
    "myohand_r": ModelRegistration(
        name="myohand_r",
        build_strategy=BuildStrategy.RIGHT_HAND,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + right hand derived from pruned right arm",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
    ),
    "myohands": ModelRegistration(
        name="myohands",
        build_strategy=BuildStrategy.BOTH_HANDS,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + right hand + mirrored-left hand from pruned arms",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
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
    "myolegs": ModelRegistration(
        name="myolegs",
        build_strategy=BuildStrategy.LEGS_BODY,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Passive anatomical torso scaffold + legs",
        include_left_arm_contacts=False,
        include_arm_contacts=False,
        include_legs=True,
    ),
    "myotorso_abdomen": ModelRegistration(
        name="myotorso_abdomen",
        build_strategy=BuildStrategy.TORSO_ABDOMEN,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Simple abdomen scaffold",
        include_left_arm_contacts=False,
    ),
    "myolegs_abdomen": ModelRegistration(
        name="myolegs_abdomen",
        build_strategy=BuildStrategy.LEGS_ABDOMEN,
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Simple abdomen scaffold plus legs",
        include_left_arm_contacts=False,
    ),
}


def pair_is_supported(pair: dict, include_left_arm_contacts: bool) -> bool:
    if include_left_arm_contacts:
        return True
    return not (pair.get("geom1", "").endswith("_l") or pair.get("geom2", "").endswith("_l"))


def load_torso_spec(registration: ModelRegistration, actuation: str = ACTUATION_MUSCLE) -> mujoco.MjSpec:
    """Build the single-torso model from component XMLs."""
    if actuation == ACTUATION_TORQUE:
        tendons_xml, muscles_xml = None, TORSO_TORQUE_XML
    else:
        tendons_xml, muscles_xml = TORSO_TENDONS_XML, TORSO_MUSCLES_XML
    torso_xml = build_child_xml_from_components(
        model_name="myotorso_attach",
        compiler_meshdir=ROOT,
        assets_xml=TORSO_ASSETS_XML,
        tendons_xml=tendons_xml,
        muscles_xml=muscles_xml,
        chain_xml=TORSO_CHAIN_XML,
        root_body_name=TORSO_ROOT_BODY,
        root_site_name="torso_root_attach",
        extra_assets_xmls=(HEAD_SIMPLE_ASSETS_XML,),
        scene_xmls=(SCENE_XML,),
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


def build_torso_body_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    modes = resolve_actuation(actuation, registration)
    return load_torso_spec(registration, modes[PART_TORSO]).compile()


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
    return torso


def load_passive_torso_spec(registration: ModelRegistration) -> mujoco.MjSpec:
    torso = load_torso_spec(registration)
    make_torso_passive(torso)
    return torso


def load_right_arm_spec(actuation: str = ACTUATION_MUSCLE, disable_fingers: bool = False) -> mujoco.MjSpec:
    if actuation == ACTUATION_TORQUE:
        tendons_xml, muscles_xml = None, RIGHT_ARM_TORQUE_XML
    else:
        tendons_xml, muscles_xml = RIGHT_ARM_TENDONS_XML, RIGHT_ARM_MUSCLES_XML
    right_arm_xml = build_child_xml_from_components(
        model_name="myoarm_r_attach",
        compiler_meshdir=ROOT,
        assets_xml=RIGHT_ARM_ASSETS_XML,
        tendons_xml=tendons_xml,
        muscles_xml=muscles_xml,
        chain_xml=RIGHT_ARM_CHAIN_XML,
        root_body_name="myoarm_r_root",
        root_site_name="arm_root_attach_r",
    )
    right_arm = mujoco.MjSpec.from_string(right_arm_xml)
    right_arm.compiler.balanceinertia = True
    if disable_fingers:
        disable_fingers_on_arm_spec(right_arm, "r")
    return right_arm


def build_mirrored_left_arm_xml(mirror_rules: MirrorRules, actuation: str = ACTUATION_MUSCLE) -> str:
    """Build a left-arm MJCF child by mirroring the right-arm XML in memory."""
    if actuation == ACTUATION_TORQUE:
        source_tendons_xml, source_muscles_xml = None, RIGHT_ARM_TORQUE_XML
    else:
        source_tendons_xml, source_muscles_xml = RIGHT_ARM_TENDONS_XML, RIGHT_ARM_MUSCLES_XML
    return build_mirrored_child_xml(
        model_name="myoarm_l_mirrored_from_r",
        compiler_meshdir=ROOT,
        source_assets_xml=RIGHT_ARM_ASSETS_XML,
        source_tendons_xml=source_tendons_xml,
        source_muscles_xml=source_muscles_xml,
        source_chain_xml=RIGHT_ARM_CHAIN_XML,
        root_body_name="myoarm_l_root",
        root_site_name="arm_root_attach_l",
        rules=mirror_rules,
    )


def load_mirrored_left_arm_spec(
    mirror_rules: MirrorRules,
    actuation: str = ACTUATION_MUSCLE,
    disable_fingers: bool = False,
) -> mujoco.MjSpec:
    left_arm = mujoco.MjSpec.from_string(build_mirrored_left_arm_xml(mirror_rules, actuation))
    left_arm.compiler.balanceinertia = True
    if disable_fingers:
        disable_fingers_on_arm_spec(left_arm, "l")
    return left_arm


def load_legs_spec(actuation: str = ACTUATION_MUSCLE) -> mujoco.MjSpec:
    if actuation == ACTUATION_TORQUE:
        tendons_xml, muscles_xml = None, LEGS_TORQUE_XML
    else:
        tendons_xml, muscles_xml = LEGS_TENDONS_XML, LEGS_MUSCLES_XML
    legs_xml = build_child_xml_from_components(
        model_name="myolegs_attach",
        compiler_meshdir=ROOT,
        assets_xml=LEGS_ASSETS_XML,
        tendons_xml=tendons_xml,
        muscles_xml=muscles_xml,
        chain_xml=LEGS_CHAIN_XML,
        root_body_name="myolegs_root",
        root_site_name="legs_root_attach",
    )
    legs = mujoco.MjSpec.from_string(legs_xml)
    legs.compiler.balanceinertia = True
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


def build_arms_body_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    modes = resolve_actuation(actuation, registration)
    torso = load_passive_torso_spec(registration)
    torso.attach(
        load_right_arm_spec(modes[PART_ARMS], disable_fingers),
        prefix="",
        suffix="",
        site=find_site(torso, RIGHT_ARM_ATTACH_SITE),
    )
    torso.attach(
        load_mirrored_left_arm_spec(registration.mirror_rules, modes[PART_ARMS], disable_fingers),
        prefix="",
        suffix="",
        site=find_site(torso, LEFT_ARM_ATTACH_SITE),
    )
    add_contact_pairs(torso, ARM_CONTACTS_XML)

    return torso.compile()


def build_right_arm_body_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    modes = resolve_actuation(actuation, registration)
    torso = load_passive_torso_spec(registration)
    torso.attach(
        load_right_arm_spec(modes[PART_ARMS], disable_fingers),
        prefix="",
        suffix="",
        site=find_site(torso, RIGHT_ARM_ATTACH_SITE),
    )

    return torso.compile()


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


def build_right_hand_from_arm_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    _reject_disable_fingers_for_hand_builds(registration, disable_fingers)
    torso = load_passive_torso_spec(registration)
    torso.attach(load_right_hand_from_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    add_contact_pairs(
        torso,
        HAND_CONTACTS_XML,
        include_pair=lambda pair: pair_is_supported(pair, include_left_arm_contacts=False),
    )
    return torso.compile()


def build_both_hands_from_arm_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    _reject_disable_fingers_for_hand_builds(registration, disable_fingers)
    torso = load_passive_torso_spec(registration)
    torso.attach(load_right_hand_from_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    torso.attach(load_left_hand_from_arm_spec(), prefix="", suffix="", site=find_site(torso, LEFT_ARM_ATTACH_SITE))
    add_contact_pairs(torso, HAND_CONTACTS_XML)
    return torso.compile()


def load_left_arm_spec(
    registration: ModelRegistration,
    actuation: str = ACTUATION_MUSCLE,
    disable_fingers: bool = False,
) -> mujoco.MjSpec | None:
    if registration.left_arm_strategy == LEFT_ARM_STRATEGY_MIRROR_RIGHT:
        return load_mirrored_left_arm_spec(registration.mirror_rules, actuation, disable_fingers)
    if registration.left_arm_strategy == LEFT_ARM_STRATEGY_NONE:
        return None
    raise ValueError(f"Unknown left arm strategy for {registration.name}: {registration.left_arm_strategy}")


def _reject_disable_fingers_for_hand_builds(registration: ModelRegistration, disable_fingers: bool) -> None:
    if disable_fingers and registration.build_strategy in {BuildStrategy.RIGHT_HAND, BuildStrategy.BOTH_HANDS}:
        raise ValueError(f"disable_fingers is not supported for hand-only model {registration.name!r}")


def build_torso_arms_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    modes = resolve_actuation(actuation, registration)
    torso = load_torso_spec(registration, modes[PART_TORSO])
    right_arm = load_right_arm_spec(modes[PART_ARMS], disable_fingers)
    left_arm = load_left_arm_spec(registration, modes[PART_ARMS], disable_fingers)

    torso.attach(right_arm, prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
    if left_arm is not None:
        torso.attach(left_arm, prefix="", suffix="", site=find_site(torso, LEFT_ARM_ATTACH_SITE))

    return torso.compile()


def build_fullbody_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    modes = resolve_actuation(actuation, registration)
    torso = load_torso_spec(registration, modes[PART_TORSO])
    torso.attach(
        load_right_arm_spec(modes[PART_ARMS], disable_fingers),
        prefix="",
        suffix="",
        site=find_site(torso, RIGHT_ARM_ATTACH_SITE),
    )
    left_arm = load_left_arm_spec(registration, modes[PART_ARMS], disable_fingers)
    if left_arm is not None:
        torso.attach(left_arm, prefix="", suffix="", site=find_site(torso, LEFT_ARM_ATTACH_SITE))

    full_body = find_body(torso, "Full Body")
    legs_frame = full_body.add_frame(name="legs_attach")
    torso.attach(load_legs_spec(modes[PART_LEGS]), prefix="", suffix="", frame=legs_frame)

    return torso.compile()


def build_legs_body_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    modes = resolve_actuation(actuation, registration)
    torso = load_passive_torso_spec(registration)
    root_body = find_body(torso, "Full Body")
    root_body.add_freejoint(name="root")
    legs_frame = root_body.add_frame(name="legs_attach")
    torso.attach(load_legs_spec(modes[PART_LEGS]), prefix="", suffix="", frame=legs_frame)

    return torso.compile()


def build_torso_abdomen_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    abdomen = load_torso_abdomen_spec()
    root_body = find_body(abdomen, "root")
    root_body.pos = registration.root_pos
    root_body.quat = TORSO_ABDOMEN_ROOT_QUAT
    return abdomen.compile()


def build_legs_abdomen_model(
    registration: ModelRegistration,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    modes = resolve_actuation(actuation, registration)
    abdomen = load_torso_abdomen_spec()

    root_body = find_body(abdomen, "root")
    root_body.pos = registration.root_pos
    root_body.quat = TORSO_ABDOMEN_ROOT_QUAT
    root_body.add_freejoint(name="root")
    legs_frame = root_body.add_frame(name="legs_attach")
    abdomen.attach(load_legs_spec(modes[PART_LEGS]), prefix="", suffix="", frame=legs_frame)

    return abdomen.compile()


BUILDERS = {
    BuildStrategy.TORSO_BODY: build_torso_body_model,
    BuildStrategy.TORSO_ARMS: build_torso_arms_model,
    BuildStrategy.ARMS_BODY: build_arms_body_model,
    BuildStrategy.RIGHT_ARM_BODY: build_right_arm_body_model,
    BuildStrategy.RIGHT_HAND: build_right_hand_from_arm_model,
    BuildStrategy.BOTH_HANDS: build_both_hands_from_arm_model,
    BuildStrategy.FULLBODY: build_fullbody_model,
    BuildStrategy.LEGS_BODY: build_legs_body_model,
    BuildStrategy.TORSO_ABDOMEN: build_torso_abdomen_model,
    BuildStrategy.LEGS_ABDOMEN: build_legs_abdomen_model,
}


def build_model(
    model_name: str,
    actuation: str | dict[str, str] | None = None,
    disable_fingers: bool = False,
) -> mujoco.MjModel:
    """Build a registered model.

    `actuation` selects per-part actuation mode: `None` uses each part's
    muscle-driven default, a string (e.g. `"torque"`) applies uniformly to
    every part, and a dict keyed by `PART_TORSO`/`PART_ARMS`/`PART_LEGS`
    (e.g. `{"arms": "torque", "legs": "muscle"}`) mixes modes within one build.

    `disable_fingers` removes finger joints and their actuators from arm parts,
    leaving shoulder, elbow, forearm, and wrist controls. Unsupported for
    hand-only models (`myohand_r`, `myohands`).
    """
    try:
        registration = MODEL_REGISTRY[model_name]
    except KeyError as exc:
        available = ", ".join(sorted(MODEL_REGISTRY))
        raise ValueError(f"Unknown model selection: {model_name}. Available: {available}") from exc
    return BUILDERS[registration.build_strategy](registration, actuation, disable_fingers)


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
    parser.add_argument("--view", action="store_true", help="Open MuJoCo viewer")
    parser.add_argument(
        "--actuation",
        action="append",
        default=None,
        help=(
            "Actuation mode: pass once as 'torque' or 'muscle' to apply uniformly, "
            "or repeat as 'part=mode' (e.g. --actuation arms=torque --actuation legs=muscle) "
            "to mix modes within one build. Parts: torso, arms, legs."
        ),
    )
    parser.add_argument(
        "--disable-fingers",
        action="store_true",
        help="Remove finger joints and actuators from arm parts (shoulder through wrist remain).",
    )
    args = parser.parse_args()

    actuation: str | dict[str, str] | None = None
    if args.actuation:
        if len(args.actuation) == 1 and "=" not in args.actuation[0]:
            actuation = args.actuation[0]
        else:
            actuation = {}
            for item in args.actuation:
                part, _, mode = item.partition("=")
                if not mode:
                    raise SystemExit(f"--actuation entries must be 'part=mode' when mixing modes, got: {item!r}")
                actuation[part] = mode

    model = build_model(args.model, actuation, disable_fingers=args.disable_fingers)
    registration = MODEL_REGISTRY[args.model]
    print(f"compiled {args.model}: nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}")
    print(f"description: {registration.description}")

    if args.view:
        view_model(model)


if __name__ == "__main__":
    main()
