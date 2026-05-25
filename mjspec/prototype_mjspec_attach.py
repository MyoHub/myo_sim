"""MjSpec.attach() prototype for composing torso + arm models.

Run from the repository root:

    python mjspec/prototype_mjspec_attach.py
    python mjspec/prototype_mjspec_attach.py --model myotorso_right_arm
    python mjspec/prototype_mjspec_attach.py --view

The model registry below controls how each composed model is built. The default
`myotorso_arms` model loads the right arm and mirrors it in memory to create the
left arm before attaching both specs to the torso.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse
import sys
import time

import mujoco

try:
    from .hand import prune_arm_spec_to_hand
    from .utils import (
        MirrorRules,
        add_contact_pairs,
        attach_to_frame,
        attach_to_site,
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
        attach_to_frame,
        attach_to_site,
        build_child_xml_from_components,
        build_mirrored_child_xml,
        find_body,
        find_site,
    )


ROOT = Path(__file__).resolve().parents[1]

TORSO_XML = ROOT / "torso" / "myotorso.xml"
TORSO_BASE_XML = ROOT / "torso" / "myotorso_base.xml"
RIGHT_ARM_XML = ROOT / "arm" / "myoarm_r.xml"
ARM_CONTACTS_XML = ROOT / "arm" / "assets" / "myoarm_contacts.xml"
LEG_CONTACTS_XML = ROOT / "leg" / "assets" / "myolegs_contacts.xml"
FULLBODY_CONTACTS_XML = ROOT / "body" / "assets" / "myofullbody_contacts.xml"
RIGHT_ARM_ASSETS_XML = ROOT / "arm" / "assets" / "myoarm_r_assets.xml"
RIGHT_ARM_TENDONS_XML = ROOT / "arm" / "assets" / "myoarm_r_tendons.xml"
RIGHT_ARM_MUSCLES_XML = ROOT / "arm" / "assets" / "myoarm_r_muscles.xml"
RIGHT_ARM_CHAIN_XML = ROOT / "arm" / "assets" / "myoarm_r_chain.xml"
LEGS_ASSETS_XML = ROOT / "leg" / "assets" / "myolegs_assets.xml"
LEGS_TENDONS_XML = ROOT / "leg" / "assets" / "myolegs_tendon.xml"
LEGS_MUSCLES_XML = ROOT / "leg" / "assets" / "myolegs_muscle.xml"
LEGS_CHAIN_XML = ROOT / "leg" / "assets" / "myolegs_chain.xml"

TORSO_ROOT_BODY = "Torso"
RIGHT_ARM_ATTACH_SITE = "arm_attach_r"
LEFT_ARM_ATTACH_SITE = "arm_attach_l"

LEFT_ARM_STRATEGY_MIRROR_RIGHT = "mirror_right_to_left"
LEFT_ARM_STRATEGY_NONE = "none"

@dataclass(frozen=True)
class ModelRegistration:
    name: str
    left_arm_strategy: str
    description: str
    include_left_arm_contacts: bool
    include_legs: bool = False
    include_fullbody_contacts: bool = False
    add_root_freejoint: bool = False
    root_pos: tuple[float, float, float] = (0, 0, 1)
    mirror_rules: MirrorRules = MirrorRules()
    arms_body_only: bool = False
    right_hand_from_arm: bool = False
    both_hands_from_arm: bool = False


MODEL_REGISTRY = {
    "myotorso_arms": ModelRegistration(
        name="myotorso_arms",
        left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
        description="Torso + right arm + mirrored-right left arm",
        include_left_arm_contacts=True,
    ),
    "myoarms": ModelRegistration(
        name="myoarms",
        left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
        description="Base scene + torso scaffold + mirrored arms",
        include_left_arm_contacts=True,
        arms_body_only=True,
    ),
    "myotorso_right_arm": ModelRegistration(
        name="myotorso_right_arm",
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Torso + right arm only",
        include_left_arm_contacts=False,
    ),
    "myohand_r": ModelRegistration(
        name="myohand_r",
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Torso base + right hand derived from pruned right arm",
        include_left_arm_contacts=False,
        right_hand_from_arm=True,
    ),
    "myohands": ModelRegistration(
        name="myohands",
        left_arm_strategy=LEFT_ARM_STRATEGY_NONE,
        description="Torso base + right hand + mirrored-left hand from pruned arms",
        include_left_arm_contacts=False,
        both_hands_from_arm=True,
    ),
    "myofullbody": ModelRegistration(
        name="myofullbody",
        left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
        description="Full body: torso + mirrored arms + legs",
        include_left_arm_contacts=True,
        include_legs=True,
        include_fullbody_contacts=True,
        add_root_freejoint=True,
        root_pos=(-0.025, 0.1, 1),
    ),
}


def pair_is_supported(pair, include_left_arm_contacts: bool):
    if include_left_arm_contacts:
        return True
    return not (
        pair.get("geom1", "").endswith("_l")
        or pair.get("geom2", "").endswith("_l")
    )


def load_torso_spec(registration: ModelRegistration):
    """Load the single-torso model and make it match the bimanual root pose."""
    torso = mujoco.MjSpec.from_file(str(TORSO_XML))
    torso.compiler.balanceinertia = True

    root_body = find_body(torso, TORSO_ROOT_BODY)
    root_body.name = "Full Body"
    root_body.pos = registration.root_pos
    if registration.add_root_freejoint:
        root_body.add_freejoint(name="root")

    add_contact_pairs(
        torso,
        ARM_CONTACTS_XML,
        include_pair=lambda pair: pair_is_supported(
            pair, registration.include_left_arm_contacts
        ),
    )
    if registration.include_legs:
        add_contact_pairs(torso, LEG_CONTACTS_XML)
    if registration.include_fullbody_contacts:
        add_contact_pairs(torso, FULLBODY_CONTACTS_XML)
    return torso


def load_right_arm_spec():
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


def build_mirrored_left_arm_xml(mirror_rules: MirrorRules):
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


def load_mirrored_left_arm_spec(mirror_rules: MirrorRules):
    left_arm = mujoco.MjSpec.from_string(build_mirrored_left_arm_xml(mirror_rules))
    left_arm.compiler.balanceinertia = True
    return left_arm


def load_legs_spec():
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
    return legs


def build_arms_body_model(registration: ModelRegistration):
    torso = mujoco.MjSpec.from_file(str(TORSO_BASE_XML))
    torso.compiler.balanceinertia = True
    attach_to_site(
        torso,
        load_right_arm_spec(),
        find_site(torso, RIGHT_ARM_ATTACH_SITE),
    )
    attach_to_site(
        torso,
        load_mirrored_left_arm_spec(registration.mirror_rules),
        find_site(torso, LEFT_ARM_ATTACH_SITE),
    )
    add_contact_pairs(torso, ARM_CONTACTS_XML)

    return torso.compile()


def load_right_hand_from_arm_spec():
    hand = mujoco.MjSpec.from_file(str(RIGHT_ARM_XML))
    hand.modelname = "myohand_r_from_myoarm_r"
    hand.compiler.balanceinertia = True
    prune_arm_spec_to_hand(hand, "r")
    return hand


def load_left_hand_from_arm_spec():
    hand = load_mirrored_left_arm_spec(MirrorRules())
    hand.modelname = "myohand_l_from_mirrored_myoarm_r"
    prune_arm_spec_to_hand(hand, "l")
    return hand


def build_right_hand_from_arm_model():
    torso = mujoco.MjSpec.from_file(str(TORSO_BASE_XML))
    torso.compiler.balanceinertia = True
    attach_to_site(
        torso,
        load_right_hand_from_arm_spec(),
        find_site(torso, RIGHT_ARM_ATTACH_SITE),
    )
    return torso.compile()


def build_both_hands_from_arm_model():
    torso = mujoco.MjSpec.from_file(str(TORSO_BASE_XML))
    torso.compiler.balanceinertia = True
    attach_to_site(
        torso,
        load_right_hand_from_arm_spec(),
        find_site(torso, RIGHT_ARM_ATTACH_SITE),
    )
    attach_to_site(
        torso,
        load_left_hand_from_arm_spec(),
        find_site(torso, LEFT_ARM_ATTACH_SITE),
    )
    return torso.compile()


def load_left_arm_spec(registration: ModelRegistration):
    if registration.left_arm_strategy == LEFT_ARM_STRATEGY_MIRROR_RIGHT:
        return load_mirrored_left_arm_spec(registration.mirror_rules)
    if registration.left_arm_strategy == LEFT_ARM_STRATEGY_NONE:
        return None
    raise ValueError(
        f"Unknown left arm strategy for {registration.name}: "
        f"{registration.left_arm_strategy}"
    )


def build_registered_model(registration: ModelRegistration):
    if registration.right_hand_from_arm:
        return build_right_hand_from_arm_model()

    if registration.both_hands_from_arm:
        return build_both_hands_from_arm_model()

    if registration.arms_body_only:
        return build_arms_body_model(registration)

    torso = load_torso_spec(registration)
    right_arm = load_right_arm_spec()
    left_arm = load_left_arm_spec(registration)

    attach_to_site(torso, right_arm, find_site(torso, RIGHT_ARM_ATTACH_SITE))
    if left_arm is not None:
        attach_to_site(torso, left_arm, find_site(torso, LEFT_ARM_ATTACH_SITE))
    if registration.include_legs:
        full_body = find_body(torso, "Full Body")
        legs_frame = full_body.add_frame(name="legs_attach")
        attach_to_frame(torso, load_legs_spec(), legs_frame)

    return torso.compile()


def build_model(model_name: str):
    try:
        registration = MODEL_REGISTRY[model_name]
    except KeyError as exc:
        available = ", ".join(sorted(MODEL_REGISTRY))
        raise ValueError(
            f"Unknown model selection: {model_name}. Available: {available}"
        ) from exc
    return build_registered_model(registration)


def view_model(model):
    import mujoco.viewer

    data = mujoco.MjData(model)
    try:
        viewer_context = mujoco.viewer.launch_passive(model, data)
    except RuntimeError as exc:
        if sys.platform == "darwin" and "mjpython" in str(exc):
            script = Path(__file__).name
            raise SystemExit(
                "MuJoCo passive viewer requires mjpython on macOS.\n"
                f"Run: mjpython mjspec/{script} --view"
            ) from exc
        raise

    with viewer_context as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        choices=tuple(sorted(MODEL_REGISTRY)),
        default="myotorso_arms",
        help="Which registered MjSpec-composed model to compile",
    )
    parser.add_argument("--view", action="store_true", help="Open MuJoCo viewer")
    args = parser.parse_args()

    model = build_model(args.model)
    registration = MODEL_REGISTRY[args.model]
    print(
        f"compiled {args.model}: "
        f"nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}"
    )
    print(f"description: {registration.description}")

    if args.view:
        view_model(model)


if __name__ == "__main__":
    main()
