"""Helpers for deriving hand-only specs from full arm specs."""

from __future__ import annotations


RIGHT_HAND_ACTUATORS = frozenset(
    {
        "ECRL",
        "ECRB",
        "ECU",
        "FCR",
        "FCU",
        "PL",
        "PT",
        "PQ",
        "FDS5",
        "FDS4",
        "FDS3",
        "FDS2",
        "FDP5",
        "FDP4",
        "FDP3",
        "FDP2",
        "EDC5",
        "EDC4",
        "EDC3",
        "EDC2",
        "EDM",
        "EIP",
        "EPL",
        "EPB",
        "FPL",
        "APL",
        "OP",
        "RI2",
        "LU_RB2",
        "UI_UB2",
        "RI3",
        "LU_RB3",
        "UI_UB3",
        "RI4",
        "LU_RB4",
        "UI_UB4",
        "RI5",
        "LU_RB5",
        "UI_UB5",
    }
)

RIGHT_HAND_REMOVED_JOINTS = frozenset(
    {
        "sternoclavicular_r2_r",
        "sternoclavicular_r3_r",
        "unrotscap_r3_r",
        "unrotscap_r2_r",
        "acromioclavicular_r2_r",
        "acromioclavicular_r3_r",
        "acromioclavicular_r1_r",
        "unrothum_r1_r",
        "unrothum_r3_r",
        "unrothum_r2_r",
        "elv_angle_r",
        "shoulder_elv_r",
        "shoulder1_r2_r",
        "shoulder_rot_r",
        "elbow_flexion_r",
    }
)


def add_side_suffix(name: str, side: str):
    suffix = f"_{side}"
    return name if name.endswith(suffix) else f"{name}{suffix}"


def side_name(name: str, side: str):
    if side == "r":
        return name
    if name.endswith("_r"):
        return f"{name[:-2]}_l"
    return name


def base_actuator_name(name: str, side: str):
    suffix = f"_{side}"
    return name[: -len(suffix)] if name.endswith(suffix) else name


def tendon_wrap_geom_names(spec):
    geom_names = set()
    for tendon in spec.tendons:
        for index in range(len(tendon.path)):
            target = tendon.path[index].target
            if target is not None and target.__class__.__name__ == "MjsGeom":
                geom_names.add(target.name)
    return geom_names


def prune_arm_spec_to_hand(spec, side: str):
    """Remove proximal arm dynamics from an arm spec, leaving wrist/hand controls."""
    removed_joints = {side_name(name, side) for name in RIGHT_HAND_REMOVED_JOINTS}

    for equality in list(spec.equalities):
        if equality.name1 in removed_joints or equality.name2 in removed_joints:
            spec.delete(equality)

    for actuator in list(spec.actuators):
        if base_actuator_name(actuator.name, side) not in RIGHT_HAND_ACTUATORS:
            spec.delete(actuator)

    tendon_name_map = {actuator.target: add_side_suffix(actuator.target, side) for actuator in spec.actuators}
    for tendon in list(spec.tendons):
        if tendon.name not in tendon_name_map:
            spec.delete(tendon)
        else:
            tendon.name = tendon_name_map[tendon.name]

    wrapped_geom_names = tendon_wrap_geom_names(spec)
    for geom in list(spec.geoms):
        if geom.group == 3 and geom.name not in wrapped_geom_names:
            spec.delete(geom)

    for actuator in spec.actuators:
        actuator.target = tendon_name_map[actuator.target]
        actuator.name = add_side_suffix(actuator.name, side)

    for joint_name in removed_joints:
        joint = spec.joint(joint_name)
        if joint is not None:
            spec.delete(joint)
    for joint in spec.joints:
        joint.name = add_side_suffix(joint.name, side)

    return spec
