"""Helpers for deriving hand-only specs from full arm specs."""

from __future__ import annotations

import mujoco
import numpy as np

HAND_PREVIEW_JOINT_POSE = {
    "elv_angle_r": 1.57,
    "shoulder_elv_r": 1.57,
    "shoulder_rot_r": -1.57,
}


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


def add_side_suffix(name: str, side: str) -> str:
    suffix = f"_{side}"
    return name if name.endswith(suffix) else f"{name}{suffix}"


def side_name(name: str, side: str) -> str:
    if side == "r":
        return name
    if name.endswith("_r"):
        return f"{name[:-2]}_l"
    return name


def base_actuator_name(name: str, side: str) -> str:
    suffix = f"_{side}"
    return name[: -len(suffix)] if name.endswith(suffix) else name


def tendon_wrap_geom_names(spec: object) -> set[str]:
    geom_names = set()
    for tendon in spec.tendons:
        for index in range(len(tendon.path)):
            target = tendon.path[index].target
            if target is not None and target.__class__.__name__ == "MjsGeom":
                geom_names.add(target.name)
    return geom_names


def quaternion_inverse(quaternion: np.ndarray) -> np.ndarray:
    return np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    result = np.zeros(4)
    mujoco.mju_mulQuat(result, left, right)
    return result


def rotate_by_inverse_quaternion(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    matrix = np.zeros(9)
    mujoco.mju_quat2Mat(matrix, quaternion)
    return matrix.reshape(3, 3).T @ vector


def named_body_names(spec: object) -> list[str]:
    return [body.name for body in spec.bodies if body.name]


def apply_joint_pose(data: mujoco.MjData, model: mujoco.MjModel, joint_pose: dict[str, float], side: str) -> None:
    for joint_name, value in joint_pose.items():
        sided_joint_name = side_name(joint_name, side)
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, sided_joint_name)
        if joint_id < 0:
            continue
        data.qpos[model.jnt_qposadr[joint_id]] = value


def bake_current_body_poses(spec: object, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    for body_name in named_body_names(spec):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id <= 0:
            continue
        body = spec.body(body_name)
        if body is None:
            continue

        parent_id = model.body_parentid[body_id]
        body.pos = rotate_by_inverse_quaternion(data.xquat[parent_id], data.xpos[body_id] - data.xpos[parent_id])
        body.quat = quaternion_multiply(quaternion_inverse(data.xquat[parent_id]), data.xquat[body_id])


def bake_hand_preview_pose(spec: object, side: str) -> None:
    model = spec.compile()
    data = mujoco.MjData(model)
    apply_joint_pose(data, model, HAND_PREVIEW_JOINT_POSE, side)
    mujoco.mj_forward(model, data)
    bake_current_body_poses(spec, model, data)


def prune_arm_spec_to_hand(spec: object, side: str) -> None:
    """Remove proximal arm dynamics from an arm spec, leaving wrist/hand controls."""
    removed_joints = {side_name(name, side) for name in RIGHT_HAND_REMOVED_JOINTS}

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

    bake_hand_preview_pose(spec, side)

    for equality in list(spec.equalities):
        if equality.name1 in removed_joints or equality.name2 in removed_joints:
            spec.delete(equality)

    for joint_name in removed_joints:
        joint = spec.joint(joint_name)
        if joint is not None:
            spec.delete(joint)
    for joint in spec.joints:
        joint.name = add_side_suffix(joint.name, side)
