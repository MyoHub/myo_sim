"""Dependency-safe deletion of joints and tendons from an editable ``MjSpec``."""

from __future__ import annotations

import mujoco


def _delete_actuator(spec: mujoco.MjSpec, actuator: mujoco.MjsActuator) -> None:
    for sensor in list(spec.sensors):
        if sensor.objtype == mujoco.mjtObj.mjOBJ_ACTUATOR and sensor.objname == actuator.name:
            spec.delete(sensor)
    spec.delete(actuator)


def delete_tendon(spec: mujoco.MjSpec, tendon: mujoco.MjsTendon) -> None:
    """Delete *tendon* together with the actuators, equalities and sensors on it.

    Args:
        spec: Spec that owns *tendon*.
        tendon: Tendon to delete.
    """
    name = tendon.name
    for a in list(spec.actuators):
        if a.trntype == mujoco.mjtTrn.mjTRN_TENDON and a.target == name:
            _delete_actuator(spec, a)
    for e in list(spec.equalities):
        if e.type == mujoco.mjtEq.mjEQ_TENDON and name in (e.name1, e.name2):
            spec.delete(e)
    for s in list(spec.sensors):
        if s.objtype == mujoco.mjtObj.mjOBJ_TENDON and s.objname == name:
            spec.delete(s)
    spec.delete(tendon)


def delete_joint(spec: mujoco.MjSpec, joint: mujoco.MjsJoint) -> None:
    """Delete *joint* together with every element that references it.

    Removes the fixed tendons that use the joint (with their own dependents,
    see :func:`delete_tendon`), joint equalities, joint actuators and joint
    sensors, so the spec still compiles afterwards.

    Args:
        spec: Spec that owns *joint*.
        joint: Joint to delete.
    """
    name = joint.name
    for t in list(spec.tendons):
        if any(w.type == mujoco.mjtWrap.mjWRAP_JOINT and getattr(w.target, "name", None) == name for w in t.path):
            delete_tendon(spec, t)
    for e in list(spec.equalities):
        if e.type == mujoco.mjtEq.mjEQ_JOINT and name in (e.name1, e.name2):
            spec.delete(e)
    for a in list(spec.actuators):
        if a.trntype in (mujoco.mjtTrn.mjTRN_JOINT, mujoco.mjtTrn.mjTRN_JOINTINPARENT) and a.target == name:
            _delete_actuator(spec, a)
    for s in list(spec.sensors):
        if s.objtype == mujoco.mjtObj.mjOBJ_JOINT and s.objname == name:
            spec.delete(s)
    spec.delete(joint)


def immobilize_subtree(spec: mujoco.MjSpec, body: mujoco.MjsBody) -> list[str]:
    """Delete every joint of *body* and its descendants with :func:`delete_joint`.

    Sites, geoms and bodies are kept, so the subtree becomes rigid.

    Args:
        spec: Spec that owns *body*.
        body: Root body of the subtree.

    Returns:
        Names of the deleted joints, in traversal order (joints of a body
        before those of its child bodies).
    """
    removed: list[str] = []
    for joint in list(body.joints):
        removed.append(joint.name)
        delete_joint(spec, joint)
    for child in list(body.bodies):
        removed.extend(immobilize_subtree(spec, child))
    return removed
