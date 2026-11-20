"""Opt-in cervical joint articulation and ocular/IMU sensor rig for composed specs."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import mujoco

_HALF_SQRT2 = math.sqrt(0.5)
# Inside the ``head`` body frame (+x anterior, +y superior, +z anatomical right),
# rotating -90 deg about +y aligns MuJoCo's camera axes (+X right, +Y up, -Z forward).
_HEAD_CAMERA_QUAT: list[float] = [_HALF_SQRT2, 0.0, -_HALF_SQRT2, 0.0]


def _require_body(spec: mujoco.MjSpec, name: str) -> mujoco.MjsBody:
    body = spec.body(name)
    if body is None:
        raise KeyError(f"Body {name!r} not found in spec {spec.modelname!r}.")
    return body


def _set_poly_or_scalar(obj: object, attr: str, value: float) -> None:
    current = getattr(obj, attr)
    if isinstance(current, (int, float)):
        setattr(obj, attr, float(value))
    else:
        setattr(obj, attr, [float(value), 0.0, 0.0])


def unlock_head_joints(
    spec: mujoco.MjSpec,
    *,
    include_roll: bool = False,
    stiffness: float = 6.0,
    damping: float = 0.8,
    armature: float = 0.01,
    add_actuators: bool = False,
    kp: float = 25.0,
    kv: float = 2.0,
) -> mujoco.MjSpec:
    """Unlock cervical degrees of freedom on ``neck`` and ``head``, in place.

    Restores the ``neck_rotation`` (axial yaw on ``neck``) and ``neck_flexion``
    (sagittal pitch on ``head``) hinge joints commented out in
    ``myohead_rigid_chain.xml``, with optional coronal roll (``neck_bending``)
    and position-servo actuators for active head control.

    Args:
        spec: Uncompiled composed ``MjSpec`` containing ``neck`` and ``head``.
        include_roll: Also add ``neck_bending`` lateral roll on ``head``.
        stiffness: Passive cervical joint stiffness [N m / rad].
        damping: Passive cervical joint damping [N m s / rad].
        armature: Reflected rotor/segment armature [kg m^2].
        add_actuators: Add position-servo actuators for each unlocked neck joint.
        kp: Position actuator gain [N m / rad] when ``add_actuators=True``.
        kv: Velocity damping gain [N m s / rad] when ``add_actuators=True``.

    Returns:
        *spec*, mutated in place.
    """
    if stiffness < 0.0 or damping < 0.0 or armature < 0.0:
        raise ValueError("stiffness, damping, and armature must be non-negative")
    if add_actuators and (kp <= 0.0 or kv < 0.0):
        raise ValueError("kp must be positive and kv non-negative when add_actuators=True")

    import mujoco

    neck = _require_body(spec, "neck")
    head = _require_body(spec, "head")

    joint_defs: list[tuple[mujoco.MjsBody, str, list[float], list[float], list[float]]] = [
        (neck, "neck_rotation", [-0.02, 0.4, 0.0], [0.2, 1.0, 0.0], [-1.4, 1.4]),
        (head, "neck_flexion", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [-0.87, 1.05]),
    ]
    if include_roll:
        joint_defs.append((head, "neck_bending", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [-0.6, 0.6]))

    existing_joints = {j.name: j for j in spec.joints}
    existing_actuators = {a.name: a for a in spec.actuators}

    for body, jname, jpos, jaxis, jrange in joint_defs:
        jnt = existing_joints.get(jname)
        if jnt is None:
            jnt = body.add_joint(name=jname)
        jnt.type = mujoco.mjtJoint.mjJNT_HINGE
        jnt.pos = jpos
        jnt.axis = jaxis
        jnt.limited = mujoco.mjtLimited.mjLIMITED_TRUE
        jnt.range = jrange
        _set_poly_or_scalar(jnt, "stiffness", stiffness)
        _set_poly_or_scalar(jnt, "damping", damping)
        jnt.armature = float(armature)

        if add_actuators:
            act_name = f"{jname}_act"
            act = existing_actuators.get(act_name)
            if act is None:
                act = spec.add_actuator(name=act_name)
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.target = jname
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.gainprm[0] = float(kp)
            act.biasprm[0] = 0.0
            act.biasprm[1] = -float(kp)
            act.biasprm[2] = -float(kv)
            act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
            act.ctrlrange = jrange

    return spec


def add_ocular_rig(
    spec: mujoco.MjSpec,
    *,
    ipd: float = 0.063,
    fovy_fovea: float = 24.0,
    fovy_periphery: float = 90.0,
    fovy_binocular: float = 75.0,
    add_head_imu: bool = True,
) -> mujoco.MjSpec:
    """Attach gaze sites, binocular/fovea-periphery cameras, and head IMU to ``head``.

    Adds ``gaze_origin`` and ``gaze_forward`` sites (10 cm anterior offset along
    the optical axis), ``eye_l`` / ``eye_r`` binocular cameras separated by ``ipd``,
    dual-acuity ``head_fovea`` and ``head_periphery`` cameras at the cyclopean
    origin, and optional ``head_imu`` accelerometer/gyroscope/orientation sensors.

    Args:
        spec: Uncompiled composed ``MjSpec`` containing a ``head`` body.
        ipd: Interpupillary distance [m] (positive).
        fovy_fovea: Vertical field of view [deg] of ``head_fovea``.
        fovy_periphery: Vertical field of view [deg] of ``head_periphery``.
        fovy_binocular: Vertical field of view [deg] of ``eye_l`` and ``eye_r``.
        add_head_imu: Attach ``head_imu`` site and ``head_accel``, ``head_gyro``,
            and ``head_quat`` sensors.

    Returns:
        *spec*, mutated in place.
    """
    if ipd <= 0.0 or not math.isfinite(ipd):
        raise ValueError(f"ipd must be positive and finite, got {ipd}")
    for label, val in (
        ("fovy_fovea", fovy_fovea),
        ("fovy_periphery", fovy_periphery),
        ("fovy_binocular", fovy_binocular),
    ):
        if not (0.0 < float(val) < 180.0):
            raise ValueError(f"{label} must lie in (0, 180), got {val}")

    import mujoco

    head = _require_body(spec, "head")
    existing_sites = {s.name: s for s in spec.sites}
    existing_cams = {c.name: c for c in spec.cameras}
    existing_sensors = {s.name: s for s in spec.sensors}

    eye_x, eye_y = 0.085, 0.065
    for sname, spos in (
        ("gaze_origin", [eye_x, eye_y, 0.0]),
        ("gaze_forward", [eye_x + 0.10, eye_y, 0.0]),
    ):
        s = existing_sites.get(sname)
        if s is None:
            s = head.add_site(name=sname)
        s.pos = spos
        s.size = [0.004, 0.004, 0.004]
        s.group = 3

    half_ipd = 0.5 * float(ipd)
    cam_defs = (
        ("eye_l", [eye_x, eye_y, -half_ipd], float(fovy_binocular)),
        ("eye_r", [eye_x, eye_y, half_ipd], float(fovy_binocular)),
        ("head_fovea", [eye_x, eye_y, 0.0], float(fovy_fovea)),
        ("head_periphery", [eye_x, eye_y, 0.0], float(fovy_periphery)),
    )
    for cname, cpos, cfovy in cam_defs:
        cam = existing_cams.get(cname)
        if cam is None:
            cam = head.add_camera(name=cname)
        cam.pos = cpos
        cam.quat = list(_HEAD_CAMERA_QUAT)
        cam.fovy = cfovy

    if add_head_imu:
        imu_site = existing_sites.get("head_imu")
        if imu_site is None:
            imu_site = head.add_site(name="head_imu")
        imu_site.pos = [0.0, eye_y, 0.0]
        imu_site.quat = list(_HEAD_CAMERA_QUAT)
        imu_site.size = [0.005, 0.005, 0.005]
        imu_site.group = 3

        sensor_defs = (
            ("head_accel", mujoco.mjtSensor.mjSENS_ACCELEROMETER),
            ("head_gyro", mujoco.mjtSensor.mjSENS_GYRO),
            ("head_quat", mujoco.mjtSensor.mjSENS_FRAMEQUAT),
        )
        for sname, stype in sensor_defs:
            sen = existing_sensors.get(sname)
            if sen is None:
                sen = spec.add_sensor(name=sname)
            sen.type = stype
            sen.objtype = mujoco.mjtObj.mjOBJ_SITE
            sen.objname = "head_imu"

    return spec


__all__ = ["add_ocular_rig", "unlock_head_joints"]
