"""Derive the hand-only specs from the full arm spec by pruning (removal only, model-authoring policy 1)."""

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
        "AdPt",
        "AdPo",
        "FPBs",
        "FPBd",
        "APB",
        "DI1d",
        "DI1s",
        "LUM1",
        "PI1",
        "DI2",
        "LUM2",
        "DI3",
        "PI2",
        "LUM3",
        "DI4",
        "PI3",
        "LUM4",
        "ADM",
        "FDM",
        "ODM",
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

# Phantom/scaffold bodies that use the "tiny mass (1g) + huge [1,1,1] kg*m^2
# inertia" idiom -- a deliberate MuJoCo trick that makes a body's absurd
# rotational inertia harmless as long as the body's own joints fully
# constrain its configuration (it never actually rotates freely under real
# dynamics). Each of these bodies' only joints are in
# RIGHT_HAND_REMOVED_JOINTS, so prune_arm_spec_to_hand() leaves them
# jointless -- welded fixed to their parent. Once jointless, MuJoCo fuses a
# fixed body's inertia into its parent's composite inertia at compile time,
# turning the idiom's [1,1,1] into a real (and wildly wrong) contribution to
# the mass matrix instead of a harmless joint-local value. See
# https://github.com/MyoHub/myo_sim/issues/128.
PHANTOM_SCAFFOLD_BODIES = frozenset({"clavphant", "scapphant", "humphant", "humphant1"})

# Physically sane inertia floor substituted for PHANTOM_SCAFFOLD_BODIES once
# jointless, matching the legacy static-XML numerical-conditioning
# convention (see myo_sim.build.compose.build_spec(inertia_floor=...)).
PHANTOM_SCAFFOLD_INERTIA_FLOOR = 0.0001


def add_side_suffix(name: str, side: str) -> str:
    """*name* with the side suffix, unless it already names the side (``x_r`` or a muscle tendon ``x_r_tendon``)."""
    suffix = f"_{side}"
    return name if name.endswith(suffix) or name.endswith(f"{suffix}_tendon") else f"{name}{suffix}"


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


# --- generic reference handling ------------------------------------------------------------------------
#
# MjSpec keeps cross-references by name and does not follow a rename. String references (actuator,
# equality, sensor, contact pair and exclude targets, camera and light target bodies) are rewritten in
# place. A tendon's path cannot be re-pointed, so every tendon whose path names a renamed element is
# re-created with the same attributes and wraps; the tendons after it are re-created too, so the tendon
# order (and every tendon id) is unchanged.

# Settable MjsTendon attributes copied when a tendon is re-created.
TENDON_ATTRIBUTES = (
    "actfrclimited",
    "actfrcrange",
    "armature",
    "damping",
    "frictionloss",
    "group",
    "info",
    "limited",
    "margin",
    "material",
    "range",
    "rgba",
    "solimp_friction",
    "solimp_limit",
    "solref_friction",
    "solref_limit",
    "springlength",
    "stiffness",
    "userdata",
    "width",
)

OBJ = mujoco.mjtObj
WRAP = mujoco.mjtWrap
# Object type named by each actuator transmission type.
TRANSMISSION_OBJECT = {
    mujoco.mjtTrn.mjTRN_JOINT: OBJ.mjOBJ_JOINT,
    mujoco.mjtTrn.mjTRN_JOINTINPARENT: OBJ.mjOBJ_JOINT,
    mujoco.mjtTrn.mjTRN_SLIDERCRANK: OBJ.mjOBJ_SITE,
    mujoco.mjtTrn.mjTRN_TENDON: OBJ.mjOBJ_TENDON,
    mujoco.mjtTrn.mjTRN_SITE: OBJ.mjOBJ_SITE,
    mujoco.mjtTrn.mjTRN_BODY: OBJ.mjOBJ_BODY,
}
# Lookup of an element by name, per object type.
FIND = {
    OBJ.mjOBJ_BODY: lambda spec, name: spec.body(name),
    OBJ.mjOBJ_JOINT: lambda spec, name: spec.joint(name),
    OBJ.mjOBJ_GEOM: lambda spec, name: spec.geom(name),
    OBJ.mjOBJ_SITE: lambda spec, name: spec.site(name),
    OBJ.mjOBJ_TENDON: lambda spec, name: spec.tendon(name),
    OBJ.mjOBJ_ACTUATOR: lambda spec, name: spec.actuator(name),
    OBJ.mjOBJ_CAMERA: lambda spec, name: spec.camera(name),
}


def _object_type(objtype: object) -> object:
    """Map MjSpec's frame-specific object types onto the element type that owns the name."""
    return OBJ.mjOBJ_BODY if objtype in (OBJ.mjOBJ_XBODY, OBJ.mjOBJ_UNKNOWN) else objtype


def equality_object_type(equality: object) -> object:
    if equality.type == mujoco.mjtEq.mjEQ_JOINT:
        return OBJ.mjOBJ_JOINT
    if equality.type == mujoco.mjtEq.mjEQ_TENDON:
        return OBJ.mjOBJ_TENDON
    if equality.type == mujoco.mjtEq.mjEQ_FLEX:
        return OBJ.mjOBJ_FLEX
    return _object_type(equality.objtype)


def tendon_path(tendon: object) -> list[tuple[object, str | None, str | None, float, float | None]]:
    """(wrap type, target name, side-site name, coefficient, pulley divisor) of every wrap of *tendon*."""
    path = []
    for index in range(len(tendon.path)):
        wrap = tendon.path[index]
        path.append(
            (
                wrap.type,
                wrap.target.name if wrap.target is not None else None,
                wrap.sidesite.name if wrap.sidesite is not None else None,
                wrap.coef,
                wrap.divisor,
            )
        )
    return path


def wrap_object_type(wrap_type: object) -> object:
    if wrap_type == WRAP.mjWRAP_JOINT:
        return OBJ.mjOBJ_JOINT
    if wrap_type == WRAP.mjWRAP_SITE:
        return OBJ.mjOBJ_SITE
    return OBJ.mjOBJ_GEOM


def add_tendon(spec: object, name: str, attributes: dict, path: list) -> object:
    """Add a tendon with *attributes* and the wraps of *path* (as returned by ``tendon_path``)."""
    tendon = spec.add_tendon(name=name)
    for attribute, value in attributes.items():
        setattr(tendon, attribute, value)
    for wrap_type, target, sidesite, coef, divisor in path:
        if wrap_type == WRAP.mjWRAP_JOINT:
            tendon.wrap_joint(target, coef)
        elif wrap_type == WRAP.mjWRAP_SITE:
            tendon.wrap_site(target)
        elif wrap_type == WRAP.mjWRAP_PULLEY:
            tendon.wrap_pulley(divisor)
        else:
            tendon.wrap_geom(target, sidesite or "")
    return tendon


def rename_elements(spec: object, renames: dict[object, dict[str, str]]) -> None:
    """Rename elements of *spec* and every reference to them, in place.

    Args:
        spec: The MjSpec to edit.
        renames: Old to new name per object type (``mujoco.mjtObj``): bodies, joints, geoms, sites,
            tendons, actuators and cameras.
    """
    renames = {objtype: mapping for objtype, mapping in renames.items() if mapping}
    if not renames:
        return

    def new(objtype: object, name: str | None) -> str | None:
        return renames.get(objtype, {}).get(name, name) if name else name

    paths = [(tendon.name, tendon_path(tendon)) for tendon in spec.tendons]
    for objtype, mapping in renames.items():
        for old, renamed in mapping.items():
            element = FIND[objtype](spec, old)
            if element is None:
                raise ValueError(f"cannot rename {old!r}: no such {objtype.name}")
            element.name = renamed

    for actuator in spec.actuators:
        actuator.target = new(TRANSMISSION_OBJECT[actuator.trntype], actuator.target)
        actuator.slidersite = new(OBJ.mjOBJ_SITE, actuator.slidersite)
        actuator.refsite = new(OBJ.mjOBJ_SITE, actuator.refsite)
    for equality in spec.equalities:
        objtype = equality_object_type(equality)
        equality.name1 = new(objtype, equality.name1)
        equality.name2 = new(objtype, equality.name2)
    for sensor in spec.sensors:
        sensor.objname = new(_object_type(sensor.objtype), sensor.objname)
        sensor.refname = new(_object_type(sensor.reftype), sensor.refname)
    for pair in spec.pairs:
        pair.geomname1 = new(OBJ.mjOBJ_GEOM, pair.geomname1)
        pair.geomname2 = new(OBJ.mjOBJ_GEOM, pair.geomname2)
    for exclude in spec.excludes:
        exclude.bodyname1 = new(OBJ.mjOBJ_BODY, exclude.bodyname1)
        exclude.bodyname2 = new(OBJ.mjOBJ_BODY, exclude.bodyname2)
    for element in (*spec.cameras, *spec.lights):
        element.targetbody = new(OBJ.mjOBJ_BODY, element.targetbody)

    def renamed_path(path: list) -> list:
        return [
            (wrap_type, new(wrap_object_type(wrap_type), target), new(OBJ.mjOBJ_SITE, sidesite), coef, divisor)
            for wrap_type, target, sidesite, coef, divisor in path
        ]

    first = next((i for i, (_, path) in enumerate(paths) if renamed_path(path) != path), None)
    if first is None:
        return
    tendons = list(spec.tendons)[first:]
    unresolved = [name for name, path in paths[first:] if any(w[0] != WRAP.mjWRAP_PULLEY and w[1] is None for w in path)]
    if unresolved:
        # A wrap whose target lies outside this spec (e.g. a torso site before attachment) exposes no name.
        raise ValueError(f"cannot re-create tendons with unresolved wraps: {unresolved}")
    saved = [(tendon.name, {a: getattr(tendon, a) for a in TENDON_ATTRIBUTES}) for tendon in tendons]
    for tendon in tendons:
        spec.delete(tendon)
    for (name, attributes), (_, path) in zip(saved, paths[first:]):
        add_tendon(spec, name, attributes, renamed_path(path))


def _ancestors(body: object) -> list[object]:
    chain = [body]
    while chain[-1].name != "world":
        chain.append(chain[-1].parent)
    return chain


def joints_between(body1: object, body2: object) -> list[str]:
    """Names of the joints on the kinematic path between two bodies of one spec."""
    chain1, chain2 = _ancestors(body1), _ancestors(body2)
    names2 = [b.name for b in chain2]
    common = next(b.name for b in chain1 if b.name in names2)
    joints = []
    for chain in (chain1, chain2):
        for body in chain:
            if body.name == common:
                break
            joints.extend(joint.name for joint in body.joints)
    return joints


def tendon_joints(spec: object, tendon: object) -> set[str]:
    """Joints a tendon's length can depend on: its joint wraps, or the joints between its attachment bodies."""
    joints = set()
    bodies = []
    for wrap_type, target, sidesite, _, _ in tendon_path(tendon):
        if wrap_type == WRAP.mjWRAP_JOINT:
            joints.add(target)
        elif wrap_type == WRAP.mjWRAP_SITE:
            bodies.append(spec.site(target).parent)
        elif wrap_type != WRAP.mjWRAP_PULLEY:
            bodies.append(spec.geom(target).parent)
    for other in bodies[1:]:
        joints.update(joints_between(bodies[0], other))
    return joints


def element_references(spec: object) -> list[tuple[object, list[tuple[object, str]]]]:
    """Every actuator, equality, sensor, contact pair and exclude with the (object type, name) it references."""
    references = []
    for actuator in spec.actuators:
        refs = [(TRANSMISSION_OBJECT[actuator.trntype], actuator.target)]
        refs += [(OBJ.mjOBJ_SITE, site) for site in (actuator.slidersite, actuator.refsite) if site]
        references.append((actuator, refs))
    for equality in spec.equalities:
        objtype = equality_object_type(equality)
        references.append((equality, [(objtype, name) for name in (equality.name1, equality.name2) if name]))
    for sensor in spec.sensors:
        refs = [(_object_type(sensor.objtype), sensor.objname)] if sensor.objname else []
        refs += [(_object_type(sensor.reftype), sensor.refname)] if sensor.refname else []
        references.append((sensor, refs))
    for pair in spec.pairs:
        references.append((pair, [(OBJ.mjOBJ_GEOM, pair.geomname1), (OBJ.mjOBJ_GEOM, pair.geomname2)]))
    for exclude in spec.excludes:
        references.append((exclude, [(OBJ.mjOBJ_BODY, exclude.bodyname1), (OBJ.mjOBJ_BODY, exclude.bodyname2)]))
    return references


def delete_dangling_references(spec: object) -> list[str]:
    """Delete every actuator, equality, sensor, pair and exclude that names a missing element.

    Returns:
        The names (or geom/body pairs) of the deleted elements.
    """
    deleted = []
    for element, refs in element_references(spec):
        if any(objtype in FIND and FIND[objtype](spec, name) is None for objtype, name in refs):
            deleted.append(element.name or "/".join(name for _, name in refs))
            spec.delete(element)
    return deleted


def prune_rigid_pairs(spec: object) -> None:
    """Delete contact pairs whose geoms are missing or cannot move relative to each other."""
    for pair in list(spec.pairs):
        geom1, geom2 = spec.geom(pair.geomname1), spec.geom(pair.geomname2)
        if geom1 is None or geom2 is None or not joints_between(geom1.parent, geom2.parent):
            spec.delete(pair)


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


def fix_up_welded_phantom_inertia(spec: object, side: str) -> None:
    """Replace idiom-style inertia on phantom bodies left jointless by pruning.

    PHANTOM_SCAFFOLD_BODIES rely on their own joints to make a nonphysical
    [1,1,1] kg*m^2 inertia harmless. If pruning removed every joint on one
    of these bodies (see RIGHT_HAND_REMOVED_JOINTS), it is now welded fixed
    to its parent and that inertia would otherwise be fused directly into
    the parent's composite inertia at compile time. Replace it with
    PHANTOM_SCAFFOLD_INERTIA_FLOOR (matching the legacy numerical-
    conditioning convention) on any such now-jointless body. Bodies that
    still have a joint of their own (e.g. an unpruned full-arm spec) are
    left untouched, since the idiom is safe there.

    Args:
        spec: A composed MjSpec being pruned, mutated in place.
        side: "r" or "l" -- resolves each base name to its sided body name.
    """
    for base_name in PHANTOM_SCAFFOLD_BODIES:
        body = spec.body(add_side_suffix(base_name, side))
        if body is not None and len(body.joints) == 0:
            body.inertia = [PHANTOM_SCAFFOLD_INERTIA_FLOOR] * 3


def prune_arm_spec_to_hand(spec: object, side: str) -> None:
    """Remove the proximal arm dynamics from an arm spec, leaving the wrist and hand, in place.

    Removal only: the kept actuators are listed in ``RIGHT_HAND_ACTUATORS`` and the removed joints in
    ``RIGHT_HAND_REMOVED_JOINTS``. Everything else follows generically:

    - tendons driven by a removed actuator go; passive tendons (springs, ligaments) stay when their
      length depends on a kept joint and they name no removed joint;
    - equalities, sensors, contact pairs and excludes stay unless they name a removed element, and
      contact pairs between geoms that can no longer move relative to each other go;
    - every kept joint, actuator and actuated tendon carries the side suffix, with all references
      renamed (``rename_elements``).
    """
    removed_joints = {side_name(name, side) for name in RIGHT_HAND_REMOVED_JOINTS}

    actuated_tendons = {a.target for a in spec.actuators if a.trntype == mujoco.mjtTrn.mjTRN_TENDON}
    for actuator in list(spec.actuators):
        if base_actuator_name(actuator.name, side) not in RIGHT_HAND_ACTUATORS:
            spec.delete(actuator)
    kept_tendons = {a.target for a in spec.actuators if a.trntype == mujoco.mjtTrn.mjTRN_TENDON}
    for tendon in list(spec.tendons):
        if tendon.name in kept_tendons:
            continue
        if tendon.name in actuated_tendons:
            spec.delete(tendon)
            continue
        fixed_on_removed = any(w[0] == WRAP.mjWRAP_JOINT and w[1] in removed_joints for w in tendon_path(tendon))
        if fixed_on_removed or not tendon_joints(spec, tendon) - removed_joints:
            spec.delete(tendon)

    wrapped_geom_names = tendon_wrap_geom_names(spec)
    for geom in list(spec.geoms):
        if geom.group == 3 and geom.name not in wrapped_geom_names:
            spec.delete(geom)

    bake_hand_preview_pose(spec, side)

    for joint_name in removed_joints:
        joint = spec.joint(joint_name)
        if joint is not None:
            spec.delete(joint)
    delete_dangling_references(spec)
    prune_rigid_pairs(spec)

    fix_up_welded_phantom_inertia(spec, side)

    rename_elements(
        spec,
        {
            OBJ.mjOBJ_JOINT: {j.name: add_side_suffix(j.name, side) for j in spec.joints},
            OBJ.mjOBJ_TENDON: {t: add_side_suffix(t, side) for t in kept_tendons},
            OBJ.mjOBJ_ACTUATOR: {a.name: add_side_suffix(a.name, side) for a in spec.actuators},
        },
    )
