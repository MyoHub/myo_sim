#!/usr/bin/env python3
"""Write ``myo_sim/build/data/postures.json``; ``--report`` prints the wiki tables.

Each posture is a short declaration: target joint angles, what supports it, and why those
angles. From ``qpos0`` of each composed model, the solver then

1. sets the targets and puts every joint coupling (``<joint>`` equality) on its polynomial,
2. where the feet are the support, solves the ankles (and subtalar joints, a knee or the
   pelvis pitch if declared) until the feet are level, at equal height, and the centre of
   mass is where it is declared to be,
3. lowers the root until the feet are ``FLOOR_CLEARANCE_M`` above the floor,
4. refuses the result if ``posture_failures`` finds a physical problem.

Run from the repository root: ``python scripts/generate_postures.py``.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from myo_sim.build.compose import build_spec  # noqa: E402
from myo_sim.build.keyframes import POSTURES_PATH, ROOT_JOINT  # noqa: E402

FLOOR_CLEARANCE_M = 1e-4  # inside the 1 mm contact margin of the foot geoms, so the feet carry the weight at t = 0
ROUND_DECIMALS = 10
SIDES = ("r", "l")
MAX_COUPLING_ERROR = 1e-7
MAX_RANGE_ERROR_RAD = 1e-6
MAX_DEPTH_M = 1e-3
MAX_FOOT_TILT_M = 2e-3
MAX_FOOT_FORCE_N = 250.0
MAX_FOOT_LOAD_PER_WEIGHT = 1.1
MAX_SOLVER_ITERATIONS = 100
MAX_SOLVER_STEP_RAD = 0.1
LIMITED_JOINT_TYPES = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
COLLAPSE_DROP_M = 0.05
COLLAPSE_DRIFT_RAD = np.radians(20.0)
COLLAPSE_HORIZON_S = 2.0


@dataclass(frozen=True)
class Posture:
    """One starting posture.

    Attributes:
        name: Keyframe name.
        description: What the posture is and what it helps with.
        source: Where the angles come from, or that they are chosen and why.
        targets_deg: Joint angles in degrees. A key names a joint, or both sides of one
            (``hip_flexion`` sets ``hip_flexion_r`` and ``hip_flexion_l``). Keys a model
            has no joint for are skipped.
        feet_support: The feet stand on the floor and the ankles are solved to level them.
        root_pitch_deg: Forward pitch of the pelvis, if it is not solved from ``balance``.
        balance: Solve the pelvis pitch to put the centre of mass this fraction of the way
            from the calcaneus origins (0) to the toes origins (1) of the feet.
        level_roll: Also solve the subtalar joints to level each foot from side to side.
        solve_knee: Knee solved so that both feet reach the floor (for uneven legs).
    """

    name: str
    description: str
    source: str
    targets_deg: Mapping[str, float] = field(default_factory=dict)
    feet_support: bool = False
    root_pitch_deg: float = 0.0
    balance: float | None = None
    level_roll: bool = False
    solve_knee: str | None = None


# Lee and Jung 2010, neutral forearm: wrist in extension and ulnar deviation, finger flexion means.
# Thumb CMC and finger abduction are chosen, see the wiki page. In this model wrist flexion is
# positive, radial deviation is positive and thumb flexion is negative.
HAND_RELAXED = {
    "flexion": -15.6,
    "deviation": -6.4,
    "cmc_flexion": -30.0,
    "cmc_abduction": 5.0,
    "mp_flexion": -42.8,
    "ip_flexion": -9.4,
    "mcp2_flexion": 27.2,
    "pm2_flexion": 26.4,
    "md2_flexion": 9.6,
    "mcp3_flexion": 31.8,
    "pm3_flexion": 27.3,
    "md3_flexion": 9.9,
    "mcp4_flexion": 23.8,
    "pm4_flexion": 32.1,
    "md4_flexion": 8.2,
    "mcp5_flexion": 28.5,
    "pm5_flexion": 33.9,
    "md5_flexion": 11.6,
}
HAND_OPEN = {
    "flexion": -15.6,
    "deviation": -6.4,
    "cmc_flexion": -15.0,
    "cmc_abduction": 15.0,
    "mp_flexion": -10.0,
    "ip_flexion": -5.0,
    **{f"mcp{digit}_flexion": 10.0 for digit in range(2, 6)},
    **{f"pm{digit}_flexion": 10.0 for digit in range(2, 6)},
    **{f"md{digit}_flexion": 5.0 for digit in range(2, 6)},
}
SHOULDER_CLEAR = {"shoulder_elv": float(np.degrees(0.10))}  # the humerus clears the thorax by 6.2 mm
ARM_AT_SIDE = {**SHOULDER_CLEAR, "elbow_flexion": 30.0, **HAND_RELAXED}
ARM_FOREARM_RAISED = {**SHOULDER_CLEAR, "elbow_flexion": 80.0, **HAND_RELAXED}
ARM_REACH = {"elv_angle": 90.0, "shoulder_elv": 45.0, "elbow_flexion": 70.0, **HAND_RELAXED}
ARM_GUARD = {"elv_angle": 60.0, "shoulder_elv": 20.0, "elbow_flexion": 70.0, **HAND_RELAXED}

STANDING = Posture(
    name="standing",
    description="Upright, feet flat, arms just clear of the thorax. The reference start, as in 0.2.4.",
    source="qpos0 with the couplings evaluated; shoulder elevation is a clearance value, not a measured arm posture",
    targets_deg=SHOULDER_CLEAR,
    feet_support=True,
)
READY = Posture(
    name="ready",
    description="Slightly flexed athletic stance, feet apart. Twice the balance margin of standing, room to move in any direction.",
    source="chosen: hip, knee and trunk values with the centre of mass mid-foot; no measured general ready stance was found",
    targets_deg={
        "hip_flexion": 25.0,
        "hip_adduction": -6.0,
        "knee_angle": 30.0,
        "flex_extension": -5.0,
        **ARM_GUARD,
    },
    feet_support=True,
    balance=0.45,
    level_roll=True,
)
STAGGERED = Posture(
    name="staggered",
    description="Right foot ahead, both feet flat. Breaks the symmetry of standing for gait and direction changes.",
    source="chosen: hip angles inside the peak range of walking (Winiarski et al. 2019), feet kept flat, not a gait event",
    targets_deg={
        "hip_flexion_r": 20.0,
        "knee_angle_r": 15.0,
        "hip_flexion_l": -8.0,
        "knee_angle_l": 5.0,
        **ARM_AT_SIDE,
    },
    feet_support=True,
    balance=0.5,
    level_roll=True,
    solve_knee="knee_angle_r",
)
SEAT_OFF = Posture(
    name="seat_off",
    description="Thighs near horizontal, as at the moment of leaving a chair. A sit-to-stand start.",
    source="hip 93.3, knee 82.7 deg at seat-off (Kinoshita et al. 2015); pelvis pitch set so that the ankle comes near their 9.0 deg",
    targets_deg={"hip_flexion": 93.3, "knee_angle": 82.7, "flex_extension": -7.0, **ARM_REACH},
    feet_support=True,
    root_pitch_deg=19.4,
    level_roll=True,
)
SQUAT = Posture(
    name="squat",
    description="Same joint depth as seat_off with the centre of mass over the feet. A loaded start that stays on its feet longer.",
    source="hip and knee as seat_off (Kinoshita et al. 2015); pelvis pitch solved for balance, so the ankle is a result",
    targets_deg={"hip_flexion": 93.3, "knee_angle": 82.7, **ARM_REACH},
    feet_support=True,
    balance=0.45,
    level_roll=True,
)
ARMS_AT_SIDE = Posture(
    name="arms_at_side",
    description="Arm hanging, elbow slightly flexed, forearm neutral, relaxed hand. The default start of the arm models.",
    source="hand from Lee and Jung 2010; elbow chosen off its stop; shoulder elevation is a clearance value",
    targets_deg=ARM_AT_SIDE,
)
FOREARM_RAISED = Posture(
    name="forearm_raised",
    description="Upper arm down, forearm raised, relaxed hand. Starts near the middle of the elbow range.",
    source="elbow at the middle of the functional arc of Morrey et al. 1981, forearm neutral; hand from Lee and Jung 2010",
    targets_deg=ARM_FOREARM_RAISED,
)
REACH = Posture(
    name="reach",
    description="Upper arm forward, elbow flexed, hand in front of the chest. Start for reaching and manipulation tasks.",
    source="chosen inside the shoulder and elbow ranges of seated daily tasks (Inagaki et al. 2025); hand from Lee and Jung 2010",
    targets_deg=ARM_REACH,
)
RELAXED = Posture(
    name="relaxed",
    description="Resting hand with flexed fingers and an extended wrist. The default start of the hand models.",
    source="wrist and finger flexion from Lee and Jung 2010; thumb CMC and finger abduction chosen",
    targets_deg=HAND_RELAXED,
)
OPEN = Posture(
    name="open",
    description="Fingers off their extension stops, thumb abducted, relaxed wrist. Start for reaching to grasp.",
    source="chosen: small flexion values off the joint limits; wrist from Lee and Jung 2010",
    targets_deg=HAND_OPEN,
)

BODY_POSTURES = (STANDING, READY, STAGGERED, SEAT_OFF, SQUAT)
ARM_POSTURES = (ARMS_AT_SIDE, FOREARM_RAISED, REACH)
HAND_POSTURES = (RELAXED, OPEN)
MODELS: dict[str, tuple[Posture, ...]] = {
    "myofullbody": BODY_POSTURES,
    "myolegs": BODY_POSTURES,
    "myolegs26": BODY_POSTURES[:3],  # deeper postures leave the ankle and knee-coupling ranges of this model
    "myolegs_abdomen": BODY_POSTURES,
    "myoarm_r": ARM_POSTURES,
    "myoarms": ARM_POSTURES,
    "myotorso_arm_r": ARM_POSTURES,
    "myotorso_arms": ARM_POSTURES,
    "myohand_r": HAND_POSTURES,
    "myohands": HAND_POSTURES,
}
# In the 26-muscle legs the knee flexes towards negative angles, and without a subtalar joint a wider stance would roll the feet.
TARGET_SIGNS: dict[str, dict[str, float]] = {"myolegs26": {"knee_angle_r": -1.0, "knee_angle_l": -1.0}}
SKIPPED_TARGETS: dict[str, set[str]] = {"myolegs26": {"hip_adduction"}}


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def _joint_adr(model: mujoco.MjModel, name: str) -> int:
    return int(model.jnt_qposadr[_joint_id(model, name)])


def _has_joint(model: mujoco.MjModel, name: str) -> bool:
    return _joint_id(model, name) >= 0


def _geom_name(model: mujoco.MjModel, geom: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""


def _target_joints(model: mujoco.MjModel, key: str) -> list[str]:
    return [name for name in (key, f"{key}_r", f"{key}_l") if _has_joint(model, name)]


def resolve_targets(model: mujoco.MjModel, model_name: str, posture: Posture) -> dict[str, float]:
    """Joint names of *model* a posture's targets apply to, with their values in radians."""
    signs = TARGET_SIGNS.get(model_name, {})
    targets = {}
    for key, degrees in posture.targets_deg.items():
        if key not in SKIPPED_TARGETS.get(model_name, ()):
            for name in _target_joints(model, key):
                targets[name] = signs.get(name, 1.0) * np.radians(degrees)
    return targets


def coupling_values(model: mujoco.MjModel, qpos: np.ndarray) -> np.ndarray:
    """Return *qpos* with every active joint coupling on its polynomial."""
    qpos = qpos.copy()
    for eq in range(model.neq):
        if model.eq_type[eq] != mujoco.mjtEq.mjEQ_JOINT or not model.eq_active0[eq]:
            continue
        adr1 = model.jnt_qposadr[model.eq_obj1id[eq]]
        adr2 = model.jnt_qposadr[model.eq_obj2id[eq]]
        dq2 = qpos[adr2] - model.qpos0[adr2] if model.eq_obj2id[eq] >= 0 else 0.0
        qpos[adr1] = model.qpos0[adr1] + np.polyval(model.eq_data[eq, :5][::-1], dq2)
    return qpos


def coupling_residual(model: mujoco.MjModel, qpos: np.ndarray) -> float:
    """Largest violation of a joint coupling at *qpos* (m or rad)."""
    return float(np.abs(coupling_values(model, qpos) - qpos).max())


def _floor_distance(model: mujoco.MjModel, data: mujoco.MjData, geom: int) -> float:
    """Distance of a geom to the nearest collidable world geom (the floor, or the stage of the default scene)."""
    world = [
        g for g in range(model.ngeom) if model.geom_bodyid[g] == 0 and (model.geom_contype[g] or model.geom_conaffinity[g])
    ]
    return min(mujoco.mj_geomDistance(model, data, g, geom, 3.0, np.zeros(6)) for g in world)


def foot_geoms(model: mujoco.MjModel, side: str) -> dict[str, int]:
    """Collision geoms that level one foot: ``heel`` and ``forefoot``, and ``medial`` and ``lateral`` if it has rails."""
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"foot_col1_{side}") >= 0:
        names = {
            "heel": f"foot_col1_{side}",
            "forefoot": f"bofoot_col2_{side}",
            "medial": f"foot_col1_{side}",
            "lateral": f"foot_col3_{side}",
        }
    else:
        names = {"heel": f"calcn_{side}_geom_1", "forefoot": f"toes_{side}_geom_1"}
    return {key: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for key, name in names.items()}


def _foot_collision_geoms(model: mujoco.MjModel, side: str) -> list[int]:
    bodies = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{part}_{side}") for part in ("calcn", "toes")}
    return [
        g for g in range(model.ngeom) if model.geom_bodyid[g] in bodies and (model.geom_contype[g] or model.geom_conaffinity[g])
    ]


def _root_pitch_quat(model: mujoco.MjModel, pitch: float) -> np.ndarray:
    adr = _joint_adr(model, ROOT_JOINT)
    quat = np.zeros(4)
    half = 0.5 * pitch
    mujoco.mju_mulQuat(quat, np.array([np.cos(half), np.sin(half), 0.0, 0.0]), model.qpos0[adr + 3 : adr + 7])
    return quat


def com_along_foot(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Whole-body centre of mass along the mean foot: 0 at the calcaneus origins, 1 at the toes origins."""
    heel = np.mean([data.body(f"calcn_{side}").xpos for side in SIDES], axis=0)
    toes = np.mean([data.body(f"toes_{side}").xpos for side in SIDES], axis=0)
    direction = toes[:2] - heel[:2]
    return float(np.dot(data.subtree_com[0][:2] - heel[:2], direction) / np.dot(direction, direction))


def _gauss_newton(residual: Callable[[np.ndarray], np.ndarray], start: np.ndarray, step: float = 1e-5) -> np.ndarray:
    x = start.copy()
    for _ in range(MAX_SOLVER_ITERATIONS):
        value = residual(x)
        jacobian = np.stack(
            [(residual(x + step * unit) - residual(x - step * unit)) / (2 * step) for unit in np.eye(x.size)], axis=1
        )
        delta = np.clip(np.linalg.lstsq(jacobian, value, rcond=None)[0], -MAX_SOLVER_STEP_RAD, MAX_SOLVER_STEP_RAD)
        x = x - delta
        if np.abs(delta).max() < 1e-11:
            return x
    raise ValueError(f"No solution: residual {residual(x)}")


def solve_pose(model: mujoco.MjModel, model_name: str, posture: Posture) -> tuple[np.ndarray, float]:
    """Solve a posture on a compiled model.

    Args:
        model: Compiled model without keyframes.
        model_name: Registry name, for the sign conventions.
        posture: The declaration.

    Returns:
        ``qpos`` and the pelvis pitch in radians.
    """
    targets = resolve_targets(model, model_name, posture)
    start = model.qpos0.copy()
    for name, value in targets.items():
        start[_joint_adr(model, name)] = value
    if not posture.feet_support:
        return coupling_values(model, start), 0.0

    data = mujoco.MjData(model)
    root = _joint_adr(model, ROOT_JOINT)
    symmetric = posture.solve_knee is None
    ankles = [[f"ankle_angle_{side}" for side in SIDES]] if symmetric else [[f"ankle_angle_{side}"] for side in SIDES]
    rolls = []
    if posture.level_roll and _has_joint(model, "subtalar_angle_r"):
        rolls = [[f"subtalar_angle_{side}" for side in SIDES]] if symmetric else [[f"subtalar_angle_{side}"] for side in SIDES]
    solved_joints = ankles + rolls + ([[posture.solve_knee]] if posture.solve_knee else [])
    n_joints = len(solved_joints)
    pitch_fixed = np.radians(posture.root_pitch_deg)
    sides = SIDES[:1] if symmetric else SIDES

    def pose(x: np.ndarray) -> np.ndarray:
        qpos = start.copy()
        for names, value in zip(solved_joints, x[:n_joints]):
            for name in names:
                qpos[_joint_adr(model, name)] = value
        qpos = coupling_values(model, qpos)
        qpos[root + 3 : root + 7] = _root_pitch_quat(model, x[n_joints] if posture.balance is not None else pitch_fixed)
        return qpos

    def residual(x: np.ndarray) -> np.ndarray:
        data.qpos[:] = pose(x)
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        out = []
        heights = []
        for side in sides:
            geoms = foot_geoms(model, side)
            heel = _floor_distance(model, data, geoms["heel"])
            out.append(heel - _floor_distance(model, data, geoms["forefoot"]))
            if rolls:
                out.append(_floor_distance(model, data, geoms["medial"]) - _floor_distance(model, data, geoms["lateral"]))
            heights.append(heel)
        if posture.solve_knee:
            out.append(heights[0] - heights[1])
        if posture.balance is not None:
            out.append(com_along_foot(model, data) - posture.balance)
        return np.array(out)

    x0 = np.zeros(n_joints + (posture.balance is not None))
    for index, names in enumerate(solved_joints):
        x0[index] = start[_joint_adr(model, names[0])]
    x = _gauss_newton(residual, x0)
    qpos = pose(x)
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    lowest = min(_floor_distance(model, data, g) for side in SIDES for g in _foot_collision_geoms(model, side))
    qpos[root + 2] -= lowest - FLOOR_CLEARANCE_M
    return qpos, float(x[n_joints]) if posture.balance is not None else float(pitch_fixed)


def model_entry(model: mujoco.MjModel, qpos: np.ndarray, pitch: float) -> dict:
    """Table entry of a solved pose: the root and the joints away from ``qpos0``."""
    joints = {}
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        adr = model.jnt_qposadr[joint]
        if name != ROOT_JOINT and abs(qpos[adr] - model.qpos0[adr]) > 10.0 ** (-ROUND_DECIMALS):
            joints[name] = round(float(qpos[adr]), ROUND_DECIMALS)
    entry: dict = {}
    if _has_joint(model, ROOT_JOINT):
        entry["root_z"] = round(float(qpos[_joint_adr(model, ROOT_JOINT) + 2]), ROUND_DECIMALS)
        if pitch:
            entry["root_pitch"] = round(pitch, ROUND_DECIMALS)
    entry["joints"] = joints
    return entry


def t0_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[list[tuple[str, str, float, float]], float]:
    """Contacts at the current state as (geom1, geom2, normal force in N, distance in m), and the body weight in N."""
    force = np.zeros(6)
    contacts = []
    for index in range(data.ncon):
        contact = data.contact[index]
        mujoco.mj_contactForce(model, data, index, force)
        contacts.append(
            (_geom_name(model, contact.geom[0]), _geom_name(model, contact.geom[1]), float(force[0]), float(contact.dist))
        )
    return contacts, float(mujoco.mj_getTotalmass(model) * -model.opt.gravity[2])


def state_at(model: mujoco.MjModel, qpos: np.ndarray) -> mujoco.MjData:
    """Data with *qpos* set and forward dynamics evaluated."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    return data


def posture_failures(model: mujoco.MjModel, qpos: np.ndarray, feet_support: bool) -> list[str]:
    """Physical problems of a posture: empty if it is a clean start.

    Checks that the joint couplings hold, that joints are within range, that nothing is
    deeply penetrating and, if the feet are the support, that both are level and on the
    floor and that only they touch it.

    Args:
        model: Compiled model.
        qpos: The posture.
        feet_support: Whether the feet carry the body.

    Returns:
        One line for each problem.
    """
    failures = []
    error = coupling_residual(model, qpos)
    if error > MAX_COUPLING_ERROR:
        failures.append(f"coupling off by {error:.2e}")
    for joint in range(model.njnt):
        if model.jnt_limited[joint] and int(model.jnt_type[joint]) in LIMITED_JOINT_TYPES:
            value = qpos[model.jnt_qposadr[joint]]
            low, high = model.jnt_range[joint]
            if value < low - MAX_RANGE_ERROR_RAD or value > high + MAX_RANGE_ERROR_RAD:
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                failures.append(f"{name} = {value:.4f} outside [{low:.4f}, {high:.4f}]")
    data = state_at(model, qpos)
    contacts, weight = t0_contacts(model, data)
    for geom1, geom2, _, dist in contacts:
        if dist < -MAX_DEPTH_M:
            failures.append(f"{geom1} and {geom2} overlap by {-dist * 1e3:.1f} mm")
    if not feet_support:
        return failures
    bodies = set()
    foot_geom_ids = {side: set(_foot_collision_geoms(model, side)) for side in SIDES}
    foot_contact = {side: 0 for side in SIDES}
    for index in range(data.ncon):
        geoms = set(data.contact[index].geom.tolist())
        for side in SIDES:
            if geoms & foot_geom_ids[side]:
                foot_contact[side] += 1
                break
        else:
            bodies.add(tuple(sorted(_geom_name(model, g) for g in geoms)))
    if bodies:
        failures.append(f"non-foot contacts: {sorted(bodies)}")
    if not all(foot_contact.values()):
        failures.append(f"a foot is off the floor: {foot_contact}")
    forces = [contact[2] for contact in contacts]
    if forces and max(forces) > MAX_FOOT_FORCE_N:
        failures.append(f"foot contact force {max(forces):.0f} N")
    if sum(forces) > MAX_FOOT_LOAD_PER_WEIGHT * weight:
        failures.append(f"foot load {sum(forces) / weight:.2f} x body weight")
    for side in SIDES:
        geoms = foot_geoms(model, side)
        distance = {key: _floor_distance(model, data, g) for key, g in geoms.items()}
        tilt = abs(distance["heel"] - distance["forefoot"])
        if "medial" in distance:
            tilt = max(tilt, abs(distance["medial"] - distance["lateral"]))
        if tilt > MAX_FOOT_TILT_M:
            failures.append(f"foot {side} tilted by {tilt * 1e3:.1f} mm")
        lowest = min(_floor_distance(model, data, g) for g in foot_geom_ids[side])
        if not -MAX_DEPTH_M < lowest < MAX_DEPTH_M:
            failures.append(f"foot {side} is {lowest * 1e3:.1f} mm from the floor")
    return failures


def build_table() -> dict:
    """The complete table as written to the data file."""
    used = set()
    models = {}
    for name, postures in MODELS.items():
        model = build_spec(name).compile()
        models[name] = {}
        for posture in postures:
            qpos, pitch = solve_pose(model, name, posture)
            failures = posture_failures(model, qpos, posture.feet_support)
            if failures:
                raise ValueError(f"{name}/{posture.name}: {failures}")
            models[name][posture.name] = model_entry(model, qpos, pitch)
            used |= {key for key in posture.targets_deg if _target_joints(model, key)}
    unused = {key for postures in MODELS.values() for posture in postures for key in posture.targets_deg} - used
    if unused:
        raise ValueError(f"Targets that match no joint of any model: {sorted(unused)}")
    every_posture = {posture.name: posture for postures in MODELS.values() for posture in postures}
    return {
        "provenance": {
            "script": "scripts/generate_postures.py",
            "documentation": "docs/wiki/postures.md",
            "floor_clearance_m": FLOOR_CLEARANCE_M,
            "method": (
                "targets set from qpos0 of each composed model, joint couplings evaluated from their polynomials, "
                "ankles and declared joints solved until the feet are level and at equal height, root lowered to "
                "the floor clearance; model parameters are not changed"
            ),
        },
        "postures": {
            name: {"description": posture.description, "source": posture.source, "feet_support": posture.feet_support}
            for name, posture in every_posture.items()
        },
        "models": models,
    }


def normalised_lengths(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Normalised fibre length of every muscle, in units of the optimal fibre length."""
    low, high = model.actuator_gainprm[:, 0], model.actuator_gainprm[:, 1]
    length0 = (model.actuator_lengthrange[:, 1] - model.actuator_lengthrange[:, 0]) / (high - low)
    tendon = model.actuator_lengthrange[:, 0] - low * length0
    return (data.actuator_length - tendon) / length0


def _first_body_dof(model: mujoco.MjModel) -> int:
    return 6 if _has_joint(model, ROOT_JOINT) else 0


def rollout_extremes(model: mujoco.MjModel, qpos: np.ndarray, duration: float = 0.05) -> tuple[float, str, float]:
    """Roll out at zero activation: peak joint speed (rad/s or m/s), its joint, deepest contact (m)."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    first = _first_body_dof(model)
    peak, joint, deepest = 0.0, "", 0.0
    for _ in range(round(duration / model.opt.timestep)):
        mujoco.mj_step(model, data)
        speeds = np.abs(data.qvel[first:])
        if speeds.max() > peak:
            peak = float(speeds.max())
            joint = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, model.dof_jntid[first + speeds.argmax()])
        if data.ncon:
            deepest = min(deepest, float(data.contact.dist.min()))
    return peak, joint, deepest


def collapse_time(model: mujoco.MjModel, qpos: np.ndarray, horizon: float = COLLAPSE_HORIZON_S) -> tuple[float | None, str]:
    """Time until a posture is lost at zero activation, or None if it holds over *horizon*.

    A free-standing body is lost when the root has dropped by ``COLLAPSE_DROP_M``. A
    fixed-base model is lost when a joint other than a coupled one has drifted by
    ``COLLAPSE_DRIFT_RAD``, and that joint is returned.
    """
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    free_standing = _has_joint(model, ROOT_JOINT)
    if free_standing:
        height = _joint_adr(model, ROOT_JOINT) + 2
    else:
        coupled = {model.eq_obj1id[eq] for eq in range(model.neq) if model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT}
        free = [j for j in range(model.njnt) if j not in coupled]
        addresses = model.jnt_qposadr[free]
    for _ in range(round(horizon / model.opt.timestep)):
        mujoco.mj_step(model, data)
        if free_standing:
            if qpos[height] - data.qpos[height] > COLLAPSE_DROP_M:
                return float(data.time), ""
        else:
            drift = np.abs(data.qpos[addresses] - qpos[addresses])
            if drift.max() > COLLAPSE_DRIFT_RAD:
                return float(data.time), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, free[int(drift.argmax())])
    return None, ""


def floor_time(model: mujoco.MjModel, qpos: np.ndarray, horizon: float = COLLAPSE_HORIZON_S) -> float | None:
    """Time until something other than the feet of a free-standing body touches the floor, at zero activation."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    feet = {g for side in SIDES for g in _foot_collision_geoms(model, side)}
    for _ in range(round(horizon / model.opt.timestep)):
        mujoco.mj_step(model, data)
        for index in range(data.ncon):
            geoms = data.contact[index].geom
            if (model.geom_bodyid[geoms[0]] == 0) != (model.geom_bodyid[geoms[1]] == 0) and not set(geoms.tolist()) & feet:
                return float(data.time)
    return None


def support_margin(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Distance of the centre of mass inside the convex hull of the contact points, negative outside (m)."""
    points = np.array([data.contact[i].pos[:2] for i in range(data.ncon)])
    points = points[np.lexsort(points.T)]

    def half(pts: np.ndarray) -> list[np.ndarray]:
        hull: list[np.ndarray] = []
        for point in pts:
            while len(hull) > 1 and np.cross(hull[-1] - hull[-2], point - hull[-2]) <= 0:
                hull.pop()
            hull.append(point)
        return hull[:-1]

    hull = np.array(half(points) + half(points[::-1]))
    com = data.subtree_com[0][:2]
    edges = np.roll(hull, -1, axis=0) - hull
    normals = np.stack([-edges[:, 1], edges[:, 0]], axis=1) / np.linalg.norm(edges, axis=1, keepdims=True)
    return float(np.min(np.sum(normals * (com - hull), axis=1)))


def trunk_lean_deg(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Rotation of the torso body from its orientation at ``qpos0``, in degrees."""
    relative = state_at(model, model.qpos0).body("torso").xmat.reshape(3, 3).T @ data.body("torso").xmat.reshape(3, 3)
    return float(np.degrees(np.arccos(np.clip(0.5 * (np.trace(relative) - 1.0), -1.0, 1.0))))


def _words(names: str) -> list[str]:
    return [word.rstrip("0123456789") for word in names.split("_")]


def _speed_row(name: str, model: mujoco.MjModel, label: str, qpos: np.ndarray, feet_support: bool) -> str:
    peak = rollout_extremes(model, qpos)
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    off = rollout_extremes(model, qpos)
    model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    lost, joint = collapse_time(model, qpos)
    lost_text = f"holds {COLLAPSE_HORIZON_S:.0f} s" if lost is None else f"{lost:.2f}" + (f" ({joint})" if joint else "")
    floor, margin = "-", "-"
    if feet_support:
        down = floor_time(model, qpos)
        floor = f"over {COLLAPSE_HORIZON_S:.0f}" if down is None else f"{down:.2f}"
        data = state_at(model, qpos)
        if data.ncon >= 3:
            margin = f"{support_margin(model, data) * 100:.1f}"
    return (
        f"| {name} | {label} | {peak[0]:.2f} ({peak[1]}) | {off[0]:.2f} ({off[1]}) | {-peak[2] * 1e3:.2f} | "
        f"{lost_text} | {floor} | {margin} |"
    )


def report() -> None:
    """Print the measurements of the wiki page."""
    measured = []
    for name, postures in MODELS.items():
        model = build_spec(name).compile()
        for posture in postures:
            qpos, pitch = solve_pose(model, name, posture)
            measured.append((name, model, posture, qpos, pitch, state_at(model, qpos)))
    print(
        "| Model | Posture | Peak speed, 50 ms | Muscles off, 50 ms | Deepest in 50 ms (mm) | Lost at (s) "
        "| Other than feet on the floor at (s) | COM inside support (cm) |"
    )
    print("|---|---|---|---|---|---|---|---|")
    seen = set()
    for name, model, posture, qpos, _, _ in measured:
        if name not in seen:
            seen.add(name)
            print(_speed_row(name, model, "`qpos0`", model.qpos0, posture.feet_support))
        print(_speed_row(name, model, f"`{posture.name}`", qpos, posture.feet_support))
    print()
    print(
        "| Model | Posture | Root z (m) | Pelvis pitch (deg) | Trunk rotation (deg) | Ankle r, l (deg) | Knee r, l (deg) "
        "| Stance width (m) | Step length (m) | Contacts at t = 0 | Largest force (N) | Sum / weight | Heel share of foot load "
        "| COM ahead of the ankle (cm) |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for name, model, posture, qpos, pitch, data in measured:
        if not posture.feet_support:
            continue
        contacts, weight = t0_contacts(model, data)
        forces = [c[2] for c in contacts]
        heel = sum(force for geom1, geom2, force, _ in contacts if not {"bofoot", "toes"} & set(_words(geom1 + "_" + geom2)))
        ankles = [np.degrees(qpos[_joint_adr(model, f"ankle_angle_{side}")]) for side in SIDES]
        knees = [abs(np.degrees(qpos[_joint_adr(model, f"knee_angle_{side}")])) for side in SIDES]
        right, left = data.body("talus_r").xpos, data.body("talus_l").xpos
        ahead = 0.5 * (right[1] + left[1]) - data.subtree_com[0][1]  # the models face -y
        print(
            f"| {name} | `{posture.name}` | {qpos[_joint_adr(model, ROOT_JOINT) + 2]:.3f} | {np.degrees(pitch):.1f} | "
            f"{trunk_lean_deg(model, data):.1f} | {ankles[0]:.1f}, {ankles[1]:.1f} | {knees[0]:.1f}, {knees[1]:.1f} | "
            f"{abs(right[0] - left[0]):.3f} | {abs(right[1] - left[1]):.3f} | {len(contacts)} | {max(forces):.0f} | "
            f"{sum(forces) / weight:.2f} | {heel / sum(forces):.0%} | {ahead * 100:.1f} |"
        )
    print()
    print("| Model | Posture | Shortest | Longest | Outside [lmin, lmax] | Over 1.2 | Largest passive force (N), muscle |")
    print("|---|---|---|---|---|---|---|")
    for name, model, posture, _, _, data in measured:
        lengths = normalised_lengths(model, data)
        outside = int(((lengths < model.actuator_gainprm[:, 4]) | (lengths > model.actuator_gainprm[:, 5])).sum())
        strongest = int(np.abs(data.actuator_force).argmax())
        muscle = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, strongest)
        print(
            f"| {name} | `{posture.name}` | {lengths.min():.3f} | {lengths.max():.3f} | {outside} | {int((lengths > 1.2).sum())} | "
            f"{abs(data.actuator_force[strongest]):.0f}, {muscle} |"
        )
    print()
    print("Where the fixed-base models settle at zero activation, in degrees, after 4 s from their first posture:")
    print()
    for name in ("myoarm_r", "myohand_r"):
        model = build_spec(name).compile()
        qpos, _ = solve_pose(model, name, MODELS[name][0])
        data = mujoco.MjData(model)
        data.qpos[:] = qpos
        for _ in range(round(4.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        coupled = {model.eq_obj1id[eq] for eq in range(model.neq) if model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT}
        joints = (j for j in range(model.njnt) if j not in coupled)
        settled = ", ".join(
            f"{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)} {np.degrees(qpos[model.jnt_qposadr[j]]):.1f} to "
            f"{np.degrees(data.qpos[model.jnt_qposadr[j]]):.1f}"
            for j in joints
        )
        print(f"- {name}, peak speed at the end {np.abs(data.qvel).max():.3f} rad/s: {settled}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--report", action="store_true", help="Print the wiki tables instead of writing the table.")
    args = parser.parse_args()
    if args.report:
        report()
        return
    POSTURES_PATH.write_text(json.dumps(build_table(), indent=2) + "\n")
    print(f"wrote {POSTURES_PATH}")


if __name__ == "__main__":
    main()
