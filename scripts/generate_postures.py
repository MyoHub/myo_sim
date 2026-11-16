#!/usr/bin/env python3
"""Write ``myo_sim/build/data/postures.json``; ``--report`` prints the wiki tables.

The table holds one posture, ``standing``. Start from ``qpos0`` and change only what makes it unusable as a start:

1. raise each shoulder by ``SHOULDER_ELV_RAD`` so that the humerus clears the thorax,
2. put every joint coupling (``<joint>`` equality) on its polynomial,
3. dorsiflex the ankles until heel and forefoot are level,
4. lower the root until the feet are ``FLOOR_CLEARANCE_M`` above the floor,
5. put each thumb where its passive moments and gravity balance at zero activation,
6. where the trunk is free to hinge on a light pelvis (``PELVIC_TILT_MODELS``), tilt the pelvis forward until the trunk's
   hinge is above the hip axis, with the legs and trunk kept upright, so that the trunk's weight does not turn the pelvis.

Run from the repository root: ``python scripts/generate_postures.py``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from myo_sim.build.compose import build_spec  # noqa: E402
from myo_sim.build.keyframes import POSTURES_PATH, ROOT_JOINT  # noqa: E402

MODELS = ("myofullbody", "myolegs", "myolegs_abdomen")
SHOULDER_ELV_RAD = 0.10  # 5.7 deg: humerus to thorax clearance of 6.2 mm, see the wiki page
FLOOR_CLEARANCE_M = 1e-4  # inside the 1 mm contact margin of the foot geoms, so the feet carry the weight at t = 0
HEEL_GEOM = "foot_col1_{side}"
FOREFOOT_GEOM = "bofoot_col2_{side}"
ROUND_DECIMALS = 10
SIDES = ("r", "l")
THUMB_JOINTS = ("cmc_flexion", "cmc_abduction", "mp_flexion", "ip_flexion")
PELVIC_TILT_MODELS = ("myolegs_abdomen",)
TRUNK_HINGE = "flex_extension"


def _joint_adr(model: mujoco.MjModel, name: str) -> int:
    return int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])


def _floor_distance(model: mujoco.MjModel, data: mujoco.MjData, geom_name: str) -> float:
    """Distance of a geom to the nearest collidable world geom (the floor, or the stage of the default scene)."""
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    world = [
        g for g in range(model.ngeom) if model.geom_bodyid[g] == 0 and (model.geom_contype[g] or model.geom_conaffinity[g])
    ]
    return min(mujoco.mj_geomDistance(model, data, g, geom, 3.0, np.zeros(6)) for g in world)


def foot_geoms(model: mujoco.MjModel) -> list[str]:
    """Names of the foot and toe collision geoms."""
    names = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(model.ngeom))
    return [name for name in names if "foot_col" in name]


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


def thumb_moments(model: mujoco.MjModel, data: mujoco.MjData, qpos: np.ndarray, side: str) -> np.ndarray:
    """Passive moments plus gravity on the thumb joints of *side* at *qpos*, zero activation and velocity (N m)."""
    data.qpos[:], data.qvel[:], data.act[:] = qpos, 0.0, 0.0
    mujoco.mj_forward(model, data)
    dofs = [model.joint(f"{joint}_{side}").dofadr[0] for joint in THUMB_JOINTS]
    return (data.qfrc_passive + data.qfrc_actuator - data.qfrc_bias)[dofs]


def thumbs_at_equilibrium(model: mujoco.MjModel, qpos: np.ndarray) -> np.ndarray:
    """Return *qpos* with both thumbs at the root of ``thumb_moments``, the left one with the right one's angles."""
    from scipy.optimize import least_squares

    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{THUMB_JOINTS[0]}_r") < 0:
        return qpos
    data = mujoco.MjData(model)
    adr = {side: [_joint_adr(model, f"{joint}_{side}") for joint in THUMB_JOINTS] for side in SIDES}
    ranges = np.array([model.joint(f"{joint}_r").range for joint in THUMB_JOINTS]).T

    def with_thumbs(angles: np.ndarray) -> np.ndarray:
        out = qpos.copy()
        for side in SIDES:
            out[adr[side]] = angles
        return out

    solved = least_squares(
        lambda angles: thumb_moments(model, data, with_thumbs(angles), "r"),
        qpos[adr["r"]],
        bounds=ranges,
        xtol=1e-13,
        ftol=1e-13,
        gtol=1e-13,
    )
    return with_thumbs(solved.x)


def tilt_pelvis(model: mujoco.MjModel, qpos: np.ndarray) -> np.ndarray:
    """Return *qpos* with the pelvis turned about the hip axis until the trunk hinge is above it; legs and trunk stay upright."""
    if not np.allclose(qpos[3:7], (1.0, 0.0, 0.0, 0.0)):
        raise ValueError("The pelvic tilt expects an upright root")
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    hip = data.joint("hip_flexion_r").xanchor.copy()
    behind, above = (data.joint(TRUNK_HINGE).xanchor - hip)[1:]  # the models face -y
    tilt = float(np.arctan2(behind, above))
    out = qpos.copy()
    for joint in ("hip_flexion_r", "hip_flexion_l", TRUNK_HINGE):
        out[_joint_adr(model, joint)] = tilt
    out[3:7] = np.cos(tilt / 2), np.sin(tilt / 2), 0.0, 0.0
    relative = qpos[:3] - hip
    out[1] = hip[1] + np.cos(tilt) * relative[1] - np.sin(tilt) * relative[2]
    out[2] = hip[2] + np.sin(tilt) * relative[1] + np.cos(tilt) * relative[2]
    return coupling_values(model, out)


def solve_pose(model: mujoco.MjModel, name: str) -> np.ndarray:
    """Return the standing ``qpos`` of *model* (see the module docstring)."""
    data = mujoco.MjData(model)

    def pose(ankle: float) -> np.ndarray:
        qpos = model.qpos0.copy()
        for side in SIDES:
            qpos[_joint_adr(model, f"ankle_angle_{side}")] = ankle
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"shoulder_elv_{side}") >= 0:
                qpos[_joint_adr(model, f"shoulder_elv_{side}")] = SHOULDER_ELV_RAD
        return coupling_values(model, qpos)

    def heel_minus_forefoot(ankle: float) -> float:
        data.qpos[:] = pose(ankle)
        mujoco.mj_kinematics(model, data)
        side = SIDES[0]
        return _floor_distance(model, data, HEEL_GEOM.format(side=side)) - _floor_distance(
            model, data, FOREFOOT_GEOM.format(side=side)
        )

    low, high = -0.1, 0.1
    if heel_minus_forefoot(low) * heel_minus_forefoot(high) > 0:
        raise ValueError("Heel and forefoot do not level within +-0.1 rad of ankle angle")
    for _ in range(50):
        mid = 0.5 * (low + high)
        if heel_minus_forefoot(low) * heel_minus_forefoot(mid) <= 0:
            high = mid
        else:
            low = mid
    qpos = pose(0.5 * (low + high))
    if name in PELVIC_TILT_MODELS:
        qpos = tilt_pelvis(model, qpos)
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    lowest = min(_floor_distance(model, data, name) for name in foot_geoms(model))
    qpos[_joint_adr(model, ROOT_JOINT) + 2] -= lowest - FLOOR_CLEARANCE_M
    return thumbs_at_equilibrium(model, qpos)


def model_entry(model: mujoco.MjModel, qpos: np.ndarray) -> dict:
    """Table entry of a solved pose: the root height and the joints away from ``qpos0``."""
    joints = {}
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        adr = model.jnt_qposadr[joint]
        if name != ROOT_JOINT and abs(qpos[adr] - model.qpos0[adr]) > 10.0 ** (-ROUND_DECIMALS):
            joints[name] = round(float(qpos[adr]), ROUND_DECIMALS)
    root = _joint_adr(model, ROOT_JOINT)
    entry = {"root_z": round(float(qpos[root + 2]), ROUND_DECIMALS), "joints": joints}
    if not np.allclose(qpos[root + 3 : root + 7], model.qpos0[root + 3 : root + 7]):
        entry["root_y_offset"] = round(float(qpos[root + 1] - model.qpos0[root + 1]), ROUND_DECIMALS)
        entry["root_quat"] = [round(float(v), ROUND_DECIMALS) for v in qpos[root + 3 : root + 7]]
    return entry


def build_table() -> dict:
    """The complete table as written to the data file."""
    models = {}
    for name in MODELS:
        model = build_spec(name).compile()
        models[name] = {"standing": model_entry(model, solve_pose(model, name))}
    return {
        "provenance": {
            "script": "scripts/generate_postures.py",
            "source": "qpos0 of each composed model; model parameters are not changed",
            "shoulder_elv_rad": SHOULDER_ELV_RAD,
            "floor_clearance_m": FLOOR_CLEARANCE_M,
            "thumb_equilibrium_joints": list(THUMB_JOINTS),
            "pelvic_tilt_models": list(PELVIC_TILT_MODELS),
            "method": (
                "shoulder elevation raised to clear the thorax, joint couplings evaluated from their "
                "polynomials, ankles bisected until heel and forefoot are level, root lowered to the "
                "floor clearance, thumb joints at the equilibrium of the passive moments and gravity at zero "
                "activation, computed by the script, pelvis tilted until the trunk hinge is above the hip axis "
                "where the model is listed"
            ),
            "documentation": "docs/wiki/postures.md",
        },
        "postures": {
            "standing": {
                "description": "Upright, feet flat, arms just clear of the thorax.",
                "source": "qpos0 with the couplings evaluated; shoulder elevation is a clearance value, not a measured arm posture",
            },
        },
        "models": models,
    }


def normalised_lengths(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Normalised fibre length of every muscle, in units of the optimal fibre length."""
    low, high = model.actuator_gainprm[:, 0], model.actuator_gainprm[:, 1]
    length0 = (model.actuator_lengthrange[:, 1] - model.actuator_lengthrange[:, 0]) / (high - low)
    tendon = model.actuator_lengthrange[:, 0] - low * length0
    return (data.actuator_length - tendon) / length0


def t0_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[list[tuple[str, str, float, float]], float]:
    """Contacts at the current state as (geom1, geom2, normal force in N, distance in m), and the body weight in N."""
    force = np.zeros(6)
    contacts = []
    for index in range(data.ncon):
        contact = data.contact[index]
        mujoco.mj_contactForce(model, data, index, force)
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in contact.geom]
        contacts.append((names[0], names[1], float(force[0]), float(contact.dist)))
    return contacts, float(mujoco.mj_getTotalmass(model) * -model.opt.gravity[2])


def rollout_extremes(model: mujoco.MjModel, qpos: np.ndarray, duration: float = 0.05) -> tuple[float, str, float]:
    """Roll out at zero activation: peak joint speed (rad/s or m/s), its joint, deepest contact (m)."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ROOT_JOINT)
    first = 6 if root >= 0 else 0
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


def report() -> None:
    """Print the measurements of the wiki page."""
    print(
        "| Model | Pose | Root z (m) | Coupling error (mm or mrad) | Contacts at t = 0 | Largest force (N) | Sum / weight | Deepest (mm) |"
    )
    print("|---|---|---|---|---|---|---|---|")
    stored = []
    for name in MODELS:
        model = build_spec(name).compile()
        standing = solve_pose(model, name)
        for label, qpos in (("`qpos0`", model.qpos0), ("`standing`", standing)):
            data = mujoco.MjData(model)
            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            contacts, weight = t0_contacts(model, data)
            forces = [c[2] for c in contacts] or [0.0]
            deepest = min([c[3] for c in contacts] or [0.0])
            print(
                f"| {name} | {label} | {qpos[_joint_adr(model, ROOT_JOINT) + 2]:.4f} | "
                f"{coupling_residual(model, qpos) * 1e3:.1f} | {len(contacts)} | {max(forces):.0f} | "
                f"{sum(forces) / weight:.2f} | {-deepest * 1e3:.2f} |"
            )
            stored.append((name, label, model, qpos, data))
    print()
    print("| Model | Pose | Peak speed, 10 ms | Peak speed, 50 ms | Peak speed, 50 ms, muscles off | Deepest in 50 ms (mm) |")
    print("|---|---|---|---|---|---|")
    for name, label, model, qpos, _ in stored:
        fast = rollout_extremes(model, qpos, 0.01)
        slow = rollout_extremes(model, qpos, 0.05)
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        off = rollout_extremes(model, qpos, 0.05)
        model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        print(
            f"| {name} | {label} | {fast[0]:.2f} ({fast[1]}) | {slow[0]:.2f} ({slow[1]}) | "
            f"{off[0]:.2f} ({off[1]}) | {-slow[2] * 1e3:.2f} |"
        )
    print()
    print(
        "| Model | Pose | Shortest | Longest | Outside [lmin, lmax] | Over 1.2 | Largest passive force (N), muscle | Largest change from `qpos0` |"
    )
    print("|---|---|---|---|---|---|---|---|")
    at_qpos0 = {}
    for name, label, model, _, data in stored:
        lengths = normalised_lengths(model, data)
        at_qpos0.setdefault(name, lengths)
        change = np.abs(lengths - at_qpos0[name])
        moved = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(change.argmax()))
        outside = int(((lengths < model.actuator_gainprm[:, 4]) | (lengths > model.actuator_gainprm[:, 5])).sum())
        strongest = int(np.abs(data.actuator_force).argmax())
        muscle = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, strongest)
        print(
            f"| {name} | {label} | {lengths.min():.3f} | {lengths.max():.3f} | {outside} | {int((lengths > 1.2).sum())} | "
            f"{abs(data.actuator_force[strongest]):.0f}, {muscle} | {change.max():.3f}, {moved} |"
        )
    print()
    print("| Model | Heel share of foot load at t = 0 | Forefoot share | Centre of mass ahead of the ankle (cm) |")
    print("|---|---|---|---|")
    for name, label, model, _, data in stored:
        if label != "`standing`":
            continue
        contacts, _ = t0_contacts(model, data)
        heel = sum(force for geom1, geom2, force, _ in contacts if "bofoot" not in geom1 + geom2)
        forefoot = sum(force for geom1, geom2, force, _ in contacts if "bofoot" in geom1 + geom2)
        ankle = 0.5 * (data.body("talus_r").xpos[1] + data.body("talus_l").xpos[1])
        ahead = ankle - data.subtree_com[0][1]  # the models face -y
        print(f"| {name} | {heel / (heel + forefoot):.0%} | {forefoot / (heel + forefoot):.0%} | {ahead * 100:.1f} |")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--report", action="store_true", help="Print the wiki tables instead of writing the table.")
    args = parser.parse_args()
    if args.report:
        report()
        return
    POSTURES_PATH.parent.mkdir(exist_ok=True)
    POSTURES_PATH.write_text(json.dumps(build_table(), indent=2) + "\n")
    print(f"wrote {POSTURES_PATH}")


if __name__ == "__main__":
    main()
