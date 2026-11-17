#!/usr/bin/env python3
"""One specific tension for the shoulder muscles, calibrated to measured isometric shoulder moments.

``build/data/shoulder_strength.json`` holds the PCSA of the glenohumeral muscles kept from MoBL-ARMS (Holzbaur et al.
2005 Table 1), the strength targets with their postures and the fit. The rotator cuff (Ward et al. 2006) and the
scapular muscles (Klein Breteler et al. 1999) carry their PCSA in ``scapular_muscles.json``. Every one of these
muscles gets peak force = PCSA x the fitted value; the elbow muscles (biceps, triceps) keep their forces, and the
passive force of each muscle stays a fixed fraction of its peak force.

The maximal moment of a target is the sum over the muscles crossing the glenohumeral joint of their active moment
about the target's axis where it is positive, plus every muscle's passive moment (no co-contraction, no gravity, as a
gravity-corrected dynamometer). The glenohumeral hinges are replaced by a ball joint so the torque vector on the humerus
has no coordinate singularity; the humerus is placed at the target's humerothoracic angles (ISB order: plane,
elevation, axial rotation) and the girdle at the frame of Seth et al. 2019's measured motions nearest in
humerothoracic elevation (the rest posture below 20 deg). ``--fit`` stores these girdle poses, so the check and the
test need no archive.

    uv run python scripts/fit_shoulder_strength.py --zip ThoracoscapularShoulderPaperMaterials.zip --fit
    uv run python scripts/fit_shoulder_strength.py --write
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import zipfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import myo_sim
from myo_sim.build.compose import RIGHT_ARM_MUSCLES_XML
from myo_sim.build.hand_data import DATA_DIR

sys.path.insert(0, str(Path(__file__).parent))
from fit_scapulothoracic import axis_rot  # noqa: E402

TABLE = DATA_DIR / "shoulder_strength.json"
SCAPULAR = DATA_DIR / "scapular_muscles.json"
GH = ("elv_angle_r", "shoulder_elv_r", "shoulder1_r2_r", "shoulder_rot_r")
GIRDLE = ("sternoclavicular_r2_r", "sternoclavicular_r3_r", "sternoclavicular_r1_r", "acromioclavicular_r")


def load_table() -> dict[str, Any]:
    return json.loads(TABLE.read_text())


def pcsa(table: dict[str, Any]) -> dict[str, float]:
    """PCSA (cm2) of every muscle the value applies to, by actuator name without side."""
    scapular = json.loads(SCAPULAR.read_text())["muscles"]
    return {k: v["cm2"] for k, v in table["pcsa"].items()} | {r["name"]: r["pcsa_cm2"] for r in scapular}


class Rig:
    """myoarm_r with the glenohumeral hinges replaced by a ball joint at the same centre."""

    def __init__(self) -> None:
        spec = myo_sim.load_spec("myoarm_r")
        for eq in list(spec.equalities):
            if eq.name == "shoulder1_r2_con_r":
                spec.delete(eq)
        for joint in list(spec.joints):
            if joint.name in GH:
                spec.delete(joint)
        spec.body("humerus_r").add_joint(name="gh_ball", type=mujoco.mjtJoint.mjJNT_BALL, pos=[0, 0, 0])
        self.model = m = spec.compile()
        self.data = mujoco.MjData(m)
        ref = myo_sim.load_model("myoarm_r")
        self.axes = [ref.jnt_axis[ref.joint(j).id] for j in ("elv_angle_r", "shoulder_elv_r", "shoulder_rot_r")]
        self.ball_dof = m.jnt_dofadr[m.joint("gh_ball").id]
        self.ball_qpos = m.jnt_qposadr[m.joint("gh_ball").id]
        self.names = [m.actuator(i).name.removesuffix("_r") for i in range(m.nu)]

    def humerothoracic(self, plane: float, elevation: float, rotation: float) -> np.ndarray:
        """Humerus orientation in the thorax frame (deg; the glenohumeral zero frame is the thorax frame at rest)."""
        ay, ax, ar = self.axes
        p, e, r = np.radians([plane, elevation, rotation])
        return axis_rot(ay, p) @ axis_rot(ax, e) @ axis_rot(ay, -p) @ axis_rot(ar, r)

    def qpos(self, posture: dict[str, Any], girdle: dict[str, list[float]]) -> tuple[np.ndarray, np.ndarray]:
        m, d = self.model, self.data
        qp = m.qpos0.copy()
        for name, values in girdle.items():
            a = m.jnt_qposadr[m.joint(name).id]
            qp[a : a + len(values)] = values
        qp[m.jnt_qposadr[m.joint("elbow_flexion_r").id]] = np.radians(posture["elbow_deg"])
        d.qpos[:] = qp
        mujoco.mj_kinematics(m, d)
        parent = d.xmat[m.body("humphant1_r").id].reshape(3, 3)
        chest = d.xmat[m.body("chest_r").id].reshape(3, 3)
        target = self.humerothoracic(posture["plane_deg"], posture["elevation_deg"], posture["rotation_deg"])
        qp[self.ball_qpos : self.ball_qpos + 4] = np.roll(Rotation.from_matrix(parent.T @ chest @ target).as_quat(), 1)
        return qp, chest

    def axis(self, posture: dict[str, Any], chest: np.ndarray, direction: str) -> np.ndarray:
        """World axis of the target's direction: elevation (+) / depression about the plane's normal, internal (+) /
        external rotation about the humerus."""
        h = 1e-5
        key = "rotation_deg" if direction in ("internal_rotation", "external_rotation") else "elevation_deg"

        def at(delta: float) -> np.ndarray:
            p = dict(posture, **{key: posture[key] + np.degrees(delta)})
            return self.humerothoracic(p["plane_deg"], p["elevation_deg"], p["rotation_deg"])

        u = chest @ Rotation.from_matrix(at(h) @ at(-h).T).as_rotvec() / (2 * h)
        return -u if direction in ("depression", "external_rotation") else u

    def moments_per_newton(self, target: dict[str, Any], girdle: dict[str, list[float]]) -> tuple[np.ndarray, np.ndarray]:
        """Each actuator's active and passive moment about the target axis per newton of its peak force (N m / N)."""
        m, d = self.model, self.data
        qp, chest = self.qpos(target["posture"], girdle)
        u = self.axis(target["posture"], chest, target["direction"])
        torque = {}
        for act in (0.0, 1.0):
            d.qpos[:], d.qvel[:], d.act[:] = qp, 0.0, act
            mujoco.mj_forward(m, d)
            moment = np.zeros((m.nu, m.nv))
            mujoco.mju_sparse2dense(moment, d.actuator_moment, d.moment_rownnz, d.moment_rowadr, d.moment_colind)
            local = moment[:, self.ball_dof : self.ball_dof + 3] * d.actuator_force[:, None]
            rh = d.xmat[m.body("humerus_r").id].reshape(3, 3)
            torque[act] = local @ rh.T @ u
        peak = m.actuator_gainprm[:, 2]
        return (torque[1.0] - torque[0.0]) / peak, torque[0.0] / peak


def forces_at(table: dict[str, Any], rig: Rig, sigma: float) -> np.ndarray:
    """Peak forces at specific tension *sigma*; muscles outside the table keep the model's."""
    areas = pcsa(table)
    return np.array([areas[n] * sigma if n in areas else f for n, f in zip(rig.names, rig.model.actuator_gainprm[:, 2])])


def model_moment(table: dict[str, Any], per_n: tuple[np.ndarray, np.ndarray], rig: Rig, sigma: float) -> float:
    """Maximal moment (N m) at specific tension *sigma*."""
    active, passive = per_n
    forces = forces_at(table, rig, sigma)
    return float(np.clip(active * forces, 0.0, None).sum() + (passive * forces).sum())


def girdle_poses(table: dict[str, Any], zf: zipfile.ZipFile) -> dict[str, dict[str, list[float]]]:
    """Girdle joint values per target: the Seth frame of the target's plane nearest in elevation, rest below 20 deg."""
    import generate_scapular_muscles as gsm

    ref = myo_sim.load_model("myoarm_r")
    frames, trials, elevation = gsm.seth_frames(zf, ref, info=True)
    out = {}
    for t in table["targets"]:
        p = t["posture"]
        if p["elevation_deg"] < 20:
            out[t["name"]] = {}
            continue
        prefix = "flx" if p["seth_trials"] == "flexion" else "abd"
        k = min((i for i, n in enumerate(trials) if n.startswith(prefix)), key=lambda i: abs(elevation[i] - p["elevation_deg"]))
        out[t["name"]] = {
            j: [
                round(float(v), 8) for v in frames[k][ref.jnt_qposadr[ref.joint(j).id] :][: 4 if j.startswith("acromio") else 1]
            ]
            for j in GIRDLE
        }
    return out


def measure(table: dict[str, Any], rig: Rig) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    return {t["name"]: rig.moments_per_newton(t, table["girdle_poses"][t["name"]]) for t in table["targets"]}


def fit(table: dict[str, Any], rig: Rig) -> None:
    """Weighted least squares of (model - mean) / SD over the fit targets (the moment is linear in the value)."""
    per_n = measure(table, rig)
    fit_rows = [t for t in table["targets"] if t["fit"]]
    a = np.array(
        [model_moment(table, per_n[t["name"]], rig, 1.0) - model_moment(table, per_n[t["name"]], rig, 0.0) for t in fit_rows]
    )
    b = np.array([model_moment(table, per_n[t["name"]], rig, 0.0) for t in fit_rows])
    mean = np.array([t["mean_Nm"] for t in fit_rows])
    w = 1.0 / np.array([t["sd_Nm"] for t in fit_rows]) ** 2
    sigma = round(float(np.sum(w * a * (mean - b)) / np.sum(w * a * a)), 1)
    results = {}
    for t in table["targets"]:
        value = model_moment(table, per_n[t["name"]], rig, sigma)
        row = {"model_Nm": round(value, 1)}
        if t.get("sd_Nm"):
            row["z"] = round((value - t["mean_Nm"]) / t["sd_Nm"], 2)
        results[t["name"]] = row
    z = np.array([results[t["name"]]["z"] for t in fit_rows])
    table["fit"]["result"] = {"value_N_per_cm2": sigma, "rms_z": round(float(np.sqrt(np.mean(z**2))), 2), "targets": results}


def girdle_check(table: dict[str, Any], rig: Rig) -> None:
    """Shrug and protraction force at the table's value (reported, not fitted): the largest moment of the girdle
    muscles about the sternoclavicular hinge, divided by its lever to the point where the force is measured."""
    m, d = rig.model, rig.data
    sigma = table["fit"]["result"]["value_N_per_cm2"]
    forces = forces_at(table, rig, sigma)
    out = {}
    for check in table["girdle_checks"]:
        qp, chest = rig.qpos(check["posture"], table["girdle_poses"][check["posture"]["pose_of"]])
        jid = m.joint(check["joint"]).id
        torque = {}
        for act in (0.0, 1.0):
            d.qpos[:], d.qvel[:], d.act[:] = qp, 0.0, act
            mujoco.mj_forward(m, d)
            moment = np.zeros((m.nu, m.nv))
            mujoco.mju_sparse2dense(moment, d.actuator_moment, d.moment_rownnz, d.moment_rowadr, d.moment_colind)
            torque[act] = moment[:, m.jnt_dofadr[jid]] * d.actuator_force / m.actuator_gainprm[:, 2] * forces
        total = float(np.clip(torque[1.0] - torque[0.0], 0.0, None).sum() + torque[0.0].sum())
        if check["point"] == "acromioclavicular":
            body = m.body("scapula_r").id
            point = d.xpos[body] + d.xmat[body].reshape(3, 3) @ m.jnt_pos[m.joint("acromioclavicular_r").id]
        else:
            point = d.xpos[m.body("humerus_r").id]
        lever = float(np.cross(point - d.xanchor[jid], chest @ np.array(check["force_direction_thorax"])) @ d.xaxis[jid])
        out[check["name"]] = {"moment_Nm": round(total, 1), "lever_mm": round(lever * 1e3, 1), "force_N": round(total / lever)}
    table["fit"]["girdle_checks"] = out


def muscles_xml(text: str, table: dict[str, Any], sigma: float) -> str:
    """Peak force of the kept glenohumeral muscles (gainprm and biasprm third value)."""
    for name, row in table["pcsa"].items():
        pattern = re.compile(rf'(<general name="{name}_r"[^>]*>)')
        (tag,) = pattern.findall(text)
        new = tag
        for key in ("gainprm", "biasprm"):
            values = re.search(rf'{key}="([^"]*)"', new).group(1).split()
            values[2] = f"{row['cm2'] * sigma:.1f}"
            new = new.replace(re.search(rf'{key}="[^"]*"', new).group(0), f'{key}="{" ".join(values)}"')
        text = text.replace(tag, new)
    return text


def write(table: dict[str, Any]) -> None:
    """Forces at the table's value: kept muscles in the arm muscle file, cuff and scapular muscles in their table."""
    import generate_scapular_muscles as gsm

    sigma = table["fit"]["result"]["value_N_per_cm2"]
    RIGHT_ARM_MUSCLES_XML.write_text(muscles_xml(RIGHT_ARM_MUSCLES_XML.read_text(), table, sigma))
    scapular = json.loads(SCAPULAR.read_text())
    scapular["specific_tension_N_per_cm2"] = sigma
    scapular["specific_tension_note"] = gsm.TENSION_NOTE
    for row in scapular["muscles"]:
        row["force_N"] = round(row["pcsa_cm2"] * sigma, 1)
    SCAPULAR.write_text(json.dumps(scapular, indent=2) + "\n")
    gsm.write(scapular)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--zip", type=Path, help="ThoracoscapularShoulderPaperMaterials.zip from SimTK (for --fit)")
    parser.add_argument("--fit", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    table = load_table()
    if args.fit:
        with zipfile.ZipFile(args.zip) as zf:
            table["girdle_poses"] = girdle_poses(table, zf)
        rig = Rig()
        fit(table, rig)
        girdle_check(table, rig)
        TABLE.write_text(json.dumps(table, indent=2) + "\n")
        write(table)
    elif args.write:
        write(table)
    result = load_table()["fit"]
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
