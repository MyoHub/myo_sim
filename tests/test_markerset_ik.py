"""Inverse kinematics with native MuJoCo primitives: each marker site is connected to a mocap target by a soft equality,
and the model settles onto the targets. A known pose is recovered from its own marker positions."""

import mujoco
import numpy as np
import pytest

from myo_sim import load_spec
from myo_sim.markersets import add_markerset, list_markersets, markerset_path

POSE = {
    "hip_flexion_r": 0.6, "hip_adduction_r": -0.15, "hip_rotation_r": 0.2, "knee_angle_r": 0.8, "ankle_angle_r": -0.2,
    "hip_flexion_l": -0.3, "hip_adduction_l": 0.1, "knee_angle_l": 0.3, "ankle_angle_l": 0.25,
    "flex_extension": 0.2, "lat_bending": 0.1, "axial_rotation": -0.1,
    "shoulder_elv_r": 0.6, "elv_angle_r": 0.5, "elbow_flexion_r": 1.0, "shoulder_elv_l": 0.4, "elbow_flexion_l": 0.6,
}  # fmt: skip
ROOT_SHIFT = np.array([0.1, -0.05, 0.02])


def marker_names(name: str) -> list[str]:
    overlay = mujoco.MjSpec.from_file(str(markerset_path(name)))  # keep the spec alive while reading its sites
    return [s.name for s in overlay.sites]


def build(name: str, with_targets: bool) -> mujoco.MjModel:
    spec = load_spec("myofullbody")
    add_markerset(spec, name)
    if with_targets:
        for n in marker_names(name):
            body = spec.worldbody.add_body(name="target_" + n, mocap=True)
            body.add_site(name="target_" + n)
            spec.add_equality(
                name="ik_" + n, type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_SITE, name1=n, name2="target_" + n
            )
    model = spec.compile()
    off = mujoco.mjtDisableBit
    model.opt.disableflags |= off.mjDSBL_CONTACT | off.mjDSBL_ACTUATION | off.mjDSBL_GRAVITY | off.mjDSBL_SPRING
    model.dof_damping[:] = 5.0
    return model


def settle(model: mujoco.MjModel, data: mujoco.MjData, steps: int) -> None:
    for _ in range(steps):
        mujoco.mj_step(model, data)
        data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def observed_joints(model: mujoco.MjModel, name: str) -> list[str]:
    """Joints of POSE whose body subtree carries at least one marker."""
    bodies = {int(model.site_bodyid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n)]) for n in marker_names(name)}
    out = []
    for j in POSE:
        root = int(model.jnt_bodyid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)])
        for b in bodies:
            while b > root:
                b = int(model.body_parentid[b])
            if b == root:
                out.append(j)
                break
    return out


@pytest.mark.parametrize("name", list_markersets())
def test_known_pose_is_recovered_from_markers(name):
    names = marker_names(name)
    ref = build(name, with_targets=False)
    ref_data = mujoco.MjData(ref)
    for j, v in POSE.items():
        ref_data.qpos[ref.jnt_qposadr[mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_JOINT, j)]] = v
    ref_data.qpos[:3] += ROOT_SHIFT
    settle(ref, ref_data, 300)  # let the joint couplings settle into a consistent pose
    target = {n: ref_data.site_xpos[mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_SITE, n)].copy() for n in names}

    model = build(name, with_targets=True)
    data = mujoco.MjData(model)
    for n in names:
        data.mocap_pos[model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_" + n)]] = target[n]
    settle(model, data, 2000)

    sid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n) for n in names]
    residual = np.linalg.norm(data.site_xpos[sid] - np.array([target[n] for n in names]), axis=1)
    assert np.sqrt(np.mean(residual**2)) < 0.003 and residual.max() < 0.01
    joints = observed_joints(model, name)
    assert len(joints) >= 9
    for j in joints:
        jm, jr = (mujoco.mj_name2id(x, mujoco.mjtObj.mjOBJ_JOINT, j) for x in (model, ref))
        assert abs(data.qpos[model.jnt_qposadr[jm]] - ref_data.qpos[ref.jnt_qposadr[jr]]) < np.radians(2), j
