"""Tests for opt-in cervical joint articulation and ocular/IMU head rig helpers."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.head import add_ocular_rig, unlock_head_joints


def test_default_models_keep_locked_head_unchanged() -> None:
    model = myo_sim.load_spec("myotorso_arms").compile()
    joint_names = {model.joint(j).name for j in range(model.njnt)}
    assert "neck_rotation" not in joint_names
    assert "neck_flexion" not in joint_names


def test_unlock_head_joints_and_ocular_rig_orientations_and_postures() -> None:
    base_model = myo_sim.load_spec("myofullbody").compile()

    spec = myo_sim.load_spec(
        "myofullbody",
        build_kwargs={
            "unlock_head_joints": True,
            "head_roll": True,
            "head_actuators": True,
            "add_ocular_rig": True,
            "postures": True,
        },
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    assert model.nq == base_model.nq + 3
    assert model.nv == base_model.nv + 3
    assert model.nu == base_model.nu + 3
    assert model.nkey > 0
    assert model.key_qpos.shape == (model.nkey, model.nq)

    # Verify optical axes: in neutral standing pose, +X_cam points right (-X_world),
    # +Y_cam points up (+Z_world), and -Z_cam (optical forward) points anterior (-Y_world).
    for cname in ("eye_l", "eye_r", "head_fovea", "head_periphery"):
        cid = model.camera(cname).id
        assert cid >= 0
        cmat = data.cam_xmat[cid].reshape(3, 3)
        np.testing.assert_allclose(cmat[:, 0], [-1.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(cmat[:, 1], [0.0, 0.0, 1.0], atol=1e-6)
        np.testing.assert_allclose(-cmat[:, 2], [0.0, -1.0, 0.0], atol=1e-6)

    # Interpupillary separation matches 63 mm along anatomical right-left axis.
    pos_l = data.cam_xpos[model.camera("eye_l").id]
    pos_r = data.cam_xpos[model.camera("eye_r").id]
    assert np.linalg.norm(pos_l - pos_r) == pytest.approx(0.063, abs=1e-6)
    assert pos_l[0] > pos_r[0]  # +X_world is anatomical left on myofullbody/myotorso

    # Gaze ray from gaze_origin -> gaze_forward matches camera optical forward axis.
    g_orig = data.site_xpos[model.site("gaze_origin").id]
    g_fwd = data.site_xpos[model.site("gaze_forward").id]
    g_dir = (g_fwd - g_orig) / np.linalg.norm(g_fwd - g_orig)
    np.testing.assert_allclose(g_dir, [0.0, -1.0, 0.0], atol=1e-6)

    # Head IMU sensors exist and produce finite readings.
    for sname in ("head_accel", "head_gyro", "head_quat"):
        sid = model.sensor(sname).id
        assert sid >= 0
    assert np.all(np.isfinite(data.sensordata))

    # Idempotent re-application does not duplicate joints, cameras, or sensors.
    spec2 = myo_sim.load_spec("myotorso_arms")
    unlock_head_joints(spec2, add_actuators=True)
    unlock_head_joints(spec2, add_actuators=True)
    add_ocular_rig(spec2)
    add_ocular_rig(spec2)
    m2 = spec2.compile()
    assert sum(1 for j in range(m2.njnt) if m2.joint(j).name == "neck_flexion") == 1
    assert sum(1 for c in range(m2.ncam) if m2.camera(c).name == "head_fovea") == 1

    # Hand-only fragment without a head body raises KeyError clearly.
    hand_spec = myo_sim.FRAGMENT_SPEC_BUILDERS["myohand_r"]()
    with pytest.raises(KeyError, match="neck"):
        unlock_head_joints(hand_spec)
