"""The knee against its source model.

The legs fragment was converted from Rajagopal et al.'s full-body OpenSim model (Rajagopal2015.osim). Its walker_knee anchors the
knee's rotations at a joint frame 8.95 mm from the tibia's origin, so where the femur sits relative to the tibia
depends on where the knee's hinges sit. The test checks position only. The reference values below are the femur's origin in the
tibia's frame, computed with
OpenSim 4.6 on Rajagopal2015.osim with the knee angle set and the joint's couplings enforced. The left knee mirrors the right in z.
"""

import mujoco
import numpy as np
import pytest

from myo_sim import load_model

# femur_r's origin in tibia_r's frame (m), OpenSim 4.6, Rajagopal2015.osim, by knee flexion in degrees
OPENSIM_FEMUR_IN_TIBIA_R = {
    0: (4.6e-07, 0.4044252, 0.00126526),
    30: (-0.207409106613, 0.349768489496, -0.029175461244),
    90: (-0.407269807194, -0.005492294638, -0.074626560278),
    120: (-0.354442279096, -0.208015098299, -0.053421166102),
}
TOLERANCE_M = 5e-5
MODELS_WITH_RAJAGOPAL_KNEE = ("myolegs", "myolegs_abdomen", "myofullbody")


def femur_in_tibia(model, side, degrees):
    """The femur's origin in the tibia's frame with the knee at `degrees` and its coupled coordinates on their polynomials."""
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    knee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"knee_angle_{side}")
    assert knee >= 0
    active = {
        int(model.eq_obj1id[e])
        for e in range(model.neq)
        if model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT and model.eq_obj2id[e] == knee and model.eq_active0[e]
    }
    for coordinate in ("translation1", "translation2", "rotation2", "rotation3"):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"knee_angle_{coordinate}_{side}")
        assert joint in active, f"knee_angle_{coordinate}_{side} is not an active coupling of the knee"
    knee_adr = model.jnt_qposadr[knee]
    data.qpos[knee_adr] = np.radians(degrees)
    for eq in range(model.neq):
        if model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT and model.eq_obj2id[eq] == knee:
            follower_adr = model.jnt_qposadr[model.eq_obj1id[eq]]
            offset = data.qpos[knee_adr] - model.qpos0[knee_adr]
            data.qpos[follower_adr] = model.qpos0[follower_adr] + np.polyval(model.eq_data[eq, 4::-1], offset)
    mujoco.mj_kinematics(model, data)
    femur = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"femur_{side}")
    tibia = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"tibia_{side}")
    return data.xmat[tibia].reshape(3, 3).T @ (data.xpos[femur] - data.xpos[tibia])


@pytest.mark.parametrize("name", MODELS_WITH_RAJAGOPAL_KNEE)
@pytest.mark.parametrize("side", ("r", "l"))
def test_femur_position_matches_opensim(name, side):
    model = load_model(name)
    mirror = np.array([1.0, 1.0, 1.0 if side == "r" else -1.0])
    errors_mm = {
        degrees: 1000 * np.linalg.norm(femur_in_tibia(model, side, degrees) - mirror * np.array(reference))
        for degrees, reference in OPENSIM_FEMUR_IN_TIBIA_R.items()
    }
    assert max(errors_mm.values()) < 1000 * TOLERANCE_M, f"{name} knee_{side}, femur off OpenSim by (mm): {errors_mm}"
