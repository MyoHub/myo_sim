import mujoco
import numpy as np
import pytest

from myo_sim.build.compose import build_model

ARM_BODY_PAIRS = (
    "clavicle",
    "scapula",
    "humerus",
    "ulna",
    "radius",
    "capitate",
    "thirdmc",
    "distph3",
)

ARM_JOINT_PAIRS = (
    "sternoclavicular_r2",
    "sternoclavicular_r3",
    "acromioclavicular_r1",
    "shoulder_elv",
    "elbow_flexion",
    "pro_sup",
    "flexion",
    "mcp3_flexion",
    "mcp3_abduction",
)


def body_id(model, name):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert body >= 0, name
    return body


def joint_id(model, name):
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    assert joint >= 0, name
    return joint


def mirror_plane_x(model, data):
    right = body_id(model, "clavicle_r")
    left = body_id(model, "clavicle_l")
    return float((data.xipos[right, 0] + data.xipos[left, 0]) / 2.0)


@pytest.mark.parametrize("model_name", ("myoarms", "myofullbody"))
def test_mirrored_arm_body_centers_reflect_across_sagittal_plane(model_name):
    model = build_model(model_name)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    plane_x = mirror_plane_x(model, data)

    for base_name in ARM_BODY_PAIRS:
        right = body_id(model, f"{base_name}_r")
        left = body_id(model, f"{base_name}_l")

        mirrored_right = data.xipos[right].copy()
        mirrored_right[0] = 2.0 * plane_x - mirrored_right[0]

        np.testing.assert_allclose(
            data.xipos[left],
            mirrored_right,
            atol=1e-9,
            err_msg=f"{model_name}: {base_name}_l is not the reflection of {base_name}_r",
        )


@pytest.mark.parametrize("model_name", ("myoarms", "myofullbody"))
def test_mirrored_arm_joint_axes_reflect_across_sagittal_plane(model_name):
    model = build_model(model_name)

    for base_name in ARM_JOINT_PAIRS:
        right = joint_id(model, f"{base_name}_r")
        left = joint_id(model, f"{base_name}_l")

        mirrored_right_axis = model.jnt_axis[right].copy()
        mirrored_right_axis[0] *= -1.0
        mirrored_right_axis[1] *= -1.0

        np.testing.assert_allclose(
            model.jnt_axis[left],
            mirrored_right_axis,
            atol=1e-9,
            err_msg=f"{model_name}: {base_name}_l axis is not mirrored from {base_name}_r",
        )
