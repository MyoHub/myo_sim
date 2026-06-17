import mujoco

from myo_sim.build.compose import PART_ARMS, PART_LEGS, PART_TORSO, build_model


def joint_count(model: mujoco.MjModel, *prefixes: str) -> int:
    return sum(
        1
        for i in range(model.njnt)
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i).startswith(prefixes)
    )


def test_myofullbody_torque_has_one_motor_per_independent_joint():
    muscle_model = build_model("myofullbody")
    torque_model = build_model("myofullbody", "torque")

    assert torque_model.nu < muscle_model.nu
    assert torque_model.nu == 71

    actuator_names = {
        mujoco.mj_id2name(torque_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(torque_model.nu)
    }
    assert all(name.startswith("mot_") for name in actuator_names)


def test_myofullbody_supports_mixed_actuation():
    mixed_model = build_model("myofullbody", {PART_TORSO: "torque", PART_ARMS: "torque", PART_LEGS: "muscle"})
    all_muscle = build_model("myofullbody")
    all_torque = build_model("myofullbody", "torque")

    assert all_torque.nu < mixed_model.nu < all_muscle.nu


def test_myolegs_and_myoarms_accept_uniform_torque_string():
    legs_model = build_model("myolegs", "torque")
    arms_model = build_model("myoarms", "torque")

    assert legs_model.nu == 14
    assert arms_model.nu == 54
