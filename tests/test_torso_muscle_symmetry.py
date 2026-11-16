import mujoco
import numpy as np
import pytest
from muscle_analysis_utils import (
    compute_force_length_curve,
    compute_moment_arm_curve,
    pair_discrepancy_summary,
    parse_model_joint_equalities,
)
from muscle_symmetry_checks import compare_muscle_pair, discover_shared_joint_pairs

import myo_sim

# Mirroring across the sagittal plane (normal to the local z axis) negates these angles, so a left muscle at q must match its
# right twin at -q with the moment arm's sign flipped.
MIRROR_FLIPPED_JOINTS = ("lat_bending", "axial_rotation")

SKIP_JOINTS = {
    "lat_bending",
    "axial_rotation",
    "L4_L5_LB",
    "L4_L5_AR",
    "Abs_t1",
    "L3_L4_LB",
    "L3_L4_AR",
    "Abs_t2",
    "L2_L3_LB",
    "L2_L3_AR",
    "Abs_r3",
    "L1_L2_LB",
    "L1_L2_AR",
}


@pytest.fixture(scope="module")
def torso_model_context():
    model, data = myo_sim.load("myotorso")
    eq_map = parse_model_joint_equalities(model)
    return model, data, eq_map


@pytest.fixture(scope="module")
def discovered_torso_pairs(torso_model_context):
    model, data, eq_map = torso_model_context
    pairs = discover_shared_joint_pairs(
        model,
        data,
        eq_map,
        skip_joints=SKIP_JOINTS,
        limit=None,
    )
    assert pairs
    return pairs


def test_discovered_torso_muscle_pairs_have_symmetric_curves(torso_model_context, discovered_torso_pairs):
    model, data, eq_map = torso_model_context
    failures = []
    for pair in discovered_torso_pairs:
        summary = compare_muscle_pair(model, data, eq_map, pair)
        if not summary["ok"]:
            failures.append((pair.label, summary))

    assert not failures


def test_torso_muscle_pairs_mirror_in_lateral_bending_and_axial_rotation(torso_model_context):
    model, data, eq_map = torso_model_context
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) for joint in range(model.njnt)}
    pairs = discover_shared_joint_pairs(model, data, eq_map, skip_joints=names - set(MIRROR_FLIPPED_JOINTS))
    assert {pair.right_joint for pair in pairs} == set(MIRROR_FLIPPED_JOINTS)

    failures = []
    for pair in pairs:
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, pair.right_joint)
        axis = model.jnt_axis[joint] / np.linalg.norm(model.jnt_axis[joint])
        low, high = model.jnt_range[joint]
        assert np.isclose(axis[2], 0.0) and np.isclose(low, -high), pair.label

        curves = {}
        for side, muscle in (("right", pair.right_muscle), ("left", pair.left_muscle)):
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
            _, moment_arms = compute_moment_arm_curve(
                model, data, model.actuator_trnid[actuator, 0], joint, n=50, eq_map=eq_map
            )
            _, forces = compute_force_length_curve(model, data, actuator, joint, n=50, eq_map=eq_map)
            curves[side] = {"moment_arms": moment_arms, "forces": forces}
        right = curves["right"]
        curves["right"] = {"moment_arms": -right["moment_arms"][::-1], "forces": right["forces"][::-1]}
        summary = pair_discrepancy_summary(curves)
        if not summary["ok"]:
            failures.append((pair.label, summary))

    assert not failures


def test_ltpt_t12_left_points_sit_on_the_left(torso_model_context):
    """The test above compares the two sides with each other, so it also passes with the right muscle flipped instead."""
    model, _, _ = torso_model_context
    for point in ("R-P1", "R-P2", "R-P3", "R-P2_0", "R-P2_1"):
        right = model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"LTpT_T12_LTpT_t12_{point}_r")]
        left = model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"LTpT_T12_LTpT_t12_{point}_l")]
        assert left[2] < 0 < right[2], point
        assert np.allclose(left, right * np.array([1.0, 1.0, -1.0]), atol=1e-6), point
