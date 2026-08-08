import pytest
from muscle_analysis_utils import parse_model_joint_equalities
from muscle_symmetry_checks import compare_muscle_pair, discover_shared_joint_pairs

import myo_sim

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
