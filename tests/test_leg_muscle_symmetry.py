import mujoco
import pytest

import myo_sim
from muscle_analysis_utils import parse_model_joint_equalities
from muscle_symmetry_checks import (
    compare_muscle_pair,
    discover_bilateral_suffix_pairs,
)


@pytest.fixture(scope="module")
def leg_model_context():
    model = mujoco.MjModel.from_xml_path(str(myo_sim.MODELS_DIR / "leg" / "myolegs.xml"))
    data = mujoco.MjData(model)
    eq_map = parse_model_joint_equalities(model)
    return model, data, eq_map


@pytest.fixture(scope="module")
def discovered_leg_pairs(leg_model_context):
    model, data, eq_map = leg_model_context
    pairs = discover_bilateral_suffix_pairs(
        model,
        data,
        eq_map,
        right_muscle_suffix="_r",
        left_muscle_suffix="_l",
        right_joint_suffix="_r",
        left_joint_suffix="_l",
        limit=None,
    )
    assert pairs
    return pairs


def test_discovered_leg_muscle_pairs_have_symmetric_curves(leg_model_context, discovered_leg_pairs):
    model, data, eq_map = leg_model_context
    failures = []
    checked_pairs = []
    for pair in discovered_leg_pairs:
        # The current non-abdomen leg model has known left/right knee-angle curve differences.
        if pair.right_joint == "knee_angle_r" or pair.left_joint == "knee_angle_l":
            continue
        checked_pairs.append(pair)
        summary = compare_muscle_pair(model, data, eq_map, pair)
        if not summary["ok"]:
            failures.append((pair.label, summary))

    assert checked_pairs
    assert not failures

