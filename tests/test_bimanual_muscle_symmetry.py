import mujoco
import pytest
from muscle_analysis_utils import parse_model_joint_equalities
from muscle_symmetry_checks import (
    compare_muscle_pair,
    discover_bilateral_suffix_pairs,
)

from myo_sim import load_model


@pytest.fixture(scope="module")
def bimanual_model_context():
    model = load_model("myoarms")
    data = mujoco.MjData(model)
    eq_map = parse_model_joint_equalities(model)
    return model, data, eq_map


@pytest.fixture(scope="module")
def discovered_bimanual_pairs(bimanual_model_context):
    model, data, eq_map = bimanual_model_context
    pairs = discover_bilateral_suffix_pairs(
        model,
        data,
        eq_map,
        right_muscle_suffix="",
        left_muscle_suffix="_l",
        right_joint_suffix="_r",
        left_joint_suffix="_l",
        limit=None,
    )
    assert pairs
    return pairs


def test_discovered_bimanual_muscle_pairs_have_symmetric_curves(
    bimanual_model_context,
    discovered_bimanual_pairs,
):
    model, data, eq_map = bimanual_model_context
    failures = []
    for pair in discovered_bimanual_pairs:
        summary = compare_muscle_pair(model, data, eq_map, pair)
        if not summary["ok"]:
            failures.append((pair.label, summary))

    assert not failures
