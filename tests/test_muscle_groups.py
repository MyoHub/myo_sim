"""Automatic muscle-group discovery recovers known functional units."""

from __future__ import annotations

import numpy as np
import pytest

from myo_sim.analysis.muscle_groups import (
    auto_scope_joints,
    discover_muscle_groups,
    group_for_actuator,
    moment_arm_signatures,
)
from myo_sim.build.compose import build_model

RIGHT_LEG_JOINTS = [
    "hip_flexion_r",
    "hip_adduction_r",
    "hip_rotation_r",
    "knee_angle_r",
    "ankle_angle_r",
    "subtalar_angle_r",
    "mtp_angle_r",
]


@pytest.fixture(scope="module")
def fullbody():
    return build_model("myofullbody")


@pytest.fixture(scope="module")
def leg_groups(fullbody):
    return discover_muscle_groups(fullbody, joints=RIGHT_LEG_JOINTS)


def _group_containing(groups, actuator):
    for group in groups:
        if actuator in group.actuators:
            return group
    return None


@pytest.mark.parametrize(
    ("seed", "expected"),
    [
        ("soleus_r", {"soleus_r", "gasmed_r", "gaslat_r"}),
        ("vaslat_r", {"vasint_r", "vaslat_r", "vasmed_r"}),
        ("tibant_r", {"edl_r", "ehl_r", "tibant_r"}),
        ("tibpost_r", {"fdl_r", "fhl_r", "tibpost_r"}),
        ("perlong_r", {"perbrev_r", "perlong_r"}),
        ("semimem_r", {"bflh_r", "semimem_r", "semiten_r"}),
        ("psoas_r", {"iliacus_r", "psoas_r"}),
    ],
)
def test_known_functional_units_are_recovered(leg_groups, seed, expected):
    """Anatomical groups fall out of geometry alone, with no muscle names used."""
    group = _group_containing(leg_groups, seed)
    assert group is not None, f"{seed} was not assigned to any group"
    assert set(group.actuators) == expected


def test_group_for_actuator_auto_scopes(fullbody):
    """The seeded helper works without the caller supplying a joint list."""
    group = group_for_actuator(fullbody, "soleus_r")
    assert group is not None
    assert set(group.actuators) == {"soleus_r", "gasmed_r", "gaslat_r"}
    assert group.dominant_joint == "ankle_angle_r"


def test_auto_scope_includes_the_knee_for_soleus(fullbody):
    """One-hop expansion must reach the knee, which separates mono- from bi-articular."""
    scope = auto_scope_joints(fullbody, "soleus_r")
    assert "ankle_angle_r" in scope
    assert "knee_angle_r" in scope


def test_target_share_is_fmax_proportional(leg_groups):
    group = _group_containing(leg_groups, "soleus_r")
    order = {name: i for i, name in enumerate(group.actuators)}
    share = group.target_share
    assert share.sum() == pytest.approx(1.0)
    # Soleus is by far the largest of the three and must dominate the prior.
    assert share[order["soleus_r"]] > share[order["gasmed_r"]] > share[order["gaslat_r"]]
    assert share[order["soleus_r"]] == pytest.approx(0.60, abs=0.02)


def test_triceps_surae_gastrocnemius_split_is_not_identifiable(leg_groups):
    """The two gastrocnemius heads are near-collinear, so their split is arbitrary."""
    group = _group_containing(leg_groups, "soleus_r")
    assert group.identifiable_rank() == 2
    merged = dict(group.identifiable_targets())
    assert ("soleus_r",) in merged
    pooled = [names for names in merged if set(names) == {"gaslat_r", "gasmed_r"}]
    assert pooled, f"gastrocnemii should pool, got {list(merged)}"


def test_ankle_moment_arms_are_physiological(leg_groups):
    """Sanity check on joint filtering: Achilles moment arm is roughly 40 mm."""
    group = _group_containing(leg_groups, "soleus_r")
    index = group.joints.index("ankle_angle_r")
    arms = np.abs(group.mean_moment_arm[:, index])
    assert np.all((arms > 0.02) & (arms < 0.07)), arms


def test_slide_and_constrained_joints_are_excluded(fullbody):
    """Coupled knee DOFs must not appear, or they swamp genuine moment arms."""
    signature = moment_arm_signatures(fullbody, joints=None, n_postures=2)
    assert not [j for j in signature.joints if "translation" in j]
    assert not [j for j in signature.joints if "beta" in j]


def test_runtime_quantities_are_consistent(fullbody):
    """shares/share_error/effort agree with forces on a concrete state."""
    import mujoco

    group = group_for_actuator(fullbody, "soleus_r")
    data = mujoco.MjData(fullbody)
    data.ctrl[:] = 0.5
    mujoco.mj_forward(fullbody, data)

    forces = group.forces(data)
    assert np.all(forces >= 0.0)
    shares = group.shares(data)
    assert shares.sum() == pytest.approx(1.0)
    np.testing.assert_allclose(group.share_error(data), shares - group.target_share, atol=1e-12)
    np.testing.assert_allclose(group.relative_stress(data), forces / group.fmax, rtol=1e-12)
    assert group.observation(data).shape == (2 * len(group.actuators),)
    assert group.effort(data) == pytest.approx(float(np.sum((forces / group.fmax) ** 3)))


def test_unloaded_group_reports_zero_share_error(fullbody):
    """A slack group must not report a spurious share error."""
    import mujoco

    group = group_for_actuator(fullbody, "soleus_r")
    data = mujoco.MjData(fullbody)
    mujoco.mj_forward(fullbody, data)
    if group.forces(data).sum() <= 0.0:
        np.testing.assert_allclose(group.share_error(data), 0.0)
