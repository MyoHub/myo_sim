"""Cross-model numerical equivalence test: myo_sim vs musclemimic fullbody.

Validates that body/myofullbody.xml produces identical moment arm curves and
force-length relationships as the reference musclemimic model for every shared
actuator.

Naming differences between the two models are resolved via explicit mappings
discovered by comparing compiled model element names.
"""

import sys
from pathlib import Path

import mujoco
import musclemimic_models
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from muscle_analysis_utils import (
    compute_force_length_curve,
    compute_moment_arm_curve,
    pair_discrepancy_summary,
    parse_model_joint_equalities,
    plot_pair,
)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
REF_XML = musclemimic_models.MODELS_DIR / "body/myofullbody.xml"
TGT_XML = REPO_ROOT / "body/myofullbody.xml"

OUT_DIR = Path(__file__).parent / "output" / "cross_model"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ---------------------------------------------------------------------------
# Name mappings: ref (musclemimic) -> tgt (myo_sim)
# musclemimic uses _left suffix for left arm muscles; myo_sim uses _l
# ---------------------------------------------------------------------------

def _ref_to_tgt_actuator(name: str) -> str:
    if name.endswith("_left"):
        return name[: -len("_left")] + "_l"
    return name


_JOINT_MAP_REF_TO_TGT = {
    "cmc_abduction_r": "cmc_abduction",
    "cmc_flexion_r": "cmc_flexion",
    "md3_flexion_r": "md3_flexion",
    "pm4_flexion_r": "pm4_flexion",
    "elbow_flex_r": "elbow_flexion_r",
    "elbow_flex_l": "elbow_flexion_l",
}


def _ref_to_tgt_joint(name: str) -> str:
    return _JOINT_MAP_REF_TO_TGT.get(name, name)


# ---------------------------------------------------------------------------
# Module-scoped fixtures: load models once per test session
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def ref_model():
    return mujoco.MjModel.from_xml_path(str(REF_XML))


@pytest.fixture(scope="module")
def tgt_model():
    return mujoco.MjModel.from_xml_path(str(TGT_XML))


@pytest.fixture(scope="module")
def ref_eq_map(ref_model):
    return parse_model_joint_equalities(ref_model)


@pytest.fixture(scope="module")
def tgt_eq_map(tgt_model):
    return parse_model_joint_equalities(tgt_model)


# ---------------------------------------------------------------------------
# Shared actuator list — cheap to build, just name lookups
# ---------------------------------------------------------------------------

def _shared_actuator_names() -> list[str]:
    """Return ref-side actuator names whose tgt mapping also exists."""
    ref = mujoco.MjModel.from_xml_path(str(REF_XML))
    tgt = mujoco.MjModel.from_xml_path(str(TGT_XML))
    tgt_names = {
        mujoco.mj_id2name(tgt, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(tgt.nu)
    }
    shared = []
    for i in range(ref.nu):
        ref_name = mujoco.mj_id2name(ref, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if _ref_to_tgt_actuator(ref_name) in tgt_names:
            shared.append(ref_name)
    return shared


_SHARED_MUSCLES = _shared_actuator_names()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("ref_act_name", _SHARED_MUSCLES)
def test_muscle_equivalence(ref_act_name, ref_model, tgt_model, ref_eq_map, tgt_eq_map):
    """For each shared actuator, find joints with non-zero moment arm and compare curves."""
    tgt_act_name = _ref_to_tgt_actuator(ref_act_name)

    ref_data = mujoco.MjData(ref_model)
    tgt_data = mujoco.MjData(tgt_model)

    ref_act_id = mujoco.mj_name2id(ref_model, mujoco.mjtObj.mjOBJ_ACTUATOR, ref_act_name)
    tgt_act_id = mujoco.mj_name2id(tgt_model, mujoco.mjtObj.mjOBJ_ACTUATOR, tgt_act_name)
    ref_tendon_id = ref_model.actuator_trnid[ref_act_id, 0]
    tgt_tendon_id = tgt_model.actuator_trnid[tgt_act_id, 0]

    tgt_joint_names = {
        mujoco.mj_id2name(tgt_model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(tgt_model.njnt)
    }

    joints_checked = 0
    failures = []

    for jnt_id in range(ref_model.njnt):
        ref_jnt_name = mujoco.mj_id2name(ref_model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        tgt_jnt_name = _ref_to_tgt_joint(ref_jnt_name)
        if tgt_jnt_name not in tgt_joint_names:
            continue

        q0, q1 = ref_model.jnt_range[jnt_id]
        if q0 == q1:
            continue

        _, ref_ma = compute_moment_arm_curve(
            ref_model, ref_data, ref_tendon_id, jnt_id, eq_map=ref_eq_map
        )
        if ref_ma is None or np.allclose(ref_ma, 0, atol=1e-6):
            continue

        tgt_jnt_id = mujoco.mj_name2id(tgt_model, mujoco.mjtObj.mjOBJ_JOINT, tgt_jnt_name)
        _, tgt_ma = compute_moment_arm_curve(
            tgt_model, tgt_data, tgt_tendon_id, tgt_jnt_id, eq_map=tgt_eq_map
        )
        ref_l, ref_f = compute_force_length_curve(
            ref_model, ref_data, ref_act_id, jnt_id, eq_map=ref_eq_map
        )
        tgt_l, tgt_f = compute_force_length_curve(
            tgt_model, tgt_data, tgt_act_id, tgt_jnt_id, eq_map=tgt_eq_map
        )

        ref_jnt_range = np.linspace(q0, q1, 100)
        tgt_q0, tgt_q1 = tgt_model.jnt_range[tgt_jnt_id]
        tgt_jnt_range = np.linspace(tgt_q0, tgt_q1, 100)

        curves = {
            "right": {
                "muscle": f"ref:{ref_act_name}",
                "jnt_range": ref_jnt_range,
                "moment_arms": ref_ma,
                "mtu_lengths": ref_l,
                "forces": ref_f,
            },
            "left": {
                "muscle": f"tgt:{tgt_act_name}",
                "jnt_range": tgt_jnt_range,
                "moment_arms": tgt_ma,
                "mtu_lengths": tgt_l,
                "forces": tgt_f,
            },
        }

        plot_pair(
            curves,
            title=f"{ref_act_name} @ {ref_jnt_name}",
            out_path=OUT_DIR / f"{ref_act_name}_{ref_jnt_name}.png",
        )

        s = pair_discrepancy_summary(curves)
        joints_checked += 1
        if not s["ok"]:
            failures.append(
                f"  {ref_jnt_name}: MA={s['moment_arm']:.2e}m (ok={s['moment_arm_ok']})"
                f"  F={s['force']:.2e}N {s['force_pct']:.1f}% (ok={s['force_ok']})"
            )

    if joints_checked == 0:
        pytest.skip("no joints with non-zero moment arm found")

    assert not failures, (
        f"{ref_act_name}: {len(failures)} joint(s) failed:\n" + "\n".join(failures)
    )


def test_actuator_coverage():
    """All actuators in ref must map to tgt and vice versa."""
    ref = mujoco.MjModel.from_xml_path(str(REF_XML))
    tgt = mujoco.MjModel.from_xml_path(str(TGT_XML))

    ref_names = {mujoco.mj_id2name(ref, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(ref.nu)}
    tgt_names = {mujoco.mj_id2name(tgt, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(tgt.nu)}

    ref_mapped = {_ref_to_tgt_actuator(n) for n in ref_names}
    only_ref = sorted(ref_mapped - tgt_names)
    only_tgt = sorted(tgt_names - ref_mapped)

    assert not only_ref, f"Actuators in ref but not tgt (after name mapping): {only_ref}"
    assert not only_tgt, f"Actuators in tgt but not ref (after name mapping): {only_tgt}"
