"""Finger extensor mechanism (option a): generated XML, branch fractions, moment arms, mirroring and the ORL.

The table is ``build/data/extensor_mechanism.json``; ``scripts/fit_extensor_mechanism.py`` writes it into the
right-arm XML (method: ``docs/wiki/extensor-mechanism.md``).
"""

from __future__ import annotations

import importlib.util
from functools import cache
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build import compose
from myo_sim.build.hand import tendon_path
from myo_sim.build.hand_data import read_generated_block

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


@cache
def _script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("fit_extensor_mechanism", SCRIPTS / "fit_extensor_mechanism.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@cache
def _model(name: str = "myoarm_r") -> mujoco.MjModel:
    return myo_sim.load_spec(name).compile()


def test_generated_blocks_are_what_the_script_writes() -> None:
    em = _script()
    table = em.load_table()
    for body, block in em.chain_blocks(table, em.skeleton()).items():
        assert read_generated_block(compose.RIGHT_ARM_CHAIN_XML, em.SCRIPT, body) == block, body
    for row in table["muscles"]:
        assert read_generated_block(compose.RIGHT_ARM_TENDONS_XML, em.SCRIPT, row["name"]) == em.tendon_block(table, row)
    assert read_generated_block(compose.RIGHT_ARM_TENDONS_XML, em.SCRIPT) == em.ligament_block(table)
    model = _model()
    for name, (lo, hi) in em.intrinsic_lengthranges(table, model).items():
        np.testing.assert_allclose(model.actuator_lengthrange[model.actuator(name).id], [lo, hi], rtol=1e-6, err_msg=name)


def test_branches_carry_the_tabled_fractions() -> None:
    """Every branched tendon has one pulley per branch with divisor 1 / fraction, and nothing else changes counts."""
    em = _script()
    table = em.load_table()
    spec = myo_sim.load_spec("myoarm_r")
    for row in table["muscles"]:
        divisors = [w[4] for w in tendon_path(spec.tendon(f"{row['name']}_r_tendon")) if w[0] == mujoco.mjtWrap.mjWRAP_PULLEY]
        expected = [1.0 / f for f in em.branch_fractions(table, row).values()]
        np.testing.assert_allclose(divisors, expected, rtol=1e-5, err_msg=row["name"])
        assert sum(em.branch_fractions(table, row).values()) <= 1.0 + 1e-9


def test_extensor_share_reproduces_the_recorded_fit() -> None:
    """The index EDC2 and EIP IP moment arms give the recorded z-scores against An 1983 and Lee 2008b."""
    em = _script()
    table = em.load_table()
    c = table["fractions"]["extensor"]["c"]
    model = _model()
    for name, targets in table["targets"].items():
        arms = em.index_arms(model, table, name)
        for key, rows in targets.items():
            for t in rows:
                z = (arms[key][t["source"]] - t["mean_mm"]) / t["sd_mm"]
                assert z == pytest.approx(c["result"]["z"][f"{name} {key} {t['source']}"], abs=0.02)
    assert c["result"]["rms_z"] < 1.0


@pytest.mark.parametrize("digit", (2, 3, 4, 5))
def test_branched_tendons_extend_both_ip_joints_over_their_ranges(digit: int) -> None:
    """No branched tendon turns into an IP flexor anywhere in the PIP and DIP ranges (the band wraps hold the bands dorsal)."""
    em = _script()
    model = _model()
    data = mujoco.MjData(model)
    for row in em.load_table()["muscles"]:
        if row["digit"] != digit:
            continue
        for joint in (f"pm{digit}_flexion_r", f"md{digit}_flexion_r"):
            hi = np.degrees(model.jnt_range[model.joint(joint).id][1])
            for deg in np.linspace(0.0, hi, 12):
                assert em.moment_arm(model, data, f"{row['name']}_r_tendon", joint, {joint: deg}) > 0.5, (
                    row["name"],
                    joint,
                    deg,
                )


def test_left_hand_mirrors_the_mechanism() -> None:
    """In myoarms the left branched tendons and ligaments have the right ones' lengths and divisors."""
    em = _script()
    table = em.load_table()
    model = _model("myoarms")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    spec = myo_sim.load_spec("myoarms")
    names = [f"{row['name']}_r_tendon" for row in table["muscles"]] + [f"lig_ORL{d}_r" for d in table["digits"]]
    for right in names:
        left = right.replace("_r_tendon", "_l_tendon") if right.endswith("_r_tendon") else right[:-2] + "_l"
        assert data.ten_length[model.tendon(left).id] == pytest.approx(data.ten_length[model.tendon(right).id], abs=1e-9)
        assert [w[4] for w in tendon_path(spec.tendon(left))] == [w[4] for w in tendon_path(spec.tendon(right))]
    for key in table["digits"]:
        for site in ("H", "CSP", "RB", "UB", "RBD", "UBD", "CS", "T", "TI"):
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"EM{key}_{site}_l") >= 0


def test_orl_is_slack_in_unresisted_flexion_and_taut_in_ueba_protocol() -> None:
    """Harris & Rutledge 1972: no ORL force while DIP - PIP <= 30 deg; Ueba 2011's posture (PIP 0, DIP 45) stretches it."""
    em = _script()
    table = em.load_table()
    model = _model()
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    for key, digit in table["digits"].items():
        t = model.tendon(f"lig_ORL{key}_r").id
        pip, dip = (model.joint(digit["joints"][j]) for j in ("pip", "dip"))

        def stretch(p: float, d: float) -> float:
            data.qpos[:] = model.qpos0
            data.qpos[pip.qposadr[0]], data.qpos[dip.qposadr[0]] = np.radians(p), np.radians(d)
            mujoco.mj_fwdPosition(model, data)
            return float(data.ten_length[t] - model.tendon_lengthspring[t][1])

        for _ in range(200):
            p = rng.uniform(0.0, 115.0)
            d = rng.uniform(0.0, min(90.0, p + 30.0))
            assert stretch(p, d) <= 1e-9
        assert stretch(0.0, 45.0) > 0.0
