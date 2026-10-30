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


def _band_muscles() -> dict[str, int]:
    """Every muscle with a branched tendon (extensors and the band-inserting intrinsics) and its digit."""
    from myo_sim.build.hand_data import load_intrinsic_table

    out = {row["name"]: int(row["digit"]) for row in _script().load_table()["muscles"]}
    for row in load_intrinsic_table()["muscles"]:
        if "insertion" in row:
            out[row["name"]] = int(row["insertion"]["hood"]["digit"])
    return out


def test_lumbrical_fractions_reproduce_the_recorded_fit() -> None:
    """The index lumbrical's DIP and PIP arms (An 1983 range averages) are the recorded fit."""
    em = _script()
    table = em.load_table()
    lum = table["fractions"]["lumbrical"]
    arms = em.index_arms(_model(), table, "LUM1")
    for key in ("dip", "pip"):
        assert arms[key]["an1983"] == pytest.approx(lum["result"]["model_mm"][key]["an1983"], abs=0.02), key
        assert abs(arms[key]["an1983"] - lum["targets"][key]["mean_mm"]) < 0.05 * lum["targets"][key]["sd_mm"] + 0.02, key
    assert lum["central"]["value"] + lum["band"]["value"] <= 1.0


def test_branches_carry_the_tabled_fractions() -> None:
    """Every branched tendon has one pulley per branch with divisor 1 / fraction (the bands' common terminal tendon
    one branch), and nothing else changes counts."""
    em = _script()
    table = em.load_table()
    spec = myo_sim.load_spec("myoarm_r")
    for row in table["muscles"]:
        divisors = [w[4] for w in tendon_path(spec.tendon(f"{row['name']}_r_tendon")) if w[0] == mujoco.mjtWrap.mjWRAP_PULLEY]
        expected = [1.0 / f for f in em.pulley_fractions(em.branch_fractions(table, row)).values()]
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
    for name, row_digit in _band_muscles().items():
        if row_digit != digit:
            continue
        row = {"name": name}
        # Extensors: at least 0.5 mm. The intrinsics carry only part of their force into the bands (S5: DI2 0.48 x
        # 1/3 to the terminal tendon, the lumbricals 0.23 fitted to An 1983's DIP arm of 0.7 +- 0.6 mm), so their
        # smallest DIP arms are 0.37-0.48 mm; for them the check is 0.25 mm, still an extensor everywhere.
        least = 0.5 if name in {r["name"] for r in em.load_table()["muscles"]} else 0.25
        for joint in (f"pm{digit}_flexion_r", f"md{digit}_flexion_r"):
            hi = np.degrees(model.jnt_range[model.joint(joint).id][1])
            for deg in np.linspace(0.0, hi, 12):
                assert em.moment_arm(model, data, f"{row['name']}_r_tendon", joint, {joint: deg}) > least, (
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
    names = [f"{name}_r_tendon" for name in _band_muscles()] + [f"lig_ORL{d}_r" for d in table["digits"]]
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
