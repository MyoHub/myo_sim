"""Interossei and lumbricals of digits 2-5 (hand rebuild S5): names, branch fractions and fitted moment arms.

The rows are in ``build/data/hand_intrinsic_muscles.json`` (those with an ``insertion``); the tendons are written
by ``scripts/generate_hand_intrinsic_muscles.py``, the band branches from ``build/data/extensor_mechanism.json``.
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
from myo_sim.build.hand import tendon_path
from myo_sim.build.hand_data import load_intrinsic_table

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
RENAMED = {
    "UI_UB2": "PI1",
    "LU_RB2": "LUM1",
    "RI3": "DI2",
    "LU_RB3": "LUM2",
    "UI_UB3": "DI3",
    "RI4": "PI2",
    "LU_RB4": "LUM3",
    "UI_UB4": "DI4",
    "RI5": "PI3",
    "LU_RB5": "LUM4",
}


@cache
def _gen() -> ModuleType:
    spec = importlib.util.spec_from_file_location("generate_hand_intrinsic_muscles", SCRIPTS / "generate_hand_intrinsic_muscles.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _rows() -> list[dict]:
    return [row for row in load_intrinsic_table()["muscles"] if "insertion" in row]


def test_every_renamed_bundle_has_a_branched_row() -> None:
    assert {row["replaces"]: row["name"] for row in _rows()} == RENAMED


@pytest.mark.parametrize("name", ("myohand_r", "myohands", "myoarm_r", "myofullbody"))
def test_models_carry_the_new_names_on_each_hand(name: str) -> None:
    model = myo_sim.load_spec(name).compile()
    actuators = {model.actuator(i).name for i in range(model.nu)}
    sides = ("r", "l") if name in ("myohands", "myofullbody") else ("r",)
    for old, new in RENAMED.items():
        for side in sides:
            assert f"{new}_{side}" in actuators and f"{old}_{side}" not in actuators, (new, side)
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{new}_{side}_tendon") >= 0


def test_divisors_compose_hood_and_band_fractions() -> None:
    """Bony branch 1 / (1 - f), hood branch 1 / f, then 1 / (f x branch fraction) for each band branch."""
    gen = _gen()
    em = gen.em_table()
    spec = myo_sim.load_spec("myoarm_r")
    for row in _rows():
        f = gen.hood_fraction(row)
        expected = [1.0 / (1.0 - f)] if "bony" in row["insertion"] else []
        expected += [1.0 / f] if "bony" in row["insertion"] or f < 1.0 else []
        expected += [1.0 / v for v in gen.band_fractions(row, em).values()]
        path = tendon_path(spec.tendon(f"{row['name']}_r_tendon"))
        divisors = [w[4] for w in path if w[0] == mujoco.mjtWrap.mjWRAP_PULLEY]
        np.testing.assert_allclose(divisors, expected, rtol=1e-5, err_msg=row["name"])
        assert row["insertion"]["hood"]["fraction"]["source"] == "salsbury1937"


def test_tendons_have_the_fitted_moment_arms() -> None:
    """MuJoCo's moment arms of the generated tendons equal the fit's (the band branches do not cross the MCP)."""
    gen = _gen()
    model = myo_sim.load_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    for row in _rows():
        tendon = model.tendon(f"{row['name']}_r_tendon").id
        for joint, target in row["fit"]["targets"].items():
            values = (
                np.radians(np.linspace(0.0, 90.0, 10) if "flexion" in joint else np.linspace(-15.0, 15.0, 7))
                if "over" in target
                else np.radians(target["posture_deg"])
            )
            arms = []
            for q in values:
                lengths = []
                for h in (1e-5, -1e-5):
                    data.qpos[:] = model.qpos0
                    data.qpos[model.joint(joint).qposadr[0]] = q + h
                    mujoco.mj_fwdPosition(model, data)
                    lengths.append(data.ten_length[tendon])
                arms.append((lengths[0] - lengths[1]) / 2e-5 * 1e3)
            value = np.mean(arms) if "over" in target else arms
            np.testing.assert_allclose(value, row["fit"]["result"]["model_mm"][joint], atol=0.05, err_msg=(row["name"], joint))
        assert gen.junction(row) in {model.site(i).name for i in range(model.nsite)}
