"""Passive tenodesis after McFarland et al. 2023 (``scripts/validate_hand_function.py``, ``build/data/hand_function.json``).

The protocol needs the extrinsic slack lengths and fpmax of ``build/data/muscle_passive.json`` in the model and
the passive joint springs fitted on those muscles. Both are read from the data: the tabled fpmax in the model, and
the muscle baseline each spring row recorded at its fit equal to the current model's. Otherwise the tests skip.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from functools import cache
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

import myo_sim
from myo_sim.build.hand_function import HAND_FUNCTION_DATA_PATH, load_hand_function_table

SCRIPTS = Path(__file__).parents[1] / "scripts"
MUSCLE_PASSIVE_PATH = HAND_FUNCTION_DATA_PATH.with_name("muscle_passive.json")
PASSIVE_MOMENTS_PATH = HAND_FUNCTION_DATA_PATH.with_name("passive_joint_moments.json")


@cache
def _load(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve their module
    spec.loader.exec_module(module)
    return module


def _missing_prerequisite() -> str | None:
    """Why this model cannot run the tenodesis protocol as intended, or None."""
    if not MUSCLE_PASSIVE_PATH.exists() or not PASSIVE_MOMENTS_PATH.exists():
        return "needs the fitted extrinsic muscles (muscle_passive.json) and the passive joint springs"
    model = myo_sim.load_spec("myoarm_r").compile()
    for row in json.loads(MUSCLE_PASSIVE_PATH.read_text())["muscles"]:
        if model.actuator_gainprm[model.actuator(row["names"]["r"]).id][7] != row["fpmax"]["value"]:
            return f"the model does not carry the fitted fpmax of {row['muscle']}"
    fit = _load("fit_passive_moments")
    for row in json.loads(PASSIVE_MOMENTS_PATH.read_text())["joints"]:
        baseline = row.get("source_curve", {}).get("subtract_model_muscles", {}).get("baseline_Nm")
        if baseline is None:
            continue
        q = np.linspace(*baseline["q_rad"])[::20]
        now = fit.muscle_baseline(row["source_curve"], row["names"]["r"], q)
        if not np.allclose(now, baseline["moment"][::20], atol=1e-5):
            return f"the springs were not fitted on the current muscles ({row['joint']}): rerun scripts/fit_passive_moments.py"
    return None


MISSING = _missing_prerequisite()
pytestmark = pytest.mark.skipif(MISSING is not None, reason=MISSING or "")


@cache
def _tenodesis(mutation: str | None = None) -> dict:
    v = _load("validate_hand_function")
    table = load_hand_function_table()
    return v.tenodesis_metrics(v.tenodesis(v.myohand(), table, v.MUTATIONS.get(mutation)), table)


def _checks(mutation: str | None = None) -> dict[str, bool]:
    return _load("validate_hand_function").tenodesis_checks(_tenodesis(mutation), load_hand_function_table())


def test_tenodesis_table_is_sourced() -> None:
    table = load_hand_function_table()
    assert table["tenodesis"]["source"] in table["sources"]
    assert "justification" in table["tenodesis"]["tolerances"]


def test_tenodesis_fingers_flex_monotonically_as_the_wrist_extends() -> None:
    assert _checks()["direction"]


@pytest.mark.parametrize("digit", (2, 3, 4, 5))
def test_tenodesis_mcp_range_of_motion(digit: int) -> None:
    assert _checks()[f"mcp_rom{digit}"]


def test_tenodesis_pip_flexes_and_dip_stays() -> None:
    checks = _checks()
    assert checks["mean_pip_rom"] and checks["dip_constant"]


def test_tenodesis_inter_digit_spread() -> None:
    assert _checks()["mcp_spread"]


def test_tenodesis_release_opens_the_fingers() -> None:
    assert _checks()["release"]


@pytest.mark.parametrize("mutation", ("without_springs", "reference_slack"))
def test_tenodesis_tolerances_reject_known_bad_models(mutation: str) -> None:
    """Every tolerance group catches a known-bad model: springs removed, or the extrinsic slack lengths and fpmax of #123."""
    checks = _checks(mutation)
    assert not all(checks.values()), checks
