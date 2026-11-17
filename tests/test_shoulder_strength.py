"""Shoulder specific tension: forces as tabled and the calibration reproduced from the stored girdle poses."""

from __future__ import annotations

import copy
import importlib.util
import sys
from pathlib import Path

import pytest

import myo_sim

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "fit_shoulder_strength.py"


def _script():
    sys.path.insert(0, str(SCRIPT.parent))
    spec = importlib.util.spec_from_file_location("fit_shoulder_strength", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_forces_are_pcsa_times_the_shoulder_value() -> None:
    script = _script()
    table = script.load_table()
    sigma = table["fit"]["result"]["value_N_per_cm2"]
    model = myo_sim.load_model("myoarm_r")
    for name, area in script.pcsa(table).items():
        assert model.actuator_gainprm[model.actuator(f"{name}_r").id, 2] == pytest.approx(area * sigma, abs=0.051), name


def test_calibration_reproduces_the_table() -> None:
    """Refitting from the stored girdle poses gives the tabled value, model moments and girdle checks."""
    pytest.importorskip("scipy")
    script = _script()
    table = script.load_table()
    refit = copy.deepcopy(table)
    rig = script.Rig()
    script.fit(refit, rig)
    script.girdle_check(refit, rig)
    old, new = table["fit"]["result"], refit["fit"]["result"]
    assert new["value_N_per_cm2"] == pytest.approx(old["value_N_per_cm2"], abs=0.1)
    for name, row in old["targets"].items():
        assert new["targets"][name]["model_Nm"] == pytest.approx(row["model_Nm"], abs=0.2), name
    for name, row in table["fit"]["girdle_checks"].items():
        assert refit["fit"]["girdle_checks"][name]["force_N"] == pytest.approx(row["force_N"], rel=0.01), name
    assert all(abs(old["targets"][t["name"]]["z"]) < 2.0 for t in table["targets"] if t["fit"])
