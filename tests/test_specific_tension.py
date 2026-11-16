"""One specific tension for the wrist and hand muscles: every peak force is PCSA x the tabled value on both hands."""

from __future__ import annotations

import importlib.util
import sys
from functools import cache
from pathlib import Path
from types import ModuleType

import pytest

import myo_sim


@cache
def _script() -> ModuleType:
    path = Path(__file__).parents[1] / "scripts" / "fit_specific_tension.py"
    spec = importlib.util.spec_from_file_location("fit_specific_tension", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("side", ("r", "l"))
def test_peak_forces_are_pcsa_times_one_value(side: str) -> None:
    gen = _script()
    table = gen.load_table()
    sigma = table["value_N_per_cm2"]["value"]
    model = myo_sim.load_spec("myoarms").compile()
    for name, area in gen.pcsa(table).items():
        force = model.actuator_gainprm[model.actuator(f"{name}_{side}").id][2]
        assert force == pytest.approx(area * sigma, abs=0.06), name


def test_elbow_and_rotation_muscles_are_left_out() -> None:
    assert not {"BRD", "PT", "PQ", "SUP"} & set(_script().pcsa(_script().load_table()))


def test_recorded_fit_is_the_minimum_of_its_curves() -> None:
    gen = _script()
    table = gen.load_table()
    fit = table["fit"]
    value = gen.weighted_fit(fit["result"]["curves"], fit["grid_N_per_cm2"], table["targets"], fit["fit_targets"])
    assert round(value) == table["value_N_per_cm2"]["value"]


def test_index_abduction_measure_rejects_a_missing_head() -> None:
    """Negative control: the moment counts both heads."""
    gen = _script()
    model = myo_sim.load_spec("myoarm_r").compile()
    data = gen._at(model, 0.0)
    full = gen._moment(model, data, "mcp2_abduction_r", 1.0, ("DI1d", "DI1s"))
    assert gen._moment(model, data, "mcp2_abduction_r", 1.0, ("DI1s",)) < 0.8 * full
