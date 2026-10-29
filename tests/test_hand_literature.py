"""The literature comparison page is generated from its data file and stays current with the model.

``docs/wiki/hand-literature-comparison.md`` is written by ``scripts/hand_literature_comparison.py`` from
``build/data/hand_literature.json``. The geometric values of the latest recorded step (fingertip and palm
soft tissue, active fist) are re-measured here on the importable model; the hand-function values (grip,
pinch, tenodesis) are covered by their own tests.
"""

from __future__ import annotations

import importlib.util
import sys
from functools import cache
from pathlib import Path
from types import ModuleType

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "hand_literature_comparison.py"


@cache
def _script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("hand_literature_comparison", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules["hand_literature_comparison"] = module
    spec.loader.exec_module(module)
    return module


def test_wiki_page_matches_the_data_file() -> None:
    script = _script()
    assert script.wiki_block() == script.render(script.load_data())


def test_every_literature_value_cites_a_known_source() -> None:
    data = _script().load_data()
    for key, quantity in data["quantities"].items():
        assert quantity["literature"], key
        for entry in quantity["literature"]:
            assert entry["source"] is None or entry["source"] in data["sources"], key
            assert entry["protocol"], key
    for step in data["steps"]:
        assert set(step["values"]) <= set(data["quantities"]), step["id"]


def _close(recorded: object, measured: object, tol: float) -> bool:
    if isinstance(recorded, dict):
        return (
            isinstance(measured, dict)
            and recorded.keys() == measured.keys()
            and all(_close(recorded[k], measured[k], tol) for k in recorded)
        )
    if isinstance(recorded, str) or recorded is None:
        return recorded == measured
    return abs(float(recorded) - float(measured)) <= tol


@pytest.mark.parametrize(
    "key,tol",
    [("fingertip_pad_thickness", 0.02), ("palm_pad_thickness", 0.02), ("fist_tip_to_palm", 0.1), ("fist_max_penetration", 0.1)],
)
def test_latest_step_matches_the_current_model(key: str, tol: float) -> None:
    script = _script()
    latest = script.load_data()["steps"][-1]["values"]
    measured = script._jsonable(_measured())
    assert _close(latest[key], measured[key], tol), f"{key}: recorded {latest[key]}, model {measured[key]}; rerun the script"


@cache
def _measured() -> dict:
    return _script().measure()


def test_fingertip_measurement_rejects_a_thinner_pad() -> None:
    """Negative control: moving the index fingertip's skin geoms 1 mm towards the bone thins it by about 1 mm."""
    import mujoco

    import myo_sim

    script = _script()
    model = myo_sim.load_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    before = script.fingertip_soft_tissue_mm(model, data, "distph2_r")
    geoms = script._body_geoms(model, "distph2_r")
    pad = next(g for g in geoms if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_ELLIPSOID)
    bone = next(g for g in geoms if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH)
    shift = data.xmat[model.geom_bodyid[pad]].reshape(3, 3).T @ script._pad_axis(model, data, pad, bone)
    for g in geoms:
        if script._collision(model, g):
            model.geom_pos[g] -= 0.001 * shift
    mujoco.mj_forward(model, data)
    after = script.fingertip_soft_tissue_mm(model, data, "distph2_r")
    assert after == pytest.approx(before - 1.0, abs=0.2)
