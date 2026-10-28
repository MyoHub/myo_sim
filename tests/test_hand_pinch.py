"""Lateral pinch after McFarland et al. 2023 (``scripts/validate_hand_function.py``, ``build/data/hand_function.json``).

Needs the thenar muscles (AdP, FPB, APB); skipped on a model without them.
"""

from __future__ import annotations

import importlib.util
import sys
from functools import cache
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.hand_function import load_hand_function_table

THENAR = ("AdP", "FPB", "APB")


@cache
def _script() -> ModuleType:
    path = Path(__file__).parents[1] / "scripts" / "validate_hand_function.py"
    spec = importlib.util.spec_from_file_location("validate_hand_function", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve their module
    spec.loader.exec_module(module)
    return module


def _has_thenar() -> bool:
    actuators = {a.name for a in myo_sim.load_spec("myoarm_r").actuators}
    return all(name in actuators for name in THENAR)


pytestmark = pytest.mark.skipif(not _has_thenar(), reason="needs the thenar muscles AdP, FPB and APB")


@cache
def _pinch(without_thenar: bool = False) -> dict:
    v = _script()
    hand = v.myohand()
    if without_thenar:
        kept = tuple(n for n in hand.muscles["thumb_intrinsic"] if n not in THENAR)
        hand = v.Hand(
            hand.load, hand.joints, hand.segments, hand.metacarpal3, hand.thumb_distal, hand.muscles | {"thumb_intrinsic": kept}
        )
    return v.pinch(hand, load_hand_function_table())


def test_pinch_table_is_sourced() -> None:
    table = load_hand_function_table()
    assert table["pinch"]["source"] in table["sources"]
    assert "justification" in table["pinch"]["tolerances"]
    assert table["sources"]["mcfarland2023"]["doi"] == "10.1109/TBME.2022.3217722"


def test_lateral_pinch_force() -> None:
    table = load_hand_function_table()
    tol = table["pinch"]["tolerances"]
    r = _pinch()
    assert r["feasible"]
    assert tol["force_N"][0] <= r["force_N"] <= tol["force_N"][1]
    assert r["off_axis"] <= table["pinch"]["off_axis_max"] + 1e-6
    assert r["sim_residual_Nm"] <= tol["max_residual_Nm"]


def test_lateral_pinch_needs_the_thenar_muscles() -> None:
    r = _pinch(without_thenar=True)
    assert not r["feasible"] or r["force_N"] < load_hand_function_table()["pinch"]["tolerances"]["force_N"][0]


def test_pinch_posture_maps_the_anatomical_angles() -> None:
    """CMC 15 deg extension / 20 deg adduction and MP / IP flexion land on MyoHand's joints with its signs."""
    v = _script()
    table = load_hand_function_table()
    scene = v.build_scene(v.myohand(), {}, table)
    data = v.start(scene, table["pinch"]["posture_deg"])
    qpos = {k: np.degrees(data.qpos[scene.qadr(k)]) for k in v.THUMB}
    assert qpos == pytest.approx({"cmc_flexion": 15.0, "cmc_abduction": -20.0, "mp_flexion": -20.0, "ip_flexion": -40.0})
    for name, (key, sign) in {"FPL": ("ip_flexion", 1), "APB": ("cmc_abduction", 1), "EPB": ("cmc_flexion", -1)}.items():
        v.set_activation(scene, data, {name: 1.0})
        mujoco.mj_forward(scene.model, data)
        assert sign * scene.joint(key)[1] * data.qfrc_actuator[scene.dof(key)] > 0, name
