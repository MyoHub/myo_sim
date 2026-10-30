"""Maximum power grip after McFarland et al. 2023 (``scripts/validate_hand_function.py``, ``build/data/hand_function.json``).

The fast tests re-evaluate the cached optimum of this model (the table entry whose fingerprint matches it);
``MYO_SIM_SLOW=1`` adds the optimisation and the posture sensitivity. The thumb is locked with its intrinsics at 0,
so the thenar muscles play no part. Without a matching entry the tests skip.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from functools import cache
from pathlib import Path
from types import ModuleType

import mujoco
import pytest

import myo_sim
from myo_sim.build.hand_function import load_hand_function_table

SLOW = pytest.mark.skipif(os.environ.get("MYO_SIM_SLOW") != "1", reason="grip optimisation; set MYO_SIM_SLOW=1")


def _fingerprint(model: mujoco.MjModel) -> dict:
    """What tells the tabled models apart: actuator and position counts, passive springs, the index MCP damping, EDC3's passive
    scale, the palm pad, the passive scale of the opponens pollicis and the EDC4-EDC5 junctura stiffness."""
    return {
        "nu": int(model.nu),
        "nq": int(model.nq),
        "passive_springs": any(model.tendon(t).name.startswith("passive_") for t in range(model.ntendon)),
        "mcp2_flexion_damping": round(float(model.dof_damping[model.joint("mcp2_flexion_r").dofadr[0]]), 6),
        "edc3_fpmax": round(float(model.actuator_gainprm[model.actuator("EDC3_r").id][7]), 6),
        # The dynamometer is placed against the palm, so the palm pad moves it.
        "palm_pad_m": round(float(model.geom_size[model.geom("palm_pad_r").id][2]), 6)
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_pad_r") >= 0
        else None,
        "op_fpmax": round(float(model.actuator_gainprm[model.actuator("OP_r").id][7]), 6),
        "jt45_stiffness": round(float(model.tendon_stiffness[model.tendon("lig_JT45_r").id]), 3)
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "lig_JT45_r") >= 0
        else None,
    }


def _cached_entry() -> dict | None:
    """The cached grip optimum of the importable model, or None."""
    fingerprint = _fingerprint(myo_sim.load_spec("myoarm_r").compile())
    entries = load_hand_function_table()["grip"]["cached"]["entries"]
    return next((e for e in entries if e["fingerprint"] == fingerprint), None)


CACHED = _cached_entry()
pytestmark = pytest.mark.skipif(
    CACHED is None, reason="no cached grip optimum for this model; run scripts/validate_hand_function.py"
)


@cache
def _script() -> ModuleType:
    path = Path(__file__).parents[1] / "scripts" / "validate_hand_function.py"
    spec = importlib.util.spec_from_file_location("validate_hand_function", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve their module
    spec.loader.exec_module(module)
    return module


@cache
def _grip_scene(posture: str):
    v = _script()
    table = load_hand_function_table()
    return v.grip_scene(v.myohand(), table, table["grip"]["wrist_postures_deg"][posture])


def _cached_grip(scale_flexors: float = 1.0) -> dict:
    v = _script()
    table = load_hand_function_table()
    gs = _grip_scene("self_selected")
    model = gs.scene.model
    saved = model.actuator_gainprm.copy()
    for name in gs.scene.hand.muscles["FDS"] + gs.scene.hand.muscles["FDP"]:
        model.actuator_gainprm[model.actuator(name).id, 2] *= scale_flexors
    try:
        return v.grip_evaluate(gs, v.grip_levels(gs.scene.hand, CACHED["levels"]), table)
    finally:
        model.actuator_gainprm[:] = saved


def test_grip_table_is_sourced() -> None:
    table = load_hand_function_table()
    assert table["grip"]["source"] in table["sources"]
    assert "justification" in table["grip"]["tolerances"]
    assert table["sources"]["mcfarland2023"]["doi"] == "10.1109/TBME.2022.3217722"
    assert table["grip"]["tolerances"]["grip_N"] == pytest.approx(
        [table["grip"]["targets"]["experiment_N"][0] - table["grip"]["targets"]["experiment_N"][1], 533.5]
    )


def test_grip_at_the_cached_optimum() -> None:
    table = load_hand_function_table()
    tol, targets = table["grip"]["tolerances"], table["grip"]["targets"]
    r = _cached_grip()
    assert tol["grip_N"][0] <= r["grip"] <= tol["grip_N"][1]
    assert r["grip"] == pytest.approx(CACHED["grip_N"], rel=0.02)
    for digit, share in targets["digit_share"].items():  # measured shares, mean +- 1 SD (Kong et al. 2011)
        band = tol["digit_share_sd_band"] * targets["digit_share_sd"][digit]
        assert abs(r["share"][int(digit)] - share) <= band, (digit, r["share"])
    assert max(abs(m) for m in r["wrist_lock_Nm"]) <= tol["max_wrist_lock_Nm"]
    assert r["penetration_m"] <= tol["max_penetration_m"]


def test_grip_tolerance_rejects_weak_flexors() -> None:
    assert _cached_grip(scale_flexors=0.5)["grip"] < load_hand_function_table()["grip"]["tolerances"]["grip_N"][0]


@SLOW
def test_grip_optimum_and_posture_sensitivity() -> None:
    v = _script()
    table = load_hand_function_table()
    tol = table["grip"]["tolerances"]
    grip = {p: v.optimise_grip(_grip_scene(p), table, balance=True)["grip"] for p in table["grip"]["wrist_postures_deg"]}
    assert grip["self_selected"] == pytest.approx(CACHED["grip_N"], rel=0.05)
    drop = {p: 1.0 - grip[p] / grip["self_selected"] for p in ("neutral", "flexed")}
    assert all(tol["posture_drop"][0] <= d <= tol["posture_drop"][1] for d in drop.values()), drop
    assert drop["flexed"] >= drop["neutral"] - 0.05, drop
