"""Fourth and fifth carpometacarpal joints: XML from the table, coupling, crossing muscles, both hands."""

from __future__ import annotations

import copy
import importlib.util
from functools import cache
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build import compose
from myo_sim.build.hand_data import read_generated_block

MODELS = ("myohand_r", "myohands", "myoarm_r", "myoarms", "myofullbody")
JOINTS = ("cmc4_flexion", "cmc5_flexion")
# dL/dq sign on CMC4 / CMC5 (negative = flexor; 0 = does not cross): the crossing muscles and their anatomy.
CROSSING = {
    "FDS4_r_tendon": (-1, 0),
    "FDP4_r_tendon": (-1, 0),
    "EDC4_r_tendon": (1, 0),
    "FDS5_r_tendon": (0, -1),
    "FDP5_r_tendon": (0, -1),
    "EDC5_r_tendon": (0, 1),
    "EDM_r_tendon": (0, 1),
    "ECU_r_tendon": (0, 1),
    "FCU_r_tendon": (0, -1),
    "PI2_r_tendon": (0, 0),
    "DI4_r_tendon": (0, 0),
    "PI3_r_tendon": (0, 0),
    "LUM4_r_tendon": (0, 0),
    "ADM_r_tendon": (0, -1),
    "FDM_r_tendon": (0, -1),
    "ODM_r_tendon": (0, -1),
}


@cache
def _script() -> ModuleType:
    path = Path(__file__).parents[1] / "scripts" / "fit_cmc_joints.py"
    spec = importlib.util.spec_from_file_location("fit_cmc_joints", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_xml_is_what_the_script_writes() -> None:
    script = _script()
    table = script.load_table()
    text = compose.RIGHT_ARM_CHAIN_XML.read_text()
    assert script.chain_with_joints(text, table) == text
    assert read_generated_block(script.ASSETS_XML, script.SCRIPT) == script.equality_block(table)


def test_fit_reproduces_the_table() -> None:
    script = _script()
    table = script.load_table()
    refit = copy.deepcopy(table)
    script.fit(refit)
    for old, new in zip(table["joints"], refit["joints"]):
        np.testing.assert_allclose(new["fitted"]["pos"], old["fitted"]["pos"], atol=2e-6)
        np.testing.assert_allclose(new["fitted"]["axis"], old["fitted"]["axis"], atol=2e-6)
    for old, new in zip(table["site_moves"], refit["site_moves"]):
        np.testing.assert_allclose(new["pos"], old["pos"], atol=2e-6)


def test_table_is_sourced() -> None:
    table = _script().load_table()
    for joint in table["joints"]:
        assert joint["axis"]["source"] in table["sources"] and joint["range_deg"]["source"] in table["sources"]
        assert joint["damping"]["source"] in table["sources"]
    coupling = table["coupling"]
    assert all(s in table["sources"] for s in coupling["sources"])
    # The CMC5 range is the CMC4 range times the ratio, so joint limits never fight the equality.
    cmc4, cmc5 = (j["range_deg"]["value"] for j in table["joints"])
    np.testing.assert_allclose(cmc5, np.round(np.multiply(cmc4, coupling["ratio"]), 1), atol=0.05)


@pytest.mark.parametrize("name", MODELS)
def test_models_carry_the_joints_and_the_coupling(name: str) -> None:
    model = myo_sim.load_model(name)
    sides = ("r",) if name.endswith("_r") else ("r", "l")
    for side in sides:
        for joint in JOINTS:
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{joint}_{side}") >= 0
        eq = model.equality(f"cmc45_coupling_{side}")
        assert model.eq_obj1id[eq.id] == model.joint(f"cmc5_flexion_{side}").id
        assert model.eq_obj2id[eq.id] == model.joint(f"cmc4_flexion_{side}").id
        np.testing.assert_allclose(model.eq_data[eq.id][:5], [0.0, 2.06, 0.0, 0.0, 0.0])


def _arms(model: mujoco.MjModel, tendon: str) -> tuple[float, float]:
    data = mujoco.MjData(model)

    def length(q4: float, q5: float) -> float:
        data.qpos[:] = model.qpos0
        data.qpos[model.joint("cmc4_flexion_r").qposadr[0]] = q4
        data.qpos[model.joint("cmc5_flexion_r").qposadr[0]] = q5
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_tendon(model, data)
        return float(data.ten_length[model.tendon(tendon).id])

    h = 1e-6
    return (length(h, 0) - length(-h, 0)) / (2 * h), (length(0, h) - length(0, -h)) / (2 * h)


def test_crossing_muscles_act_as_anatomy_says() -> None:
    """Flexors flex and extensors extend CMC4/5; the moved interosseous origins cross neither joint."""
    model = myo_sim.load_model("myoarm_r")
    for tendon, signs in CROSSING.items():
        for arm, sign in zip(_arms(model, tendon), signs):
            if sign == 0:
                assert abs(arm) < 1e-9, tendon
            else:
                assert sign * arm > 1e-3, (tendon, arm)


def test_positive_angles_flex_and_the_coupling_holds() -> None:
    model = myo_sim.load_model("myoarm_r")
    data = mujoco.MjData(model)
    flexors = [model.actuator(n).id for n in ("FDS5_r", "FDP5_r", "ADM_r")]
    data.ctrl[flexors] = 1.0
    for _ in range(500):
        mujoco.mj_step(model, data)
    q4 = data.qpos[model.joint("cmc4_flexion_r").qposadr[0]]
    q5 = data.qpos[model.joint("cmc5_flexion_r").qposadr[0]]
    assert q5 > 0.05  # the little-finger flexors cup the palm
    assert q5 == pytest.approx(2.06 * q4, abs=2e-3)


def test_relaxed_hand_rests_near_neutral() -> None:
    """With the end-range springs the muscles' passive tension no longer cups the relaxed hand."""
    model = myo_sim.load_model("myohands")
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    for _ in range(1500):
        mujoco.mj_step(model, data)
    for side in ("r", "l"):
        for joint in JOINTS:
            assert abs(data.qpos[model.joint(f"{joint}_{side}").qposadr[0]]) < np.radians(2.0)
