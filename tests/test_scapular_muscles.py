"""Scapular muscles and cuff sub-regions after Seth et al. 2019: XML equals the table, cervical origins, mirroring."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.hand_data import blocks_match, read_generated_block

ROOT = Path(__file__).resolve().parents[1]
TABLE = json.loads((ROOT / "myo_sim" / "build" / "data" / "scapular_muscles.json").read_text())


def _script():
    spec = importlib.util.spec_from_file_location(
        "generate_scapular_muscles", ROOT / "scripts" / "generate_scapular_muscles.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_xml_blocks_are_what_the_table_renders() -> None:
    gen = _script()
    for (path, label), block in gen.blocks(TABLE).items():
        assert blocks_match(read_generated_block(path, gen.SCRIPT, label), block), f"{path.name} ({label}): rerun --write"


def test_forces_are_pcsa_times_the_specific_tension() -> None:
    tension = TABLE["specific_tension_N_per_cm2"]
    model = myo_sim.load_model("myoarm_r")
    for row in TABLE["muscles"]:
        assert row["force_N"] == pytest.approx(row["pcsa_cm2"] * tension, abs=0.1)
        assert model.actuator_gainprm[model.actuator(f"{row['name']}_r").id, 2] == pytest.approx(row["force_N"])
        lo, hi = row["fibre_length_seth_motions"]
        assert 0.5 - 1e-3 <= lo and hi <= 1.5 + 5e-3, row["name"]


@pytest.mark.parametrize(("name", "neck"), [("myoarms", "rigid"), ("myofullbody", "hyoid")])
def test_cervical_origins_and_mirroring(name: str, neck: str) -> None:
    """Origins on the cervical spine sit on the vertebra (muscular neck) or the rigid neck, at Seth's point at rest;
    every new site of the left arm is the reflection of the right one."""
    model = myo_sim.load_model(name)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    chest = model.body("chest_r").id
    rc, pc = data.xmat[chest].reshape(3, 3), data.xpos[chest]
    flip = np.diag([1.0, 1.0, -1.0])
    for row in TABLE["muscles"]:
        for k, end in ((1, row["origin"]), (2, row["insertion"])):
            right, left = model.site(f"{row['name']}-P{k}_r").id, model.site(f"{row['name']}-P{k}_l").id
            p_r = rc.T @ (data.site_xpos[right] - pc)
            p_l = rc.T @ (data.site_xpos[left] - pc)
            np.testing.assert_allclose(p_l, flip @ p_r, atol=1e-6, err_msg=f"{name} {row['name']}-P{k}")
            if neck in end:
                body = end[neck]["body"]
                assert model.body(model.site_bodyid[right]).name == _script().COMPOSED.get(body, body)
                np.testing.assert_allclose(p_r, end["pos"], atol=1e-6)
    assert all(model.actuator(f"{row['name']}_{s}").id >= 0 for row in TABLE["muscles"] for s in "rl")
