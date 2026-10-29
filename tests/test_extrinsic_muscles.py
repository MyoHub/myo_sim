"""Extrinsic-muscle revisions (S7, ``build/data/extrinsic_muscles.json``).

- The generated blocks are what ``scripts/generate_extrinsic_muscles.py`` writes.
- Juncturae: every hand has the EDC3-5 nodes, their ties and the two links, mirrored left to right; the links
  carry no force at q0 or in the relaxed resting posture, and reproduce the recorded traction transfers.
- FCU: neutral moment arms within Horii 1993's bands and independent of forearm rotation.
- FDS5: force as derived in the table.
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
from myo_sim.build.compose import RIGHT_ARM_ASSETS_XML, RIGHT_ARM_CHAIN_XML, RIGHT_ARM_TENDONS_XML
from myo_sim.build.hand_data import place_tendon_nodes, read_generated_block

SCRIPT = Path(__file__).parents[1] / "scripts" / "generate_extrinsic_muscles.py"
NODES = ("node_JT3", "node_JT4", "node_JT5")
LINKS = ("lig_JT34", "lig_JT45")


@cache
def _script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("generate_extrinsic_muscles", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@cache
def _model(name: str) -> mujoco.MjModel:
    return myo_sim.load_spec(name).compile()


def _link_force(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    t = model.tendon(name).id
    return float(model.tendon_stiffness[t] * max(0.0, data.ten_length[t] - model.tendon_lengthspring[t][1]))


def test_generated_blocks_are_what_the_script_writes() -> None:
    gen = _script()
    table = gen.load_table()
    gen.fds5_force(table)
    name = gen.SCRIPT
    for node_name, node in table["juncturae"]["nodes"].items():
        assert read_generated_block(RIGHT_ARM_CHAIN_XML, name, f"node {node_name}") == gen.chain_node_block(node_name, node)
    assert read_generated_block(RIGHT_ARM_CHAIN_XML, name, "FCU hamate") == gen.fcu_block(table)
    assert read_generated_block(RIGHT_ARM_ASSETS_XML, name, "juncturae ties") == gen.equality_block(table)
    assert read_generated_block(RIGHT_ARM_TENDONS_XML, name) == gen.tendon_block(table)


@pytest.mark.parametrize("name", ["myohand_r", "myohands", "myoarm_r", "myoarms", "myofullbody"])
def test_every_hand_carries_the_juncturae(name: str) -> None:
    model = _model(name)
    sides = ("r", "l") if name in ("myohands", "myoarms", "myofullbody") else ("r",)
    for side in sides:
        for node in NODES:
            assert model.joint(f"{node}_{side}").type[0] == mujoco.mjtJoint.mjJNT_SLIDE
            tie = model.equality(f"{node}_tie_{side}")
            assert model.tendon(tie.obj2id[0]).name == f"{node}_edc_{side}"
        for link in LINKS:
            assert model.tendon_stiffness[model.tendon(f"{link}_{side}").id] > 0.0


def test_links_are_slack_at_rest_and_mirror_left_to_right() -> None:
    gen = _script()
    model = _model("myoarms")
    data = mujoco.MjData(model)
    for q in (0.0, 1.0):  # q0, then the relaxed resting posture
        data.qpos[:] = model.qpos0
        for side in ("r", "l"):
            for d in (2, 3, 4, 5):
                for j, deg in gen.REST_DEG.items():
                    data.qpos[model.joint(f"{j}{d}_flexion_{side}").qposadr[0]] = q * np.radians(deg)
        place_tendon_nodes(model, data)
        mujoco.mj_forward(model, data)
        for link in LINKS:
            assert _link_force(model, data, f"{link}_r") == pytest.approx(0.0, abs=1e-6)
            r, left = model.tendon(f"{link}_r").id, model.tendon(f"{link}_l").id
            assert data.ten_length[r] == pytest.approx(data.ten_length[left], abs=1e-7)


def test_links_reproduce_the_recorded_traction_transfer() -> None:
    gen = _script()
    table = gen.load_table()
    result = table["juncturae"]["fit"]["result"]["with_links_deg"]["EDC4_traction"]
    stiffness = {k: v["stiffness_N_per_m"]["value"] for k, v in table["juncturae"]["links"].items()}
    values = gen.traction_transfer(stiffness, "EDC4")
    for joint, recorded in result.items():
        assert values[joint] == pytest.approx(recorded, abs=0.5)
    assert values["mcp5_flexion"] > 15.0  # von Schroeder & Botte: junctura share 22 deg


def test_fcu_moment_arms_match_horii_and_ignore_forearm_rotation() -> None:
    gen = _script()
    model = _model("myoarm_r")
    target = gen.load_table()["fcu"]["targets"]
    arms = [gen.fcu_arms(model, {"pro_sup_r": np.radians(deg)}) for deg in (-90.0, 0.0, 90.0)]
    for flexion, ulnar in arms:
        assert abs(flexion - target["flexion_mm"]["mean"]) <= target["flexion_mm"]["sd"]
        assert abs(ulnar - target["ulnar_mm"]["mean"]) <= target["ulnar_mm"]["sd"]
    assert np.ptp([a[0] for a in arms]) < 0.1


def test_fds5_force_follows_its_group() -> None:
    gen = _script()
    table = gen.load_table()
    gen.fds5_force(table)
    model = _model("myoarm_r")
    force = model.actuator_gainprm[model.actuator("FDS5_r").id][2]
    assert force == pytest.approx(table["fds5"]["force_N"]["value"])
    assert force / model.actuator_gainprm[model.actuator("FDS4_r").id][2] == pytest.approx(0.40 / 1.61, abs=0.05)
