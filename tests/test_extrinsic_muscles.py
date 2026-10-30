"""Extrinsic-muscle revisions (S7, ``build/data/extrinsic_muscles.json``).

- The generated blocks are what ``scripts/generate_extrinsic_muscles.py`` writes.
- Juncturae: every hand has the EDC3-5 nodes, their ties and the two links, mirrored left to right; each node
  moves with its EDC over the MCP range (a helper from the node's rest point, which misses the MCP wrap, does
  not); the links carry no force at q0 or in the relaxed resting posture, and reproduce the recorded traction
  transfers.
- FCU: neutral moment arms within Horii 1993's bands and independent of forearm rotation.
- FDS5: force as derived in the table.
- Wrap sides (EDC2, EDC4, EDC5, EDM, ECRB, FCR, PQ): each side site lies outside its geom, as tabled, so
  MuJoCo and mujoco-warp resolve the wraps alike (an inside wrap, side site at the centre, differs between them).
- ECU: its groove via point is on the ulna in every arm, its moment arms reproduce the recorded fit and change
  with forearm rotation as measured; its path has no wrap geom left.
- The forearm rotation axis passes through the radial and ulnar heads of the meshes.
"""

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
    assert read_generated_block(RIGHT_ARM_CHAIN_XML, name, "ECU groove") == gen.ecu_block(table)
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


def _node_excursion_error(model: mujoco.MjModel, node: str) -> float:
    """Largest |node slide - EDC excursion| (m) over the MCP range, IPs straight and at von Schroeder & Botte's rest.

    Wrist and CMC at 0, so the EDC's length change is all distal of P4: a rigid tendon's material moves by it.
    """
    gen = _script()
    digit = int(node[-1])
    data = mujoco.MjData(model)
    slide = model.joint(f"node_{node}_r").qposadr[0]
    edc = model.tendon(f"EDC{digit}_r_tendon").id
    mcp = model.joint(f"mcp{digit}_flexion_r")
    worst = 0.0
    for ip in (0.0, 1.0):
        for q in np.linspace(*model.jnt_range[mcp.id], 13):
            data.qpos[:] = model.qpos0
            data.qpos[mcp.qposadr[0]] = q
            for j in ("pm", "md"):
                data.qpos[model.joint(f"{j}{digit}_flexion_r").qposadr[0]] = ip * np.radians(gen.REST_DEG[j])
            place_tendon_nodes(model, data)
            mujoco.mj_tendon(model, data)
            s = data.qpos[slide] - model.qpos0[slide]
            worst = max(worst, abs(s - (data.ten_length[edc] - model.tendon_length0[edc])))
    return worst


@pytest.mark.parametrize("node", ["JT3", "JT4", "JT5"])
def test_nodes_move_with_their_tendon_over_the_mcp_range(node: str) -> None:
    assert _node_excursion_error(_model("myohand_r"), node) < 1e-6


@pytest.mark.parametrize("node", ["JT3", "JT5"])
def test_node_excursion_check_catches_a_helper_from_the_rest_point(node: str) -> None:
    """Negative control: the S7 helper started at the node's rest point near the MC head (the link site at slide 0)."""
    model = copy.deepcopy(_model("myohand_r"))
    helper = model.tendon(f"node_{node}_edc_r").id
    model.wrap_objid[model.tendon_adr[helper]] = model.site(f"node_{node}_link_r").id
    data = mujoco.MjData(model)
    mujoco.mj_tendon(model, data)  # qpos0: the link site at the rest point
    model.tendon_length0[helper] = data.ten_length[helper]
    assert _node_excursion_error(model, node) > 5e-3  # S7: 6-11 mm at MCP 90 deg


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


WRAP_SIDES = ("EDC2", "EDC4", "EDC5", "EDM", "ECRB", "FCR", "PQ")


@pytest.mark.parametrize("muscle", WRAP_SIDES)
def test_wrap_side_site_is_tabled_and_outside_its_geom(muscle: str) -> None:
    side = _script().load_table()["wrap_sides"]["muscles"][muscle]
    model = _model("myoarm_r")
    site, geom = model.site(side["site"]), model.geom(side["geom"])
    np.testing.assert_allclose(model.site_pos[site.id], side["pos"], atol=1e-9)
    assert model.site_bodyid[site.id] == model.geom_bodyid[geom.id] == model.body(side["body"]).id
    offset = model.site_pos[site.id] - model.geom_pos[geom.id]
    if geom.type[0] == mujoco.mjtGeom.mjGEOM_CYLINDER:  # distance from the axis
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, model.geom_quat[geom.id])
        axis = rot.reshape(3, 3)[:, 2]
        offset = offset - (offset @ axis) * axis
    assert np.linalg.norm(offset) > 1.5 * model.geom_size[geom.id][0]


def _cpu_warp_differences(model: mujoco.MjModel) -> dict[str, float]:
    """Largest length difference (m) per wrap-side muscle between MuJoCo and mujoco-warp over random arm postures."""
    mjw = pytest.importorskip("mujoco_warp")
    wp = pytest.importorskip("warp")
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    names = ("flexion_r", "deviation_r", "pro_sup_r", "elbow_flexion_r", "mcp2_flexion_r", "mcp2_abduction_r", "mcp5_flexion_r")
    joints = [model.joint(n) for n in names]
    tendons = [model.tendon(f"{m}_r_tendon").id for m in WRAP_SIDES]
    qpos, lengths = [], []
    for _ in range(128):
        data.qpos[:] = model.qpos0
        for j in joints:
            data.qpos[j.qposadr[0]] = rng.uniform(*model.jnt_range[j.id])
        mujoco.mj_fwdPosition(model, data)
        qpos.append(data.qpos.copy())
        lengths.append(data.ten_length[tendons])
    with wp.ScopedDevice("cpu"):
        model_w = mjw.put_model(model)
        data_w = mjw.put_data(model, data, nworld=len(qpos), nconmax=64, njmax=256)
        data_w.qpos.assign(wp.array(np.array(qpos, dtype=np.float32)))
        mjw.kinematics(model_w, data_w)
        mjw.com_pos(model_w, data_w)
        mjw.tendon(model_w, data_w)
        warp_lengths = data_w.ten_length.numpy()[:, tendons]
    return dict(zip(WRAP_SIDES, np.max(np.abs(warp_lengths - np.array(lengths)), axis=0).tolist()))


def test_wrap_side_lengths_agree_on_cpu_and_mujoco_warp() -> None:
    model = myo_sim.load_spec("myoarm_r").compile()
    assert max(_cpu_warp_differences(model).values()) < 1e-6
    # Negative control: each side site at its geom's centre (inside wrap) does not agree.
    table = _script().load_table()["wrap_sides"]["muscles"]
    for side in table.values():
        model.site_pos[model.site(side["site"]).id] = model.geom_pos[model.geom(side["geom"]).id]
    worst = _cpu_warp_differences(model)
    assert sum(v > 1e-5 for v in worst.values()) >= 5, worst


@pytest.mark.parametrize("name", ["myoarm_r", "myoarms", "myofullbody"])
def test_ecu_groove_is_on_the_ulna_without_a_wrap(name: str) -> None:
    model = _model(name)
    sides = ("r", "l") if name in ("myoarms", "myofullbody") else ("r",)
    for side in sides:
        assert model.body(model.site_bodyid[model.site(f"ECU-P5_{side}").id]).name == f"ulna_{side}"
        t = model.tendon(f"ECU_{side}_tendon").id
        adr, num = model.tendon_adr[t], model.tendon_num[t]
        assert all(model.wrap_type[adr : adr + num] == mujoco.mjtWrap.mjWRAP_SITE)


def test_ecu_moment_arms_follow_forearm_rotation() -> None:
    """The recorded fit is reproduced; the extension arm falls in pronation (Horii 1993, Loren 1996) and stays in
    supination, the neutral ulnar arm lies within Horii's band, and the fit beats the radius-fixed path."""
    gen = _script()
    table = gen.load_table()
    model = _model("myoarm_r")
    ext, uln = gen.ecu_rotation_arms(model, table)
    result = table["ecu"]["fit"]["result"]
    assert ext == pytest.approx(result["extension_mm"], abs=0.01)
    assert uln == pytest.approx(result["ulnar_mm"], abs=0.01)
    assert ext[1] - ext[2] > 4.0  # Horii 3.4 mm, Loren 6.3 mm
    assert abs(ext[0] - ext[1]) < 1.0  # Horii 0, Loren 0.6 mm
    assert abs(uln[1] - 17.0) <= 3.0  # Horii Table II, 17 +- 3 mm
    assert np.sqrt(np.mean(gen.ecu_residuals(model, table) ** 2)) < table["ecu"]["before"]["rms_z"]


def test_ecu_is_symmetric_left_to_right() -> None:
    model = _model("myoarms")
    data = mujoco.MjData(model)
    for deg in (-90.0, 0.0, 90.0):
        for flexion in (-0.6, 0.0, 0.6):
            data.qpos[:] = model.qpos0
            for side in ("r", "l"):
                data.qpos[model.joint(f"pro_sup_{side}").qposadr[0]] = np.radians(deg)
                data.qpos[model.joint(f"flexion_{side}").qposadr[0]] = flexion
            mujoco.mj_fwdPosition(model, data)
            r, left = model.tendon("ECU_r_tendon").id, model.tendon("ECU_l_tendon").id
            assert data.ten_length[r] == pytest.approx(data.ten_length[left], abs=1e-7)


def test_rotation_axis_passes_through_the_radial_and_ulnar_heads() -> None:
    """Within 2 mm of the radial head centre and the ulnar head centre (the meshes fit them with 1.8-2 mm rms)."""
    gen = _script()
    offsets = gen.axis_offsets(_model("myoarm_r"), gen.load_table())
    assert offsets["radial_head_mm"] < 2.0
    assert offsets["ulnar_head_mm"] < 2.0
