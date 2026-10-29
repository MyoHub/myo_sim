"""Lumbrical junction nodes (S6): each lumbrical arises from a slide on its FDP tendon, tied by a tendon equality.

- The generated node blocks, ties and helper tendons are what the script writes (``test_passive_thenar.py``
  covers the blocks as a whole); here: every model with a hand carries four nodes per hand, mirrored.
- The tie holds the node on the FDP's distal length in a fist and with the lumbrical active.
- The lumbrical lengthens by about Ranney & Wells' 31 mm from the straight finger to the fist.
"""

from __future__ import annotations

from functools import cache

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.hand_data import fold_node_forces, load_intrinsic_table, place_tendon_nodes

NODES = tuple(f"node_LUM{n}" for n in (1, 2, 3, 4))


@cache
def _model(name: str) -> mujoco.MjModel:
    return myo_sim.load_spec(name).compile()


def _ties(model: mujoco.MjModel) -> list[int]:
    return [e for e in range(model.neq) if model.eq_type[e] == mujoco.mjtEq.mjEQ_TENDON]


def _residual(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    out = 0.0
    for e in _ties(model):
        t1, t2 = model.eq_obj1id[e], model.eq_obj2id[e]
        out = max(out, abs((data.ten_length[t1] - model.tendon_length0[t1]) - (data.ten_length[t2] - model.tendon_length0[t2])))
    return out


@pytest.mark.parametrize("name", ["myohand_r", "myohands", "myoarm_r", "myoarms", "myofullbody"])
def test_every_hand_carries_its_nodes_and_ties(name: str) -> None:
    model = _model(name)
    sides = ("r", "l") if name in ("myohands", "myoarms", "myofullbody") else ("r",)
    for side in sides:
        for node in NODES:
            joint = model.joint(f"{node}_{side}")
            assert joint.type[0] == mujoco.mjtJoint.mjJNT_SLIDE
            tie = model.equality(f"{node}_tie_{side}")
            assert model.tendon(tie.obj1id[0]).name == f"{node}_slide_{side}"
            assert model.tendon(tie.obj2id[0]).name == f"{node}_fdp_{side}"
    assert len(_ties(model)) == 4 * len(sides)


def test_left_nodes_mirror_the_right_ones() -> None:
    """In a fist both hands' nodes slide by the same amount (the slide axis mirrors as a translation)."""
    model = _model("myoarms")
    data = mujoco.MjData(model)
    for side in ("r", "l"):
        for digit in (2, 3, 4, 5):
            for joint, deg in ((f"mcp{digit}_flexion", 80.0), (f"pm{digit}_flexion", 100.0), (f"md{digit}_flexion", 60.0)):
                data.qpos[model.joint(f"{joint}_{side}").qposadr[0]] = np.radians(deg)
    place_tendon_nodes(model, data)
    mujoco.mj_forward(model, data)
    for node in NODES:
        right = data.qpos[model.joint(f"{node}_r").qposadr[0]]
        assert right < -0.02  # the FDP material moves proximally as the finger flexes
        assert data.qpos[model.joint(f"{node}_l").qposadr[0]] == pytest.approx(right, abs=1e-9)
        lum = node.removeprefix("node_")
        assert data.actuator_length[model.actuator(f"{lum}_l").id] == pytest.approx(
            data.actuator_length[model.actuator(f"{lum}_r").id], abs=1e-9
        )
    assert _residual(model, data) < 1e-12


def _run(levels: dict[str, float], seconds: float = 1.0) -> tuple[mujoco.MjModel, mujoco.MjData, float, float]:
    """``myoarm_r`` from rest with *levels*: final state, largest tie residual (m) and LUM1 force step (N)."""
    model = _model("myoarm_r")
    data = mujoco.MjData(model)
    ids = [model.actuator(f"{n}_r").id for n in levels]
    worst, forces = 0.0, []
    for _ in range(int(round(seconds / model.opt.timestep))):
        data.ctrl[ids] = list(levels.values())
        mujoco.mj_step(model, data)
        worst = max(worst, _residual(model, data))
        forces.append(data.actuator_force[model.actuator("LUM1_r").id])
    tail = np.array(forces[len(forces) // 2 :])
    return model, data, worst, float(np.abs(np.diff(tail)).max())


def test_tie_holds_in_a_fist() -> None:
    _, _, worst, _ = _run({f"FD{m}{d}": 1.0 for m in "SP" for d in (2, 3, 4, 5)})
    assert worst < 1e-3  # m; prototype and S6 runs: below 0.3 mm


def test_lumbrical_alone_flexes_the_mcp_and_extends_the_ips_without_chatter() -> None:
    model, data, worst, chatter = _run({"LUM1": 1.0})
    deg = {j: np.degrees(data.qpos[model.joint(f"{j}2_flexion_r").qposadr[0]]) for j in ("mcp", "pm", "md")}
    assert deg["mcp"] > 10.0 and deg["pm"] < 2.0 and deg["md"] < 2.0
    assert worst < 1e-3
    assert chatter < 0.05  # N per step at 2 ms (1e-2 kg without damping chattered by 2.8 N)


def test_lumbrical_lengthens_in_the_fist() -> None:
    """Straight finger to fist: about +31 mm (Ranney & Wells 1988, index, 25 -> 56 mm); a fixed origin shortens."""
    model = _model("myoarm_r")
    data = mujoco.MjData(model)
    place_tendon_nodes(model, data)
    mujoco.mj_forward(model, data)
    lum = model.actuator("LUM1_r").id
    straight = data.actuator_length[lum]
    for joint in ("mcp2_flexion_r", "pm2_flexion_r", "md2_flexion_r"):
        data.qpos[model.joint(joint).qposadr[0]] = model.jnt_range[model.joint(joint).id][1]
    place_tendon_nodes(model, data)
    mujoco.mj_forward(model, data)
    assert 1e3 * (data.actuator_length[lum] - straight) == pytest.approx(31.0, abs=8.0)


def test_folded_node_force_equals_the_tie_reaction() -> None:
    """Statics: a lumbrical force on its node reaches the joints as the force times the FDP's distal arms."""
    model = _model("myoarm_r")
    data = mujoco.MjData(model)
    data.qpos[model.joint("mcp2_flexion_r").qposadr[0]] = 0.5
    place_tendon_nodes(model, data)
    mujoco.mj_forward(model, data)
    qfrc = np.zeros(model.nv)
    dof = model.joint("node_LUM1_r").dofadr[0]
    qfrc[dof] = 2.0
    folded = fold_node_forces(model, data, qfrc)
    jac = np.zeros((model.ntendon, model.nv))
    mujoco.mju_sparse2dense(jac, data.ten_J, model.ten_J_rownnz, model.ten_J_rowadr, model.ten_J_colind)
    np.testing.assert_allclose(folded, 2.0 * jac[model.tendon("node_LUM1_fdp_r").id] * (np.arange(model.nv) != dof))


def test_nodes_are_tabled() -> None:
    nodes = load_intrinsic_table()["lumbrical_nodes"]["nodes"]
    assert sorted(nodes) == ["LUM1", "LUM2", "LUM3", "LUM4"]
    for node in nodes.values():
        assert np.linalg.norm(node["axis"]) == pytest.approx(1.0, abs=1e-5)
