"""Deep hip muscles (``build/data/deep_hip_muscles.json``, ``scripts/generate_deep_hip_muscles.py``).

Part of the leg model: the generated blocks equal what the script writes, the muscles come after the other leg muscles on
both sides, their paths are mirrored, their moment arms follow OpenSim's Gait2392, the rigid-tendon map holds and
a rollout stays finite.
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
from myo_sim.build.hand_data import read_generated_block

SCRIPT = Path(__file__).parents[1] / "scripts" / "generate_deep_hip_muscles.py"
HIP = ("hip_flexion", "hip_adduction", "hip_rotation")


@cache
def _script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("generate_deep_hip_muscles", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@cache
def _legs() -> mujoco.MjModel:
    return myo_sim.load_spec("myolegs").compile()


def _table() -> dict:
    return _script().load_table()


def _tendon_length(model: mujoco.MjModel, data: mujoco.MjData, name: str, q: dict[str, float]) -> float:
    data.qpos[:] = model.qpos0
    for joint, value in q.items():
        data.qpos[model.joint(joint).qposadr[0]] = value
    mujoco.mj_kinematics(model, data)
    mujoco.mj_tendon(model, data)
    return float(data.ten_length[model.tendon(f"{name}_tendon").id])


def _moment_arm(model: mujoco.MjModel, data: mujoco.MjData, muscle: str, joint: str, h: float = 1e-5) -> float:
    plus = _tendon_length(model, data, muscle, {joint: h})
    minus = _tendon_length(model, data, muscle, {joint: -h})
    return -(plus - minus) / (2 * h)


def test_generated_blocks_are_what_the_script_writes() -> None:
    gen = _script()
    table = _table()
    pairs = [
        (gen.CHAIN_XML, gen.chain_block(table, "pelvis"), "pelvis"),
        (gen.CHAIN_XML, gen.chain_block(table, "femur", "r"), "femur_r"),
        (gen.CHAIN_XML, gen.chain_block(table, "femur", "l"), "femur_l"),
        (gen.TENDON_XML, gen.tendon_block(table), None),
        (gen.MUSCLE_XML, gen.muscle_block(table), None),
    ]
    for path, block, label in pairs:
        assert read_generated_block(path, gen.SCRIPT, label) == block, label


def test_muscles_come_after_the_other_leg_muscles() -> None:
    model = _legs()
    names = [model.actuator(i).name for i in range(model.nu)]
    new = [f"{m['name']}_{s}" for s in ("r", "l") for m in _table()["muscles"]]
    assert names[-len(new) :] == new


def test_full_body_and_part_models_carry_them() -> None:
    for name in ("myofullbody", "myolegs_abdomen"):
        model = myo_sim.load_spec(name).compile()
        assert all(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"pect_{s}") >= 0 for s in "rl")


@pytest.mark.parametrize("muscle", ("pect", "quad_fem", "gem"))
def test_moment_arms_follow_gait2392(muscle: str) -> None:
    """Neutral moment arms within the frame-transfer error of the points (11 mm) of the values OpenSim gives."""
    model = _legs()
    data = mujoco.MjData(model)
    row = next(m for m in _table()["muscles"] if m["name"] == muscle)
    arms = [1e3 * _moment_arm(model, data, f"{muscle}_r", f"{j}_r") for j in HIP]
    np.testing.assert_allclose(arms, row["gait2392_neutral_moment_arms_mm"], atol=11.0)


@pytest.mark.parametrize("muscle", ("pect", "quad_fem", "gem"))
def test_left_right_lengths_are_mirrored(muscle: str) -> None:
    model = _legs()
    data = mujoco.MjData(model)
    q = {"flexion": 0.4, "adduction": 0.2, "rotation": -0.3}
    right = _tendon_length(model, data, f"{muscle}_r", {f"hip_{k}_r": v for k, v in q.items()})
    left = _tendon_length(model, data, f"{muscle}_l", {f"hip_{k}_l": v for k, v in q.items()})
    assert right == pytest.approx(left, abs=1e-6)


def test_length_map_is_the_rigid_tendon_map_of_gait2392() -> None:
    """Normalised fibre length (L - L_ts) / L_fo, inside the force-length bounds; the fit clips slack postures, hence 0.5 %."""
    model = _legs()
    for m in _table()["muscles"]:
        a = model.actuator(f"{m['name']}_r").id
        r0, r1, lmin, lmax = model.actuator_gainprm[a, [0, 1, 4, 5]]
        l0, l1 = model.actuator_lengthrange[a]
        assert lmin < r0 < r1 < lmax
        slope = (r1 - r0) / (l1 - l0)
        assert 1.0 / slope == pytest.approx(m["optimal_fibre_m"], rel=5e-3)
        assert l0 - r0 / slope == pytest.approx(m["tendon_slack_m"], abs=5e-4)


def test_rollout_is_finite() -> None:
    model = _legs()
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    for step in range(500):
        if step % 20 == 0:
            data.ctrl[:] = rng.random(model.nu)
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
