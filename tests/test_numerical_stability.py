"""The muscles stay stable under Euler at the model's own 2 ms step, at every activation (``myo_sim/build/stability.py``).

A period-two oscillation of the wrist and digit joints appears when the force-velocity damping of the muscles exceeds the
explicit limit. The bounds come from measured values: over these 2 s the worst joint reverses on at most 3.5 % of the steps
(5.7 % over random thumb subsets), all of it while the hand closes on itself, against 39-100 % before the fix.
"""

from __future__ import annotations

import json
import re

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.stability import alternation_share, force_velocity_rate, muscle_ids

MODELS = ("myohand_r", "myoarm_r")
LEVELS = (0.0, 0.05, 0.1, 0.3, 0.5, 1.0)
STEPS = 1000  # 2 s
MAX_SHARE = 0.10
GROUPS = {
    "thumb_extrinsic": r"(FPL|EPL|EPB|APL)_",
    "thenar": r"(OP|APB|FPBs|FPBd|AdPo|AdPt)_",
    "finger_flexors": r"(FDS|FDP)\d_",
    "finger_extensors": r"(EDC\d|EDM|EIP)_",
    "intrinsics": r"(DI\d[ds]?|PI\d|LUM\d)_",
    "hypothenar": r"(ADM|FDM|ODM)_",
    "wrist": r"(FCU|FCR|PL|ECRL|ECRB|ECU)_",
    "forearm_rotation": r"(PT|PQ|SUP)_",
}
THUMB_MUSCLES = r"(FPL|EPL|EPB|APL|OP|APB|FPBs|FPBd|AdPo|AdPt|DI1s|DI1d)_"
THUMB_JOINTS = ("cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r", "ip_flexion_r")


def _mask(model: mujoco.MjModel, pattern: str) -> np.ndarray:
    return np.array([re.match(pattern, model.actuator(a).name) is not None for a in range(model.nu)])


def _worst(model: mujoco.MjModel, ctrl: np.ndarray) -> tuple[float, str]:
    data = mujoco.MjData(model)
    data.ctrl[:] = ctrl
    share = alternation_share(model, data, STEPS)
    dof = int(np.argmax(share))
    return float(share[dof]), model.joint(int(model.dof_jntid[dof])).name


@pytest.fixture(scope="module", params=MODELS)
def model(request: pytest.FixtureRequest) -> mujoco.MjModel:
    return myo_sim.load_spec(request.param).compile()


@pytest.mark.parametrize("level", LEVELS)
def test_no_oscillation_with_all_muscles(model: mujoco.MjModel, level: float) -> None:
    share, joint = _worst(model, np.full(model.nu, level))
    assert share <= MAX_SHARE, f"activation {level}: {joint} reverses on {share:.2f} of the steps"


@pytest.mark.parametrize("group", GROUPS)
@pytest.mark.parametrize("level", (0.3, 1.0))
def test_no_oscillation_with_one_muscle_group(model: mujoco.MjModel, group: str, level: float) -> None:
    share, joint = _worst(model, level * _mask(model, GROUPS[group]))
    assert share <= MAX_SHARE, f"{group} at {level}: {joint} reverses on {share:.2f} of the steps"


@pytest.mark.parametrize("seed", range(6))
def test_no_oscillation_with_thumb_subsets_at_full_activation(seed: int) -> None:
    """Random thumb poses and random subsets of the thumb muscles at activation 1: the case that sets the CMC armature."""
    model = myo_sim.load_spec("myohand_r").compile()
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(model)
    for name in THUMB_JOINTS:
        data.qpos[model.joint(name).qposadr[0]] = rng.uniform(*model.joint(name).range)
    ids = np.flatnonzero(_mask(model, THUMB_MUSCLES))
    data.ctrl[ids[rng.random(len(ids)) < 0.25 + 0.75 * rng.random()]] = 1.0
    share = alternation_share(model, data, STEPS)
    assert share.max() <= MAX_SHARE, model.joint(int(model.dof_jntid[int(np.argmax(share))])).name


def test_force_velocity_rate_below_the_limit(model: mujoco.MjModel) -> None:
    """All muscles at 0.3 stay below the explicit limit in every sampled pose; at 1 the thumb CMC exceeds it (4.5, wiki page)."""
    hinges = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and model.jnt_limited[j]]
    rng = np.random.default_rng(0)
    rates = []
    for sample in range(20):
        data = mujoco.MjData(model)
        if sample:
            for j in hinges:
                if model.joint(j).name.startswith(("cmc", "mp_", "ip_", "mcp", "pm", "md", "flexion", "deviation")):
                    data.qpos[model.jnt_qposadr[j]] = rng.uniform(*model.jnt_range[j])
        mujoco.mj_forward(model, data)
        rates.append(force_velocity_rate(model, data, np.ones(model.nu)))
    assert max(r for r, _ in rates) * 0.3 < 2.0
    assert max(r for r, _ in rates) < 5.0


def test_negative_control_the_old_force_velocity_slope_oscillates() -> None:
    """MuJoCo's default vmax and the class armature bring the oscillation back, so the measures above can fail."""
    model = myo_sim.load_spec("myohand_r").compile()
    for a in muscle_ids(model):
        model.actuator_gainprm[a, 6] = model.actuator_biasprm[a, 6] = 1.5
    for name in ("cmc_flexion_r", "cmc_abduction_r"):
        model.dof_armature[model.joint(name).dofadr[0]] = 1e-4
    share, joint = _worst(model, np.full(model.nu, 0.3))
    assert share > 2 * MAX_SHARE, joint
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert force_velocity_rate(model, data, np.full(model.nu, 0.3))[0] > 5.0


def test_parameters_hold_their_tabled_values_on_both_sides() -> None:
    root = myo_sim.MODELS_DIR.parent / "build" / "data"
    vmax = json.loads((root / "muscle_passive.json").read_text())["force_velocity"]["vmax"]["value"]
    rows = {r["joint"]: r for r in json.loads((root / "passive_joint_moments.json").read_text())["joints"]}
    model = myo_sim.load_spec("myoarms").compile()
    assert np.all(model.actuator_gainprm[muscle_ids(model), 6] == vmax)
    for side in "rl":
        for row, joint in (("thumb_cmc_flexion", "cmc_flexion"), ("thumb_cmc_abduction", "cmc_abduction")):
            assert model.dof_armature[model.joint(f"{joint}_{side}").dofadr[0]] == rows[row]["armature"]["value"]
