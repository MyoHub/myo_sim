"""Smoke test: every composed model must compile and step without going non-finite.

Compiling proves the XML is well-formed; this steps the dynamics briefly to catch
models that compile but blow up on the first integration step (bad inertia,
degenerate constraints, exploding contacts).
"""

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from myo_sim import load_model
from myo_sim.build.compose import MODEL_REGISTRY

STEPS = 50


@pytest.mark.parametrize("name", sorted(MODEL_REGISTRY))
def test_model_steps_without_going_nonfinite(name):
    model = load_model(name)
    data = mujoco.MjData(model)

    for _ in range(STEPS):
        mujoco.mj_step(model, data)

    assert np.all(np.isfinite(data.qpos)), f"{name}: non-finite qpos after {STEPS} steps"
    assert np.all(np.isfinite(data.qvel)), f"{name}: non-finite qvel after {STEPS} steps"
    assert np.all(np.isfinite(data.qacc)), f"{name}: non-finite qacc after {STEPS} steps"


# Random-activation rollouts of the full body. "biclong_elbow_graze" starts from a state reached in
# such a rollout, where the right BIClong path grazed its elbow wrap sphere with the elbow past its
# extension limit and MuJoCo < 3.16 returned a NaN tendon length (fixed by taking that wrap out of the path).
ROLLOUT_STEPS = 500
CTRL_HOLD = 100
ROLLOUT_SEEDS = (0, 1, 2)
GRAZE_STATE = Path(__file__).parent / "data" / "myofullbody_biclong_elbow_graze.json"


@pytest.fixture(scope="module")
def fullbody():
    return load_model("myofullbody")


def _set_state(model: mujoco.MjModel, data: mujoco.MjData, path: Path) -> None:
    """Load a name-keyed qpos / qvel / act state written from a myofullbody rollout."""
    state = json.loads(path.read_text())
    for name, values in state["qpos"].items():
        joint = model.joint(name)
        data.qpos[joint.qposadr[0] : joint.qposadr[0] + len(values)] = values
    for name, values in state["qvel"].items():
        joint = model.joint(name)
        data.qvel[joint.dofadr[0] : joint.dofadr[0] + len(values)] = values
    for name, value in state["act"].items():
        data.act[model.actuator_actadr[model.actuator(name).id]] = value
    data.time = state["time"]


@pytest.mark.parametrize("seed", ROLLOUT_SEEDS)
@pytest.mark.parametrize("start", ["qpos0", "biclong_elbow_graze"])
def test_myofullbody_random_activation_stays_finite(fullbody, start, seed):
    model = fullbody
    data = mujoco.MjData(model)
    if start == "biclong_elbow_graze":
        _set_state(model, data, GRAZE_STATE)
    mujoco.mj_forward(model, data)
    assert np.all(np.isfinite(data.ten_length)), f"{start}: non-finite tendon length before stepping"

    rng = np.random.default_rng(seed)
    for step in range(ROLLOUT_STEPS):
        if step % CTRL_HOLD == 0:
            data.ctrl[:] = (rng.random(model.nu) < 0.3) * rng.random(model.nu)
        mujoco.mj_step(model, data)
        # MuJoCo resets the state after a bad qacc, so check its warning counter, not only qpos.
        assert data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0, f"{start}, seed {seed}: bad qacc at step {step}"
        assert np.all(np.isfinite(data.ten_length)), f"{start}, seed {seed}: non-finite tendon length at step {step}"
