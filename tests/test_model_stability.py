"""Smoke test: every composed model must compile and step without going non-finite.

Compiling proves the XML is well-formed; this steps the dynamics briefly to catch
models that compile but blow up on the first integration step (bad inertia,
degenerate constraints, exploding contacts).
"""

import mujoco
import numpy as np
import pytest

from myo_sim.build.compose import MODEL_REGISTRY
from myo_sim import load_model


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
