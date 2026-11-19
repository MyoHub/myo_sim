# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Portable Hill muscle curves match native MuJoCo actuator forces and generalized torques."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from myo_sim.build.compose import build_spec
from myo_sim.physics.hill_muscle import (
    HillMuscleModel,
    MuscleParams,
    dense_actuator_moment,
    muscle_activation_dot,
    step_muscle_activation,
    strip_muscles_for_joint_torque_sim,
)
from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings


@pytest.mark.parametrize("model_name", ["myohand_r", "myoarm_r"])
def test_hill_muscle_exact_parity_against_mujoco(model_name: str) -> None:
    spec = build_spec(model_name)
    model = spec.compile()
    data = mujoco.MjData(model)
    rng = np.random.default_rng(7)

    couplings = JointCouplings.from_model(model)
    for j in range(model.njnt):
        if model.jnt_limited[j]:
            lo, hi = model.jnt_range[j]
            data.qpos[model.jnt_qposadr[j]] = rng.uniform(0.3 * lo + 0.7 * 0.0, 0.3 * hi + 0.7 * 0.0)
    project_joint_couplings(couplings, data.qpos)
    data.qvel[:] = rng.normal(scale=0.15, size=model.nv)
    data.act[:] = rng.uniform(0.05, 0.95, size=model.na)
    data.ctrl[:] = rng.uniform(0.0, 1.0, size=model.nu)
    mujoco.mj_forward(model, data)

    params = MuscleParams.from_model(model)
    hill = HillMuscleModel(model)

    act_dot = muscle_activation_dot(data.ctrl, data.act, params)
    np.testing.assert_allclose(act_dot, data.act_dot, atol=1e-12, rtol=1e-12)

    moment = dense_actuator_moment(model, data)
    qfrc = hill.generalized_forces(data.actuator_length, data.actuator_velocity, data.act, moment)
    np.testing.assert_allclose(qfrc, data.qfrc_actuator, atol=1e-10, rtol=1e-10)

    next_act, _ = step_muscle_activation(data.act, data.ctrl, params, model.opt.timestep, substeps=1)
    assert next_act.shape == (model.nu,)
    assert np.all(np.isfinite(next_act))


def test_strip_muscles_for_joint_torque_sim_preserves_kinematics_and_couplings() -> None:
    orig_spec = build_spec("myoarm_r")
    orig = orig_spec.compile()
    stripped_spec = strip_muscles_for_joint_torque_sim(orig_spec, add_joint_motors=True)
    stripped = stripped_spec.compile()

    assert stripped.ntendon == 0
    assert stripped.na == 0
    assert stripped.nbody == orig.nbody
    assert stripped.njnt == orig.njnt
    assert stripped.nsite == orig.nsite
    assert stripped.neq == orig.neq
    # Independent 1-DOF joints receive direct unit-gear joint motors (27 = 38 total - 11 coupled).
    assert stripped.nu == 27
    assert np.all(stripped.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)
