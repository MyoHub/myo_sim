"""Numerical stability of the muscles at a model's own timestep: ``tests/test_numerical_stability.py``.

MuJoCo's Euler integrator treats joint damping implicitly but the muscles' force-velocity term explicitly. A joint mode
with damping D, inertia M and joint damping B is therefore multiplied per step by 1 - h D / (M + h B), which oscillates
with period two and grows above h D / (M + h B) = 2. The force-velocity curve flattens away from zero velocity, so the
growth saturates in a limit cycle instead of diverging. See ``docs/wiki/hand-literature-comparison.md``.
"""

from __future__ import annotations

import mujoco
import numpy as np


def muscle_ids(model: mujoco.MjModel) -> np.ndarray:
    """Actuator ids of the muscles of *model*."""
    return np.flatnonzero(model.actuator_gaintype == int(mujoco.mjtGain.mjGAIN_MUSCLE))


def force_velocity_damping(model: mujoco.MjModel, data: mujoco.MjData, activation: np.ndarray) -> np.ndarray:
    """Tendon-space damping of each actuator at zero velocity at the current lengths.

    Args:
        model: The model.
        data: State with up-to-date actuator lengths.
        activation: Activation per actuator.

    Returns:
        ``-dF/dv`` per actuator (N s/m), 0 for non-muscles.
    """
    damping = np.zeros(model.nu)
    for a in muscle_ids(model):
        prm, lr, acc0 = model.actuator_gainprm[a][:9], model.actuator_lengthrange[a], model.actuator_acc0[a]
        length = data.actuator_length[a]
        speed = 1e-5 * (lr[1] - lr[0]) / (prm[1] - prm[0]) * prm[6]
        slope = (mujoco.mju_muscleGain(length, speed, lr, acc0, prm) - mujoco.mju_muscleGain(length, -speed, lr, acc0, prm)) / (
            2.0 * speed
        )
        damping[a] = -activation[a] * slope
    return damping


def _moment_matrix(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
    return moment


def mass_matrix(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Joint-space inertia matrix of *data* with the armature on its diagonal."""
    mass, unit, column = np.zeros((model.nv, model.nv)), np.zeros(model.nv), np.zeros(model.nv)
    for i in range(model.nv):
        unit[i] = 1.0
        mujoco.mj_mulM(model, data, column, unit)
        mass[:, i] = column
        unit[i] = 0.0
    return mass


def force_velocity_rate(model: mujoco.MjModel, data: mujoco.MjData, activation: np.ndarray) -> tuple[float, str]:
    """Largest eigenvalue of h (M + h B)^-1 D, with D the muscles' force-velocity damping in joint space.

    Explicit Euler is stable below 2. Joint limits and contacts, which the solver treats implicitly, are not counted, and a
    moving muscle has a smaller slope: this bounds the state at rest in free space.

    Args:
        model: The model.
        data: State after ``mj_forward``.
        activation: Activation per actuator.

    Returns:
        The rate and the joint that dominates its mode.
    """
    moment = _moment_matrix(model, data)
    damping = moment.T @ (force_velocity_damping(model, data, activation)[:, None] * moment)
    inertia = mass_matrix(model, data) + model.opt.timestep * np.diag(model.dof_damping)
    values, vectors = np.linalg.eig(np.linalg.solve(inertia, model.opt.timestep * damping))
    i = int(np.argmax(values.real))
    dof = int(np.argmax(np.abs(vectors[:, i])))
    return float(values[i].real), model.joint(int(model.dof_jntid[dof])).name


def alternation_share(model: mujoco.MjModel, data: mujoco.MjData, steps: int, threshold: float = 1.0) -> np.ndarray:
    """Step *data* and return, per degree of freedom, the share of steps with a period-two velocity oscillation.

    Args:
        model: The model.
        data: Start state with its controls set; it is stepped in place.
        steps: Number of steps.
        threshold: Size of the velocity change per step that counts (rad/s or m/s).

    Returns:
        Share per degree of freedom of the steps whose velocity change reverses the previous one, both above *threshold*.
    """
    velocity = np.zeros((steps, model.nv))
    for k in range(steps):
        mujoco.mj_step(model, data)
        velocity[k] = data.qvel
    change = np.diff(velocity, axis=0)
    reversal = (change[1:] * change[:-1] < 0) & (np.abs(change[1:]) > threshold) & (np.abs(change[:-1]) > threshold)
    return reversal.mean(axis=0)
