"""Shared helpers for the myo_sim.diff tests: models, random states and MuJoCo reference values."""

from __future__ import annotations

import functools
import warnings
from pathlib import Path

import mujoco
import numpy as np

import myo_sim

FEATURE_XML = Path(__file__).parent / "data" / "diff_features.xml"
# registered models the parity tests cover, plus one composition
MODELS = ("myohand_r", "myoarm_r", "myolegs", "myofullbody", "myoarms")


@functools.lru_cache(maxsize=None)
def load(name: str) -> mujoco.MjModel:
    """Compiled model by registry name, or ``"features"`` for the feature test model (cached per process)."""
    if name == "features":
        return mujoco.MjModel.from_xml_path(str(FEATURE_XML))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")  # mujoco-warp version warning
        return myo_sim.load_model(name)


def random_qpos(m: mujoco.MjModel, n: int, rng: np.random.Generator) -> np.ndarray:
    """Random postures: limited hinges and slides uniform in their range, others in [-2, 2]; free and ball
    joints with random, unnormalised quaternions (MuJoCo normalises them) and free positions near the origin."""
    Q = np.tile(m.qpos0, (n, 1))
    for j in range(m.njnt):
        a, typ = m.jnt_qposadr[j], int(m.jnt_type[j])
        if typ in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            Q[:, a] = rng.uniform(*m.jnt_range[j], n) if m.jnt_limited[j] else rng.uniform(-2, 2, n)
        else:
            o = 0
            if typ == int(mujoco.mjtJoint.mjJNT_FREE):
                Q[:, a : a + 3] = rng.normal(0, 0.5, (n, 3))
                o = 3
            q = rng.normal(0, 1, (n, 4))
            Q[:, a + o : a + o + 4] = q / np.linalg.norm(q, axis=1, keepdims=True) * rng.uniform(0.5, 2, (n, 1))
    return Q


def random_state(m: mujoco.MjModel, n: int, seed: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Random (qpos, qvel, act, ctrl); activations and controls also outside their nominal ranges."""
    rng = np.random.default_rng(seed)
    Q = random_qpos(m, n, rng)
    return Q, rng.normal(0, 1.5, (n, m.nv)), rng.uniform(-0.2, 1.2, (n, m.na)), rng.uniform(-0.3, 1.3, (n, m.nu))


def _wrap_signature(m: mujoco.MjModel, d: mujoco.MjData) -> list[tuple[int, ...]]:
    flat = d.wrap_obj.reshape(-1)  # MuJoCo indexes wrap_obj flat by ten_wrapadr
    return [tuple(int(x) for x in flat[d.ten_wrapadr[t] : d.ten_wrapadr[t] + d.ten_wrapnum[t]]) for t in range(m.ntendon)]


def mujoco_reference(m: mujoco.MjModel, Q, V, A, U, eps: float = 1e-7, ndir: int = 2) -> dict[str, np.ndarray]:
    """MuJoCo's values at each state, and per tendon whether it is at a wrap onset.

    A tendon is at an onset when its wrap state (``wrap_obj``) changes under perturbations of ``qpos`` by
    ``eps`` along ``ndir`` random directions; there the discrete wrap decision depends on round-off.
    """
    d = mujoco.MjData(m)
    keys = ("ten_length", "ten_J", "ten_velocity", "actuator_length", "actuator_moment", "actuator_velocity",
            "actuator_force", "qfrc_actuator", "act_dot")  # fmt: skip
    out = {k: [] for k in keys}
    onset = np.zeros((len(Q), m.ntendon), dtype=bool)
    rng = np.random.default_rng(12345)
    for i in range(len(Q)):
        d.qpos[:], d.qvel[:], d.ctrl[:] = Q[i], V[i], U[i]
        if m.na:
            d.act[:] = A[i]
        mujoco.mj_forward(m, d)
        for k in keys:
            out[k].append(np.array(getattr(d, k)))
        base = _wrap_signature(m, d)
        for _ in range(ndir):
            dq = rng.normal(0, 1, m.nq)
            for sgn in (-1, 1):
                d.qpos[:] = Q[i] + sgn * eps * dq
                mujoco.mj_kinematics(m, d)
                mujoco.mj_comPos(m, d)
                mujoco.mj_tendon(m, d)
                onset[i] |= np.array([a != b for a, b in zip(base, _wrap_signature(m, d))])
    res = {k: np.array(v) for k, v in out.items()}
    res["onset"] = onset
    return res


def actuator_onset(m: mujoco.MjModel, onset: np.ndarray) -> np.ndarray:
    """Per-actuator onset flags (actuators on tendons at an onset), (n, nu)."""
    flags = np.zeros((len(onset), m.nu), dtype=bool)
    for a in range(m.nu):
        if int(m.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_TENDON):
            flags[:, a] = onset[:, m.actuator_trnid[a, 0]]
    return flags
