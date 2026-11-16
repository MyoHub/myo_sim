"""The models with the scapulothoracic girdle stay stable under random activation, on CPU MuJoCo and on mujoco-warp.

Rollouts at the default timestep from ``qpos0`` and from postures that move the girdle and the glenohumeral joint
(both sides), three seeds each, with random activation held for 0.2 s. Besides non-finite values and MuJoCo's bad-qacc
warning, every check step bounds the largest joint speed, the deepest contact penetration and the largest equality and
limit violation, with tighter bounds for the scapulothoracic contact and the girdle and glenohumeral limits. The bounds are about twice the largest values seen on MuJoCo 4.0.1 (``shoulder-complex.md``,
"Integrated stability"). The mujoco-warp rollouts run on the GPU (skipped without CUDA) and are measured with CPU
kinematics from their states.
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np
import pytest

from myo_sim import load_model
from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings

MODELS = ("myoarms", "myotorso_arms", "myofullbody")
SEEDS = (0, 1, 2)
STEPS = 500  # 1 s at the 2 ms default timestep
CTRL_HOLD = 100
CHECK_EVERY = 10
# Joint values per side (rad), the same on both sides.
POSTURES: dict[str, dict[str, float]] = {
    "qpos0": {},
    "abduction_90": {"elv_angle": 0.0, "shoulder_elv": 1.57, "elbow_flexion": 0.3},
    "flexion_120": {"elv_angle": 1.57, "shoulder_elv": 2.1, "elbow_flexion": 1.0},
    "shrug_reach": {
        "sternoclavicular_r3": 0.35,
        "sternoclavicular_r2": -0.7,
        "elv_angle": 1.2,
        "shoulder_elv": 1.4,
        "shoulder_rot": 0.5,
    },
}
MAX_JOINT_SPEED = 100.0  # rad/s, any dof except a free joint (finger joints reach 41)
MAX_PENETRATION = 0.05  # m, any contact (a falling myofullbody's forearm on the floor 27 mm)
MAX_SCAPULOTHORACIC_PENETRATION = 0.004  # m, scapula on the thoracic ellipsoid (2.0 mm)
MAX_EQUALITY_VIOLATION = 0.1  # rad or m, any equality row (0.043, knee and CMC4-5 couplings)
MAX_LIMIT_VIOLATION = 1.0  # rad or m, any joint or tendon limit row (finger MCP 0.53)
MAX_GIRDLE_LIMIT_VIOLATION = 0.15  # rad, girdle and glenohumeral limits (sternoclavicular protraction 0.083)
GIRDLE_JOINTS = ("sternoclavicular", "acromioclavicular", "elv_angle", "shoulder")


def start_state(model: mujoco.MjModel, posture: str) -> np.ndarray:
    """qpos of a posture: the standing keyframe for free-floating models, then the posture's joints, on the couplings."""
    qpos = model.qpos0.copy()
    if posture != "qpos0" and model.nkey and model.key("standing").id >= 0:
        qpos[:] = model.key("standing").qpos
    for side in "rl":
        for joint, value in POSTURES[posture].items():
            qpos[model.joint(f"{joint}_{side}").qposadr[0]] = value
    project_joint_couplings(JointCouplings.from_model(model), qpos)
    return qpos


MEASURES = ("joint_speed", "penetration", "scapulothoracic_penetration", "equality", "limit", "girdle_limit")


def _free_dofs(model: mujoco.MjModel) -> np.ndarray:
    free = np.zeros(model.nv, bool)
    for j in np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE):
        free[model.jnt_dofadr[j] : model.jnt_dofadr[j] + 6] = True
    return free


def measures(model: mujoco.MjModel, data: mujoco.MjData, free: np.ndarray) -> dict[str, float]:
    """Joint speed, contact penetration and constraint violation of the state in *data* (after mj_forward)."""
    kind = data.efc_type[: data.nefc]
    pos = data.efc_pos[: data.nefc]
    eq = kind == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    lim = np.isin(kind, (mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT, mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON))
    girdle = (kind == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT) & np.array(
        [model.joint(int(i)).name.startswith(GIRDLE_JOINTS) if k == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT else False
         for k, i in zip(kind, data.efc_id[: data.nefc])], dtype=bool
    )  # fmt: skip
    dist = data.contact.dist[: data.ncon]
    glide = np.array(
        [model.geom(int(g)).name.startswith("scapula_glide") for g in data.contact.geom[: data.ncon, 0]], dtype=bool
    )
    glide |= np.array(
        [model.geom(int(g)).name.startswith("scapula_glide") for g in data.contact.geom[: data.ncon, 1]], dtype=bool
    )
    return {
        "joint_speed": float(np.abs(data.qvel[~free]).max()),
        "penetration": float(max(0.0, -dist.min())) if data.ncon else 0.0,
        "scapulothoracic_penetration": float(max(0.0, -dist[glide].min())) if glide.any() else 0.0,
        "equality": float(np.abs(pos[eq]).max()) if eq.any() else 0.0,
        "limit": float(max(0.0, -pos[lim].min())) if lim.any() else 0.0,
        "girdle_limit": float(max(0.0, -pos[girdle].min())) if girdle.any() else 0.0,
    }


def _controls(rng: np.random.Generator, nu: int) -> np.ndarray:
    return (rng.random(nu) < 0.3) * rng.random(nu)


def _check(m: dict[str, float], where: str) -> None:
    assert m["joint_speed"] < MAX_JOINT_SPEED, f"{where}: joint speed {m['joint_speed']:.1f} rad/s"
    assert m["penetration"] < MAX_PENETRATION, f"{where}: penetration {1e3 * m['penetration']:.1f} mm"
    assert m["scapulothoracic_penetration"] < MAX_SCAPULOTHORACIC_PENETRATION, (
        f"{where}: scapulothoracic penetration {1e3 * m['scapulothoracic_penetration']:.2f} mm"
    )
    assert m["equality"] < MAX_EQUALITY_VIOLATION, f"{where}: equality violation {m['equality']:.4f}"
    assert m["limit"] < MAX_LIMIT_VIOLATION, f"{where}: limit violation {m['limit']:.4f}"
    assert m["girdle_limit"] < MAX_GIRDLE_LIMIT_VIOLATION, f"{where}: girdle limit violation {m['girdle_limit']:.4f}"


def cpu_rollout(model: mujoco.MjModel, posture: str, seed: int, check: bool = True) -> dict[str, float]:
    """Random-activation rollout on CPU MuJoCo; returns the largest value of each measure."""
    data = mujoco.MjData(model)
    data.qpos[:] = start_state(model, posture)
    mujoco.mj_forward(model, data)
    free = _free_dofs(model)
    rng = np.random.default_rng(seed)
    worst = dict.fromkeys(MEASURES, 0.0)
    for step in range(STEPS):
        if step % CTRL_HOLD == 0:
            data.ctrl[:] = _controls(rng, model.nu)
        mujoco.mj_step(model, data)
        where = f"{posture}, seed {seed}, step {step}"
        if check:
            assert data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0, f"{where}: bad qacc"
            assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)), f"{where}: non-finite state"
        if step % CHECK_EVERY == CHECK_EVERY - 1:
            mujoco.mj_forward(model, data)
            m = measures(model, data, free)
            worst = {k: max(worst[k], m[k]) for k in worst}
            if check:
                _check(m, where)
    return worst


@pytest.fixture(scope="module", params=MODELS)
def model(request: Any) -> mujoco.MjModel:
    return load_model(request.param)


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("posture", sorted(POSTURES))
def test_girdle_models_stay_stable_on_cpu(model: mujoco.MjModel, posture: str, seed: int) -> None:
    cpu_rollout(model, posture, seed)


def warp_rollout(model: mujoco.MjModel, check: bool = True) -> dict[str, float]:
    """All postures x seeds as worlds of one mujoco-warp rollout on the GPU, measured with CPU kinematics."""
    import mujoco_warp as mjw
    import warp as wp

    cases = [(p, s) for p in sorted(POSTURES) for s in SEEDS]
    nworld = len(cases)
    qpos = np.stack([start_state(model, p) for p, _ in cases])
    rngs = [np.random.default_rng(s) for _, s in cases]
    with wp.ScopedDevice("cuda:0"):
        m = mjw.put_model(model)
        d = mjw.make_data(model, nworld=nworld, nconmax=512, njmax=2048)
        d.qpos = wp.array(qpos, dtype=float)
        mjw.forward(m, d)
        data = mujoco.MjData(model)
        free = _free_dofs(model)
        worst = dict.fromkeys(MEASURES, 0.0)
        for step in range(STEPS):
            if step % CTRL_HOLD == 0:
                d.ctrl = wp.array(np.stack([_controls(r, model.nu) for r in rngs]), dtype=float)
            mjw.step(m, d)
            if step % CHECK_EVERY != CHECK_EVERY - 1:
                continue
            q, v, act = d.qpos.numpy(), d.qvel.numpy(), d.act.numpy()
            for w, (posture, seed) in enumerate(cases):
                where = f"warp {posture}, seed {seed}, step {step}"
                if check:
                    assert np.all(np.isfinite(q[w])) and np.all(np.isfinite(v[w])), f"{where}: non-finite state"
                data.qpos[:], data.qvel[:], data.act[:] = q[w], v[w], act[w]
                mujoco.mj_forward(model, data)
                meas = measures(model, data, free)
                worst = {k: max(worst[k], meas[k]) for k in worst}
                if check:
                    _check(meas, where)
    return worst


def _has_cuda() -> bool:
    try:
        import warp as wp

        wp.config.quiet = True
        wp.init()
        return wp.get_cuda_device_count() > 0
    except ImportError:
        return False


@pytest.mark.skipif(not _has_cuda(), reason="mujoco-warp rollouts need a CUDA device")
def test_girdle_models_stay_stable_on_warp(model: mujoco.MjModel) -> None:
    pytest.importorskip("mujoco_warp")
    warp_rollout(model)
