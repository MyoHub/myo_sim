"""CPU MuJoCo and MuJoCo Warp must give the same tendon lengths on the hand and arm models.

Policies trained on the GPU with mujoco-warp are replayed on the CPU, so every tendon path has to
agree between the two backends over the whole joint range, not only at qpos0.
"""

import importlib.metadata

import mujoco
import numpy as np
import pytest

from myo_sim import load_model

wp = pytest.importorskip("warp")
mjw = pytest.importorskip("mujoco_warp")

pytestmark = pytest.mark.skipif(not wp.is_cuda_available(), reason="mujoco_warp parity needs a CUDA device")

NWORLD = 64
SEED = 0
# float32 rounding stays below 1e-6 m on these models; real path differences are above 3e-5 m.
ATOL = 1e-5
# mujoco_warp before 3.16 returns a wrong contact point for inside wrapping (sidesite inside the wrap
# geom) when its float32 Newton solve stalls: google-deepmind/mujoco_warp#1713, fixed in #1761.
INSIDE_WRAP_FIXED_IN = (3, 16)


def _random_postures(model: mujoco.MjModel, n: int, seed: int) -> np.ndarray:
    """qpos0 with every limited hinge joint drawn uniformly from its range."""
    rng = np.random.default_rng(seed)
    qpos = np.tile(model.qpos0, (n, 1))
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and model.jnt_limited[j]:
            qpos[:, model.jnt_qposadr[j]] = rng.uniform(*model.jnt_range[j], n)
    return qpos


def _inside_wrap_tendons(model: mujoco.MjModel) -> set[str]:
    """Tendons with a sidesite inside its sphere or cylinder wrap geom (MuJoCo's inside wrapping)."""
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    names = set()
    for t in range(model.ntendon):
        for w in range(model.tendon_adr[t], model.tendon_adr[t] + model.tendon_num[t]):
            side = int(model.wrap_prm[w])
            if model.wrap_type[w] not in (mujoco.mjtWrap.mjWRAP_SPHERE, mujoco.mjtWrap.mjWRAP_CYLINDER) or side < 0:
                continue
            g = model.wrap_objid[w]
            local = data.geom_xmat[g].reshape(3, 3).T @ (data.site_xpos[side] - data.geom_xpos[g])
            if np.linalg.norm(local) < model.geom_size[g, 0]:
                names.add(model.tendon(t).name)
    return names


def _warp_version() -> tuple[int, ...]:
    return tuple(int(x) for x in importlib.metadata.version("mujoco-warp").split(".")[:2])


@pytest.mark.parametrize("name", ["myohands", "myoarms"])
def test_tendon_lengths_match_between_cpu_and_warp(name):
    model = load_model(name)
    qpos = _random_postures(model, NWORLD, SEED)

    # GPU: only the kinematics and tendon stages are needed for lengths.
    m = mjw.put_model(model)
    d = mjw.make_data(model, nworld=NWORLD, nconmax=64, njmax=256)
    d.qpos = wp.array(qpos, dtype=float)
    mjw.kinematics(m, d)
    mjw.com_pos(m, d)
    mjw.tendon(m, d)
    gpu = d.ten_length.numpy()

    data = mujoco.MjData(model)
    cpu = np.empty_like(gpu)
    for i in range(NWORLD):
        data.qpos[:] = qpos[i]
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_tendon(model, data)
        cpu[i] = data.ten_length

    err = np.abs(cpu - gpu).max(axis=0)
    bad = {model.tendon(t).name: float(err[t]) for t in np.flatnonzero(err > ATOL)}
    inside = _inside_wrap_tendons(model)
    other = {k: v for k, v in bad.items() if k not in inside}
    assert not other, f"{name}: CPU vs warp tendon length differs by more than {ATOL} m: {other}"
    if bad and _warp_version() < INSIDE_WRAP_FIXED_IN:
        pytest.xfail(f"mujoco_warp {_warp_version()} inside-wrap float32 fallback (mujoco_warp#1713): {bad}")
    assert not bad, f"{name}: CPU vs warp tendon length differs by more than {ATOL} m: {bad}"
