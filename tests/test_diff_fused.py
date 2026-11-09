"""myo_sim.diff's opt-in fused tendon kernel (``myo_sim.diff.fused``) against the torch replica and MuJoCo.

Runs on CUDA, or on the CPU with ``TRITON_INTERPRET=1`` (Triton's interpreter, slow: small batches).
"""

import importlib.util
import os

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from diff_utils import MODELS, load, mujoco_reference, random_state  # noqa: E402

from myo_sim.diff import MuscleTendonModel  # noqa: E402

DEV = "cuda" if torch.cuda.is_available() else "cpu"
needs_kernel = pytest.mark.skipif(
    importlib.util.find_spec("triton") is None or (DEV == "cpu" and os.environ.get("TRITON_INTERPRET") != "1"),
    reason="needs Triton and CUDA (or TRITON_INTERPRET=1)",
)
N = 48 if DEV == "cuda" else 8


def test_fused_off_by_default_and_fallback(monkeypatch):
    """The torch path is the default; fused=True without CUDA (and no interpreter) warns and falls back to it."""
    m = load("features")
    assert MuscleTendonModel(m, "cpu").tendons.fused is None
    monkeypatch.delenv("TRITON_INTERPRET", raising=False)
    with pytest.warns(UserWarning, match="fused=True ignored"):
        mt = MuscleTendonModel(m, "cpu", fused=True)
    assert mt.tendons.fused is None


def _pair(m, rule, dtype=torch.float64):
    return (MuscleTendonModel(m, DEV, dtype, rule), MuscleTendonModel(m, DEV, dtype, rule, fused=True))


def _tendons(mt, q, v):
    return mt.tendons.forward(mt.kinematics.forward(q), q, v)


@needs_kernel
@pytest.mark.parametrize("rule", ("mujoco3", "mujoco4"))
@pytest.mark.parametrize("name", MODELS + ("features",))
def test_fused_matches_torch(name, rule):
    """Lengths, Jacobian, velocities and wrap flags equal to the torch replica's to float64 round-off."""
    m = load(name)
    ref, fu = _pair(m, rule)
    Q, V, _, _ = random_state(m, N, seed=0)
    q, v = torch.tensor(Q, device=DEV), torch.tensor(V, device=DEV)
    a, b = _tendons(ref, q, v), _tendons(fu, q, v)
    torch.testing.assert_close(b.length, a.length, atol=1e-12, rtol=0)
    torch.testing.assert_close(b.J, a.J, atol=1e-12, rtol=0)
    torch.testing.assert_close(b.velocity, a.velocity, atol=1e-11, rtol=0)
    assert torch.equal(a.wrapped, b.wrapped)


@needs_kernel
@pytest.mark.parametrize("name", ("myoarm_r", "features"))
def test_fused_matches_mujoco(name):
    """The fused model's tendon lengths, Jacobian and actuator forces against MuJoCo, away from wrap onsets."""
    m = load(name)
    mt = MuscleTendonModel(m, DEV, fused=True)
    Q, V, A, U = random_state(m, N, seed=1)
    ref = mujoco_reference(m, Q, V, A, U)
    out = mt.forward(*(torch.tensor(x, device=DEV) for x in (Q, V, A, U)))
    ok = ~ref["onset"]
    np.testing.assert_allclose(out.ten_length.cpu().numpy()[ok], ref["ten_length"][ok], atol=1e-9, rtol=0)
    okJ = ok[:, mt.tendons.row_of_slot.cpu().numpy()]
    np.testing.assert_allclose(out.ten_J.cpu().numpy()[okJ], ref["ten_J"][okJ], atol=1e-8, rtol=0)
    s = ok.all(1)
    scale = np.abs(ref["qfrc_actuator"]).max()
    np.testing.assert_allclose(out.qfrc_actuator.cpu().numpy()[s], ref["qfrc_actuator"][s], atol=1e-6 * scale, rtol=0)


@needs_kernel
def test_fused_float32():
    """float32: the fused kernel within the torch replica's float32 error of the float64 values."""
    m = load("myofullbody")
    Q, V, _, _ = random_state(m, N, seed=2)
    ref64 = MuscleTendonModel(m, DEV, torch.float64)
    q64, v64 = torch.tensor(Q, device=DEV), torch.tensor(V, device=DEV)
    a = _tendons(ref64, q64, v64)
    errs = {}
    for fused in (False, True):
        mt = MuscleTendonModel(m, DEV, torch.float32, fused=fused)
        b = _tendons(mt, q64.float(), v64.float())
        ok = (a.wrapped == b.wrapped).all(-1)  # states without a float32 wrap flip
        errs[fused] = ((b.length.double() - a.length).abs()[ok].max(), (b.J.double() - a.J).abs()[ok].max())
    assert errs[True][0] <= 3 * errs[False][0] + 1e-6 and errs[True][1] <= 3 * errs[False][1] + 1e-6, errs


@needs_kernel
@pytest.mark.parametrize("name", ("features", "myoarm_r"))
def test_fused_gradients(name):
    """The fused path's gradient (the torch replica's, recomputed) equals the torch path's and finite differences."""
    m = load(name)
    ref, fu = _pair(m, "mujoco3")
    Q, V, A, U = random_state(m, 4, seed=3)
    q = torch.tensor(Q, device=DEV)
    v, act, ctrl = (torch.tensor(x, device=DEV) for x in (V, A, U))
    w = torch.randn(m.nv, device=DEV, dtype=torch.float64)

    def objective(mt, qq):
        return (mt.forward(qq, v, act, ctrl).qfrc_actuator * w).sum()

    grads = []
    for mt in (ref, fu):
        qq = q.clone().requires_grad_()
        objective(mt, qq).backward()
        grads.append(qq.grad)
    torch.testing.assert_close(grads[1], grads[0], atol=1e-10, rtol=1e-10)
    # central differences along random directions of the joint coordinates (free/ball quaternions included)
    rng = np.random.default_rng(0)
    for _ in range(3):
        d = torch.tensor(rng.normal(0, 1, Q.shape), device=DEV)
        eps = 1e-6
        with torch.no_grad():
            fd = (objective(fu, q + eps * d) - objective(fu, q - eps * d)) / (2 * eps)
        an = (grads[1] * d).sum()
        assert abs(float(fd - an)) <= 1e-4 * max(1.0, abs(float(an))), (float(fd), float(an))
