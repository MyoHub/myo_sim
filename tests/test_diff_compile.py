"""myo_sim.diff under torch.compile: one graph without host synchronisation (CPU trace), and a compiled
step with CUDA-graph capture on the GPU (skipped without CUDA)."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from diff_utils import load, random_state  # noqa: E402

from myo_sim.diff import MuscleTendonModel  # noqa: E402


def _step(mt: MuscleTendonModel):
    def step(qpos, qvel, act, ctrl):
        out = mt.forward(qpos, qvel, act, ctrl)
        nxt, _ = mt.step_act(act, ctrl, nsteps=2)
        return out.ten_length, out.actuator_force, out.qfrc_actuator, nxt

    return step


def _inputs(m, n, device, dtype):
    return [torch.tensor(x, device=device, dtype=dtype) for x in random_state(m, n, seed=20)]


@pytest.mark.parametrize("name", ("myohand_r", "features"))
def test_traces_as_one_graph(name):
    """fullgraph=True fails on any graph break, i.e. on host syncs or data-dependent Python control flow."""
    m = load(name)
    mt = MuscleTendonModel(m)
    step = _step(mt)
    args = _inputs(m, 16, "cpu", torch.float64)
    compiled = torch.compile(step, fullgraph=True, backend="aot_eager")
    for got, want in zip(compiled(*args), step(*args)):
        torch.testing.assert_close(got, want, atol=1e-12, rtol=1e-12)


@pytest.mark.skipif(not torch.cuda.is_available(), reason="needs a CUDA device")
def test_compiled_step_on_cuda():
    """The step and its gradient compile with inductor and run as a captured CUDA graph (reduce-overhead)."""
    m = load("myoarm_r")
    mt = MuscleTendonModel(m, device="cuda", dtype=torch.float32)
    step = _step(mt)
    q, v, a, u = _inputs(m, 512, "cuda", torch.float32)

    def loss(q, v, a, u):
        L, F, qfrc, nxt = step(q, v, a, u)
        return (qfrc**2).mean() + F.mean() + L.sum() + nxt.sum()

    compiled = torch.compile(step, fullgraph=True, mode="reduce-overhead")
    for _ in range(3):  # warm-up, capture, replay
        got = compiled(q, v, a, u)
    for x, y in zip(got, step(q, v, a, u)):
        torch.testing.assert_close(x, y, atol=1e-4 * y.abs().max().item(), rtol=1e-4)  # float32, sums of large forces
    closs = torch.compile(loss, fullgraph=True)
    qg = q.clone().requires_grad_()
    g = torch.autograd.grad(closs(qg, v, a, u), qg)[0]
    qg2 = q.clone().requires_grad_()
    g2 = torch.autograd.grad(loss(qg2, v, a, u), qg2)[0]
    assert torch.isfinite(g).all()
    assert (g - g2).abs().max() <= 1e-3 * g2.abs().max()
    np.testing.assert_array_equal(got[3].shape, (512, m.na))
