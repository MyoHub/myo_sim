"""Differentiate the fatigue step against independent zero-rate limits."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from myo_sim.physics.fatigue import three_cc_r_step


def _differentiate(backend: str, points: np.ndarray, F: float, R: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Return compartments and the gradient of sum(MA + 0.3 MF) in float32."""
    params = {"F": F, "R": R, "r": 15.0, "tauact": 0.01, "taudeact": 0.04}
    if backend.startswith("torch"):
        torch = pytest.importorskip("torch")
        device = "cuda" if backend == "torch-cuda" else "cpu"
        if device == "cuda" and not torch.cuda.is_available():
            pytest.skip("CUDA unavailable")
        inputs = torch.tensor(points, dtype=torch.float32, device=device, requires_grad=True)
        out = three_cc_r_step(torch, *inputs.T, dt=dt, **{k: torch.tensor(v, device=device) for k, v in params.items()})
        gradient = torch.autograd.grad((out[0] + 0.3 * out[2]).sum(), inputs)[0]
        return np.stack([x.detach().cpu().numpy() for x in out]), gradient.detach().cpu().numpy()

    jax = pytest.importorskip("jax")
    jp = jax.numpy

    def step(inputs: Any) -> Any:
        return jp.stack(three_cc_r_step(jp, *inputs.T, dt=dt, **params))

    inputs = jp.asarray(points, dtype=jp.float32)
    out = jax.jit(step)(inputs)
    gradient = jax.jit(jax.grad(lambda x: (step(x)[0] + 0.3 * step(x)[2]).sum()))(inputs)
    return np.asarray(out), np.asarray(gradient)


@pytest.mark.parametrize("backend", ["torch", "torch-cuda", "jax"])
@pytest.mark.parametrize("dt", [0.0, 0.02])
def test_zero_rates_match_differentiable_recruitment_limits(backend: str, dt: float) -> None:
    # Free recruitment and recruitment limited by MR. Neither case crosses a regime.
    points = np.array([[0.1, 0.2, 0.5], [0.2, 0.3, 0.9], [0.01, 0.98, 0.9]], dtype=np.float64)
    ma, mf, tl = points.T
    act = 0.5 + 1.5 * ma
    ld = 1.0 / (0.01 * act)
    limited = mf > 1.0 - tl
    target = np.where(limited, 1.0 - mf, tl)
    decay = np.exp(-ld * dt)
    expected_ma = target + (ma - target) * decay
    expected_gradient = np.column_stack(
        (
            decay * (1 + (ma - target) * dt * 1.5 / (0.01 * act**2)),
            np.where(limited, decay - 1, 0.0) + 0.3,
            np.where(limited, 0.0, 1 - decay),
        )
    )

    out, gradient = _differentiate(backend, points, F=0.0, R=0.0, dt=dt)
    np.testing.assert_allclose(out[0], expected_ma, atol=2e-7)
    np.testing.assert_allclose(out[2], mf, atol=2e-7)
    np.testing.assert_allclose(gradient, expected_gradient, atol=2e-6)


@pytest.mark.parametrize("backend", ["torch", "torch-cuda", "jax"])
@pytest.mark.parametrize("rates", [(0.0, 0.0), (0.0, 0.00091), (0.0097, 0.0), (0.0097, 0.00091), (1e-19, 0.0)])
def test_degenerate_rate_gradients_remain_finite(backend: str, rates: tuple[float, float]) -> None:
    points = np.array([[0.2, 0.3, 0.9], [0.1, 0.2, 0.5], [0.6, 0.1, 0.2], [0.3, 0.2, np.nan]])
    out, gradient = _differentiate(backend, points, F=rates[0], R=rates[1], dt=0.02)
    assert np.isfinite(out).all()
    assert np.isfinite(gradient).all()
