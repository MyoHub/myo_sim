"""Compiled fatigue timesteps stay bounded without changing values or graph reuse."""

from __future__ import annotations

from typing import Any

import pytest

from myo_sim.physics import fatigue


@pytest.fixture(autouse=True)
def clear_dt_cache() -> Any:
    fatigue._torch_dt_tensor.cache_clear()
    yield
    fatigue._torch_dt_tensor.cache_clear()


@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_timestep_cache_is_bounded_and_preserves_inflight_values(device: str) -> None:
    torch = pytest.importorskip("torch")
    if device == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")
    first = fatigue._torch_dt_tensor(device, torch.float32, 0.02)
    oldest = fatigue._torch_dt_tensor(device, torch.float32, 0.03)
    for i in range(fatigue._DT_TENSOR_CACHE_SIZE):
        assert fatigue._torch_dt_tensor(device, torch.float32, 0.02) is first
        fatigue._torch_dt_tensor(device, torch.float32, 0.04 + i * 0.001)
    assert fatigue._torch_dt_tensor.cache_info().currsize <= fatigue._DT_TENSOR_CACHE_SIZE
    assert fatigue._torch_dt_tensor(device, torch.float32, 0.02) is first
    replacement = fatigue._torch_dt_tensor(device, torch.float32, 0.03)
    assert replacement is not oldest
    # Eviction cannot mutate scalar tensors retained by an in-flight autograd graph.
    torch.testing.assert_close(first, torch.tensor(0.02, device=device))
    torch.testing.assert_close(oldest, replacement)


def test_timestep_cache_keys_include_dtype_device_and_value() -> None:
    torch = pytest.importorskip("torch")
    a = fatigue._torch_dt_tensor("cpu", torch.float32, 0.02)
    b = fatigue._torch_dt_tensor("cpu", torch.float64, 0.02)
    c = fatigue._torch_dt_tensor("cpu", torch.float32, 0.03)
    assert a is not b and a is not c
    assert a.dtype == torch.float32 and b.dtype == torch.float64
    assert a.item() != c.item()
    if torch.cuda.is_available():
        gpu = fatigue._torch_dt_tensor("cuda:0", torch.float32, 0.02)
        assert gpu.device.type == "cuda"
        assert gpu is not a
        torch.testing.assert_close(gpu.cpu(), a)


def test_variable_timesteps_reuse_one_torch_graph(monkeypatch: pytest.MonkeyPatch) -> None:
    torch = pytest.importorskip("torch")
    compilations = 0

    def backend(graph: Any, inputs: Any) -> Any:
        nonlocal compilations
        compilations += 1
        return graph.forward

    # Exercise real TorchDynamo graph guards without requiring a native compiler.
    compiled = torch.compile(fatigue._torch_step, backend=backend, fullgraph=True, dynamic=False)
    monkeypatch.setattr(fatigue, "_compiled_torch_step", compiled)
    monkeypatch.setattr(fatigue, "_eager_keys", set())
    ma = torch.tensor([[0.2, 0.3]])
    mf = torch.tensor([[0.3, 0.2]])
    tl = torch.tensor([[0.9, 0.6]])
    params = {k: torch.tensor(v) for k, v in {"F": 0.0097, "R": 0.00091, "r": 15.0, "tauact": 0.01, "taudeact": 0.04}.items()}
    for i in range(fatigue._DT_TENSOR_CACHE_SIZE + 2):
        dt = 0.01 + i * 0.001
        out = fatigue.torch_three_cc_r_step(ma, mf, tl, dt=dt, compile=True, **params)
        eager = fatigue.torch_three_cc_r_step(ma, mf, tl, dt=dt, compile=False, **params)
        for actual, expected in zip(out, eager):
            torch.testing.assert_close(actual, expected)
    assert compilations == 1
    assert not fatigue._eager_keys
    assert fatigue._torch_dt_tensor.cache_info().currsize <= fatigue._DT_TENSOR_CACHE_SIZE
