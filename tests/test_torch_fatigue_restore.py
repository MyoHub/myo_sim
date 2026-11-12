"""Rejected Torch fatigue checkpoints must not change a running simulation."""

from copy import deepcopy

import pytest

from myo_sim.physics.fatigue import TorchFatigueState


@pytest.fixture(params=["cpu", "cuda"])
def torch_device(request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch) -> str:
    torch = pytest.importorskip("torch")
    if request.param == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")
    monkeypatch.setattr("myo_sim.physics.fatigue.TORCH_FATIGUE_COMPILE", "0")
    return request.param


@pytest.mark.parametrize("field", ["MA", "MR", "MF"])
@pytest.mark.parametrize("failure", ["missing", "nonnumeric", "shape"])
def test_rejected_restore_preserves_state_and_next_step(torch_device: str, field: str, failure: str) -> None:
    import torch

    state = TorchFatigueState(2, 3, device=torch_device)
    reference = TorchFatigueState(2, 3, device=torch_device)
    command = torch.full((2, 3), 0.7, device=torch_device)
    for model in (state, reference):
        model.reset(fatigue_reset_vec=[0.3, 0.4, 0.5])
        model.step(command, 0.02)
    before = {name: getattr(state, name).clone() for name in ("MA", "MR", "MF")}
    buffers = {name: getattr(state, name) for name in before}
    incoming = {"MA": [[0.2] * 3] * 2, "MR": [[0.5] * 3] * 2, "MF": [[0.3] * 3] * 2}
    if failure == "missing":
        del incoming[field]
        error = KeyError
    elif failure == "nonnumeric":
        incoming[field] = [["bad"] * 3] * 2
        error = (TypeError, ValueError)
    else:
        incoming[field] = [[0.1] * 4]
        error = RuntimeError
    with pytest.raises(error):
        state.load_state_dict(incoming)
    for name in before:
        assert getattr(state, name) is buffers[name]
        torch.testing.assert_close(getattr(state, name), before[name], rtol=0, atol=0)
    state.step(command, 0.02)
    reference.step(command, 0.02)
    for name in before:
        torch.testing.assert_close(getattr(state, name), getattr(reference, name), rtol=0, atol=0)


@pytest.mark.parametrize("shape", ["full", "row", "vector", "column", "scalar"])
def test_valid_restore_preserves_broadcasting_and_buffer_identity(torch_device: str, shape: str) -> None:
    import torch

    state = TorchFatigueState(2, 3, device=torch_device)
    reference = TorchFatigueState(2, 3, device=torch_device)
    values = {
        "full": [[0.1, 0.2, 0.3], [0.15, 0.25, 0.35]],
        "row": [[0.1, 0.2, 0.3]],
        "vector": [0.1, 0.2, 0.3],
        "column": [[0.1], [0.2]],
        "scalar": 0.2,
    }[shape]
    ma = torch.tensor(values)
    incoming = {"MA": values, "MF": (ma / 2).tolist(), "MR": (1 - 1.5 * ma).tolist()}
    buffers = {name: getattr(state, name) for name in incoming}
    state.load_state_dict(deepcopy(incoming))
    for name in incoming:
        expected = torch.tensor(incoming[name], device=torch_device).expand(2, 3)
        assert getattr(state, name) is buffers[name]
        torch.testing.assert_close(getattr(state, name), expected, rtol=0, atol=0)
        getattr(reference, name).copy_(expected)
    command = torch.full((2, 3), 0.6, device=torch_device)
    state.step(command, 0.02)
    reference.step(command, 0.02)
    for name in incoming:
        torch.testing.assert_close(getattr(state, name), getattr(reference, name), rtol=0, atol=0)


@pytest.mark.filterwarnings("ignore:To copy construct from a tensor:UserWarning")
def test_restore_stages_cross_aliased_live_tensors(torch_device: str) -> None:
    import torch

    state = TorchFatigueState(2, 3, device=torch_device)
    state.reset(fatigue_reset_vec=[0.3, 0.4, 0.5])
    expected = {"MA": state.MR.clone(), "MR": state.MA.clone(), "MF": state.MF.clone()}
    incoming = {"MA": state.MR, "MR": state.MA, "MF": state.MF}
    state.load_state_dict(incoming)
    for name in expected:
        torch.testing.assert_close(getattr(state, name), expected[name], rtol=0, atol=0)


def test_zero_muscle_snapshot_restores(torch_device: str) -> None:
    import torch

    state = TorchFatigueState(2, 0, device=torch_device)
    state.load_state_dict({"MA": [[], []], "MR": [[], []], "MF": [[], []]})
    assert state.step(torch.empty((2, 0), device=torch_device), 0.02).shape == (2, 0)
