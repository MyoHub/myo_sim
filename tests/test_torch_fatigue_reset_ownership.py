"""Reset vectors can refer to live fatigue compartments without losing their values."""

import pytest

from myo_sim.physics.fatigue import TorchFatigueState


@pytest.fixture(params=["cpu", "cuda"])
def device(request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch) -> str:
    torch = pytest.importorskip("torch")
    if request.param == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")
    monkeypatch.setattr("myo_sim.physics.fatigue.TORCH_FATIGUE_COMPILE", "0")
    return request.param


@pytest.mark.parametrize("compartment", ["MA", "MR", "MF"])
@pytest.mark.parametrize("selection", ["all", "slice", "ids"])
@pytest.mark.parametrize("input_type", ["numpy", "tensor"])
def test_reset_uses_original_values_of_live_views(device: str, compartment: str, selection: str, input_type: str) -> None:
    import torch

    state = TorchFatigueState(3, 3, device=device)
    reference = TorchFatigueState(3, 3, device=device)
    command = torch.tensor([[0.8, 0.7, 0.6], [0.4, 0.3, 0.2], [0.2, 0.5, 0.1]], device=device)
    for model in (state, reference):
        model.reset(fatigue_reset_vec=[0.1, 0.2, 0.3])
        model.step(command, 0.02)
    before = {name: getattr(state, name).clone() for name in ("MA", "MR", "MF")}
    source = getattr(state, compartment)[0]
    requested = source.clone()
    # On CPU this NumPy array shares the live state storage; CUDA tensors do too.
    vector = source.cpu().numpy() if input_type == "numpy" else source
    rows = {"all": None, "slice": slice(0, 2), "ids": torch.tensor([0, 2], device=device)}[selection]
    index = slice(None) if rows is None else rows
    state.reset(rows, fatigue_reset_vec=vector)
    reference.reset(rows, fatigue_reset_vec=requested)
    expected = {name: values.clone() for name, values in before.items()}
    expected["MA"][index] = 0
    expected["MF"][index] = requested
    expected["MR"][index] = 1 - requested
    for name in expected:
        torch.testing.assert_close(getattr(state, name), expected[name], rtol=0, atol=0)
    state.step(command, 0.02)
    reference.step(command, 0.02)
    for name in expected:
        torch.testing.assert_close(getattr(state, name), getattr(reference, name), rtol=0, atol=0)


def test_owning_the_reset_vector_keeps_its_gradient(device: str) -> None:
    import torch

    state = TorchFatigueState(3, 3, device=device)
    vector = torch.tensor([0.1, 0.4, 0.8], dtype=torch.float64, device=device, requires_grad=True)
    state.reset(torch.tensor([0, 2], device=device), fatigue_reset_vec=vector)
    state.MF.square().sum().backward()
    torch.testing.assert_close(vector.grad, 4 * vector.detach().float().to(vector.dtype), rtol=0, atol=0)
