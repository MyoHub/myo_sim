"""The 3CC-r fatigue model: exact step, NumPy and torch states, parameters and muscle groups."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from myo_sim.physics.fatigue import MUSCLE_FATIGUE_PARAMS, _muscle_group
from myo_sim.physics.muscle_groups import FUNCTIONAL_MUSCLE_GROUPS, MUSCLE_FMG

# ---------------------------------------------------------------------------
# CumulativeFatigue, TorchFatigueState and three_cc_r_step
# ---------------------------------------------------------------------------


def _mock_muscle_model(n: int) -> MagicMock:
    """A stand-in MjModel with n muscle actuators (tau 10/40 ms)."""
    import mujoco

    mock = MagicMock()
    mock.opt.timestep = 0.002
    mock.nu = n
    mock.actuator_dyntype = np.full(n, mujoco.mjtDyn.mjDYN_MUSCLE)
    mock.actuator_dynprm = np.tile([0.01, 0.04] + [0.0] * 8, (n, 1))
    return mock


def _make_fatigue_model(n: int, **kwargs: Any) -> CumulativeFatigue:  # noqa: F821
    """Build a CumulativeFatigue with n uniform muscle actuators (no real MjModel needed)."""
    from myo_sim.physics.fatigue import CumulativeFatigue

    return CumulativeFatigue(_mock_muscle_model(n), use_uniform_params=True, **kwargs)


def _fatigue_stepper(backend: str, n: int) -> Any:
    """Return ``step(TL, dt) -> MA`` for the numpy or torch 3CC-r model (tau 10/40 ms)."""
    if backend == "numpy":
        f = _make_fatigue_model(n)
        return lambda tl, dt: f.compute_act(tl.copy(), dt=dt)[0].copy()
    torch = pytest.importorskip("torch")
    from myo_sim.physics.fatigue import TorchFatigueState

    t = TorchFatigueState(num_envs=1, n_muscles=n)
    return lambda tl, dt: t.step(torch.tensor(tl[None], dtype=torch.float32), dt)[0].double().numpy()


def _fatigue_state_stepper(backend: str, n: int) -> Any:
    """Return ``step(TL, dt) -> (MA, MR, MF)`` for the numpy or torch model (uniform F, R, r)."""
    if backend == "numpy":
        f = _make_fatigue_model(n)
        return lambda tl, dt: tuple(x.copy() for x in f.compute_act(tl.copy(), dt=dt))
    torch = pytest.importorskip("torch")
    from myo_sim.physics.fatigue import _UNIFORM_PARAMS, TorchFatigueState

    t = TorchFatigueState(1, n, **{k: _UNIFORM_PARAMS[k] for k in "FRr"})

    def step(tl: np.ndarray, dt: float) -> tuple[np.ndarray, ...]:
        t.step(torch.tensor(tl[None], dtype=torch.float32), dt)
        return tuple(x[0].double().numpy() for x in (t.MA, t.MR, t.MF))

    return step


# Control steps of the registered myoFati* envs (10, 20 and 25 ms).
_FATIGUE_CTRL_DTS = (0.01, 0.02, 0.025)


# (MA, MF, TL) covering each controller regime and the switches between them, for
# the elbow parameters: activation, deactivation past TL, rest, a negative command,
# MR-limited recruitment ending (MF falls below 1 - TL) and starting (endurance).
_EXACT_STEP_CASES = np.array(
    [
        [0.0, 0.0, 0.4],
        [0.6, 0.1, 0.2],
        [0.5, 0.3, 0.5],
        [0.4, 0.2, 0.0],
        [0.3, 0.5, -0.5],
        [0.05, 0.9, 0.15],
        [0.1, 0.3, 0.9],
        [0.45, 0.5, 0.48],
        [0.0, 0.0, 0.7],
    ]
)
_ELBOW = {"F": 0.01086, "R": 0.00225, "r": 4.93, "tauact": 0.01, "taudeact": 0.04}


def _exact_step(
    backend: str,
    ma: np.ndarray,
    mf: np.ndarray,
    tl: np.ndarray,
    dt: float,
    **params: Any,
) -> tuple[np.ndarray, ...]:
    """``three_cc_r_step`` with the elbow parameters on numpy, torch or jax (float32)."""
    from myo_sim.physics.fatigue import three_cc_r_step

    kw = {k: np.broadcast_to(v, ma.shape) for k, v in {**_ELBOW, **params}.items()}
    if backend == "numpy":
        return three_cc_r_step(np, ma, mf, tl, dt=dt, **kw)
    if backend == "jax":
        jax = pytest.importorskip("jax")
        jnp = jax.numpy
        j = lambda x: jnp.asarray(np.asarray(x), dtype=jnp.float32)  # noqa: E731
        # jit with dt traced, as the MJX FatigueWrapper calls it
        step = jax.jit(lambda a, f, c, d, p: three_cc_r_step(jnp, a, f, c, dt=d, **p))
        out = step(j(ma), j(mf), j(tl), j(dt), {k: j(v) for k, v in kw.items()})
        return tuple(np.asarray(x, dtype=np.float64) for x in out)
    torch = pytest.importorskip("torch")
    t = lambda x: torch.tensor(np.asarray(x), dtype=torch.float32)  # noqa: E731
    kw = {k: t(v) for k, v in kw.items()}
    out = three_cc_r_step(torch, t(ma), t(mf), t(tl), dt=dt, **kw)
    return tuple(x.double().numpy() for x in out)


def _frozen_rate_reference(ma: float, mf: float, tl: float, dt: float) -> np.ndarray:
    """``(MA, MR, MF)`` from solve_ivp of the 3CC-r ODE, LD and LR frozen at ``ma``."""
    from scipy.integrate import solve_ivp

    from myo_sim.physics.fatigue import FATIGUE_REST_THRESHOLD

    p = _ELBOW
    ld = 1.0 / (p["tauact"] * (0.5 + 1.5 * ma))
    lr = (0.5 + 1.5 * ma) / p["taudeact"]
    rho = p["R"] * (p["r"] if tl <= FATIGUE_REST_THRESHOLD else 1.0)

    def rhs(_: float, y: np.ndarray) -> list[float]:
        a, f = y
        c = lr * (tl - a) if a >= tl else ld * min(tl - a, 1.0 - a - f)
        return [c - p["F"] * a, p["F"] * a - rho * f]

    def empty(_: float, y: np.ndarray) -> float:  # MA reaches 0 under TL < 0
        return y[0] if tl < 0 else 1.0

    empty.terminal = True  # type: ignore[attr-defined]
    sol = solve_ivp(rhs, (0.0, dt), [ma, mf], method="LSODA", rtol=1e-11, atol=1e-13, events=empty)
    a, f = sol.y[:, -1]
    if sol.status == 1:  # MA stays at 0, MF decays
        a, f = 0.0, f * np.exp(-rho * (dt - sol.t[-1]))
    return np.array([a, 1.0 - a - f, f])


def _rest_protocol(n: int = 3) -> np.ndarray:
    """Target loads at 20 ms: (10 s at 50%, 10 s at 20%, 10 s at rest) x 2."""
    block = np.repeat([0.5, 0.2, 0.0], 500)
    return np.tile(np.tile(block, 2)[:, None], (1, n))


def _rest_protocol_trace(backend: str, loads: np.ndarray, dt: float = 0.02, **kwargs: Any) -> dict:
    """MA, MF before and after each step of the uniform-parameter model.

    Starts with MF = 0.3 so that the recovery rate is visible from the start.
    ``kwargs`` go to the model's constructor (e.g. ``rest_threshold``).
    """
    from myo_sim.physics.fatigue import _UNIFORM_PARAMS, TorchFatigueState

    n = loads.shape[1]
    mf0 = np.full(n, 0.3)
    if backend == "numpy":
        cpu = _make_fatigue_model(n, **kwargs)
        cpu.reset(fatigue_reset_vec=mf0)

        def step(tl: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            cpu.compute_act(tl.copy(), dt=dt)
            return cpu.MA.copy(), cpu.MF.copy()

        state = (cpu.MA.copy(), cpu.MF.copy())
    else:
        torch = pytest.importorskip("torch")
        gpu = TorchFatigueState(1, n, **{k: _UNIFORM_PARAMS[k] for k in "FRr"}, **kwargs)
        gpu.reset(fatigue_reset_vec=mf0)

        def step(tl: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            gpu.step(torch.tensor(tl[None], dtype=torch.float32), dt)
            return gpu.MA[0].double().numpy(), gpu.MF[0].double().numpy()

        state = (gpu.MA[0].double().numpy(), gpu.MF[0].double().numpy())
    ma, mf = [state[0]], [state[1]]
    for tl in loads:
        new_ma, new_mf = step(tl)
        ma.append(new_ma)
        mf.append(new_mf)
    return {"MA": np.array(ma), "MF": np.array(mf), "dt": dt, **_UNIFORM_PARAMS}


def _effective_recovery_multiplier(trace: dict) -> np.ndarray:
    """``rR / R`` of each step, from ``dMF = (F MA - rR MF) dt``.

    The step integrates MF over the changing MA, so this holds where MA is
    (nearly) constant over the step.
    """
    ma, mf = trace["MA"], trace["MF"]
    dmf_dt = np.diff(mf, axis=0) / trace["dt"]
    return (trace["F"] * ma[:-1] - dmf_dt) / (trace["R"] * mf[:-1])


class TestCumulativeFatigue:
    def test_init_compartments(self) -> None:
        f = _make_fatigue_model(6)
        assert f.MA.shape == (6,)
        assert f.MF.shape == (6,)
        assert f.MR.shape == (6,)
        assert np.all(f.MA == 0.0)
        assert np.all(f.MF == 0.0)
        assert np.all(f.MR == 1.0)

    def test_step_output_shape(self) -> None:
        f = _make_fatigue_model(3)
        out = f.step(np.ones(3) * 0.5, dt=0.01)
        assert out.shape == (3,)

    def test_step_output_within_bounds(self) -> None:
        f = _make_fatigue_model(4)
        for _ in range(50):
            out = f.step(np.random.rand(4), dt=0.01)
        assert np.all(out >= 0.0)
        assert np.all(out <= 1.0)

    def test_step_monotone_accumulation(self) -> None:
        f = _make_fatigue_model(2)
        # MA should increase from zero when excitation > MA
        f.step(np.ones(2), dt=1.0)
        assert np.all(f.MA > 0.0), "Active compartment must grow"

    def test_reset_restores_initial_state(self) -> None:
        f = _make_fatigue_model(3)
        for _ in range(20):
            f.step(np.ones(3) * 0.8, dt=0.01)
        f.reset()
        assert np.all(f.MA == 0.0)
        assert np.all(f.MF == 0.0)
        assert np.all(f.MR == 1.0)

    @pytest.mark.parametrize("readonly", [False, True])
    def test_reset_vector_remains_owned_by_caller(self, readonly: bool) -> None:
        f = _make_fatigue_model(3)
        initial = np.full(3, 0.3)
        initial.setflags(write=not readonly)
        f.reset(fatigue_reset_vec=initial)
        for _ in range(100):
            f.step(np.ones(3), dt=0.02)
        np.testing.assert_array_equal(initial, np.full(3, 0.3))
        assert not np.shares_memory(f.MF, initial)
        f.reset(fatigue_reset_vec=initial)
        np.testing.assert_array_equal(f.MF, initial)

    def test_custom_rates(self) -> None:
        f = _make_fatigue_model(2)
        f.set_FatigueCoefficient(0.1)
        f.set_RecoveryCoefficient(0.05)
        assert f.F == pytest.approx(0.1)
        assert f.R == pytest.approx(0.05)

    @pytest.mark.parametrize("dt", _FATIGUE_CTRL_DTS)
    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_step_from_rest_does_not_overshoot_command(self, backend: str, dt: float) -> None:
        # LD * dt reaches 5 at a control step; an explicit Euler step then
        # drove MA from rest to 1.0 for a 0.25 command.
        tl = np.array([0.05, 0.1, 0.25, 0.5, 0.8, 1.0])
        ma = _fatigue_stepper(backend, tl.size)(tl, dt)
        assert np.all(ma > 0.0)
        assert np.all(ma <= tl + 1e-6), f"MA {ma} overshoots TL {tl}"

    @pytest.mark.parametrize("dt", _FATIGUE_CTRL_DTS)
    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_step_never_crosses_command(self, backend: str, dt: float) -> None:
        # MA moves towards TL and stops short of it (on the way down, only the
        # fatigue drain F * MA * dt < 3e-4 may take it below TL).
        step = _fatigue_stepper(backend, 8)
        rng = np.random.default_rng(0)
        ma = np.zeros(8)
        for _ in range(300):
            tl = rng.uniform(0.0, 1.0, 8)
            new = step(tl, dt)
            rising = ma < tl
            assert np.all(new[rising] <= tl[rising] + 1e-6)
            assert np.all(new[~rising] >= tl[~rising] - 1e-3)
            ma = new

    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_rest_multiplier_applies_only_at_rest(self, backend: str) -> None:
        """``r`` multiplies recovery only when TL == 0 (Rakshit et al. 2021, Eq. 7).

        On the 20% steps that follow 50% the muscle relaxes (MA >= TL) under a
        non-zero load, which the model used to treat as rest.
        """
        loads = _rest_protocol()
        trace = _rest_protocol_trace(backend, loads)
        mult = _effective_recovery_multiplier(trace)
        steady = np.abs(np.diff(trace["MA"], axis=0)) < 1e-4
        rest = loads == 0.0
        relaxing = (trace["MA"][:-1] >= loads) & ~rest
        assert (relaxing & steady).sum() > 10, "protocol must relax under load"
        np.testing.assert_allclose(mult[rest & steady], trace["r"], rtol=0.05)
        np.testing.assert_allclose(mult[~rest & steady], 1.0, atol=0.3)

    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_near_zero_command_counts_as_rest(self, backend: str) -> None:
        """Commands up to ``FATIGUE_REST_THRESHOLD`` rest (a sigmoid-mapped muscle
        never reaches zero); above it recovery runs at ``R``."""
        from myo_sim.physics.fatigue import FATIGUE_REST_THRESHOLD

        below = _rest_protocol_trace(backend, np.full((100, 3), FATIGUE_REST_THRESHOLD))
        above = _rest_protocol_trace(backend, np.full((100, 3), 2.0 * FATIGUE_REST_THRESHOLD))
        # skip the first steps, where MA still decays towards the command
        np.testing.assert_allclose(_effective_recovery_multiplier(below)[30:], below["r"], rtol=0.05)
        np.testing.assert_allclose(_effective_recovery_multiplier(above)[30:], 1.0, atol=0.3)

    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_negative_command_counts_as_rest(self, backend: str) -> None:
        """A negative command (``[-1, 1]`` muscle ctrl ranges, clamped to zero
        excitation by MuJoCo) recovers like TL = 0."""
        rest = _rest_protocol_trace(backend, np.zeros((100, 3)))
        negative = _rest_protocol_trace(backend, np.full((100, 3), -1.0))
        np.testing.assert_allclose(negative["MF"], rest["MF"], atol=1e-7)
        mult = _effective_recovery_multiplier(negative)
        np.testing.assert_allclose(mult, negative["r"], rtol=0.05)

    def test_rest_protocol_cpu_matches_torch(self) -> None:
        """CPU and torch agree on load / relax / rest cycles."""
        pytest.importorskip("torch")
        loads = _rest_protocol()
        cpu = _rest_protocol_trace("numpy", loads)
        gpu = _rest_protocol_trace("torch", loads)
        for k in ("MA", "MF"):
            np.testing.assert_allclose(gpu[k], cpu[k], atol=1e-5, err_msg=k)

    def test_default_rest_threshold_is_the_module_constant(self) -> None:
        pytest.importorskip("torch")
        from myo_sim.physics.fatigue import (
            FATIGUE_REST_THRESHOLD,
            TorchFatigueState,
        )

        assert FATIGUE_REST_THRESHOLD == 0.01
        assert _make_fatigue_model(2).rest_threshold == FATIGUE_REST_THRESHOLD
        assert TorchFatigueState(1, 2).rest_threshold == FATIGUE_REST_THRESHOLD
        model = _mock_muscle_model(2)
        gpu = TorchFatigueState.from_mj_model(model, 1, use_uniform_params=True)
        assert gpu.rest_threshold == FATIGUE_REST_THRESHOLD
        gpu = TorchFatigueState.from_mj_model(model, 1, use_uniform_params=True, rest_threshold=0.2)
        assert gpu.rest_threshold == 0.2

    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_rest_threshold_sets_what_counts_as_rest(self, backend: str) -> None:
        """A 5 % command (e.g. noise on an estimated excitation) rests only with a
        threshold at or above it; a zero threshold rests only TL <= 0."""
        loads = np.full((100, 3), 0.05)
        default = _rest_protocol_trace(backend, loads)
        raised = _rest_protocol_trace(backend, loads, rest_threshold=0.05)
        # skip the first steps, where MA still rises towards the command
        np.testing.assert_allclose(_effective_recovery_multiplier(default)[30:], 1.0, atol=0.3)
        np.testing.assert_allclose(_effective_recovery_multiplier(raised)[30:], raised["r"], rtol=0.05)
        tiny = np.full((100, 3), 0.001)
        zero = _rest_protocol_trace(backend, tiny, rest_threshold=0.0)
        np.testing.assert_allclose(_effective_recovery_multiplier(zero)[30:], 1.0, atol=0.3)
        rest = _rest_protocol_trace(backend, np.zeros((100, 3)), rest_threshold=0.0)
        np.testing.assert_allclose(_effective_recovery_multiplier(rest), rest["r"], rtol=0.05)

    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_set_rest_threshold_matches_constructor(self, backend: str) -> None:
        loads = np.vstack([_rest_protocol(), np.full((300, 3), 0.04)])
        built = _rest_protocol_trace(backend, loads, rest_threshold=0.05)
        default = _rest_protocol_trace(backend, loads)
        assert not np.allclose(built["MF"], default["MF"], atol=1e-3)

        if backend == "numpy":
            model = _make_fatigue_model(3)
        else:
            torch = pytest.importorskip("torch")
            from myo_sim.physics.fatigue import (
                _UNIFORM_PARAMS,
                TorchFatigueState,
            )

            model = TorchFatigueState(1, 3, **{k: _UNIFORM_PARAMS[k] for k in "FRr"})
        model.set_RestThreshold(0.05)
        assert model.rest_threshold == 0.05
        model.reset(fatigue_reset_vec=np.full(3, 0.3))
        mf = []
        for tl in loads:
            if backend == "numpy":
                model.compute_act(tl.copy(), dt=0.02)
                mf.append(model.MF.copy())
            else:
                model.step(torch.tensor(tl[None], dtype=torch.float32), 0.02)
                mf.append(model.MF[0].double().numpy())
        np.testing.assert_array_equal(np.array(mf), built["MF"][1:])

    def test_custom_rest_threshold_cpu_matches_torch(self) -> None:
        """CPU and torch agree when 4 % commands count as rest."""
        pytest.importorskip("torch")
        loads = np.tile(np.repeat([0.5, 0.04, 0.2, 0.0], 300)[:, None], (2, 3))
        cpu = _rest_protocol_trace("numpy", loads, rest_threshold=0.05)
        gpu = _rest_protocol_trace("torch", loads, rest_threshold=0.05)
        for k in ("MA", "MF"):
            np.testing.assert_allclose(gpu[k], cpu[k], atol=1e-5, err_msg=k)
        default = _rest_protocol_trace("numpy", loads)
        assert np.abs(cpu["MF"] - default["MF"]).max() > 1e-2

    @pytest.mark.parametrize("value", [-0.01, 1.0, 2.0, np.nan, np.inf, np.array([0.01, 0.02])])
    def test_invalid_rest_threshold_is_rejected(self, value: Any) -> None:
        pytest.importorskip("torch")
        from myo_sim.physics.fatigue import (
            FATIGUE_REST_THRESHOLD,
            TorchFatigueState,
        )

        with pytest.raises(ValueError, match="rest_threshold"):
            _make_fatigue_model(2, rest_threshold=value)
        with pytest.raises(ValueError, match="rest_threshold"):
            TorchFatigueState(1, 2, rest_threshold=value)
        with pytest.raises(ValueError, match="rest_threshold"):
            TorchFatigueState.from_mj_model(_mock_muscle_model(2), 1, use_uniform_params=True, rest_threshold=value)
        for model in (_make_fatigue_model(2), TorchFatigueState(1, 2)):
            with pytest.raises(ValueError, match="rest_threshold"):
                model.set_RestThreshold(value)
            assert model.rest_threshold == FATIGUE_REST_THRESHOLD

    def test_sustained_command_fatigue_matches_fine_step(self) -> None:
        # At the 20 ms control step MF must match the same ODE integrated at
        # the 2 ms physics step. Overshooting MA to TL used to select the
        # recovery rate r * R (then applied at MA >= TL) on every other step
        # and under-counted MF (-9%).
        coarse, fine = _make_fatigue_model(1), _make_fatigue_model(1)
        tl = np.full(1, 0.3)
        for _ in range(1500):  # 30 s
            coarse.compute_act(tl, dt=0.02)
            for _ in range(10):
                fine.compute_act(tl, dt=0.002)
        np.testing.assert_allclose(coarse.MF, fine.MF, rtol=0.02)

    @pytest.mark.parametrize("dt", (*_FATIGUE_CTRL_DTS, 1.0, 60.0, 300.0))
    @pytest.mark.parametrize("backend", ["numpy", "torch", "jax"])
    def test_step_is_exact_for_frozen_rates(self, backend: str, dt: float) -> None:
        # One step of any length is the exact solution of the 3CC-r ODE with LD, LR
        # held at their start values, through every regime switch inside the step.
        ma0, mf0, tl = _EXACT_STEP_CASES.T
        got = np.stack(_exact_step(backend, ma0, mf0, tl, dt))
        want = np.stack([_frozen_rate_reference(*c, dt) for c in _EXACT_STEP_CASES], axis=1)
        # float64 matches to < 1e-9, float32 to ~1e-7
        np.testing.assert_allclose(got, want, atol=1e-8 if backend == "numpy" else 1e-6)
        assert np.all((got >= 0) & (got <= 1))
        np.testing.assert_allclose(got.sum(0), 1.0, atol=1e-12 if backend == "numpy" else 1e-6)

    @pytest.mark.parametrize("dt", (0.02, 1.0, 30.0))
    @pytest.mark.parametrize("backend", ["numpy", "torch", "jax"])
    def test_step_keeps_the_regime_of_its_equilibrium(self, backend: str, dt: float) -> None:
        # A step starting just inside MF = 1 - TL stays in the regime that holds its
        # equilibrium. The fast MA transient used to read as a slow-mode crossing: an
        # MR-limited step switched to free recruitment at once (MA overshot to ~TL,
        # MF was clipped), and a free step with a free equilibrium switched to limited.
        cases = np.array(
            [
                [0.3776, 0.1 + 1e-6, 0.9],  # MR-limited, just past the boundary
                [0.3776, 0.1 + 1e-4, 0.9],
                [0.0, 0.4 + 1e-5, 0.6],
                [0.149994, 0.85 - 1e-10, 0.15],  # free, F MA* < R (1 - TL); float64
            ]
        )
        ma0, mf0, tl = cases.T
        got = np.stack(_exact_step(backend, ma0, mf0, tl, dt))
        want = np.stack([_frozen_rate_reference(*c, dt) for c in cases], axis=1)
        np.testing.assert_allclose(got, want, atol=1e-8 if backend == "numpy" else 1e-6)

    @pytest.mark.filterwarnings("error")
    @pytest.mark.parametrize("dt", (0.02, 300.0))
    @pytest.mark.parametrize("backend", ["numpy", "torch", "jax"])
    def test_step_degenerate_rates_stay_finite(self, backend: str, dt: float) -> None:
        # No fatigue (F = 0), no recovery (R = 0) and NaN commands (C = 0)
        ma, mf = np.array([0.2, 0.2, 0.0, 0.3]), np.array([0.3, 0.3, 0.5, 0.2])
        tl = np.array([0.6, 0.9, 0.95, np.nan])
        for params in ({"F": 0.0}, {"R": 0.0}, {"F": 0.0, "R": 0.0}):
            got = np.stack(_exact_step(backend, ma, mf, tl, dt, **params))
            assert np.all((got >= 0) & (got <= 1)), params
            np.testing.assert_allclose(got.sum(0), 1.0, atol=1e-6)
        no_fatigue = _exact_step(backend, ma, mf, tl, dt, F=0.0)
        recovered = mf[:3] * np.exp(-_ELBOW["R"] * dt)  # MF only recovers
        np.testing.assert_allclose(no_fatigue[2][:3], recovered, rtol=1e-5)

    @pytest.mark.parametrize("backend", ["numpy", "torch"])
    def test_long_step_matches_control_steps(self, backend: str) -> None:
        # One explicit step held the start-of-step MA for the whole interval:
        # a 120 s rest passed in one call kept fatiguing the muscle (MF rose)
        # instead of recovering it. The exact step integrates MA over the step;
        # the rest differs from 20 ms steps only by the frozen LR (2e-4).
        hold, rest = np.full(3, 0.4), np.zeros(3)
        one_call = _fatigue_state_stepper(backend, 3)
        control = _fatigue_state_stepper(backend, 3)
        for _ in range(3000):  # 60 s at 40 %
            one_call(hold, 0.02)
            mf_hold = control(hold, 0.02)[2]
        ma, mr, mf = one_call(rest, 120.0)
        for _ in range(6000):
            reference = control(rest, 0.02)
        assert np.all(mf < mf_hold)
        np.testing.assert_allclose(mf, reference[2], rtol=1e-3)
        # MR = 1 - MA - MF, so the sum stays 1 to rounding.
        np.testing.assert_allclose(ma + mr + mf, 1.0, atol=1e-6 if backend == "torch" else 1e-12)
        assert np.all((ma >= 0) & (mr >= 0) & (mf >= 0))

    def test_per_muscle_params_ignore_attach_prefix_and_side_suffix(self) -> None:
        """Prefixed and side-suffixed muscle names get their group's F / R / r.

        mjlab scenes prefix actuators (``robot/BIClong``); ``myohand_r.xml`` and
        the left arm of full-body models suffix the side (``ECRL_r``, ``BIClong_l``).
        """
        import mujoco

        from myo_sim.physics.fatigue import (
            MUSCLE_FATIGUE_PARAMS,
            CumulativeFatigue,
        )

        def fatigue(names: list[str]) -> CumulativeFatigue:
            mock = MagicMock()
            mock.opt.timestep = 0.002
            mock.nu = len(names)
            mock.actuator_dyntype = np.full(len(names), mujoco.mjtDyn.mjDYN_MUSCLE)
            mock.actuator_dynprm = np.tile([0.01, 0.04] + [0.0] * 8, (len(names), 1))
            mock.actuator.side_effect = lambda i: SimpleNamespace(name=names[i])
            return CumulativeFatigue(mock)

        got = fatigue(["robot/BIClong", "ECRL_r", "robot/ECRL_r", "BIClong_l", "robot/soleus_r"])
        want = fatigue(["BIClong", "ECRL", "ECRL", "BIClong", "soleus_r"])
        assert np.all(want.F != MUSCLE_FATIGUE_PARAMS["Default"]["F"])
        for p in ("F", "R", "r"):
            np.testing.assert_array_equal(getattr(got, p), getattr(want, p), p)

    def test_get_effort_is_distance_to_target_load(self) -> None:
        """get_effort() (legacy API, JAX twins) is ||MA - TL|| after compute_act."""
        f = _make_fatigue_model(3)
        tl = np.array([0.2, 0.6, 1.0])
        f.compute_act(tl, dt=0.02)
        assert f.get_effort() == pytest.approx(float(np.linalg.norm(f.MA - tl)))
        assert f.get_effort() > 0.0

    def test_torch_nan_excitation_does_not_poison_state(self) -> None:
        """One NaN excitation gives C = 0 on torch as on CPU; the state stays finite."""
        torch = pytest.importorskip("torch")
        import mujoco

        from myo_sim.physics.fatigue import CumulativeFatigue, TorchFatigueState

        model = mujoco.MjModel.from_xml_string(_FATIGUE_MUSCLE_XML)
        cpu = CumulativeFatigue(model)
        gpu = TorchFatigueState.from_mj_model(model, num_envs=2)
        rng = np.random.default_rng(0)
        for k in range(40):
            tl = rng.uniform(0.0, 1.0, cpu.na)
            if k == 5:
                tl[0] = np.nan
            cpu.compute_act(tl.copy(), dt=0.02)
            gpu.step(torch.tensor(np.stack([tl, np.nan_to_num(tl)])), 0.02)
            assert bool(torch.isfinite(gpu.MA).all())
            np.testing.assert_allclose(gpu.MA[0].numpy(), cpu.MA, atol=1e-5)
            np.testing.assert_allclose(gpu.MF[0].numpy(), cpu.MF, atol=1e-5)

    def test_torch_state_dtype_is_fixed(self) -> None:
        """A float64 excitation no longer switches the float32 state to float64."""
        torch = pytest.importorskip("torch")
        from myo_sim.physics.fatigue import TorchFatigueState

        f32 = TorchFatigueState(num_envs=1, n_muscles=3)
        f64 = TorchFatigueState(num_envs=1, n_muscles=3)
        tl = torch.tensor([[0.2, 0.5, 0.9]], dtype=torch.float64)
        out = f64.step(tl, 0.02)
        f32.step(tl.float(), 0.02)
        assert f64.MA.dtype == f64.MR.dtype == f64.MF.dtype == out.dtype == torch.float32
        torch.testing.assert_close(f64.MA, f32.MA)

    @pytest.mark.parametrize("device", ["cpu", "cuda"])
    def test_compiled_torch_step_matches_eager(self, device: str) -> None:
        """torch.compile fuses the exact step and changes only float32 round-off."""
        torch = pytest.importorskip("torch")
        if device == "cuda" and not torch.cuda.is_available():
            pytest.skip("CUDA unavailable")
        from myo_sim.physics import fatigue as mc

        g = torch.Generator().manual_seed(0)
        n_env, n = 64, 39

        def rand(*shape: int) -> Any:
            return torch.rand(*shape, generator=g, dtype=torch.float64).to(device)

        p64 = {
            "F": 0.005 + 0.015 * rand(n),
            "R": 0.0008 + 0.004 * rand(n),
            "r": 1.0 + 24.0 * rand(n),
            "tauact": torch.full((n,), 0.01, dtype=torch.float64, device=device),
            "taudeact": torch.full((n,), 0.04, dtype=torch.float64, device=device),
        }
        p = {k: v.float() for k, v in p64.items()}
        ma0 = 0.8 * rand(n_env, n)
        mf0 = (1.0 - ma0) * rand(n_env, n)
        ma0[0], mf0[1] = 0.0, 1.0 - ma0[1]  # empty MA, empty MR

        def command() -> Any:
            tl = 1.2 * rand(n_env, n) - 0.1  # rest, negative and > 1 commands
            tl[:, :5] = 0.0
            return tl

        def step(state: tuple, tl: Any, dt: float, compile: bool) -> tuple:
            if state[0].dtype == torch.float64:
                out = mc.torch_three_cc_r_step(*state, tl, dt=dt, compile=False, **p64)
            else:
                out = mc.torch_three_cc_r_step(*state, tl.float(), dt=dt, compile=compile, **p)
            return out[0], out[2]

        start = (ma0.float(), mf0.float())
        step(start, command(), 0.02, compile=True)  # compiles, or falls back
        if (str(start[0].device), torch.float32, (n_env, n)) in mc._eager_keys:
            pytest.skip(f"torch.compile unavailable on {device}")
        nan_tl = command()
        nan_tl[2, 0] = float("nan")
        for a, b in zip(step(start, nan_tl, 0.02, True), step(start, nan_tl, 0.02, False)):
            torch.testing.assert_close(a, b, atol=1e-6, rtol=0.0)
        # One step from the same state, control steps to long rests.
        for dt in (0.01, 0.02, 0.025, 1.0, 300.0):
            tl = command()
            for a, b in zip(step(start, tl, dt, True), step(start, tl, dt, False)):
                torch.testing.assert_close(a, b, atol=1e-6, rtol=0.0)
        # A 20 ms rollout with a new command every step. Near a regime switch float32
        # rounding can move either path by ~1e-4 (rarely), so also compare to float64.
        fused, eager, exact = start, start, (ma0, mf0)
        for _ in range(250):
            tl = command()
            fused = step(fused, tl, 0.02, compile=True)
            eager = step(eager, tl, 0.02, compile=False)
            exact = step(exact, tl, 0.02, compile=False)
        for a, b, c in zip(fused, eager, exact):
            torch.testing.assert_close(a, b, atol=1e-5, rtol=0.0)
            err_fused = float((a.double() - c).abs().max())
            err_eager = float((b.double() - c).abs().max())
            assert err_fused <= err_eager + 1e-6

    def test_compile_switch_reads_both_env_vars(self, monkeypatch: pytest.MonkeyPatch) -> None:
        from myo_sim.physics import fatigue as mc

        monkeypatch.delenv("MYO_SIM_FATIGUE_COMPILE", raising=False)
        monkeypatch.setenv("MYOSUITE_FATIGUE_COMPILE", "0")
        assert mc._compile_mode() == "0"
        monkeypatch.setenv("MYO_SIM_FATIGUE_COMPILE", "1")
        assert mc._compile_mode() == "1"

    def test_seed_matches_gymnasium(self) -> None:
        """The reset generator is the one gymnasium's seeding gives (MyoSuite's envs used it)."""
        seeding = pytest.importorskip("gymnasium.utils.seeding")
        f = _make_fatigue_model(5, seed=7)
        rng, seed = seeding.np_random(7)
        assert f.seed(7) == [seed]
        np.testing.assert_array_equal(f.np_random.random(5), rng.random(5))
        with pytest.raises(ValueError):
            f.seed(-1)

    def test_uniform_params_match_across_backends(self) -> None:
        """use_uniform_params: the v2.4 row and the model's tau on CPU and torch."""
        torch = pytest.importorskip("torch")
        import mujoco

        from myo_sim.physics.fatigue import (
            MUSCLE_FATIGUE_PARAMS,
            CumulativeFatigue,
            TorchFatigueState,
        )

        model = mujoco.MjModel.from_xml_string(_FATIGUE_MUSCLE_XML)
        cpu = CumulativeFatigue(model, use_uniform_params=True)
        gpu = TorchFatigueState.from_mj_model(model, 1, use_uniform_params=True)
        v24 = MUSCLE_FATIGUE_PARAMS["Default_v2_4"]
        for p in ("F", "R", "r"):
            np.testing.assert_allclose(getattr(cpu, p), v24[p])
        np.testing.assert_allclose(gpu._tauact.numpy(), model.actuator_dynprm[:, 0])
        np.testing.assert_allclose(gpu._taudeact.numpy(), model.actuator_dynprm[:, 1])
        rng = np.random.default_rng(1)
        for _ in range(200):
            tl = rng.uniform(0.0, 1.0, cpu.na)
            cpu.compute_act(tl, dt=0.02)
            gpu.step(torch.tensor(tl[None], dtype=torch.float32), 0.02)
        np.testing.assert_allclose(gpu.MA[0].numpy(), cpu.MA, atol=1e-5)
        np.testing.assert_allclose(gpu.MF[0].numpy(), cpu.MF, atol=1e-5)

    def test_torch_reset_vec_matches_cpu(self) -> None:
        """A fixed fatigue vector resets the chosen envs like the CPU model."""
        torch = pytest.importorskip("torch")
        from myo_sim.physics.fatigue import TorchFatigueState

        cpu = _make_fatigue_model(3)
        gpu = TorchFatigueState(num_envs=4, n_muscles=3)
        for _ in range(30):
            cpu.compute_act(np.full(3, 0.7), dt=0.02)
            gpu.step(torch.full((4, 3), 0.7), 0.02)
        before = gpu.MF.clone()
        vec = np.array([0.1, 0.4, 0.8])
        cpu.reset(fatigue_reset_vec=vec)
        gpu.reset(torch.tensor([1, 3]), fatigue_reset_vec=vec)
        for name in ("MA", "MR", "MF"):
            got = getattr(gpu, name)[[1, 3]].double().numpy()
            np.testing.assert_allclose(got, np.tile(getattr(cpu, name), (2, 1)), atol=1e-7)
        torch.testing.assert_close(gpu.MF[[0, 2]], before[[0, 2]])

    @pytest.mark.parametrize("device", ["cpu", "cuda"])
    @pytest.mark.parametrize("kind", ["list", "numpy", "numpy-reversed", "tensor", "tensor-grad"])
    def test_torch_reset_accepts_vectors_on_the_state_device(self, device: str, kind: str) -> None:
        """Tensor resets work without a host NumPy conversion, including CUDA tensors."""
        torch = pytest.importorskip("torch")
        from myo_sim.physics.fatigue import TorchFatigueState

        if device == "cuda" and not torch.cuda.is_available():
            pytest.skip("CUDA is unavailable")
        fatigue = TorchFatigueState(num_envs=4, n_muscles=3, device=device)
        fatigue.MA.fill_(0.2)
        fatigue.MR.fill_(0.5)
        fatigue.MF.fill_(0.3)
        before = tuple(state.clone() for state in (fatigue.MA, fatigue.MR, fatigue.MF))
        values = [0.1, 0.4, 0.8]
        if kind == "list":
            vec = values
        elif kind == "numpy":
            vec = np.array(values, dtype=np.float64)
        elif kind == "numpy-reversed":
            vec = np.array(values[::-1], dtype=np.float64)[::-1]
        else:
            vec = torch.tensor(values, dtype=torch.float64, device=device, requires_grad=kind == "tensor-grad")
        rows = torch.tensor([1, 3], device=device)
        fatigue.reset(rows, fatigue_reset_vec=vec)

        expected_mf = torch.tensor(values, dtype=torch.float32, device=device).expand(2, -1)
        for state, expected, original in zip(
            (fatigue.MA, fatigue.MR, fatigue.MF),
            (torch.zeros_like(expected_mf), 1 - expected_mf, expected_mf),
            before,
        ):
            assert state.dtype == torch.float32
            assert state.device.type == device
            torch.testing.assert_close(state[rows], expected)
            torch.testing.assert_close(state[[0, 2]], original[[0, 2]])
        if isinstance(vec, torch.Tensor):
            torch.testing.assert_close(vec.detach(), torch.tensor(values, dtype=torch.float64, device=device))

    def test_torch_reset_random_matches_cpu_distribution(self) -> None:
        """Random resets: seeded, per env, and the CPU construction of the state.

        CPU and torch draw from different generators, so the states are compared
        in distribution: ``MF = 1 - nf`` and ``MA / (MA + MR) = ap`` are uniform.
        """
        torch = pytest.importorskip("torch")
        from myo_sim.physics.fatigue import TorchFatigueState

        n = 3
        gpu = TorchFatigueState(num_envs=4000, n_muscles=n)
        gpu.reset(fatigue_reset_random=True, generator=torch.Generator().manual_seed(0))
        again = TorchFatigueState(num_envs=4000, n_muscles=n)
        again.reset(fatigue_reset_random=True, generator=torch.Generator().manual_seed(0))
        torch.testing.assert_close(gpu.MF, again.MF)
        total = (gpu.MA + gpu.MR + gpu.MF).double().numpy()
        np.testing.assert_allclose(total, 1.0, atol=1e-6)
        assert len(torch.unique(gpu.MF[:, 0])) > 3900  # one draw per env

        cpu = _make_fatigue_model(n)
        rng = np.random.default_rng(0)
        states = []
        for _ in range(4000):
            cpu.reset(fatigue_reset_random=True, np_random=rng)
            states.append((cpu.MA.copy(), cpu.MR.copy(), cpu.MF.copy()))
        ma, mr, mf = (np.stack(s) for s in zip(*states))
        q = np.linspace(0.05, 0.95, 19)
        for cpu_x, gpu_x in (
            (mf, gpu.MF.double().numpy()),
            (ma, gpu.MA.double().numpy()),
            (ma / (ma + mr), (gpu.MA / (gpu.MA + gpu.MR)).double().numpy()),
        ):
            np.testing.assert_allclose(np.quantile(gpu_x, q), np.quantile(cpu_x, q), atol=0.02)

    def test_torch_reset_rejects_invalid_options(self) -> None:
        """Both options at once, or a vector of the wrong length, raise as on CPU."""
        torch = pytest.importorskip("torch")
        from myo_sim.physics.fatigue import TorchFatigueState

        gpu = TorchFatigueState(num_envs=2, n_muscles=3)
        with pytest.raises(ValueError, match="Cannot pass"):
            gpu.reset(fatigue_reset_vec=[0.1] * 3, fatigue_reset_random=True)
        with pytest.raises(ValueError, match="length"):
            gpu.reset(fatigue_reset_vec=[0.1, 0.2])
        for vec in (torch.tensor([0.1, 0.2]), torch.zeros(1, 3), torch.tensor(0.1)):
            with pytest.raises(ValueError, match="length"):
                gpu.reset(fatigue_reset_vec=vec)
        torch.testing.assert_close(gpu.MR, torch.ones(2, 3))


# Three muscles; the first two have non-default activation time constants.
_FATIGUE_MUSCLE_XML = """
<mujoco>
  <worldbody>
    <body>
      <joint name="j" type="hinge" range="-1 1" limited="true"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <muscle name="m0" joint="j" timeconst="0.02 0.08"/>
    <muscle name="m1" joint="j" timeconst="0.005 0.03"/>
    <muscle name="m2" joint="j"/>
  </actuator>
</mujoco>
"""


@pytest.mark.parametrize(
    ("name", "group"),
    [
        ("BIClong", "Elbow-Flexor"),
        ("robot/BIClong", "Elbow-Flexor"),  # mjlab scene prefix
        ("ECRL_r", "Wrist-Extensor"),  # hand-pose models
        ("BIClong_left", "Elbow-Flexor"),  # MuscleMimic left arm
        ("a1_BIClong_left", "Elbow-Flexor"),  # two-agent scene
        ("robot/a0_soleus_r", "Ankle-Plantarflexor"),
        ("R:ECRL", "Wrist-Extensor"),  # side prefix
        ("soleus_l", "Ankle-Plantarflexor"),  # leg muscles keep their side
        ("rect_abd_r", "Trunk"),
        ("QL_post_I.2-L3_l", "Trunk"),
        ("Ps_L5_VB_r", "Trunk"),  # spinal psoas fascicle
        ("psoas_r", "Hip-Flexor"),  # leg psoas
        ("PT", "Elbow-Pronator"),
        ("APL", "Wrist-Extensor"),
        ("unknown_muscle", "Default"),
    ],
)
def test_muscle_group_name_rules(name: str, group: str) -> None:
    assert _muscle_group(name) == group


def test_groups_are_declared_and_resolve_to_a_row() -> None:
    """Every assigned group is declared and has a row of its own or of its parent joint."""
    assert set(MUSCLE_FMG.values()) <= FUNCTIONAL_MUSCLE_GROUPS
    rowless = {g for g in set(MUSCLE_FMG.values()) if g not in MUSCLE_FATIGUE_PARAMS}
    assert {g for g in rowless if g.split("-")[0] not in MUSCLE_FATIGUE_PARAMS} == set()


@pytest.mark.parametrize("name", ["myoarm_r", "myolegs", "myotorso"])
def test_every_muscle_of_the_composed_models_has_a_group(name: str) -> None:
    """No muscle of these models falls back to Default by name."""
    import mujoco

    import myo_sim

    model = myo_sim.load_spec(name).compile()
    muscles = [model.actuator(i).name for i in range(model.nu) if model.actuator_dyntype[i] == mujoco.mjtDyn.mjDYN_MUSCLE]
    assert muscles
    assert [m for m in muscles if _muscle_group(m) == "Default"] == []
