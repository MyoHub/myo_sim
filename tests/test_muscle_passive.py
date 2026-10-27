"""Opt-in passive muscle tension of the index extrinsic muscles (myo_sim.build.muscle_passive)."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.muscle_passive import load_muscle_passive_table

MODELS = ("myofullbody", "myotorso_arms", "myohands", "myoarm_r")
DEG = np.pi / 180.0


def _implied_slack(model: mujoco.MjModel, act: int) -> float:
    lo, hi = model.actuator_lengthrange[act]
    r0, r1 = model.actuator_gainprm[act][:2]
    return float(lo - r0 * (hi - lo) / (r1 - r0))


def _right(model: mujoco.MjModel, name: str) -> int:
    for candidate in (name, f"{name}_r"):
        try:
            return model.actuator(candidate).id
        except KeyError:
            continue
    raise KeyError(name)


@pytest.mark.parametrize("name", MODELS)
def test_opt_out_leaves_the_model_unchanged(name: str) -> None:
    base = myo_sim.load_spec(name).compile()
    off = myo_sim.load_spec(name, passive_muscles=False).compile()
    for field in ("actuator_lengthrange", "actuator_gainprm", "tendon_stiffness", "jnt_range", "qpos0"):
        np.testing.assert_array_equal(getattr(off, field), getattr(base, field), err_msg=field)


@pytest.mark.parametrize("name", MODELS)
def test_flag_sets_the_tabled_slack_lengths_on_both_arms(name: str) -> None:
    base = myo_sim.load_spec(name).compile()
    model = myo_sim.load_spec(name, passive_muscles=True).compile()
    rows = load_muscle_passive_table()["muscles"]
    edited = {
        base.actuator(i).name
        for i in range(base.nu)
        if not np.allclose(base.actuator_lengthrange[i], model.actuator_lengthrange[i])
    }
    sides = ("r",) if name == "myoarm_r" else ("r", "l")
    expected = set()
    for row in rows:
        for side in sides:
            act = _right(model, row["names"]["r"]) if side == "r" else model.actuator(row["names"]["l"]).id
            expected.add(model.actuator(act).name)
            assert _implied_slack(model, act) == pytest.approx(row["tendon_slack_m"]["value"], abs=1e-9)
            assert _implied_slack(base, act) == pytest.approx(row["tendon_slack_m"]["reference_value_m"], abs=1e-6)
            span = lambda m: np.diff(m.actuator_lengthrange[act])[0]  # noqa: E731
            assert span(model) == pytest.approx(span(base))  # optimal length unchanged
    assert edited == expected


def test_fit_matches_the_recorded_error() -> None:
    """The stored slack lengths reproduce the recorded fit to Knutson's extrinsic moment."""
    path = Path(__file__).parents[1] / "scripts" / "fit_muscle_passive.py"
    spec = importlib.util.spec_from_file_location("fit_muscle_passive", path)
    fit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fit)
    table = load_muscle_passive_table()
    model = myo_sim.load_spec("myoarm_r").compile()
    lengths, arms = fit.geometry(model)
    shift = np.array(
        [
            next(r for r in table["muscles"] if r["muscle"] == n)["tendon_slack_m"]["value"] - fit.implied_slack(model, n)
            for n in fit.MUSCLES
        ]
    )
    target = fit.knutson_extrinsic(fit.MCP_DEG[None, :], fit.WRIST_DEG[:, None])
    err = target - fit.model_torque(model, lengths, arms, shift)
    assert np.sqrt(np.mean(err**2)) == pytest.approx(table["fit"]["rmse_Ncm_after"], abs=1e-3)
    assert table["fit"]["rmse_Ncm_after"] < 0.25 * table["fit"]["rmse_Ncm_before"]


def _settle(flag: bool, wrist_deg: float) -> dict[str, float]:
    """Finger angles (deg) after 3 s without gravity or excitation, both wrists held at *wrist_deg*."""
    model = myo_sim.load_spec("myofullbody", passive_muscles=flag).compile()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    for side in ("r", "l"):
        for name in (f"flexion_{side}", f"deviation_{side}", f"pro_sup_{side}"):
            dof = model.joint(name).dofadr[0]
            model.dof_armature[dof] = model.dof_damping[dof] = 1e4  # hold the wrist and forearm
        data.qpos[model.joint(f"flexion_{side}").qposadr[0]] = wrist_deg * DEG
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return {
        n: float(np.degrees(data.qpos[model.joint(n).qposadr[0]]))
        for n in ("mcp2_flexion_r", "pm2_flexion_r", "mcp2_flexion_l", "pm2_flexion_l")
    }


def test_tenodesis_index_flexes_as_the_wrist_extends() -> None:
    extended, flexed = _settle(True, -60.0), _settle(True, 60.0)
    for side in ("r", "l"):
        assert extended[f"mcp2_flexion_{side}"] > flexed[f"mcp2_flexion_{side}"] + 10.0
        assert extended[f"pm2_flexion_{side}"] > flexed[f"pm2_flexion_{side}"] + 10.0
    for joint in ("mcp2_flexion", "pm2_flexion"):  # left / right symmetric
        assert extended[f"{joint}_r"] == pytest.approx(extended[f"{joint}_l"], abs=0.1)


def test_subject_scale_multiplies_slack() -> None:
    row = next(r for r in load_muscle_passive_table()["muscles"] if r["muscle"] == "FDS2")
    model = myo_sim.load_spec("myoarm_r", passive_muscles=True, subject_scale={"FDS2.slack": 1.02}).compile()
    assert _implied_slack(model, model.actuator("FDS2").id) == pytest.approx(1.02 * row["tendon_slack_m"]["value"])
