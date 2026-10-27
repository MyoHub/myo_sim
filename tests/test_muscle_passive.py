"""Extrinsic hand-muscle tendon slack lengths fitted to passive moments (myoarm_r_muscle.xml, build/data/muscle_passive.json)."""

from __future__ import annotations

import importlib.util
from functools import cache
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

import myo_sim

MODELS = ("myofullbody", "myotorso_arms", "myohands", "myoarm_r")
DEG = np.pi / 180.0


def _fit_script() -> ModuleType:
    path = Path(__file__).parents[1] / "scripts" / "fit_muscle_passive.py"
    spec = importlib.util.spec_from_file_location("fit_muscle_passive", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
def test_models_carry_the_tabled_slack_lengths_on_both_arms(name: str) -> None:
    model = myo_sim.load_spec(name).compile()
    sides = ("r",) if name == "myoarm_r" else ("r", "l")
    for row in _fit_script().load_table()["muscles"]:
        for side in sides:
            act = _right(model, row["names"]["r"]) if side == "r" else model.actuator(row["names"]["l"]).id
            assert _implied_slack(model, act) == pytest.approx(row["tendon_slack_m"]["value"], abs=1e-12)


def test_xml_is_what_the_fit_script_writes() -> None:
    """The four lengthrange attributes in myoarm_r_muscle.xml are the JSON slack lengths (rerunning --xml-only changes nothing)."""
    fit = _fit_script()
    model = myo_sim.load_spec("myoarm_r").compile()
    for name, lengthrange in fit.fitted_lengthranges(model, fit.load_table()).items():
        np.testing.assert_array_equal(model.actuator_lengthrange[model.actuator(name).id], lengthrange)
    for row in fit.load_table()["muscles"]:
        slack = row["tendon_slack_m"]
        assert 100.0 * (slack["value"] / slack["reference_value_m"] - 1.0) == pytest.approx(slack["change_percent"], abs=2e-3)


def test_fit_matches_the_recorded_error() -> None:
    """The stored slack lengths reproduce the recorded fit to Knutson's extrinsic moment."""
    fit = _fit_script()
    table = fit.load_table()
    model = myo_sim.load_spec("myoarm_r").compile()
    lengths, arms = fit.geometry(model)
    target = fit.knutson_extrinsic(fit.MCP_DEG[None, :], fit.WRIST_DEG[:, None])
    err = target - fit.model_torque(model, lengths, arms, np.zeros(len(fit.MUSCLES)))
    assert np.sqrt(np.mean(err**2)) == pytest.approx(table["fit"]["rmse_Ncm_after"], abs=1e-3)
    assert table["fit"]["rmse_Ncm_after"] < 0.25 * table["fit"]["rmse_Ncm_before"]


def test_joint_fit_matches_the_recorded_error() -> None:
    """The stored slack lengths reproduce the recorded one-sided fit to the total joint moments, and reduce every excess."""
    fit = _fit_script()
    table = fit.load_table()
    model = myo_sim.load_spec("myoarm_r").compile()
    zero = np.zeros(len(fit.JOINT_MUSCLES))
    for term in fit.joint_fit_terms(model, table):
        moment, target, recorded = fit.joint_moment(model, term, zero), term["target"], term["row"]["fit"]
        assert np.sqrt(np.mean((moment - target) ** 2)) == pytest.approx(
            recorded["rmse_Nm_after"], abs=1e-4
        )  # slack rounded to 1 um
        excess = np.max(np.abs(fit.excess_and_deficit(moment, target)[0]))
        assert excess == pytest.approx(recorded["max_excess_Nm_after"], abs=1e-4)
        assert recorded["max_excess_Nm_after"] <= recorded["max_excess_Nm_before"] + 1e-9
    for row in table["muscles"]:
        assert abs(row["tendon_slack_m"]["change_percent"]) <= 100.0 * table["joint_fit"]["max_change"] + 1e-3


HAND = ("cmc_flexion", "cmc_abduction", "mp_flexion", "ip_flexion") + tuple(
    f"{joint}{digit}_flexion" for digit in (2, 3, 4, 5) for joint in ("mcp", "pm", "md")
)


def _joint(model: mujoco.MjModel, base: str, side: str) -> int:
    for candidate in (f"{base}_{side}", base) if side == "r" else (f"{base}_{side}",):
        try:
            return model.joint(candidate).id
        except KeyError:
            continue
    raise KeyError(base)


@cache
def _settle(wrist_deg: float) -> dict[str, tuple[float, float, float]]:
    """Hand joint angles and limits (deg) after 3 s without gravity or excitation, both wrists held at *wrist_deg*."""
    model = myo_sim.load_spec("myofullbody").compile()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    for side in ("r", "l"):
        for name in (f"flexion_{side}", f"deviation_{side}", f"pro_sup_{side}"):
            dof = model.joint(name).dofadr[0]
            model.dof_armature[dof] = model.dof_damping[dof] = 1e4  # hold the wrist and forearm
        data.qpos[model.joint(f"flexion_{side}").qposadr[0]] = wrist_deg * DEG
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    out = {}
    for base in HAND:
        for side in ("r", "l"):
            j = _joint(model, base, side)
            lo, hi = np.degrees(model.jnt_range[j])
            out[f"{base}_{side}"] = (float(np.degrees(data.qpos[model.jnt_qposadr[j]])), float(lo), float(hi))
    return out


def test_tenodesis_index_flexes_as_the_wrist_extends() -> None:
    extended, flexed = _settle(-60.0), _settle(60.0)
    for side in ("r", "l"):
        assert extended[f"mcp2_flexion_{side}"][0] > flexed[f"mcp2_flexion_{side}"][0] + 10.0
        # PIP: flexes as the wrist extends (no source gives an amplitude; 2 deg margin).
        assert extended[f"pm2_flexion_{side}"][0] > flexed[f"pm2_flexion_{side}"][0] + 2.0
    for joint in ("mcp2_flexion", "pm2_flexion"):  # left / right symmetric
        assert extended[f"{joint}_r"][0] == pytest.approx(extended[f"{joint}_l"][0], abs=0.1)


@pytest.mark.parametrize("digit", (3, 4, 5))
def test_tenodesis_other_fingers_flex_as_the_wrist_extends(digit: int) -> None:
    extended, flexed = _settle(-60.0), _settle(60.0)
    for side in ("r", "l"):
        # No source gives an amplitude for digits 3-5 (joint springs shrink it); 2 deg margin.
        assert extended[f"mcp{digit}_flexion_{side}"][0] > flexed[f"mcp{digit}_flexion_{side}"][0] + 2.0


@pytest.mark.parametrize("wrist_deg", (-60.0, 0.0, 60.0))
def test_relaxed_hand_rests_off_its_limits(wrist_deg: float) -> None:
    """No finger or thumb joint is driven into a limit; only the finger extension limit (0 deg) is allowed.

    Holding the fingers off full extension is the job of joint springs, which this table does not add.
    """
    for name, (q, lo, hi) in _settle(wrist_deg).items():
        finger = not name.startswith(("cmc", "mp_", "ip_"))
        assert q < hi - 1.0, f"{name} at its upper limit ({q:.1f} deg)"
        assert finger or q > lo + 1.0, f"{name} at its lower limit ({q:.1f} deg)"
    for base in HAND:
        assert _settle(wrist_deg)[f"{base}_r"][0] == pytest.approx(_settle(wrist_deg)[f"{base}_l"][0], abs=0.1)
