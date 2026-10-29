"""Extrinsic hand-muscle slack lengths and fpmax fitted to passive moments (myoarm_r_muscle.xml, build/data/muscle_passive.json)."""

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
def test_models_carry_the_tabled_values_on_both_arms(name: str) -> None:
    model = myo_sim.load_spec(name).compile()
    sides = ("r",) if name == "myoarm_r" else ("r", "l")
    for row in _fit_script().load_table()["muscles"]:
        for side in sides:
            act = _right(model, row["names"]["r"]) if side == "r" else model.actuator(row["names"]["l"]).id
            assert _implied_slack(model, act) == pytest.approx(row["tendon_slack_m"]["value"], abs=1e-6)  # stored to 1 um
            assert model.actuator_gainprm[act][7] == model.actuator_biasprm[act][7] == row["fpmax"]["value"]


def test_xml_is_what_the_fit_script_writes() -> None:
    """The lengthrange and fpmax attributes in myoarm_r_muscle.xml are the JSON values (rerunning --xml-only changes nothing)."""
    fit = _fit_script()
    model = myo_sim.load_spec("myoarm_r").compile()
    for name, lengthrange in fit.fitted_lengthranges(model, fit.load_table()).items():
        np.testing.assert_array_equal(model.actuator_lengthrange[model.actuator(name).id], lengthrange)
    for name, fpmax in fit.fitted_fpmax(fit.load_table()).items():
        assert model.actuator_gainprm[model.actuator(name).id][7] == fpmax
    for row in fit.load_table()["muscles"]:
        slack = row["tendon_slack_m"]
        assert 100.0 * (slack["value"] / slack["reference_value_m"] - 1.0) == pytest.approx(slack["change_percent"], abs=2e-3)


def test_fit_matches_the_recorded_error() -> None:
    """The stored values reproduce the recorded fit to Knutson's extrinsic moment."""
    fit = _fit_script()
    table = fit.load_table()
    model = myo_sim.load_spec("myoarm_r").compile()
    lengths, arms = fit.geometry(model)
    target = fit.knutson_extrinsic(fit.MCP_DEG[None, :], fit.WRIST_DEG[:, None])
    err = target - fit.model_torque(model, lengths, arms, np.zeros(len(fit.MUSCLES)))
    assert np.sqrt(np.mean(err**2)) == pytest.approx(table["fit"]["rmse_Ncm_after"], abs=1e-3)
    assert table["fit"]["rmse_Ncm_after"] < 0.25 * table["fit"]["rmse_Ncm_before"]


def test_joint_fit_matches_the_recorded_error() -> None:
    """The stored values reproduce the recorded one-sided fit to the total joint moments, and reduce every excess."""
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


def test_extensors_open_the_digits_in_the_release_trial() -> None:
    """EDC3-5 and EDM sit at or above the smallest fpmax that opens digits 3-5 in the tenodesis release trial."""
    fit = _fit_script()
    table = fit.load_table()
    spec = table["release_constraint"]
    bound = spec["result"]["fpmax_lower_bound"]
    model = myo_sim.load_spec("myoarm_r").compile()
    for name in spec["muscles"]:
        assert model.actuator_gainprm[model.actuator(name).id][7] >= bound, name
    if any(model.tendon(t).name.startswith("passive_") for t in range(model.ntendon)):
        pytest.skip("the bound was found without joint springs; with them, tests/test_hand_tenodesis.py checks the release")
    at_bound = fit.release_extension(table, bound)
    assert at_bound == pytest.approx(spec["result"]["mcp_extension_deg_at_bound"], abs=0.1)
    assert min(at_bound.values()) >= spec["min_mcp_extension_deg"]
    assert min(fit.release_extension(table, 0.95 * bound).values()) < spec["min_mcp_extension_deg"]


def _side(model: mujoco.MjModel, name: str, side: str) -> str:
    """Joint name of *name* (right-side name) on *side* of a composed model."""
    base = name[:-2] if name.endswith("_r") else name
    for candidate in (name, base, f"{base}_r") if side == "r" else (f"{base}_l",):
        try:
            model.joint(candidate)
            return candidate
        except KeyError:
            continue
    raise KeyError(name)


@pytest.mark.parametrize("posture", ("fist_wrist_neutral", "fist_wrist_flexed_30", "extension_wrist_extended_30"))
def test_flexors_close_and_extensors_open_the_hand_actively(posture: str) -> None:
    """MuJoCo's active force-length factor stays at least min_fl in a full fist (flexors) and full extension (extensors)."""
    spec = _fit_script().load_table()["active_constraint"]
    model = myo_sim.load_spec("myofullbody").compile()
    data = mujoco.MjData(model)
    for side in ("r", "l"):
        for joint, deg in spec["postures_deg"][posture].items():
            data.qpos[model.joint(_side(model, joint, side)).qposadr[0]] = deg * DEG
    mujoco.mj_forward(model, data)
    for name in spec["muscles"][posture]:
        for side in ("r", "l"):
            act = _right(model, name) if side == "r" else model.actuator(f"{name.removesuffix('_r')}_l").id
            prm = model.actuator_gainprm[act]
            fl = (
                -mujoco.mju_muscleGain(
                    data.actuator_length[act], 0.0, model.actuator_lengthrange[act], model.actuator_acc0[act], prm[:9]
                )
                / prm[2]
            )
            assert fl >= spec["min_fl"] - 1e-9, f"{name}_{side}: active factor {fl:.3f} in {posture}"


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
        assert extended[f"pm2_flexion_{side}"][0] > flexed[f"pm2_flexion_{side}"][0] + 10.0
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
    Known exception: with the wrist extended 60 deg the little-finger MCP and PIP reach their flexion
    limits (the PIP's equilibrium lies there: it gets there within 6 s with the class damping of 0.05
    N m s/rad, within 3 s with Kamper's finger damping). FDP5 keeps some passive force there because it
    offsets the PIP extension moment of the unfitted digit-5 intrinsics (LU_RB5, UI_UB5).
    """
    allowed = {f"{j}5_flexion_{s}" for j in ("mcp", "pm") for s in ("r", "l")} if wrist_deg <= -60.0 else set()
    for name, (q, lo, hi) in _settle(wrist_deg).items():
        finger = not name.startswith(("cmc", "mp_", "ip_"))
        assert name in allowed or q < hi - 1.0, f"{name} at its upper limit ({q:.1f} deg)"
        assert finger or q > lo + 1.0, f"{name} at its lower limit ({q:.1f} deg)"
    for base in HAND:
        assert _settle(wrist_deg)[f"{base}_r"][0] == pytest.approx(_settle(wrist_deg)[f"{base}_l"][0], abs=0.1)
