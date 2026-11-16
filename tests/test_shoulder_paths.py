"""Shoulder paths fitted by ``scripts/fit_shoulder_paths.py``: XML equals the table, the fit reproduces, no length jumps."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim

ROOT = Path(__file__).resolve().parents[1]
TABLE = json.loads((ROOT / "myo_sim" / "build" / "data" / "shoulder_paths.json").read_text())
CHANGED = ("DELT2", "DELT3", "INFSP1", "INFSP2", "SUBSC1", "SUBSC2", "SUBSC3")
REMOVED_GEOMS = ("Delt3_torus_r", "delt3hum_ellipsoid_DELT3_r", "INFSP_cylinder_r", "SUBSC_cylinder_r")
STEP = 2e-3  # rad
MAX_STEP_CHANGE = 0.5e-3  # m per step; a 30 mm arm changes the length by 0.06 mm
MARGIN = 0.13  # rad past the limits, as in test_tendon_path_continuity


def _script():
    name = "fit_shoulder_paths"
    if name not in sys.modules:
        sys.path.insert(0, str(ROOT / "scripts"))
        spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
        sys.modules[name] = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sys.modules[name])
    return sys.modules[name]


@pytest.fixture(scope="module")
def arm() -> mujoco.MjModel:
    return myo_sim.load_model("myoarm_r")


def _path_sites(model: mujoco.MjModel, tendon: str) -> list[str]:
    t = model.tendon(tendon).id
    adr, num = model.tendon_adr[t], model.tendon_num[t]
    return [
        model.site(model.wrap_objid[k]).name for k in range(adr, adr + num) if model.wrap_type[k] == mujoco.mjtWrap.mjWRAP_SITE
    ]


def test_sites_are_where_the_table_says(arm: mujoco.MjModel) -> None:
    for name, entry in TABLE["kept_sites"].items():
        sid = arm.site(name).id
        assert arm.body(arm.site_bodyid[sid]).name == entry["body"]
        np.testing.assert_allclose(arm.site_pos[sid], entry["pos"], atol=1e-9)
    for muscle, entry in TABLE["cuff_vias"].items():
        sid = arm.site(f"{muscle}-V1_r").id
        np.testing.assert_allclose(arm.site_pos[sid], entry["pos"], atol=1e-9)
        assert _path_sites(arm, f"{muscle}_r_tendon") == [f"{muscle}-P1_r", f"{muscle}-V1_r", f"{muscle}-P2_r"]
    assert _path_sites(arm, "DELT3_r_tendon") == [f"DELT3_DELT3-P{k}_r" for k in (1, 2, 3)]
    for geom in REMOVED_GEOMS:
        assert mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_GEOM, geom) == -1


def test_free_sites_stay_outside_the_head(arm: mujoco.MjModel) -> None:
    """Cuff vias 25 mm or more from the humeral-head sphere's centre (radius 23.5 mm), the DELT3 via 28 mm from the joint."""
    c = _script().centres(arm)
    for muscle in TABLE["cuff_vias"]:
        assert np.linalg.norm(arm.site_pos[arm.site(f"{muscle}-V1_r").id] - c["humeral_head"]) >= 0.025 - 1e-6
    assert np.linalg.norm(arm.site_pos[arm.site("DELT3_DELT3-P2_r").id] - c["glenohumeral"]) >= 0.028 - 1e-6


def test_fit_summary_reproduces() -> None:
    """The stored moment arms are the model's (same convention as validate_shoulder_mechanics), within 0.05 mm."""
    script = _script()
    a = script.vsm.Arm()
    grid = script.Grid(a)
    for design, stored in zip(script.DESIGNS, TABLE["designs"]):
        fit = script.Fit(design, a.model, grid)
        summary = fit.summary(fit.x0)
        for plane, row in stored["fit"]["planes"].items():
            np.testing.assert_allclose(
                summary["planes"][plane]["model_mm"], row["model_mm"], atol=0.05, err_msg=f"{design['muscle']} {plane}"
            )


@pytest.mark.parametrize("name", ["myoarm_r", "myoarms"])
def test_changed_paths_are_continuous_over_the_glenohumeral_range(name: str) -> None:
    """Each glenohumeral hinge swept past its limits at several postures of the others; no length step above 0.5 mm."""
    model = myo_sim.load_model(name)
    data = mujoco.MjData(model)
    sides = ("r", "l") if name == "myoarms" else ("r",)
    for side in sides:
        tendons = [model.tendon(f"{m}_{side}_tendon").id for m in CHANGED]
        joints = {j: model.joint(f"{j}_{side}") for j in ("elv_angle", "shoulder_elv", "shoulder_rot")}
        bases = [{}, {"shoulder_elv": 1.2}, {"shoulder_elv": 2.2, "elv_angle": 1.0}, {"shoulder_elv": 0.6, "shoulder_rot": 0.8}]
        for sweep, joint in joints.items():
            for base in bases:
                angles = np.arange(joint.range[0] - MARGIN, joint.range[1] + MARGIN, STEP)
                lengths = np.empty((len(angles), len(tendons)))
                for i, v in enumerate(angles):
                    data.qpos[:] = model.qpos0
                    for j, x in base.items():
                        data.qpos[joints[j].qposadr[0]] = x
                    data.qpos[joint.qposadr[0]] = v
                    mujoco.mj_kinematics(model, data)
                    mujoco.mj_tendon(model, data)
                    lengths[i] = data.ten_length[tendons]
                assert np.all(np.isfinite(lengths))
                jumps = np.abs(np.diff(lengths, axis=0))
                i, k = np.unravel_index(np.argmax(jumps), jumps.shape)
                assert jumps[i, k] < MAX_STEP_CHANGE, (
                    f"{CHANGED[k]}_{side} jumps by {jumps[i, k] * 1e3:.2f} mm at {sweep} {angles[i]:+.3f} rad, base {base}"
                )
