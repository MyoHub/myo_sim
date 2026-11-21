"""Shoulder-girdle ligaments as passive spatial tendons (``scripts/fit_ligaments.py``, ``build/data/ligaments.json``)."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.hand_data import blocks_match, read_generated_block

ROOT = Path(__file__).resolve().parents[1]
TABLE = json.loads((ROOT / "myo_sim" / "build" / "data" / "ligaments.json").read_text())
ROWS = TABLE["ligaments"]


def _script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("fit_ligaments", ROOT / "scripts" / "fit_ligaments.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rows_are_sourced() -> None:
    for row in ROWS:
        assert row["references"] and all(r in TABLE["references"] for r in row["references"]), row["name"]
        assert row["k_source"] and row["eps_rest_source"] and row["status"] in ("measured", "transferred")
        for end in ("origin", "insertion"):
            assert row[end]["landmark"]["source"], (row["name"], end)


def test_generated_blocks_are_what_the_script_writes() -> None:
    gen = _script()
    for (path, label), block in gen.blocks(TABLE).items():
        assert blocks_match(read_generated_block(path, gen.SCRIPT, label), block), (path.name, label)


def test_force_law_and_both_sides() -> None:
    """Tension-only quadratic toe, b = k / (2 e_t L0^2), no damping, mirrored on the left."""
    model = myo_sim.load_model("myoarms")
    for row in ROWS:
        for side in ("r", "l"):
            t = model.tendon(f"{row['name']}_ligament_{side}").id
            assert model.tendon_lengthspring[t][0] == 0.0
            assert model.tendon_lengthspring[t][1] == pytest.approx(row["l0_m"])
            b = row["k_N_per_strain"] / (2 * row["eps_t"] * row["l0_m"] ** 2)
            assert row["b_N_per_m2"] == pytest.approx(b, rel=1e-4)
            assert model.tendon_stiffness[t] == 0.0
            assert model.tendon_stiffnesspoly[t][0] == pytest.approx(row["b_N_per_m2"], rel=1e-5)  # 6 digits in the XML
            assert model.tendon_damping[t] == 0.0


def test_scapula_stays_on_the_thorax_near_rest_at_zero_activation() -> None:
    """6 s at zero activation from qpos0 (the arm settles in about 5 s): every girdle angle within 5 deg of rest, the
    scapula on its thoracic contact."""
    model = myo_sim.load_model("myoarm_r")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    scap, chest = model.body("scapula_r").id, model.body("chest_r").id
    r0 = data.xmat[chest].reshape(3, 3).T @ data.xmat[scap].reshape(3, 3)
    q0 = data.qpos.copy()
    for _ in range(int(6.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    for name in ("sternoclavicular_r2_r", "sternoclavicular_r3_r", "sternoclavicular_r1_r"):
        adr = model.joint(name).qposadr[0]
        assert abs(np.degrees(data.qpos[adr] - q0[adr])) < 5.0, name
    r = data.xmat[chest].reshape(3, 3).T @ data.xmat[scap].reshape(3, 3)
    angle = np.degrees(np.arccos(np.clip((np.trace(r0.T @ r) - 1) / 2, -1, 1)))
    assert angle < 5.0
    dist = mujoco.mj_geomDistance(
        model, data, model.geom("scapula_glide_r").id, model.geom("scapulothoracic_ellipsoid_r").id, 0.05, None
    )
    assert abs(dist) < 5e-4
