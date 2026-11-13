"""Seth et al. 2019 shoulder table: provenance, licence and the frame registration against the current model."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim

DATA = Path(myo_sim.__file__).parent / "build" / "data"


def table() -> dict:
    return json.loads((DATA / "scapulothoracic.json").read_text())


def test_source_and_licence() -> None:
    src = table()["source"]
    assert "10.3389/fnbot.2019.00090" == src["doi"]
    assert "CC BY 4.0" in src["license"]
    text = (DATA / "LICENSE-Seth2019.txt").read_text()
    assert "Creative Commons Attribution 4.0" in text and "do not endorse" in text


def test_registration_matches_model() -> None:
    """The sternoclavicular centre the table was registered on is still where the arm attaches in chest_r."""
    reg = table()["registration"]
    for name in ("clavicle", "scapula", "humerus"):
        assert reg["meshes"][name]["seth_to_ours_max_mm"] < 1e-3
    model = myo_sim.load_model("myoarm_r")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    chest = model.body("chest_r").id
    sc = data.xmat[chest].reshape(3, 3).T @ (data.xpos[model.body("clavicle_r").id] - data.xpos[chest])
    np.testing.assert_allclose(sc, reg["sc_ours_in_chest_r"], atol=1e-6)
    np.testing.assert_allclose(np.add(reg["sc_seth_in_thorax"], reg["shift_seth_thorax_to_chest_r"]), sc, atol=1e-6)


def test_girdle_matches_table() -> None:
    """Design A in the arm chain: the values of the table, and the rest posture (qpos0) on the thoracic ellipsoid."""
    gird = table()["girdle"]
    model = myo_sim.load_model("myoarms")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for side, flip in (("r", 1.0), ("l", -1.0)):
        for name, ref in zip(("sternoclavicular_r2", "sternoclavicular_r3"), gird["clavicle"]["ref"]):
            joint = model.joint(f"{name}_{side}")
            assert model.qpos0[joint.qposadr[0]] == pytest.approx(ref)
        ac = model.joint(f"acromioclavicular_{side}")
        assert ac.type[0] == mujoco.mjtJoint.mjJNT_BALL and ac.range[1] == pytest.approx(gird["scapula"]["ac_range"])
        np.testing.assert_allclose(
            model.body(f"scapula_{side}").pos, np.multiply(gird["scapula"]["pos"], [1, 1, flip]), atol=1e-6
        )
        ell, ball = model.geom(f"scapulothoracic_ellipsoid_{side}").id, model.geom(f"scapula_glide_{side}").id
        dist = mujoco.mj_geomDistance(model, data, ell, ball, 0.05, np.zeros(6))
        assert abs(dist) < 1e-5, f"{side}: rest posture {1e3 * dist:.4f} mm off the thoracic ellipsoid"
        assert model.pair(f"scapulothoracic_{side}").dim[0] == 1
    # no rhythm couplings are left on the girdle
    for eq in range(model.neq):
        if model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT:
            name = model.joint(model.eq_obj1id[eq]).name
            assert not name.startswith(("sternoclavicular", "acromioclavicular", "unrot")), name
