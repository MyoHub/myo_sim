"""Seth et al. 2019 shoulder table: provenance, licence and the frame registration against the current model."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

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
