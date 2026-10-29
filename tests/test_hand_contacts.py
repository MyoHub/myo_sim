"""Hand self-contacts and pads: generated from their tables, present in every model with a hand, quiet at rest.

- ``myohand_self_contacts.xml`` is what ``scripts/generate_hand_contacts.py`` writes from
  ``build/data/hand_contact_pairs.json``; the pad geoms in ``myoarm_r_chain.xml`` hold the values of
  ``build/data/hand_pads.json`` (``scripts/fit_hand_pads.py``).
- Every model with a hand carries all pairs on each hand, with the left pairs mirrored from the right.
- No self-contact is active at rest; in an active fist no pair penetrates more than 1 mm beyond its margin.
- The fingertip soft tissue equals its target, and each fitted palm pad just touches the fist.
"""

from __future__ import annotations

import importlib.util
import sys
from functools import cache
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.utils import MirrorRules, mirror_name

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
HAND_MODELS = {"myohand_r": "r", "myohands": "rl", "myoarm_r": "r", "myoarms": "rl", "myotorso_arms": "rl", "myofullbody": "rl"}
PAIR_FIELDS = ("pair_dim", "pair_friction", "pair_solref", "pair_solimp", "pair_margin", "pair_gap")


@cache
def _script(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@cache
def _model(name: str) -> mujoco.MjModel:
    return myo_sim.load_model(name)


def _pairs(model: mujoco.MjModel) -> dict[tuple[str, str], int]:
    return {
        tuple(sorted((model.geom(model.pair_geom1[i]).name, model.geom(model.pair_geom2[i]).name))): i
        for i in range(model.npair)
    }


def test_contacts_file_is_what_the_script_writes() -> None:
    gen = _script("generate_hand_contacts")
    assert gen.read_block() == gen.block(gen.load_table())


def test_chain_holds_the_tabled_pads() -> None:
    pads = _script("fit_hand_pads")
    text = pads.CHAIN_XML.read_text()
    assert pads.chain_with_pads(text, pads.load_table()) == text


@pytest.mark.parametrize("name", list(HAND_MODELS))
def test_every_hand_carries_every_pair(name: str) -> None:
    model = _model(name)
    pairs = _pairs(model)
    table = _script("generate_hand_contacts").load_table()["pairs"]
    for side in HAND_MODELS[name]:
        for row in table:
            geoms = (
                (row["geom1"], row["geom2"])
                if side == "r"
                else (mirror_name(row["geom1"], MirrorRules()), mirror_name(row["geom2"], MirrorRules()))
            )
            assert tuple(sorted(geoms)) in pairs, (name, geoms)


def test_left_pairs_mirror_the_right_ones() -> None:
    model = _model("myohands")
    pairs = _pairs(model)
    for (g1, g2), i in pairs.items():
        if g1.endswith("_r") and g2.endswith("_r"):
            j = pairs[tuple(sorted((mirror_name(g1, MirrorRules()), mirror_name(g2, MirrorRules()))))]
            for field in PAIR_FIELDS:
                np.testing.assert_array_equal(getattr(model, field)[i], getattr(model, field)[j], err_msg=f"{g1}/{g2} {field}")


def test_left_pairs_keep_the_right_order_and_roles() -> None:
    """In the composed full body the left hand's pairs come in the right hand's order, geom1 and geom2 alike."""
    model = _model("myofullbody")
    hand = [model.pair(i) for i in range(model.npair) if "__" in model.pair(i).name]
    right = [(model.geom(p.geom1[0]).name, model.geom(p.geom2[0]).name) for p in hand if p.name.endswith("_r")]
    left = [(model.geom(p.geom1[0]).name, model.geom(p.geom2[0]).name) for p in hand if p.name.endswith("_l")]
    assert left == [(mirror_name(a, MirrorRules()), mirror_name(b, MirrorRules())) for a, b in right]


def _active_hand_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[tuple[str, str, float]]:
    gen = _script("generate_hand_contacts")
    hand = set()
    for root in ("lunate_r", "lunate_l"):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root) >= 0:
            hand |= gen._subtree(model, root)
    return [
        (model.geom(c.geom1).name, model.geom(c.geom2).name, c.dist)
        for c in data.contact[: data.ncon]
        if c.exclude == 0 and model.geom_bodyid[c.geom1] in hand and model.geom_bodyid[c.geom2] in hand
    ]


@pytest.mark.parametrize("name", list(HAND_MODELS))
def test_no_self_contact_is_active_at_rest(name: str) -> None:
    model = _model(name)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert _active_hand_contacts(model, data) == []


def test_rest_check_rejects_the_overlapping_pairs_without_their_margin() -> None:
    """Negative control: the two side-by-side pairs that overlap at rest are active there with margin 0."""
    spec = myo_sim.load_spec("myohand_r")
    for pair in spec.pairs:
        pair.margin = 0.0
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert len(_active_hand_contacts(model, data)) == 2


def test_active_fist_does_not_penetrate() -> None:
    fist = _script("hand_literature_comparison").fist()
    assert fist["max_penetration_mm"] < 1.0


def test_fingertip_soft_tissue_is_on_target() -> None:
    pads = _script("fit_hand_pads")
    table = pads.load_table()["fingertip"]
    model = _model("myoarm_r")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for row in table["digits"]:
        measured = pads.fingertip_soft_tissue_mm(model, data, row["body"], row["flexor"], table["corner_m"])
        assert measured == pytest.approx(1000 * row["target_m"], abs=0.02), row["body"]


def test_fitted_palm_pads_just_touch_the_fist() -> None:
    pads = _script("fit_hand_pads")
    model = _model("myoarm_r")
    data = mujoco.MjData(model)
    pads.full_fist(model, data, [2, 3, 4, 5])
    for row in pads.load_table()["palm"]:
        if row["fitted"]["method"] == "fist":
            gap = pads.fingertip_to_pad_m(model, data, model.geom(row["name"]).id, row["digits"])
            assert abs(gap) < 5e-5, row["name"]
