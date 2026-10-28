"""Model policy: registered models have no feature switches; contact sets are unconditional."""

import inspect
import xml.etree.ElementTree as ET

import mujoco
import pytest

import myo_sim
from myo_sim import load_model
from myo_sim.build import compose
from myo_sim.build.compose import COMPOSITION_KWARGS, MODEL_REGISTRY



def test_registrations_use_only_composition_kwargs() -> None:
    for name, registration in MODEL_REGISTRY.items():
        assert set(registration.build_kwargs) <= COMPOSITION_KWARGS, name


def test_load_and_build_take_no_build_options() -> None:
    assert list(inspect.signature(myo_sim.load_spec).parameters) == ["name", "inertia_floor"]
    assert list(inspect.signature(compose.build_spec).parameters) == ["model_name", "inertia_floor"]


def _pairs(model: mujoco.MjModel) -> set[frozenset[str]]:
    name = lambda geom_id: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)  # noqa: E731
    return {frozenset((name(model.pair_geom1[i]), name(model.pair_geom2[i]))) for i in range(model.npair)}


@pytest.mark.parametrize("name", ["myoarm_r", "myoarms", "myohand_r", "myohands", "myolegs", "myolegs_abdomen", "myofullbody"])
def test_every_applicable_contact_pair_is_present(name: str) -> None:
    """Every arm/leg/hand/full-body pair whose two geoms exist in a model with those parts is injected."""
    model = load_model(name)
    present = _pairs(model)
    files = {
        "myoarm_r": ("myoarm_contacts.xml",),
        "myoarms": ("myoarm_contacts.xml",),
        "myohand_r": ("myohand_contacts.xml",),
        "myohands": ("myohand_contacts.xml",),
        "myolegs": ("myolegs_contacts.xml",),
        "myolegs_abdomen": ("myolegs_contacts.xml",),
        "myofullbody": ("myoarm_contacts.xml", "myolegs_contacts.xml", "myofullbody_contacts.xml"),
    }[name]
    geoms = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)}
    expected = set()
    for file in files:
        for pair in ET.parse(myo_sim.MODELS_DIR / "contacts" / file).getroot().iter("pair"):
            if {pair.get("geom1"), pair.get("geom2")} <= geoms:
                expected.add(frozenset((pair.get("geom1"), pair.get("geom2"))))
    assert expected, name
    assert expected <= present, sorted(map(sorted, expected - present))


def test_finger_collisions_are_always_on() -> None:
    for name in ("myoarms", "myohands", "myofullbody"):
        model = load_model(name)
        for base in ("proxph2_coll", "distal_thumb_coll", "5distph_coll"):
            for side in ("_r", "_l"):
                geom = model.geom(base + side)
                assert model.geom_contype[geom.id] or model.geom_conaffinity[geom.id], (name, base + side)
