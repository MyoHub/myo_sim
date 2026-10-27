import mujoco
import numpy as np
import pytest

from myo_sim import load_spec
from myo_sim.markersets import Marker, MarkerSet, add_markerset, list_markersets, load_markerset


@pytest.fixture(scope="module")
def fullbody_spec_factory():
    return lambda: load_spec("myofullbody")


def test_anchored_marker_sits_at_anchor_plus_offset(fullbody_spec_factory):
    ms = MarkerSet(
        "t",
        "t",
        (),
        (
            Marker("A", "anatomical", "d", anchor="ASI_r", pos=(0.0, 0.01, 0.0)),
            Marker("B", "virtual", "d", body="pelvis", pos=(0, 0, 0.1)),
        ),
    )
    spec = fullbody_spec_factory()
    assert add_markerset(spec, ms) == []
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    a, anchor = (data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n)] for n in ("mk_A", "ASI_r"))
    assert np.linalg.norm(a - anchor) == pytest.approx(0.01, abs=1e-6)


def test_missing_anchor_raises_or_is_skipped(fullbody_spec_factory):
    ms = MarkerSet("t", "t", (), (Marker("X", "virtual", "d", anchor="no_such_site"),))
    with pytest.raises(KeyError):
        add_markerset(fullbody_spec_factory(), ms)
    assert add_markerset(fullbody_spec_factory(), ms, skip_missing=True) == ["X"]


@pytest.mark.parametrize("name", list_markersets())
def test_packaged_set_is_well_formed_and_builds_on_the_full_body(name, fullbody_spec_factory):
    ms = load_markerset(name)
    assert ms.sources and len(set(ms.names())) == len(ms.markers)
    spec = fullbody_spec_factory()
    assert add_markerset(spec, ms) == []
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "mk_" + n) for n in ms.names()]
    assert min(ids) >= 0 and np.isfinite(data.site_xpos[ids]).all()
