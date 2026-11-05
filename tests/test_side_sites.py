"""One tendon path on MuJoCo 3 and 4: the moved side sites, and no wrap that newly differs between the two rules.

``scripts/migrate_side_sites.py`` holds a copy of both wrap rules, so all of this runs under either MuJoCo; the first
test ties the copy to the running engine.
"""

import importlib.util
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim

ROOT = Path(__file__).resolve().parents[1]
TABLE = json.loads((ROOT / "myo_sim/build/data/side_sites.json").read_text())
RULE_OF_RUNNING_MUJOCO = 4 if int(mujoco.__version__.split(".")[0]) >= 4 else 3


def _script():
    name = "migrate_side_sites"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
        sys.modules[name] = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sys.modules[name])
    return sys.modules[name]


@pytest.fixture(scope="module")
def arm() -> mujoco.MjModel:
    return myo_sim.load_model("myoarm_r")


@pytest.mark.parametrize("name", ["myoarm_r", "myolegs", "myotorso"])
def test_rule_copy_reproduces_the_running_mujoco(name: str) -> None:
    script = _script()
    model = myo_sim.load_model(name)
    qs = script.sample_postures(model, 300, 5, True)
    lengths, covered = script.rule_tendon_lengths(model, qs, RULE_OF_RUNNING_MUJOCO)
    data = mujoco.MjData(model)
    real = np.empty_like(lengths)
    for i, q in enumerate(qs):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_tendon(model, data)
        real[i] = data.ten_length
    assert covered.sum() > 0.5 * model.ntendon
    assert np.abs(lengths - real)[:, covered].max() < 1e-9


@pytest.mark.parametrize("site", TABLE["sites"], ids=lambda s: s["name"])
def test_moved_site_is_where_the_table_says(arm: mujoco.MjModel, site: dict) -> None:
    sid = arm.site(site["name"]).id
    assert arm.body(arm.site_bodyid[sid]).name == site["body"]
    np.testing.assert_allclose(arm.site_pos[sid], site["pos"], atol=1e-9)


@pytest.mark.parametrize("tier", ["A", "B"])
@pytest.mark.parametrize("site", TABLE["sites"], ids=lambda s: s["name"])
def test_moved_site_gives_the_mujoco3_path_under_both_rules(arm: mujoco.MjModel, site: dict, tier: str) -> None:
    script = _script()
    wraps = [w for w in script.model_wraps(arm) if w.side_name == site["name"]]
    assert {w.tendon_name for w in wraps} == set(site["users"])
    qs = script.tier_postures(arm, tier, 4000, 41)
    for w in wraps:
        sides = [(site["body_before"], np.array(site["pos_before"])), (site["body"], np.array(site["pos"]))]
        ends, (old, new) = script.wrap_geometry(arm, qs, w, sides)
        args = (w.radius, w.is_sphere)
        reference, _ = script.segment_length(ends[:, 0], ends[:, 1], old, *args, 3)
        for rule in (3, 4):
            length, _ = script.segment_length(ends[:, 0], ends[:, 1], new, *args, rule)
            assert np.abs(length - reference).max() < script.LENGTH_TOL, (w.tendon_name, rule)
        # negative control: the old site does differ under the MuJoCo 4 rule
        length, _ = script.segment_length(ends[:, 0], ends[:, 1], old, *args, 4)
        assert np.abs(length - reference).max() > 1e-3, w.tendon_name


@pytest.mark.parametrize("name", ["myoarm_r", "myolegs", "myotorso"])
def test_no_wrap_newly_differs_between_the_rules(name: str) -> None:
    """In range, on the joint couplings, only the wraps tabled as kept may differ, by no more than tabled."""
    script = _script()
    model = myo_sim.load_model(name)
    wraps = script.model_wraps(model)
    side = script.side_local(model, wraps)
    ends, _ = script.forward_sites(model, script.tier_postures(model, "A", 3000, 21), wraps)
    kept = {k["side_site"]: k for k in TABLE["kept"]}
    for k, w in enumerate(wraps):
        if not script.rule_decides(w, side[k]):
            continue
        args = (ends[:, k, 0], ends[:, k, 1], side[k], w.radius, w.is_sphere)
        difference = np.abs(script.segment_length(*args, 3)[0] - script.segment_length(*args, 4)[0]).max()
        source = w.side_name if w.side_name in kept else w.side_name[:-2] + "_r"
        allowed = kept[source]["difference_mm"]["A"] * 1e-3 + 1e-3 if source in kept else script.LENGTH_TOL
        assert difference <= allowed, (w.tendon_name, w.geom_name, difference)
