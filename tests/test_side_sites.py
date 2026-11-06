"""Tendon paths on MuJoCo 4: side sites as tabled, and no length jump along the sequences of nearby postures.

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
def test_moved_site_is_where_the_table_says(site: dict) -> None:
    model = myo_sim.load_model("myotorso" if site.get("part") == "torso" else "myoarm_r")
    sid = model.site(site["name"]).id
    assert model.body(model.site_bodyid[sid]).name == site["body"]
    np.testing.assert_allclose(model.site_pos[sid], site["pos"], atol=1e-9)


@pytest.mark.parametrize("removal", TABLE["removed"], ids=lambda s: s["name"])
def test_removed_side_site_is_gone(arm: mujoco.MjModel, removal: dict) -> None:
    assert mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_SITE, removal["name"]) == -1
    wraps = [w for w in _script().model_wraps(arm) if w.tendon_name == removal["tendon"] and w.geom_name == removal["geom"]]
    assert wraps and all(w.side == -1 for w in wraps)
    if "radius_m" in removal:
        assert arm.geom(removal["geom"]).size[0] == pytest.approx(removal["radius_m"])


@pytest.mark.parametrize("via", TABLE["vias"], ids=lambda s: s["name"])
def test_via_replaces_the_wrap(arm: mujoco.MjModel, via: dict) -> None:
    sid = arm.site(via["name"]).id
    assert arm.body(arm.site_bodyid[sid]).name == via["body"]
    np.testing.assert_allclose(arm.site_pos[sid], via["pos"], atol=1e-9)
    assert mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_GEOM, via["geom"]) == -1
    t = arm.tendon(via["tendon"]).id
    path = arm.wrap_objid[arm.tendon_adr[t] : arm.tendon_adr[t] + arm.tendon_num[t]]
    assert sid in path


def _site_in_geom_frame(model, wrap, qs, script) -> np.ndarray | None:
    """Side site in the geom frame per posture, where it sits on another body than the geom."""
    if wrap.side < 0 or model.site_bodyid[wrap.side] == model.geom_bodyid[wrap.geom]:
        return None
    body = model.body(model.site_bodyid[wrap.side]).name
    return script.wrap_geometry(model, qs, wrap, [(body, np.array(model.site_pos[wrap.side]))])[1][0]


@pytest.mark.parametrize("past", [False, True], ids=["in_range", "past_limits"])
@pytest.mark.parametrize("name", ["myoarm_r", "myolegs", "myotorso"])
def test_no_length_jump_on_mujoco4(name: str, past: bool) -> None:
    """The MuJoCo 4 rule, along sweeps and coupled lines; wraps known to jump are tabled with their size."""
    script = _script()
    model = myo_sim.load_model(name)
    wraps = script.model_wraps(model)
    side = script.side_local(model, wraps)
    allowed = {k: v for k, v in TABLE["jumps_mm"][name].items()} if name in TABLE["jumps_mm"] else {}
    label = "past" if past else "in"
    seen = {}
    for k, w in enumerate(wraps):
        inside = w.side >= 0 and not np.isnan(side[k][0]) and np.linalg.norm(side[k]) < w.radius
        if inside:
            continue
        qs, ids = script.continuity_postures(model, w, past, n_lines=120, line_steps=60, sweep_steps=301, n_bases=3)
        ends, _ = script.wrap_geometry(model, qs, w, [])
        ws = script.precompute(ends[:, 0], ends[:, 1], w.radius, w.is_sphere)
        same = ids[1:] == ids[:-1]
        if w.side < 0:
            st = script.states_no_side(ws, 4)[None]
        else:
            moving = _site_in_geom_frame(model, w, qs, script)
            st = script.states(ws, side[k][None] if moving is None else moving[None], 4)
        jump = float(script.state_jump(ws, st, 4, same).max())
        seen[f"{w.tendon_name}/{w.geom_name}"] = jump
    too_big = {
        key: round(j * 1e3, 2)
        for key, j in seen.items()
        if j * 1e3 > 1.5 * allowed.get(key, {}).get(label, script.JUMP_TOL * 1e3) + 0.2  # coarser sampling here than recorded
    }
    assert not too_big, too_big


@pytest.mark.parametrize("site", [s for s in TABLE["sites"] if s.get("keeps_mujoco3")], ids=lambda s: s["name"])
def test_moved_site_keeps_its_mujoco3_path_under_both_rules(arm: mujoco.MjModel, site: dict) -> None:
    script = _script()
    wraps = [w for w in script.model_wraps(arm) if w.side_name == site["name"]]
    assert {w.tendon_name for w in wraps} == set(site["users"])
    qs = script.tier_postures(arm, "A", 4000, 41)
    for w in wraps:
        sides = [(site["body_before"], np.array(site["pos_before"])), (site["body"], np.array(site["pos"]))]
        ends, (old, new) = script.wrap_geometry(arm, qs, w, sides)
        args = (w.radius, w.is_sphere)
        reference, _ = script.segment_length(ends[:, 0], ends[:, 1], old, *args, 3)
        for rule in (3, 4):
            length, _ = script.segment_length(ends[:, 0], ends[:, 1], new, *args, rule)
            assert np.abs(length - reference).max() < script.LENGTH_TOL, (w.tendon_name, rule)
