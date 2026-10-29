import importlib.util
import json
import re
from pathlib import Path

import mujoco
import numpy as np
import pytest

from myo_sim import load_spec
from myo_sim.markersets import add_markerset, list_markersets, markerset_path

CONFIDENCE = {"myomarker_anatomical", "myomarker_reference", "myomarker_fitted", "myomarker_nominal"}
# asymmetric in the source: CMU fingers (right index, left ring), gait2392 rear shank (x sign differs in the OpenSim file)
ASYMMETRIC = {"RBAK", "RBAC", "RFIN", "R.Shank.Rear"}
MIDLINE_X = -0.0245  # world x of the sagittal plane of myofullbody at the default pose


def world_positions(name: str) -> dict[str, np.ndarray]:
    spec = load_spec("myofullbody")
    assert add_markerset(spec, name) == []
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    overlay = mujoco.MjSpec.from_file(str(markerset_path(name)))
    return {s.name: data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s.name)].copy() for s in overlay.sites}


@pytest.mark.parametrize("name", list_markersets())
def test_overlay_is_valid_mjcf_with_confidence_classes(name):
    overlay = mujoco.MjSpec.from_file(str(markerset_path(name)))
    overlay.compile()
    names = [s.name for s in overlay.sites]
    assert names and len(set(names)) == len(names)
    assert {s.classname.name for s in overlay.sites} <= CONFIDENCE


@pytest.mark.parametrize("name", list_markersets())
def test_set_builds_on_the_full_body_and_is_symmetric(name):
    pos = world_positions(name)
    assert np.isfinite(np.array(list(pos.values()))).all()
    pairs = [(k, side + k[len(side) :]) for k in pos for side in ("L", "L.", "L_") if k.startswith(side.replace("L", "R", 1))]
    pairs = [(r, lft) for r, lft in pairs if lft in pos and r not in ASYMMETRIC]
    assert pairs
    for r, lft in pairs:
        mirrored = pos[lft] * np.array([-1, 1, 1]) + np.array([2 * MIDLINE_X, 0, 0])
        assert np.linalg.norm(pos[r] - mirrored) < 0.01, (r, lft)


def test_missing_bodies_raise_or_are_skipped():
    with pytest.raises(KeyError):
        add_markerset(load_spec("myolegs"), "plug_in_gait")
    skipped = add_markerset(load_spec("myolegs"), "plug_in_gait", skip_missing=True)
    assert "LASI" not in skipped and "RELB" in skipped


def test_prefix_allows_two_sets_together():
    spec = load_spec("myofullbody")
    add_markerset(spec, "plug_in_gait")
    add_markerset(spec, "cmu_41", prefix="cmu_")
    spec.compile()


def load_bath_fit_script():
    path = Path(__file__).resolve().parents[1] / "scripts" / "fit_markersets_bath.py"
    spec = importlib.util.spec_from_file_location("fit_markersets_bath", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bath_generated_blocks_match_provenance_json():
    """The generated blocks of bath_running.xml hold exactly the values of markersets_bath.json (no raw data needed)."""
    fit = load_bath_fit_script()
    record = json.loads(fit.JSON_PATH.read_text())
    text = fit.XML_PATH.read_text()
    assert fit.read_blocks(text) == fit.render_blocks(record)
    assert fit.write_blocks(text, record) == text
    overlay = mujoco.MjSpec.from_file(str(fit.XML_PATH))  # keep the spec alive while reading its sites
    fitted = {s.name for s in overlay.sites if s.classname.name == "myomarker_fitted"}
    assert fitted == set(record["markers"])  # no fitted site outside the generated blocks
    assert record["source"]["doi"] == "10.15125/BATH-01341" and record["source"]["license"] == "CC BY 4.0"


@pytest.mark.parametrize("name", list_markersets())
def test_third_party_license_files_exist(name):
    text = markerset_path(name).read_text()
    assert "Copyright" in text.split("-->")[0]
    for license_file in re.findall(r"LICENSE-[A-Z0-9-]+\.txt", text):
        assert (markerset_path(name).parent / license_file).exists(), license_file
