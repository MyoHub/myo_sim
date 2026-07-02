"""Smoke tests that verify the installed myo_sim package is complete and functional."""

from __future__ import annotations

import mujoco

import myo_sim


def test_models_dir_exists():
    assert myo_sim.MODELS_DIR.exists(), f"MODELS_DIR not found: {myo_sim.MODELS_DIR}"


def test_xml_files_present():
    xml_files = list(myo_sim.MODELS_DIR.rglob("*.xml"))
    assert len(xml_files) > 0, "No .xml files found under MODELS_DIR"


def test_stl_files_present():
    stl_files = list(myo_sim.MODELS_DIR.rglob("*.stl"))
    assert len(stl_files) > 0, "No .stl files found under MODELS_DIR"


def test_load_myolegs():
    model, _ = myo_sim.load("myolegs")
    assert model.njnt > 0, "Loaded myolegs model has no joints"
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Full Body") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor") >= 0


def test_build_compose_import():
    from myo_sim.build.compose import build_model  # noqa: F401
