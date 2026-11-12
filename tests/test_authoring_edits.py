# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Selected source-model edits preserve anatomy and fail transactionally."""

from __future__ import annotations

import copy
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import pytest
from authoring_support import (
    _assert_source_unchanged,
    _edit,
    _elbow_model,
    _fusion_spec,
    _patch,
    _source_state,
)

import myo_sim
from myo_sim.authoring import apply_model_patch, mesh_inertia, model_fingerprint
from myo_sim.authoring.interchange import _inertia


@pytest.fixture(scope="module")
def elbow() -> tuple[mujoco.MjSpec, mujoco.MjModel]:
    return _elbow_model()


def test_empty_validated_patch_compiles_once_and_preserves_source(
    elbow: tuple[mujoco.MjSpec, mujoco.MjModel], monkeypatch: pytest.MonkeyPatch
) -> None:
    spec, source = elbow
    patch = _patch(source)
    before = _source_state(spec)
    before_xml = spec.to_xml()
    compile = mujoco.MjSpec.compile
    calls = []

    def record(candidate: mujoco.MjSpec) -> mujoco.MjModel:
        calls.append(candidate)
        return compile(candidate)

    with monkeypatch.context() as context:
        context.setattr(mujoco.MjSpec, "compile", record)
        edited = apply_model_patch(spec, patch)
    assert len(calls) == 1 and calls[0] is edited
    assert edited is not spec
    assert model_fingerprint(edited.compile()) == patch["fingerprint"]
    _assert_source_unchanged(spec, before)
    assert spec.to_xml() == before_xml


def test_elbow_site_and_inertia_preserve_other_anatomy(elbow: tuple[mujoco.MjSpec, mujoco.MjModel]) -> None:
    spec, source = elbow
    patch = _edit(source)
    before = _source_state(spec)
    edited = apply_model_patch(spec, patch)
    model = edited.compile()
    assert edited is not spec
    _assert_source_unchanged(spec, before)
    assert model_fingerprint(spec.compile()) == patch["fingerprint"]
    np.testing.assert_array_equal(model.site("BRA-P2").pos, patch["sites"][0]["pos"])
    body = model.body("r_ulna_radius_hand").id
    np.testing.assert_allclose(model.body_mass[body], 1.7)
    np.testing.assert_allclose(model.body_ipos[body], patch["inertias"][0]["com"])
    np.testing.assert_allclose(
        _inertia(model.body_inertia[body], model.body_iquat[body]),
        patch["inertias"][0]["inertia"],
        atol=1e-12,
    )
    untouched_sites = np.arange(source.nsite) != source.site("BRA-P2").id
    np.testing.assert_array_equal(model.site_pos[untouched_sites], source.site_pos[untouched_sites])


@pytest.mark.parametrize("side", ["r", "l"])
def test_fullbody_edits_preserve_composition_and_unrelated_calibration(side: str) -> None:
    spec = myo_sim.load_spec("myofullbody")
    source = spec.copy().compile()
    before = _source_state(spec)
    chain = myo_sim.MODELS_DIR / "arm" / "assets" / "myoarm_r_chain.xml"
    original_xml = chain.read_bytes()
    site_name, body_name = f"DELT1_DELT1-P4_{side}", f"humerus_{side}"
    sid, bid = source.site(site_name).id, source.body(body_name).id
    patch = _patch(source)
    patch["sites"] = [dict(name=site_name, pos=(source.site_pos[sid] + [0.001, 0.002, -0.003]).tolist())]
    patch["inertias"] = [
        dict(
            name=body_name,
            mass=float(source.body_mass[bid] * 1.02),
            com=(source.body_ipos[bid] + [0.001, -0.002, 0.001]).tolist(),
            inertia=(1.03 * np.asarray(_inertia(source.body_inertia[bid], source.body_iquat[bid]))).tolist(),
        )
    ]

    edited = apply_model_patch(spec, patch)
    result = edited.compile()

    _assert_source_unchanged(spec, before)
    assert chain.read_bytes() == original_xml
    assert model_fingerprint(spec.copy().compile()) == patch["fingerprint"]
    assert result.names == source.names
    for field in (
        "body_parentid",
        "body_jntadr",
        "body_jntnum",
        "body_pos",
        "body_quat",
        "jnt_type",
        "jnt_bodyid",
        "jnt_pos",
        "jnt_axis",
        "jnt_range",
        "jnt_qposadr",
        "jnt_dofadr",
        "geom_bodyid",
        "geom_pos",
        "geom_quat",
        "geom_size",
        "geom_type",
        "site_bodyid",
        "site_quat",
        "eq_type",
        "eq_obj1id",
        "eq_obj2id",
        "eq_data",
        "eq_active0",
        "tendon_adr",
        "tendon_num",
        "wrap_type",
        "wrap_objid",
        "wrap_prm",
        "actuator_trntype",
        "actuator_trnid",
        "actuator_gainprm",
        "actuator_biasprm",
        "actuator_dynprm",
        "actuator_gear",
        "actuator_ctrlrange",
        "actuator_lengthrange",
        "qpos0",
    ):
        np.testing.assert_array_equal(getattr(result, field), getattr(source, field), err_msg=field)
    other_sites = np.arange(source.nsite) != sid
    other_bodies = np.arange(source.nbody) != bid
    np.testing.assert_array_equal(result.site_pos[other_sites], source.site_pos[other_sites])
    for field in ("body_mass", "body_ipos", "body_inertia", "body_iquat"):
        np.testing.assert_array_equal(getattr(result, field)[other_bodies], getattr(source, field)[other_bodies], err_msg=field)
    np.testing.assert_allclose(result.site_pos[sid], patch["sites"][0]["pos"], atol=1e-12)
    np.testing.assert_allclose(result.body_ipos[bid], patch["inertias"][0]["com"], atol=1e-12)
    np.testing.assert_allclose(
        _inertia(result.body_inertia[bid], result.body_iquat[bid]), patch["inertias"][0]["inertia"], atol=1e-12
    )
    np.testing.assert_allclose(result.body_mass.sum() - source.body_mass.sum(), source.body_mass[bid] * 0.02, atol=1e-12)
    # Native acceleration, force estimates and routed lengths may change from the
    # selected anatomy edits; input physiological calibration remains unchanged.
    assert not np.array_equal(result.actuator_acc0, source.actuator_acc0)
    old, new = mujoco.MjData(source), mujoco.MjData(result)
    mujoco.mj_forward(source, old)
    mujoco.mj_forward(result, new)
    assert not np.array_equal(new.ten_length, old.ten_length)


@pytest.mark.parametrize(
    "edit",
    [
        {"schema": True},
        {"schema": 2},
        {"fingerprint": "wrong"},
        {"mujoco_version": "different"},
        {"unknown": 1},
        {"sites": {}},
        {"sites": ()},
        {"inertias": {}},
        {"inertias": ()},
        {"sites": [{"name": "unknown", "pos": [0, 0, 0]}]},
        {"sites": [{"name": "BRA-P2", "pos": [0, 0, 0], "extra": 1}]},
        {"sites": [{"name": "BRA-P2", "pos": [0, 0, np.inf]}]},
        {"sites": [{"name": "BRA-P2", "pos": [0, 0]}]},
        {"sites": [{"name": "BRA-P2", "pos": ["0", "0", "0"]}]},
        {"sites": [{"name": "BRA-P2", "pos": [0, 0, 0]}] * 2},
        {
            "inertias": [
                {
                    "name": "world",
                    "mass": 1,
                    "com": [0, 0, 0],
                    "inertia": np.eye(3).tolist(),
                }
            ]
        },
    ],
)
def test_invalid_patch_does_not_mutate_source(elbow: tuple[mujoco.MjSpec, mujoco.MjModel], edit: dict[str, Any]) -> None:
    spec, model = elbow
    before = _source_state(spec)
    patch = _patch(model)
    patch.update(copy.deepcopy(edit))
    with pytest.raises(ValueError):
        apply_model_patch(spec, patch)
    _assert_source_unchanged(spec, before)


@pytest.mark.parametrize(
    "edit",
    [
        {"mass": 0},
        {"mass": np.nan},
        {"com": [0, 1, np.nan]},
        {"inertia": np.diag([1, 1, -1]).tolist()},
        {"inertia": np.diag([1, 1, 3]).tolist()},
        {"inertia": [[1, 0.1, 0], [0.2, 1, 0], [0, 0, 1]]},
        {"inertia": [[True, False, False], [False, True, False], [False, False, True]]},
    ],
)
def test_invalid_inertia_is_rejected(elbow: tuple[mujoco.MjSpec, mujoco.MjModel], edit: dict[str, Any]) -> None:
    spec, model = elbow
    patch = _edit(model)
    patch["inertias"][0].update(edit)
    with pytest.raises(ValueError):
        apply_model_patch(spec, patch)


def test_duplicate_inertias_and_changed_source_are_rejected(elbow: tuple[mujoco.MjSpec, mujoco.MjModel]) -> None:
    spec, model = elbow
    patch = _edit(model)
    patch["inertias"] *= 2
    with pytest.raises(ValueError, match="Duplicate"):
        apply_model_patch(spec, patch)
    changed = spec.copy()
    changed.site("BRA-P2").pos[0] += 0.001
    with pytest.raises(ValueError, match="fingerprint"):
        apply_model_patch(changed, _edit(model))


def test_compile_failure_keeps_original_spec(
    elbow: tuple[mujoco.MjSpec, mujoco.MjModel], monkeypatch: pytest.MonkeyPatch
) -> None:
    spec, model = elbow
    patch = _edit(model)
    before = _source_state(spec)
    compile = mujoco.MjSpec.compile
    calls = []

    def fail_edited(candidate: mujoco.MjSpec) -> mujoco.MjModel:
        calls.append(candidate)
        if len(calls) == 2:
            raise ValueError("compiler rejected the edit")
        return compile(candidate)

    with monkeypatch.context() as context:
        context.setattr(mujoco.MjSpec, "compile", fail_edited)
        with pytest.raises(ValueError, match="compiler rejected"):
            apply_model_patch(spec, patch)
    assert all(candidate is not spec for candidate in calls)
    _assert_source_unchanged(spec, before)


def test_compiler_cannot_silently_ignore_explicit_inertia() -> None:
    spec = mujoco.MjSpec.from_string(
        '<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b">'
        '<joint/><geom size=".1"/></body></worldbody></mujoco>'
    )
    patch = _patch(spec.compile())
    patch["inertias"] = [dict(name="b", mass=1, com=[0, 0, 0], inertia=np.eye(3).tolist())]
    with pytest.raises(ValueError, match="ignored"):
        apply_model_patch(spec, patch)


@pytest.mark.parametrize("nested", [False, True])
@pytest.mark.parametrize("fusestatic", [False, True])
def test_framed_site_uses_body_local_position_and_preserves_orientation(nested: bool, fusestatic: bool) -> None:
    inner = (
        '<frame name="inner" pos=".03 -.02 .1" euler="20 30 0"><site name="s" pos=".1 0 0" euler="10 0 25"/></frame>'
        if nested
        else '<site name="s" pos=".1 0 0" euler="10 0 25"/>'
    )
    spec = mujoco.MjSpec.from_string(
        f'<mujoco><compiler fusestatic="{str(fusestatic).lower()}"/><worldbody><body name="moving"><joint/><geom size=".1"/>'
        '<frame name="outer" pos=".1 .2 .3" euler="0 0 90">'
        '<site name="untouched" pos="0 .2 0"/>' + inner + "</frame></body></worldbody></mujoco>"
    )
    source = spec.copy().compile()
    before = _source_state(spec)
    patch = _patch(source)
    patch["sites"] = [dict(name="s", pos=[0.2, 0.4, 0.6])]
    result = apply_model_patch(spec, patch).compile()
    np.testing.assert_allclose(result.site("s").pos, [0.2, 0.4, 0.6], atol=1e-12)
    np.testing.assert_allclose(result.site("s").quat, source.site("s").quat, atol=1e-12)
    assert result.site("s").bodyid == source.site("s").bodyid
    np.testing.assert_array_equal(result.site("untouched").pos, source.site("untouched").pos)
    _assert_source_unchanged(spec, before)
    assert model_fingerprint(spec.compile()) == patch["fingerprint"]


def test_unsupported_fromto_frame_edit_is_transactional() -> None:
    spec = mujoco.MjSpec.from_string(
        '<mujoco><worldbody><body name="moving"><joint/><geom size=".1"/>'
        '<frame pos=".1 .2 .3" euler="0 0 90">'
        '<site name="s" type="capsule" size=".01" fromto="0 0 0 0 0 .1"/>'
        "</frame></body></worldbody></mujoco>"
    )
    patch = _patch(spec.copy().compile())
    patch["sites"] = [dict(name="s", pos=[0.2, 0.4, 0.6])]
    before = _source_state(spec)
    with pytest.raises(ValueError):
        apply_model_patch(spec, patch)
    _assert_source_unchanged(spec, before)


def test_compiled_site_postcondition_rejects_a_transformed_result(monkeypatch: pytest.MonkeyPatch) -> None:
    spec = mujoco.MjSpec.from_string('<mujoco><worldbody><site name="s"/></worldbody></mujoco>')
    patch = _patch(spec.compile())
    patch["sites"] = [dict(name="s", pos=[0.2, 0.4, 0.6])]
    before = _source_state(spec)
    compile = mujoco.MjSpec.compile
    calls = []

    def transformed(candidate: mujoco.MjSpec) -> mujoco.MjModel:
        result = compile(candidate)
        calls.append(candidate)
        if len(calls) == 2:
            result.site("s").pos[0] += 0.1
        return result

    with monkeypatch.context() as context:
        context.setattr(mujoco.MjSpec, "compile", transformed)
        with pytest.raises(ValueError, match="body-local site"):
            apply_model_patch(spec, patch)
    _assert_source_unchanged(spec, before)


def test_mesh_inertia_uses_homogeneous_watertight_mass_properties(tmp_path: Path) -> None:
    trimesh = pytest.importorskip("trimesh")
    mesh = trimesh.creation.box(extents=[0.1, 0.2, 0.3])
    mesh.apply_translation([0.2, -0.3, 0.4])
    path = tmp_path / "box.ply"
    mesh.export(path)
    result = mesh_inertia(path, 1000)
    np.testing.assert_allclose(result["mass"], 6, rtol=1e-6)
    np.testing.assert_allclose(result["com"], [0.2, -0.3, 0.4], atol=1e-7)
    np.testing.assert_allclose(result["inertia"], np.diag([0.065, 0.05, 0.025]), rtol=1e-6, atol=1e-8)
    open_mesh = trimesh.Trimesh(mesh.vertices, mesh.faces[:-1], process=False)
    open_mesh.export(path)
    with pytest.raises(ValueError, match="watertight"):
        mesh_inertia(path, 1000)
    for density in (0, -1, np.nan):
        with pytest.raises(ValueError):
            mesh_inertia(path, density)


def test_mesh_inertia_is_optional(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setitem(sys.modules, "trimesh", None)
    with pytest.raises(ImportError, match="myo-sim\\[mesh\\]"):
        mesh_inertia(tmp_path / "unused.stl", 1000)


@pytest.mark.parametrize("with_site_edit", [False, True])
def test_actual_static_fusion_is_rejected_without_changing_other_anatomy(with_site_edit: bool) -> None:
    spec = _fusion_spec()
    source = spec.copy().compile()
    before = _source_state(spec)
    patch = _patch(source)
    if with_site_edit:
        patch["sites"] = [dict(name="s", pos=[0.2, 0.4, 0.6])]
    with pytest.raises(ValueError, match="fusion"):
        apply_model_patch(spec, patch)
    _assert_source_unchanged(spec, before)
    unchanged = spec.copy().compile()
    for field in ("geom_pos", "geom_quat", "site_pos", "site_quat"):
        np.testing.assert_array_equal(getattr(unchanged, field), getattr(source, field))
