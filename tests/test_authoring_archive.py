# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Portable native archives preserve edited anatomy, model physics and source state."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest
from authoring_support import _assert_source_unchanged, _edit, _elbow_model, _fusion_spec, _patch, _source_state

import myo_sim
from myo_sim.authoring import apply_model_patch, joint_sweep, model_fingerprint
from myo_sim.authoring.archive import save_model_archive
from myo_sim.authoring.interchange import _inertia


def _curves(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    data = mujoco.MjData(model)
    lengths, moments = [], []
    for pose in joint_sweep(model, "r_elbow_flex", 21):
        data.qpos[:] = pose
        mujoco.mj_forward(model, data)
        dense = np.zeros((model.nu, model.nv))
        mujoco.mju_sparse2dense(
            dense,
            data.actuator_moment,
            data.moment_rownnz,
            data.moment_rowadr,
            data.moment_colind,
        )
        lengths.append(data.actuator_length.copy())
        moments.append(dense.copy())
    return np.array(lengths), np.array(moments)


@pytest.fixture(scope="module")
def elbow() -> tuple[mujoco.MjSpec, mujoco.MjModel]:
    return _elbow_model()


def test_archive_rejects_ambiguous_external_asset_names(tmp_path: Path) -> None:
    obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n"
    for folder, contents in (("a", obj), ("b", obj.replace("v 1 0 0", "v 2 0 0"))):
        (tmp_path / folder).mkdir()
        (tmp_path / folder / "same.obj").write_text(contents)
    xml = tmp_path / "source.xml"
    xml.write_text(
        '<mujoco><asset><mesh name="a" file="a/same.obj"/>'
        '<mesh name="b" file="b/same.obj"/></asset><worldbody>'
        '<geom type="mesh" mesh="a"/><geom type="mesh" mesh="b"/></worldbody></mujoco>'
    )
    spec = mujoco.MjSpec.from_file(str(xml))
    with pytest.raises(ValueError, match="Ambiguous"):
        save_model_archive(spec, tmp_path / "model.zip")
    assert not (tmp_path / "model.zip").exists()


@pytest.mark.parametrize(
    "vfs_key",
    [
        None,
        "assets/tetra.obj",
        "tetra.obj",
        "old/path/tetra.obj",
        "assets/old/path/tetra.obj",
    ],
)
@pytest.mark.parametrize("filename", ["old/path/tetra.obj", r"old\path\tetra.obj"])
def test_native_archive_honors_strippath(tmp_path: Path, vfs_key: str | None, filename: str) -> None:
    data = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n"
    directory = "assets" if vfs_key in (None, "tetra.obj") else Path(vfs_key).parent.as_posix()
    xml = (
        f'<mujoco><compiler strippath="true" meshdir="{directory}"/><asset>'
        f'<mesh name="tetra" file="{filename}"/></asset><worldbody>'
        '<geom type="mesh" mesh="tetra"/></worldbody></mujoco>'
    )
    if vfs_key is None:
        (tmp_path / "assets").mkdir()
        (tmp_path / "assets/tetra.obj").write_bytes(data)
        source = tmp_path / "source.xml"
        source.write_text(xml)
        spec = mujoco.MjSpec.from_file(str(source))
    else:
        spec = mujoco.MjSpec.from_string(xml, assets={vfs_key: data})
    original = spec.compile()
    archive = tmp_path / "model.zip"
    save_model_archive(spec, archive)
    loaded = mujoco.MjSpec.from_zip(archive).compile()
    for field in (
        "mesh_vert",
        "mesh_face",
        "geom_type",
        "geom_pos",
        "geom_quat",
        "geom_size",
    ):
        np.testing.assert_allclose(getattr(loaded, field), getattr(original, field), atol=1e-6, err_msg=field)


@pytest.mark.parametrize("vfs_key", ["old/path/tetra.obj", "assets/old/path/tetra.obj"])
def test_native_archive_preserves_basename_alias_when_supported(tmp_path: Path, vfs_key: str) -> None:
    """Keep newer native VFS suffix lookup coverage without requiring it on 3.4."""
    data = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n"
    spec = mujoco.MjSpec.from_string(
        '<mujoco><compiler strippath="true" meshdir="assets"/><asset>'
        '<mesh name="tetra" file="old/path/tetra.obj"/></asset><worldbody>'
        '<geom type="mesh" mesh="tetra"/></worldbody></mujoco>',
        assets={vfs_key: data},
    )
    try:
        original = spec.compile()
    except ValueError as error:
        if "Error opening file" not in str(error):
            raise
        pytest.skip(f"Native MuJoCo {mujoco.__version__} cannot compile this VFS basename alias")
    archive = tmp_path / "alias.zip"
    save_model_archive(spec, archive)
    loaded = mujoco.MjSpec.from_zip(archive).compile()
    for field in ("mesh_vert", "mesh_face", "geom_type", "geom_pos", "geom_quat", "geom_size"):
        np.testing.assert_allclose(getattr(loaded, field), getattr(original, field), atol=1e-6, err_msg=field)


@pytest.mark.parametrize("location", ["absolute-file", "absolute-directory", "parent-vfs", "child-relative", "child-strippath"])
def test_attached_child_assets_use_their_own_directory_or_fail_explicitly(tmp_path: Path, location: str) -> None:
    child_folder = tmp_path / "child"
    child_folder.mkdir()
    mesh = child_folder / "tetra.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n")
    filename = mesh.as_posix() if location == "absolute-file" else mesh.name
    if location == "child-strippath":
        filename = "old/path/tetra.obj"
    strip = ' strippath="false"' if location == "child-strippath" else ""
    compiler = (
        f'<compiler meshdir="{child_folder.as_posix()}"{strip}/>'
        if location in {"absolute-directory", "child-strippath"}
        else ""
    )
    child_xml = child_folder / "child.xml"
    child_xml.write_text(
        f'<mujoco>{compiler}<asset><mesh name="mesh" file="{filename}"/></asset>'
        '<worldbody><body name="body"><joint/><geom type="mesh" mesh="mesh"/></body></worldbody></mujoco>'
    )
    child = mujoco.MjSpec.from_file(str(child_xml))
    spec = mujoco.MjSpec()
    if location == "child-strippath":
        # strippath belongs to the assembled parent; child directories remain local.
        spec.strippath = True
    spec.attach(child, prefix="child_", frame=spec.worldbody.add_frame())
    if location == "parent-vfs":
        spec.assets = {mesh.name: mesh.read_bytes()}
    source = spec.copy().compile()
    before = _source_state(spec)
    output = tmp_path / "model.zip"
    # MuJoCo 3.4 does not expose per-element compiler directories either.
    unavailable = location == "child-relative" or (location == "absolute-directory" and not hasattr(spec.meshes[0], "compiler"))
    unavailable |= location == "child-strippath" and not hasattr(spec.meshes[0], "compiler")
    if unavailable:
        output.write_bytes(b"keep this file")
        with pytest.raises(ValueError, match="child-relative model directory"):
            save_model_archive(spec, output, overwrite=True)
        assert output.read_bytes() == b"keep this file"
    else:
        save_model_archive(spec, output)
        loaded = mujoco.MjSpec.from_zip(output).compile()
        np.testing.assert_array_equal(loaded.mesh_vert, source.mesh_vert)
        np.testing.assert_array_equal(loaded.mesh_face, source.mesh_face)
        assert loaded.names == source.names
    _assert_source_unchanged(spec, before)


def test_elbow_native_archive_preserves_edited_anatomy(elbow: tuple[mujoco.MjSpec, mujoco.MjModel], tmp_path: Path) -> None:
    spec, source = elbow
    before = _source_state(spec)
    edited = apply_model_patch(spec, _edit(source))
    model = edited.compile()
    for field in (
        "geom_type",
        "geom_size",
        "geom_pos",
        "geom_quat",
        "geom_dataid",
        "mesh_vert",
        "mesh_face",
        "actuator_gainprm",
        "actuator_biasprm",
        "actuator_dynprm",
        "actuator_lengthrange",
        "eq_data",
    ):
        np.testing.assert_array_equal(getattr(model, field), getattr(source, field), err_msg=field)
    old_length, old_moment = _curves(source)
    new_length, new_moment = _curves(model)
    selected = model.actuator("BRA").id
    assert np.max(np.abs(new_length[:, selected] - old_length[:, selected])) > 1e-4
    assert np.max(np.abs(new_moment[:, selected] - old_moment[:, selected])) > 1e-5
    others = np.arange(model.nu) != selected
    np.testing.assert_array_equal(new_length[:, others], old_length[:, others])
    np.testing.assert_allclose(new_moment[:, others], old_moment[:, others], atol=1e-12, rtol=1e-12)
    archive = tmp_path / "edited.zip"
    save_model_archive(edited, archive)
    loaded = mujoco.MjSpec.from_zip(archive).compile()
    for field in (
        "geom_type",
        "geom_size",
        "geom_pos",
        "geom_quat",
        "mesh_vert",
        "mesh_face",
        "site_pos",
    ):
        np.testing.assert_allclose(getattr(loaded, field), getattr(model, field), atol=1e-6, err_msg=field)
    np.testing.assert_allclose(_curves(loaded)[0], new_length, atol=1e-12)
    np.testing.assert_allclose(_curves(loaded)[1], new_moment, atol=1e-12)
    _assert_source_unchanged(spec, before)


def test_archive_refuses_static_fusion_without_changing_source(tmp_path: Path) -> None:
    spec = _fusion_spec()
    source = spec.copy().compile()
    before = _source_state(spec)
    with pytest.raises(ValueError, match="fusion"):
        save_model_archive(spec, tmp_path / "fused.zip")
    assert not (tmp_path / "fused.zip").exists()
    _assert_source_unchanged(spec, before)
    unchanged = spec.copy().compile()
    for field in ("geom_pos", "geom_quat", "site_pos", "site_quat"):
        np.testing.assert_array_equal(getattr(unchanged, field), getattr(source, field))


@pytest.mark.parametrize(("name", "side"), [("myoarms", "l"), ("myofullbody", "r")])
def test_composed_selected_edits_survive_native_archive(name: str, side: str, tmp_path: Path) -> None:
    spec = myo_sim.load_spec(name)
    original = spec.compile()
    before = _source_state(spec)
    before_xml = spec.to_xml()
    patch = _patch(original)
    site = original.site(f"BIClong_BIClong-P3_{side}")
    patch["sites"] = [dict(name=site.name, pos=(site.pos + [0.002, -0.001, 0.001]).tolist())]
    body = original.body(f"humerus_{side}")
    tensor = np.asarray(_inertia(body.inertia, body.iquat)) * 1.05
    patch["inertias"] = [dict(name=body.name, mass=float(body.mass[0]) * 1.01, com=body.ipos.tolist(), inertia=tensor.tolist())]
    edited = apply_model_patch(spec, patch)
    model = edited.compile()
    edited_before = _source_state(edited)
    output = save_model_archive(edited, tmp_path / f"{name}.zip")
    loaded = mujoco.MjSpec.from_zip(output).compile()
    assert (loaded.nbody, loaded.njnt, loaded.nu, loaded.ntendon, loaded.neq) == (
        model.nbody,
        model.njnt,
        model.nu,
        model.ntendon,
        model.neq,
    )
    np.testing.assert_allclose(loaded.site(site.name).pos, patch["sites"][0]["pos"], atol=1e-6)
    np.testing.assert_allclose(loaded.body(body.name).mass, patch["inertias"][0]["mass"], rtol=1e-5)
    np.testing.assert_allclose(
        _inertia(loaded.body(body.name).inertia, loaded.body(body.name).iquat), tensor, atol=1e-8, rtol=1e-5
    )
    other = "r" if side == "l" else "l"
    np.testing.assert_allclose(
        loaded.site(f"BIClong_BIClong-P3_{other}").pos, original.site(f"BIClong_BIClong-P3_{other}").pos, atol=1e-6
    )
    np.testing.assert_allclose(loaded.eq_data, model.eq_data, rtol=1e-5, atol=1e-12)
    np.testing.assert_allclose(loaded.actuator_gainprm, model.actuator_gainprm, rtol=1e-5, atol=1e-12)
    np.testing.assert_allclose(loaded.actuator_biasprm, model.actuator_biasprm, rtol=1e-5, atol=1e-12)
    np.testing.assert_allclose(loaded.tendon_width, model.tendon_width, rtol=1e-5, atol=1e-12)
    np.testing.assert_allclose(loaded.tendon_rgba, model.tendon_rgba, rtol=1e-5, atol=1e-12)
    massless = model.body_mass == 0
    np.testing.assert_allclose(loaded.body_ipos[massless], model.body_ipos[massless], atol=1e-12)
    _assert_source_unchanged(spec, before)
    _assert_source_unchanged(edited, edited_before)
    assert spec.to_xml() == before_xml
    assert model_fingerprint(spec.compile()) == model_fingerprint(original)


def test_archive_protects_existing_destination(tmp_path: Path) -> None:
    output = tmp_path / "existing.zip"
    output.write_bytes(b"keep this file")
    with pytest.raises(FileExistsError, match="overwrite"):
        save_model_archive(mujoco.MjSpec(), output)
    assert output.read_bytes() == b"keep this file"
    assert save_model_archive(mujoco.MjSpec(), output, overwrite=True) == output
    assert mujoco.MjSpec.from_zip(output).compile().nbody == 1


@pytest.mark.parametrize("loss", ["site-position", "object-name", "statistic"])
def test_archive_rejects_compilable_semantic_loss_transactionally(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, loss: str
) -> None:
    spec, _ = _elbow_model()
    before = _source_state(spec)
    output = tmp_path / "existing.zip"
    output.write_bytes(b"keep this file")
    read = mujoco.MjSpec.from_zip

    def changed(path: Path) -> mujoco.MjSpec:
        loaded = read(path)
        if loss == "object-name":
            actuator = loaded.actuators[0]
            actuator.name = "X" + actuator.name[1:]  # Same byte length leaves name-address arrays unchanged.
        elif loss == "statistic":
            loaded.stat.meaninertia = 2 * loaded.compile().stat.meaninertia
        else:
            loaded.site("BRA-P2").pos[0] += 0.01
        return loaded

    monkeypatch.setattr(mujoco.MjSpec, "from_zip", changed)
    with pytest.raises(ValueError, match="equivalence"):
        save_model_archive(spec, output, overwrite=True)
    assert output.read_bytes() == b"keep this file"
    _assert_source_unchanged(spec, before)


def test_archive_drops_flattened_include_and_unused_xml_assets(tmp_path: Path) -> None:
    part = b'<mujoco><worldbody><body name="included"><joint/><geom size=".1"/></body></worldbody></mujoco>'
    (tmp_path / "part.xml").write_bytes(part)
    source = tmp_path / "source.xml"
    source.write_text('<mujoco><include file="part.xml"/></mujoco>')
    spec = mujoco.MjSpec.from_file(str(source))
    spec.assets = {"part.xml": part, "unused.xml": b'<mujoco model="wrong"/>'}
    original = spec.copy().compile()
    output = save_model_archive(spec, tmp_path / "included.zip")
    loaded = mujoco.MjSpec.from_zip(output).compile()
    assert loaded.body("included").id == original.body("included").id
    assert loaded.names == original.names


@pytest.mark.parametrize("attached_defaults", [False, True])
@pytest.mark.parametrize("text", ["plain", "a\tb", "a\nb", "a\rb", "a\r\nb", "\u96ea\u00e9 &<>\"'\t\r\n"])
def test_archive_preserves_custom_text_bytes(tmp_path: Path, text: str, attached_defaults: bool) -> None:
    spec = mujoco.MjSpec.from_string('<mujoco><custom><text name="note" data="initial"/></custom></mujoco>')
    spec.texts[0].data = text
    if attached_defaults:
        child = mujoco.MjSpec.from_string(
            '<mujoco><default><geom rgba=".2 .3 .4 1"/></default><worldbody><geom size=".1"/></worldbody></mujoco>'
        )
        spec.attach(child, prefix="child_", frame=spec.worldbody.add_frame())
    source = spec.copy().compile()
    output = save_model_archive(spec, tmp_path / "text.zip")
    loaded = mujoco.MjSpec.from_zip(output).compile()
    assert loaded.text_data == source.text_data
    np.testing.assert_array_equal(loaded.text_adr, source.text_adr)
    np.testing.assert_array_equal(loaded.text_size, source.text_size)
    assert spec.texts[0].data == text
    assert model_fingerprint(spec.compile()) == model_fingerprint(source)


def test_archive_rejects_same_size_custom_text_loss(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    spec = mujoco.MjSpec.from_string('<mujoco><custom><text name="note" data="left right"/></custom></mujoco>')
    output = tmp_path / "existing.zip"
    output.write_bytes(b"keep this file")
    read = mujoco.MjSpec.from_zip

    def changed(path: Path) -> mujoco.MjSpec:
        loaded = read(path)
        loaded.texts[0].data = "LEFT right"
        return loaded

    monkeypatch.setattr(mujoco.MjSpec, "from_zip", changed)
    with pytest.raises(ValueError, match="custom text"):
        save_model_archive(spec, output, overwrite=True)
    assert output.read_bytes() == b"keep this file"
    assert spec.texts[0].data == "left right"


@pytest.mark.parametrize("measurement", ["length-offset", "length-nan", "moment-nan"])
def test_archive_checks_native_tendon_curves(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, measurement: str) -> None:
    spec, _ = _elbow_model()
    forward = mujoco.mj_forward
    calls = 0

    def changed(model: mujoco.MjModel, data: mujoco.MjData) -> None:
        nonlocal calls
        forward(model, data)
        calls += 1
        if measurement == "length-nan":
            data.ten_length[0] = np.nan  # Matching invalid source/reload measurements must still fail.
        elif measurement == "moment-nan":
            data.ten_J.flat[0] = np.nan
        elif calls % 2 == 0:
            data.ten_length[0] += 0.001

    monkeypatch.setattr(mujoco, "mj_forward", changed)
    output = tmp_path / "invalid.zip"
    with pytest.raises(ValueError, match="native tendon"):
        save_model_archive(spec, output)
    assert not output.exists()


def test_archive_rejects_raw_mesh_vertex_loss(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    spec, _ = _elbow_model()
    read, compile = mujoco.MjSpec.from_zip, mujoco.MjSpec.compile
    loaded_specs = []

    def loaded(path: Path) -> mujoco.MjSpec:
        result = read(path)
        loaded_specs.append(result)
        return result

    def changed(candidate: mujoco.MjSpec) -> mujoco.MjModel:
        model = compile(candidate)
        if any(candidate is source for source in loaded_specs):
            model.mesh_vert[0, 0] += 0.01
        return model

    monkeypatch.setattr(mujoco.MjSpec, "from_zip", loaded)
    monkeypatch.setattr(mujoco.MjSpec, "compile", changed)
    output = tmp_path / "invalid-mesh.zip"
    with pytest.raises(ValueError, match="mesh_vert"):
        save_model_archive(spec, output)
    assert not output.exists()


@pytest.mark.parametrize("explicit_pair", [False, True])
def test_archive_keeps_collision_mesh_hull_checks(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, explicit_pair: bool) -> None:
    flag = "0" if explicit_pair else "1"
    pair = '<contact><pair geom1="mesh" geom2="other"/></contact>' if explicit_pair else ""
    spec = mujoco.MjSpec.from_string(
        '<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" '
        'face="0 1 2 0 3 1 0 2 3 1 3 2"/></asset><worldbody><body><joint/>'
        f'<geom name="mesh" type="mesh" mesh="m" contype="{flag}" conaffinity="{flag}"/>'
        '</body><geom name="other" size=".1" pos="2 0 0" contype="0" conaffinity="0"/>'
        f"</worldbody>{pair}</mujoco>"
    )
    read, compile = mujoco.MjSpec.from_zip, mujoco.MjSpec.compile
    loaded_specs = []

    def loaded(path: Path) -> mujoco.MjSpec:
        result = read(path)
        loaded_specs.append(result)
        return result

    def changed(candidate: mujoco.MjSpec) -> mujoco.MjModel:
        model = compile(candidate)
        if any(candidate is source for source in loaded_specs):
            model.mesh_polynum[0] += 1
        return model

    monkeypatch.setattr(mujoco.MjSpec, "from_zip", loaded)
    monkeypatch.setattr(mujoco.MjSpec, "compile", changed)
    output = tmp_path / "invalid-hull.zip"
    with pytest.raises(ValueError, match="mesh_polynum"):
        save_model_archive(spec, output)
    assert not output.exists()
