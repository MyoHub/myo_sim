# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Archive the asset sources actually selected by native MuJoCo."""

from __future__ import annotations

import os
import struct
import zipfile
import zlib
from pathlib import Path

import mujoco
import numpy as np
import pytest

from myo_sim.authoring import model_fingerprint, save_model_archive


def _png(color: tuple[int, int, int]) -> bytes:
    """A tiny RGB PNG fixture without an optional image-library dependency."""

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    rows = (b"\0" + bytes(color) * 2) * 2
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">2I5B", 2, 2, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )


def _skin() -> bytes:
    """Native SKN: one triangle, with all vertices bound to body 'bone'."""
    return (
        struct.pack("<4i", 3, 0, 1, 1)
        + struct.pack("<9f", 0, 0, 0, 1, 0, 0, 0, 1, 0)
        + struct.pack("<3i", 0, 1, 2)
        + b"bone"
        + bytes(36)
        + struct.pack("<7fi", 0, 0, 0, 1, 0, 0, 0, 3)
        + struct.pack("<3i", 0, 1, 2)
        + struct.pack("<3f", 1, 1, 1)
    )


@pytest.mark.parametrize("procedural", [False, True])
def test_texture_archive_ignores_inactive_files(tmp_path: Path, procedural: bool) -> None:
    images = tmp_path / "images"
    images.mkdir()
    (images / "main.png").write_bytes(_png((30, 120, 210)))
    attributes = 'builtin="checker" width="4" file="missing-main.png"' if procedural else 'file="main.png"'
    source = tmp_path / "source.xml"
    source.write_text(
        '<mujoco><compiler texturedir="images"/><asset>'
        f'<texture name="t" type="cube" {attributes} fileleft="missing-face.png"/>'
        '<material name="m" texture="t"/></asset><worldbody><geom size=".1" material="m"/></worldbody></mujoco>'
    )
    spec = mujoco.MjSpec.from_file(str(source))
    original = spec.copy().compile()
    before = spec.to_xml()
    output = save_model_archive(spec, tmp_path / "texture.zip")
    loaded = mujoco.MjSpec.from_zip(output).compile()
    np.testing.assert_array_equal(loaded.tex_data, original.tex_data)
    with zipfile.ZipFile(output) as archive:
        files = {name for name in archive.namelist() if not name.endswith(".xml")}
        assert files == (set() if procedural else {"main.png"})
    assert spec.to_xml() == before


def test_texture_archive_preserves_all_active_cube_faces(tmp_path: Path) -> None:
    faces = ("right", "left", "up", "down", "front", "back")
    for index, face in enumerate(faces):
        (tmp_path / f"{face}.png").write_bytes(_png((20 + 30 * index, 10, 240 - 30 * index)))
    attributes = " ".join(f'file{face}="{face}.png"' for face in faces)
    source = tmp_path / "source.xml"
    source.write_text(f'<mujoco><asset><texture name="cube" type="cube" {attributes}/></asset></mujoco>')
    spec = mujoco.MjSpec.from_file(str(source))
    original = spec.copy().compile()
    loaded = mujoco.MjSpec.from_zip(save_model_archive(spec, tmp_path / "cube.zip")).compile()
    np.testing.assert_array_equal(loaded.tex_data, original.tex_data)


@pytest.mark.parametrize("filename", ["skin.skn", "nested/skin.skn"])
def test_skin_archive_uses_mesh_directory(tmp_path: Path, filename: str) -> None:
    asset = tmp_path / "assets" / filename
    asset.parent.mkdir(parents=True)
    asset.write_bytes(_skin())
    source = tmp_path / "source.xml"
    source.write_text(
        '<mujoco><compiler meshdir="assets"/><worldbody><body name="bone"><joint/><geom size=".1"/>'
        f'</body></worldbody><deformable><skin name="skin" file="{filename}"/></deformable></mujoco>'
    )
    spec = mujoco.MjSpec.from_file(str(source))
    original = spec.copy().compile()
    before = asset.read_bytes()
    loaded = mujoco.MjSpec.from_zip(save_model_archive(spec, tmp_path / "skin.zip")).compile()
    for field in ("skin_vert", "skin_face", "skin_bonebodyid", "skin_bonevertid", "skin_bonevertweight"):
        np.testing.assert_array_equal(getattr(loaded, field), getattr(original, field))
    assert asset.read_bytes() == before
    assert model_fingerprint(spec.compile()) == model_fingerprint(original)


@pytest.mark.parametrize(
    "elevation",
    ["0 .2 .5 1 .75 .9", "1000000 1000000.125 1000000.25 1000000.5 1000000.75 1000001"],
)
def test_inline_heightfield_archive_preserves_rows_and_precision(tmp_path: Path, elevation: str) -> None:
    spec = mujoco.MjSpec.from_string(
        f'<mujoco><asset><hfield name="h" nrow="2" ncol="3" elevation="{elevation}" size="1 1 1 .1"/></asset>'
        '<worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>'
    )
    original = spec.copy().compile()
    before = list(spec.hfields[0].userdata)
    loaded = mujoco.MjSpec.from_zip(save_model_archive(spec, tmp_path / "height.zip")).compile()
    np.testing.assert_array_equal(loaded.hfield_data, original.hfield_data)
    assert list(spec.hfields[0].userdata) == before


def test_binary_heightfield_archive_preserves_elevations(tmp_path: Path) -> None:
    (tmp_path / "height.bin").write_bytes(struct.pack("<2i6f", 2, 3, 0, 0.2, 0.5, 1, 0.75, 0.9))
    source = tmp_path / "source.xml"
    source.write_text(
        '<mujoco><asset><hfield name="h" file="height.bin" size="1 1 1 .1"/></asset>'
        '<worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>'
    )
    spec = mujoco.MjSpec.from_file(str(source))
    original = spec.copy().compile()
    loaded = mujoco.MjSpec.from_zip(save_model_archive(spec, tmp_path / "height.zip")).compile()
    np.testing.assert_array_equal(loaded.hfield_data, original.hfield_data)


def test_attached_texture_defaults_survive_archive(tmp_path: Path) -> None:
    if not hasattr(mujoco.MjSpec().default, "texture"):
        pytest.skip("Native MuJoCo does not support texture default classes")
    child = mujoco.MjSpec.from_string(
        '<mujoco><default><default class="image"><texture type="2d" builtin="checker" width="4" height="4"/>'
        '</default></default><asset><texture name="t" class="image"/><material name="m" texture="t"/></asset>'
        '<worldbody><geom size=".1" material="m"/></worldbody></mujoco>'
    )
    spec = mujoco.MjSpec()
    spec.attach(child, prefix="", frame=spec.worldbody.add_frame())
    original = spec.copy().compile()
    before = spec.to_xml()
    loaded = mujoco.MjSpec.from_zip(save_model_archive(spec, tmp_path / "defaults.zip")).compile()
    np.testing.assert_array_equal(loaded.tex_data, original.tex_data)
    assert spec.to_xml() == before


def _disk_asset_model(tmp_path: Path, kind: str) -> tuple[mujoco.MjSpec, Path]:
    world = '<geom size=".1"/>'
    if kind == "mesh":
        filename = "source.obj"
        data = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n"
        declaration = '<asset><mesh name="m" file="source.obj"/></asset>'
        world = '<geom type="mesh" mesh="m"/>'
    elif kind == "texture":
        filename, data = "source.png", _png((20, 80, 140))
        declaration = '<asset><texture name="t" type="2d" file="source.png"/></asset>'
    elif kind == "heightfield":
        filename, data = "source.bin", struct.pack("<2i4f", 2, 2, 0, 0.2, 0.5, 1)
        declaration = '<asset><hfield name="h" file="source.bin" size="1 1 1 .1"/></asset>'
        world = '<geom type="hfield" hfield="h"/>'
    else:
        filename, data = "source.skn", _skin()
        declaration = '<deformable><skin name="skin" file="source.skn"/></deformable>'
        world = '<body name="bone"><joint/><geom size=".1"/></body>'
    asset = tmp_path / filename
    asset.write_bytes(data)
    source = tmp_path / "source.xml"
    source.write_text(f"<mujoco>{declaration}<worldbody>{world}</worldbody></mujoco>")
    return mujoco.MjSpec.from_file(str(source)), asset


@pytest.mark.parametrize("kind", ["mesh", "texture", "heightfield", "skin"])
def test_archive_cannot_replace_a_disk_input_asset(tmp_path: Path, kind: str) -> None:
    spec, asset = _disk_asset_model(tmp_path, kind)
    original = spec.copy().compile()
    before = asset.read_bytes()
    with pytest.raises(ValueError, match="overwrite.*source asset"):
        save_model_archive(spec, asset, overwrite=True)
    assert asset.read_bytes() == before
    assert model_fingerprint(spec.compile()) == model_fingerprint(original)


def test_archive_allows_a_distinct_hard_link_output(tmp_path: Path) -> None:
    spec, asset = _disk_asset_model(tmp_path, "mesh")
    before = asset.read_bytes()
    output = tmp_path / "model.zip"
    os.link(asset, output)
    save_model_archive(spec, output, overwrite=True)
    assert asset.read_bytes() == before
    assert mujoco.MjSpec.from_zip(output).compile().nmesh == 1


@pytest.mark.parametrize("use_second_asset", [False, True])
def test_exact_vfs_key_takes_priority_over_basename_alias(tmp_path: Path, use_second_asset: bool) -> None:
    tetra = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n"
    assets = {"a/tetra.obj": tetra, "unused/tetra.obj": tetra.replace(b"v 1 0 0", b"v 2 0 0")}
    extra = '<mesh name="b" file="unused/tetra.obj"/>' if use_second_asset else ""
    spec = mujoco.MjSpec.from_string(
        f'<mujoco><asset><mesh name="a" file="a/tetra.obj"/>{extra}</asset>'
        '<worldbody><geom type="mesh" mesh="a"/></worldbody></mujoco>',
        assets=assets,
    )
    original = spec.copy().compile()
    output = tmp_path / "exact.zip"
    if use_second_asset:
        with pytest.raises(ValueError, match="Ambiguous"):
            save_model_archive(spec, output)
        assert not output.exists()
    else:
        loaded = mujoco.MjSpec.from_zip(save_model_archive(spec, output)).compile()
        np.testing.assert_array_equal(loaded.mesh_vert, original.mesh_vert)
        np.testing.assert_array_equal(loaded.mesh_face, original.mesh_face)
    assert dict(spec.assets) == assets
