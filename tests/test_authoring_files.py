# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Public authoring operations preserve input files and competing output writers."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import pytest

from myo_sim import authoring
from myo_sim.authoring import cli


@pytest.fixture
def inputs(tmp_path: Path) -> tuple[Path, Path]:
    source = tmp_path / "source.xml"
    source.write_text(
        '<mujoco><worldbody><body name="body"><joint/><geom size=".1"/>'
        '<site name="s" pos=".1 0 0"/></body></worldbody></mujoco>'
    )
    model = mujoco.MjSpec.from_file(str(source)).compile()
    patch = tmp_path / "patch.json"
    patch.write_text(
        json.dumps(
            dict(
                schema=1,
                fingerprint=authoring.model_fingerprint(model),
                mujoco_version=mujoco.__version__,
                sites=[dict(name="s", pos=[0.2, 0, 0])],
                inertias=[],
            )
        )
    )
    return source, patch


def _run(operation: str, source: Path, patch: Path, output: Path, *, overwrite: bool = False) -> Any:
    if operation == "archive":
        return authoring.save_model_archive(mujoco.MjSpec.from_file(str(source)), output, overwrite=overwrite)
    args = [operation, "--xml", str(source), "--output", str(output)]
    if operation == "apply":
        args += ["--patch", str(patch)]
    if overwrite:
        args.append("--overwrite")
    return cli.main(args)


@pytest.mark.parametrize("operation", ["export", "apply", "archive"])
def test_no_overwrite_preserves_a_writer_that_finishes_at_publication(
    inputs: tuple[Path, Path], tmp_path: Path, monkeypatch: pytest.MonkeyPatch, operation: str
) -> None:
    source, patch = inputs
    output = tmp_path / "competing-output"
    written = b"another writer completed"
    calls = []
    publish = Path.rename if os.name == "nt" else os.link

    def competing_writer(staged: Path, destination: Path) -> Any:
        if Path(destination).resolve() == output:
            assert Path(staged).stat().st_size > 0
            calls.append(staged)
            output.write_bytes(written)
        return publish(staged, destination)

    if os.name == "nt":
        monkeypatch.setattr(Path, "rename", competing_writer)
    else:
        monkeypatch.setattr(os, "link", competing_writer)
    expected = FileExistsError if operation == "archive" else SystemExit
    with pytest.raises(expected):
        _run(operation, source, patch, output)

    assert len(calls) == 1
    assert output.read_bytes() == written
    assert set(tmp_path.iterdir()) == {source, patch, output}


@pytest.mark.parametrize("operation", ["export", "apply", "archive"])
def test_existing_output_is_refused_before_any_native_compilation(
    inputs: tuple[Path, Path], tmp_path: Path, monkeypatch: pytest.MonkeyPatch, operation: str
) -> None:
    source, patch = inputs
    output = tmp_path / "existing-output"
    output.write_bytes(b"keep this output")

    def forbidden_compile(spec: mujoco.MjSpec) -> mujoco.MjModel:
        raise AssertionError("Existing outputs must be refused before compilation")

    monkeypatch.setattr(mujoco.MjSpec, "compile", forbidden_compile)
    expected = FileExistsError if operation == "archive" else SystemExit
    with pytest.raises(expected):
        _run(operation, source, patch, output)
    assert output.read_bytes() == b"keep this output"


@pytest.mark.parametrize("operation", ["export", "apply"])
def test_cli_refuses_output_aliasing_source_xml(inputs: tuple[Path, Path], tmp_path: Path, operation: str) -> None:
    source, patch = inputs
    before = source.read_bytes()
    folder = tmp_path / "alias"
    folder.mkdir()
    output = folder / ".." / source.name
    with pytest.raises(SystemExit) as error:
        _run(operation, source, patch, output, overwrite=True)
    assert error.value.code == 2
    assert source.read_bytes() == before


def test_cli_refuses_output_aliasing_patch(inputs: tuple[Path, Path], tmp_path: Path) -> None:
    source, patch = inputs
    before = patch.read_bytes()
    folder = tmp_path / "alias"
    folder.mkdir()
    with pytest.raises(SystemExit) as error:
        _run("apply", source, patch, folder / ".." / patch.name, overwrite=True)
    assert error.value.code == 2
    assert patch.read_bytes() == before


def test_in_place_archive_apply_is_explicit_and_preserves_input_on_failure(inputs: tuple[Path, Path], tmp_path: Path) -> None:
    source, patch = inputs
    archive = tmp_path / "instance.model"
    authoring.save_model_archive(mujoco.MjSpec.from_file(str(source)), archive)
    model = mujoco.MjSpec.from_zip(archive).compile()
    document = json.loads(patch.read_text())
    document["fingerprint"] = authoring.model_fingerprint(model)
    patch.write_text(json.dumps(document))
    before = archive.read_bytes()

    with pytest.raises(SystemExit):
        _run("export", archive, patch, archive, overwrite=True)
    with pytest.raises(SystemExit):
        _run("apply", archive, patch, archive)
    assert archive.read_bytes() == before

    assert _run("apply", archive, patch, archive, overwrite=True) == 0
    np.testing.assert_allclose(mujoco.MjSpec.from_zip(archive).compile().site("s").pos, [0.2, 0, 0])
    before = archive.read_bytes()
    document["fingerprint"] = "wrong"
    patch.write_text(json.dumps(document))
    with pytest.raises(SystemExit):
        _run("apply", archive, patch, archive, overwrite=True)
    assert archive.read_bytes() == before


def test_cli_apply_cannot_replace_a_referenced_disk_asset(inputs: tuple[Path, Path], tmp_path: Path) -> None:
    source, patch = inputs
    asset = tmp_path / "tetra.obj"
    asset.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n")
    source.write_text(
        '<mujoco><asset><mesh name="m" file="tetra.obj"/></asset><worldbody>'
        '<body><joint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>'
    )
    document = json.loads(patch.read_text())
    document.update(fingerprint=authoring.model_fingerprint(mujoco.MjSpec.from_file(str(source)).compile()), sites=[])
    patch.write_text(json.dumps(document))
    before = asset.read_bytes()
    with pytest.raises(SystemExit) as error:
        _run("apply", source, patch, asset, overwrite=True)
    assert error.value.code == 2
    assert asset.read_bytes() == before


def test_replacing_a_distinct_hardlink_output_keeps_source_input(inputs: tuple[Path, Path], tmp_path: Path) -> None:
    source, patch = inputs
    alias = tmp_path / "output.json"
    try:
        os.link(source, alias)
    except OSError as error:
        pytest.skip(f"Hardlinks unavailable: {error}")
    before = source.read_bytes()
    assert _run("export", source, patch, alias, overwrite=True) == 0
    assert source.read_bytes() == before
    assert json.loads(alias.read_text())["schema"] == 2


def test_failed_export_removes_partial_output_and_preserves_destination(
    inputs: tuple[Path, Path], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source, patch = inputs
    output = tmp_path / "previous.json"
    output.write_bytes(b"previous completed output")

    def partial_export(model: mujoco.MjModel, staged: Path, **kwargs: Any) -> None:
        staged.write_bytes(b"partial output")
        raise OSError("write failed")

    monkeypatch.setattr(authoring, "export_model", partial_export)
    with pytest.raises(SystemExit):
        _run("export", source, patch, output, overwrite=True)
    assert output.read_bytes() == b"previous completed output"
    assert set(tmp_path.iterdir()) == {source, patch, output}
