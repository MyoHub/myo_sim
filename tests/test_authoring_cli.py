# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Anatomy CLI staging and native model archives preserve the source model."""

import json
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

from myo_sim.authoring import cli as authoring_cli


@pytest.fixture
def cli() -> ModuleType:
    return authoring_cli


@pytest.mark.parametrize("archive_name", ["model.zip", "model.model"])
def test_cli_export_apply_and_protected_outputs(cli: ModuleType, tmp_path: Path, archive_name: str) -> None:
    xml = tmp_path / "source.xml"
    xml.write_text(
        '<mujoco><worldbody><body name="b"><joint name="q" range="-30 30"/>'
        '<geom size=".1"/><site name="s" pos=".1 0 0"/></body></worldbody></mujoco>'
    )
    output = tmp_path / "export.json"
    assert (
        cli.main(
            [
                "export",
                "--xml",
                str(xml),
                "--joint-sweep",
                "q",
                "--samples",
                "3",
                "--output",
                str(output),
            ]
        )
        == 0
    )
    document = json.loads(output.read_text())
    assert len(document["frames"]) == 3
    patch = {k: document[k] for k in ("fingerprint", "mujoco_version")}
    patch["schema"] = document["patch_schema"]
    patch.update(sites=[dict(name="s", pos=[0.2, 0, 0])], inertias=[])
    patch_file = tmp_path / "edits.json"
    patch_file.write_text(json.dumps(patch))
    archive = tmp_path / archive_name
    args = [
        "apply",
        "--xml",
        str(xml),
        "--patch",
        str(patch_file),
        "--output",
        str(archive),
    ]
    assert cli.main(args) == 0
    np.testing.assert_allclose(mujoco.MjSpec.from_zip(archive).compile().site("s").pos, [0.2, 0, 0])
    roundtrip = tmp_path / "reexport.json"
    assert cli.main(["export", "--xml", str(archive), "--output", str(roundtrip)]) == 0
    assert next(site for site in json.loads(roundtrip.read_text())["sites"] if site["source_name"] == "s")["pos"] == [0.2, 0, 0]
    before = archive.read_bytes()
    with pytest.raises(SystemExit):
        cli.main(args)
    assert archive.read_bytes() == before
    patch["fingerprint"] = "wrong"
    patch_file.write_text(json.dumps(patch))
    with pytest.raises(SystemExit):
        cli.main([*args, "--overwrite"])
    assert archive.read_bytes() == before


def test_module_entry_exports_outside_checkout(tmp_path: Path) -> None:
    """The packaged module entry locates its dependencies from another directory."""
    source = tmp_path / "source.xml"
    source.write_text('<mujoco><worldbody><geom size=".1"/></worldbody></mujoco>')
    output = tmp_path / "anatomy.json"
    result = subprocess.run(
        [sys.executable, "-m", "myo_sim.authoring", "export", "--xml", str(source), "--output", str(output)],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert json.loads(output.read_text())["schema"] == 2


def test_cli_exposes_only_application_independent_commands(cli: ModuleType, capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as result:
        cli.main(["--help"])
    assert result.value.code == 0
    help_text = capsys.readouterr().out
    assert "{export,apply}" in help_text
    for application in ("blender", "muskemo"):
        with pytest.raises(SystemExit) as result:
            cli.main([application])
        assert result.value.code == 2


def test_invalid_joint_name_reports_error_and_preserves_output(
    cli: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    source = tmp_path / "source.xml"
    source.write_text(
        '<mujoco><worldbody><body><joint name="valid" range="-30 30"/><geom size=".1"/></body></worldbody></mujoco>'
    )
    output = tmp_path / "anatomy.json"
    output.write_text("previous output")

    with pytest.raises(SystemExit) as result:
        cli.main(
            [
                "export",
                "--xml",
                str(source),
                "--joint-sweep",
                "misspelled",
                "--output",
                str(output),
                "--overwrite",
            ]
        )

    assert result.value.code == 2
    error = capsys.readouterr().err
    assert "misspelled" in error and "Traceback" not in error
    assert output.read_text() == "previous output"
