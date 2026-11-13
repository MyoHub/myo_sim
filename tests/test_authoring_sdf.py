# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Unsupported SDF exports fail without publishing incomplete native geometry."""

from __future__ import annotations

from pathlib import Path

import mujoco
import pytest

from myo_sim.authoring import export_model, model_fingerprint


@pytest.fixture(scope="module")
def native_sdf() -> mujoco.MjModel:
    """Use MuJoCo's bundled torus plugin, without an external integration."""
    xml = """<mujoco>
      <extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"/></plugin></extension>
      <asset><mesh name="torus"><plugin instance="torus"/></mesh></asset>
      <worldbody><geom name="torus_geom" type="sdf" mesh="torus">
        <plugin instance="torus"/>
      </geom></worldbody>
    </mujoco>"""
    try:
        model = mujoco.MjModel.from_xml_string(xml)
    except ValueError as error:
        message = str(error).lower()
        if "unrecognized plugin 'mujoco.sdf.torus'" in message:
            pytest.skip(f"This native MuJoCo build does not provide its bundled SDF torus plugin: {error}")
        raise
    assert model.geom_type[0] == mujoco.mjtGeom.mjGEOM_SDF
    assert model.nmesh == 1 and model.mesh_vertnum[0] > 0
    return model


@pytest.mark.parametrize("existing", [False, True])
def test_sdf_export_rejects_incomplete_geometry_without_changing_source_or_output(
    tmp_path: Path, native_sdf: mujoco.MjModel, existing: bool
) -> None:
    destination = tmp_path / "anatomy.json"
    previous = b'{"previous": "anatomy"}\n'
    if existing:
        destination.write_bytes(previous)
    fingerprint = model_fingerprint(native_sdf)

    with pytest.raises(ValueError, match="SDF"):
        export_model(native_sdf, destination)

    assert model_fingerprint(native_sdf) == fingerprint
    if existing:
        assert destination.read_bytes() == previous
    else:
        assert not destination.exists()
