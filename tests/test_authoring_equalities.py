# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Equality references retain native object namespaces in anatomy exports."""

from __future__ import annotations

from pathlib import Path

import mujoco
import pytest

from myo_sim.authoring import export_model

_SOURCE = """<mujoco><worldbody>
  <site name="padding"/>
  <body name="body_a"><joint name="driver"/><geom size=".1"/>
    <site name="site_a" pos=".2 0 0"/></body>
  <body name="body_b"><joint name="follower"/><geom size=".1"/>
    <site name="site_b" pos=".2 0 0"/></body>
</worldbody><tendon>
  <fixed name="path_a"><joint joint="driver" coef="1"/></fixed>
  <fixed name="path_b"><joint joint="follower" coef="1"/></fixed>
</tendon><equality>{equalities}</equality></mujoco>"""


def test_equality_reference_namespaces_resolve_to_native_source_objects(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string(
        _SOURCE.format(
            equalities='<connect body1="body_a" body2="body_b" anchor="0 0 0" active="false"/>'
            '<connect site1="site_a" site2="site_b" active="false"/>'
            '<joint joint1="follower" joint2="driver" active="false"/>'
            '<tendon tendon1="path_b" tendon2="path_a" active="false"/>'
        )
    )
    document = export_model(model, tmp_path / "equalities.json")

    # The CONNECT integer IDs overlap, so their namespaces are essential.
    assert model.eq_obj1id[0] == model.eq_obj1id[1]
    assert model.eq_obj2id[0] == model.eq_obj2id[1]
    expected = [
        ("CONNECT", "BODY", model.body, "body_a", "body_b"),
        ("CONNECT", "SITE", model.site, "site_a", "site_b"),
        ("JOINT", "UNKNOWN", model.joint, "follower", "driver"),
        ("TENDON", "UNKNOWN", model.tendon, "path_b", "path_a"),
    ]
    assert len(document["equalities"]) == len(expected)
    for equality, (kind, namespace, resolve, first, second) in zip(document["equalities"], expected, strict=True):
        assert equality["type"] == kind
        assert equality["object_type"] == namespace
        assert resolve(equality["object1"]).name == first
        assert resolve(equality["object2"]).name == second


@pytest.mark.parametrize("kind,name", [("joint", "follower"), ("tendon", "path_b")])
def test_missing_secondary_equality_object_preserves_native_sentinel(tmp_path: Path, kind: str, name: str) -> None:
    model = mujoco.MjModel.from_xml_string(
        _SOURCE.format(equalities=f'<{kind} {kind}1="{name}" polycoef=".2 0 0 0 0" active="false"/>')
    )
    document = export_model(model, tmp_path / "locked.json")

    equality = document["equalities"][0]
    assert equality["type"] == kind.upper()
    # Native eq_objtype is meaningful for CONNECT/WELD; the other kinds imply it.
    assert equality["object_type"] == "UNKNOWN"
    assert getattr(model, kind)(equality["object1"]).name == name
    assert model.eq_obj2id[0] == equality["object2"] == -1
