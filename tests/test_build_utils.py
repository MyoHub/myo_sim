"""XML composition preserves source trees, child order and mirror conventions."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from myo_sim.build.utils import MirrorRules, component_children, expand_component_element, mirror_element


def test_mirror_nested_tree_preserves_source_and_xml_content() -> None:
    source = ET.fromstring(
        '<body name="arm_r" pos="1 2 3" euler="0.1 0.2 0.3">root text'
        '<geom name="Thorax_ellipsoid_r" fromto="1 2 3 4 5 6" material="MatSkin"/>geom tail'
        '<body name="hand_r" quat="1 2 3 4">'
        '<joint name="wrist_r" axis="1 2 3"/>'
        '<inertial pos="4 5 6" fullinertia="1 2 3 4 5 6"/>'
        '<camera name="view_r" quat="1 2 3 4"/>'
        "</body>body tail"
        '<site name="tip_r" pos="7 8 9"/>'
        "</body>"
    )
    before = ET.tostring(source)
    expected = ET.fromstring(
        '<body name="arm_l" pos="1 2 -3" euler="0.1 0.2 0.3">root text'
        '<geom name="thorax_ellipsoid_l" fromto="1 2 -3 4 5 -6" material="MatSkin_l"/>geom tail'
        '<body name="hand_l" quat="1 -2 -3 4">'
        '<joint name="wrist_l" axis="-1 -2 3"/>'
        '<inertial pos="4 5 -6" fullinertia="1 2 3 4 -5 -6"/>'
        '<camera name="view_l" quat="3 -4 1 -2"/>'
        "</body>body tail"
        '<site name="tip_l" pos="7 8 -9"/>'
        "</body>"
    )

    mirrored = mirror_element(source, MirrorRules())

    assert ET.tostring(mirrored) == ET.tostring(expected)
    assert ET.tostring(source) == before
    for original, transformed in zip(source.iter(), mirrored.iter()):
        assert transformed is not original
        assert transformed.attrib is not original.attrib


def test_mirror_nested_tree_applies_custom_rules() -> None:
    source = ET.fromstring(
        '<default class="arm_r"><body pos="1 2 3" name="arm_r" childclass="shared">'
        '<geom name="shape_r" mesh="shared" material="MatSkin"/>'
        '<mesh name="custom_mesh" file="bone_r" scale="1 2 3"/>'
        '<site name="pref_tip"/>'
        "</body></default>"
    )
    rules = MirrorRules(
        mirrored_material="left_skin",
        common_names=frozenset({"shared"}),
        body_pos_x_mirror_names=frozenset({"arm_r"}),
        replacements=(("custom", "left"),),
        prefix_replacements=(("pref_", "left_"),),
        mirror_file_attributes=True,
    )

    mirrored = mirror_element(source, rules)

    assert mirrored.get("class") == "arm_l"
    body = mirrored[0]
    assert body.attrib == {"pos": "-1 2 3", "name": "arm_l", "childclass": "shared"}
    assert body[0].attrib == {"name": "shape_l", "mesh": "shared", "material": "left_skin"}
    assert body[1].attrib == {"name": "left_mesh", "file": "bone_l", "scale": "1 2 -3"}
    assert body[2].get("name") == "left_tip"


def test_expand_nested_includes_preserves_order_content_and_source(tmp_path: Path) -> None:
    parts = tmp_path / "parts"
    parts.mkdir()
    (parts / "inner.xml").write_text('<mujocoinclude><site name="inner"/><site name="inner2"/></mujocoinclude>')
    (parts / "outer.xml").write_text(
        '<mujocoinclude><body name="included">body text<include file="inner.xml"/>body tail</body></mujocoinclude>'
    )
    source = ET.fromstring(
        '<worldbody>root text<body name="before"><site name="tip"/>site tail</body>before tail'
        '<include file="parts/outer.xml"/><include/><body name="after"/>after tail</worldbody>'
    )
    before = ET.tostring(source)
    expected = ET.fromstring(
        '<worldbody>root text<body name="before"><site name="tip"/>site tail</body>before tail'
        '<body name="included">body text<site name="inner"/><site name="inner2"/></body>'
        '<body name="after"/>after tail</worldbody>'
    )

    (expanded,) = expand_component_element(source, tmp_path)

    assert ET.tostring(expanded) == ET.tostring(expected)
    assert ET.tostring(source) == before
    expanded[0][0].set("name", "edited")
    assert source[0][0].get("name") == "tip"
    # Every read returns an independent tree, including file-backed replacements.
    expanded[1][0].set("name", "edited")
    assert component_children(parts / "outer.xml")[0][0].get("name") == "inner"


def test_expand_missing_include_reports_its_resolved_path(tmp_path: Path) -> None:
    source = ET.fromstring('<body><include file="missing.xml"/></body>')
    with pytest.raises(FileNotFoundError, match="MJCF include not found") as error:
        expand_component_element(source, tmp_path)
    assert str(tmp_path / "missing.xml") in str(error.value)
