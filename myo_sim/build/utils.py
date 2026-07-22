"""Utilities for MuJoCo MjSpec model composition."""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco

NAME_REFERENCE_ATTRS = {
    "name",
    "mesh",
    "site",
    "geom",
    "sidesite",
    "body1",
    "body2",
    "joint1",
    "joint2",
    "joint",
    "tendon",
}


@dataclass(frozen=True)
class MirrorRules:
    """Rules for mirroring a right-side MJCF subtree into a left-side subtree."""

    mirrored_material: str = "MatSkin_l"
    common_names: frozenset[str] = frozenset({"texgeom", "geom"})
    body_pos_x_mirror_names: frozenset[str] = frozenset()
    lowercase_geom_prefixes: tuple[str, ...] = ("Thorax_ellipsoid",)
    replacements: tuple[tuple[str, str], ...] = ()
    prefix_replacements: tuple[tuple[str, str], ...] = ()
    mirror_file_attributes: bool = False


def find_body(spec: object, body_name: str) -> object:
    """Return the named body, raising ValueError if it is not present."""
    body = spec.body(body_name)
    if body is None:
        raise ValueError(f"Body not found: {body_name!r}")
    return body


def find_site(spec: object, site_name: str) -> object:
    """Return the named site, raising ValueError if it is not present."""
    site = spec.site(site_name)
    if site is None:
        raise ValueError(f"Site not found: {site_name!r}")
    return site


def float_list(value: str) -> list[float]:
    return [float(item) for item in value.split()]


def format_floats(values: list[float]) -> str:
    return " ".join(f"{value:.12g}" for value in values)


PAIR_ATTRIBUTE_PARSERS = (
    ("condim", int),
    ("solref", float_list),
    ("solreffriction", float_list),
    ("solimp", float_list),
    ("margin", float),
    ("gap", float),
    ("friction", float_list),
)

SENSOR_TYPE_BY_TAG = {
    "framelinvel": mujoco.mjtSensor.mjSENS_FRAMELINVEL,
    "frameangvel": mujoco.mjtSensor.mjSENS_FRAMEANGVEL,
    "touch": mujoco.mjtSensor.mjSENS_TOUCH,
    "jointlimitfrc": mujoco.mjtSensor.mjSENS_JOINTLIMITFRC,
}
SENSOR_OBJTYPE_BY_NAME = {
    "body": mujoco.mjtObj.mjOBJ_BODY,
    "site": mujoco.mjtObj.mjOBJ_SITE,
    "joint": mujoco.mjtObj.mjOBJ_JOINT,
}
# Default (objtype, source-attribute) per tag when objtype/objname are omitted:
# touch references a <site>, jointlimitfrc references a <joint>.
_SENSOR_DEFAULT_OBJ = {"touch": ("site", "site"), "jointlimitfrc": ("joint", "joint")}


def _existing_pair_names(spec: object) -> set[str]:
    names = set()
    for pair in getattr(spec, "pairs", []):
        if isinstance(pair, dict):
            name = pair.get("name")
        else:
            name = getattr(pair, "name", None)
        if name:
            names.add(name)
    return names


def _contact_pair_name(pair: ET.Element, index: int, existing_names: set[str]) -> str:
    name = pair.get("name")
    if name:
        return name

    base_name = f"{pair.get('geom1')}_{pair.get('geom2')}"
    candidate = f"{base_name}_{index}"
    while candidate in existing_names:
        index += 1
        candidate = f"{base_name}_{index}"
    return candidate


def _contact_pair_kwargs(pair: ET.Element, index: int, existing_names: set[str]) -> dict:
    kwargs = {
        "name": _contact_pair_name(pair, index, existing_names),
        "geomname1": pair.get("geom1"),
        "geomname2": pair.get("geom2"),
    }
    for attr_name, parser in PAIR_ATTRIBUTE_PARSERS:
        value = pair.get(attr_name)
        if value:
            kwargs[attr_name] = parser(value)
    return kwargs


def add_contact_pairs(spec: object, contacts_xml: Path, include_pair: object = None) -> None:
    """Add contact pairs from an MJCF include file."""
    existing_names = _existing_pair_names(spec)
    for index, pair in enumerate(ET.parse(contacts_xml).getroot().iter("pair")):
        if include_pair is not None and not include_pair(pair):
            continue
        kwargs = _contact_pair_kwargs(pair, index, existing_names)
        spec.add_pair(**kwargs)
        existing_names.add(kwargs["name"])


def add_sensors(spec: object, sensors_xml: Path) -> None:
    """Add supported sensors from an MJCF include file."""
    for sensor in ET.parse(sensors_xml).getroot().iter():
        if sensor.tag not in SENSOR_TYPE_BY_TAG:
            continue
        default_objtype, default_attr = _SENSOR_DEFAULT_OBJ.get(sensor.tag, ("", None))
        objtype_name = sensor.get("objtype", default_objtype)
        objname = sensor.get("objname") or (sensor.get(default_attr) if default_attr else None)
        spec.add_sensor(
            name=sensor.get("name"),
            type=SENSOR_TYPE_BY_TAG[sensor.tag],
            objtype=SENSOR_OBJTYPE_BY_NAME[objtype_name],
            objname=objname,
        )


def add_keyframes(spec: object, keyframes_xml: Path) -> None:
    """Add keyframes from an MJCF include file.

    Kept as a spec-level step (not merged into the composed child XML) because a
    keyframe's ``qpos`` is only valid once the model's DoFs are fixed -- e.g. for
    a standalone base, after the free root joint has been added.
    """
    for key in ET.parse(keyframes_xml).getroot().iter("key"):
        added = spec.add_key()
        added.name = key.get("name")
        if key.get("qpos"):
            added.qpos = float_list(key.get("qpos"))
        if key.get("qvel"):
            added.qvel = float_list(key.get("qvel"))
        if key.get("time"):
            added.time = float(key.get("time"))


def expand_component_element(element: ET.Element, base_path: Path) -> list[ET.Element]:
    """Expand local MJCF include elements inside a component tree."""
    if element.tag == "include":
        include_file = element.get("file")
        if include_file is None:
            return []
        include_path = base_path / include_file
        if not include_path.exists():
            raise FileNotFoundError(f"MJCF include not found: {include_path} (resolved from {base_path} / {include_file!r})")
        return component_children(include_path)

    expanded = copy.deepcopy(element)
    for child in list(expanded):
        expanded.remove(child)
        for replacement in expand_component_element(child, base_path):
            expanded.append(replacement)
    return [expanded]


def component_children(xml_path: Path) -> list[ET.Element]:
    """Return component children, recursively expanding local MJCF includes."""
    children = []
    root = ET.parse(xml_path).getroot()
    for child in list(root):
        children.extend(expand_component_element(child, xml_path.parent))
    return children


def mirror_name(value: str, rules: MirrorRules) -> str:
    if value in rules.common_names:
        return value
    for old, new in rules.replacements:
        if old in value:
            return value.replace(old, new)
    for old, new in rules.prefix_replacements:
        if value.startswith(old):
            return f"{new}{value[len(old) :]}"
    if value == "MatSkin":
        return rules.mirrored_material
    if value.endswith("_r"):
        return f"{value[:-2]}_l"
    return f"{value}_l"


def should_mirror_class_name(value: str, rules: MirrorRules) -> bool:
    if value.endswith("_r"):
        return True
    if any(old in value for old, _ in rules.replacements):
        return True
    return any(value.startswith(old) for old, _ in rules.prefix_replacements)


def mirror_reference(element: ET.Element, attr_name: str, attr_value: str, rules: MirrorRules) -> str:
    mirrored = mirror_name(attr_value, rules)
    should_lowercase_geom = (attr_name == "name" and element.tag == "geom") or (attr_name == "geom")
    if should_lowercase_geom:
        for prefix in rules.lowercase_geom_prefixes:
            if mirrored.startswith(prefix):
                return f"{mirrored[0].lower()}{mirrored[1:]}"
    return mirrored


def mirror_xyz_attribute(element: ET.Element, attr_name: str, rules: MirrorRules) -> None:
    values = float_list(element.get(attr_name))
    if len(values) != 3:
        return
    element_name = element.get("name")
    if element.tag == "body" and element_name in rules.body_pos_x_mirror_names and attr_name == "pos":
        values[0] *= -1
    else:
        values[2] *= -1
    element.set(attr_name, format_floats(values))


def mirror_axial_attribute(element: ET.Element, attr_name: str) -> None:
    values = float_list(element.get(attr_name))
    if len(values) != 3:
        return
    values[0] *= -1
    values[1] *= -1
    element.set(attr_name, format_floats(values))


def mirror_quaternion(element: ET.Element) -> None:
    values = float_list(element.get("quat"))
    if len(values) != 4:
        return
    values[1] *= -1
    values[2] *= -1
    element.set("quat", format_floats(values))


def mirror_fromto(element: ET.Element) -> None:
    values = float_list(element.get("fromto"))
    if len(values) != 6:
        return
    values[2] *= -1
    values[5] *= -1
    element.set("fromto", format_floats(values))


def mirror_fullinertia(element: ET.Element) -> None:
    values = float_list(element.get("fullinertia"))
    if len(values) != 6:
        return
    # Ixx Iyy Izz Ixy Ixz Iyz; mirroring z flips the xz and yz products.
    values[4] *= -1
    values[5] *= -1
    element.set("fullinertia", format_floats(values))


def mirror_element(element: ET.Element, rules: MirrorRules) -> ET.Element:
    mirrored = copy.deepcopy(element)
    for child in list(mirrored):
        index = list(mirrored).index(child)
        mirrored.remove(child)
        mirrored.insert(index, mirror_element(child, rules))

    for attr_name, attr_value in list(mirrored.attrib.items()):
        if attr_name in NAME_REFERENCE_ATTRS:
            if attr_name == "name" and mirrored.tag == "texture":
                mirrored.set(attr_name, mirror_name(attr_value, rules))
                continue
            mirrored.set(
                attr_name,
                mirror_reference(mirrored, attr_name, attr_value, rules),
            )
        elif attr_name in {"class", "childclass"}:
            if should_mirror_class_name(attr_value, rules):
                mirrored.set(attr_name, mirror_name(attr_value, rules))
        elif attr_name in {"material", "texture"}:
            mirrored.set(attr_name, mirror_name(attr_value, rules))
        elif attr_name == "file" and rules.mirror_file_attributes:
            mirrored.set(attr_name, mirror_name(attr_value, rules))
        elif attr_name in {"pos", "scale", "ipos"}:
            mirror_xyz_attribute(mirrored, attr_name, rules)
        elif attr_name == "axis":
            mirror_axial_attribute(mirrored, attr_name)
        elif attr_name == "euler":
            if mirrored.tag != "body":
                mirror_axial_attribute(mirrored, attr_name)
        elif attr_name == "quat":
            mirror_quaternion(mirrored)
        elif attr_name == "fromto":
            mirror_fromto(mirrored)
        elif attr_name == "fullinertia":
            mirror_fullinertia(mirrored)

    return mirrored


def build_mirrored_child_xml(
    *,
    model_name: str,
    compiler_meshdir: Path,
    source_assets_xml: Path,
    source_tendons_xml: Path | None,
    source_muscles_xml: Path | None,
    source_chain_xml: Path,
    root_body_name: str,
    root_site_name: str,
    rules: MirrorRules | None = None,
) -> str:
    """Build a mirrored MJCF child XML from right-side arm component files."""
    if rules is None:
        rules = MirrorRules()
    root = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(
        root,
        "compiler",
        angle="radian",
        meshdir=str(compiler_meshdir),
        texturedir=str(compiler_meshdir),
    )

    for child in component_children(source_assets_xml):
        if child.tag in {"compiler", "size", "option"}:
            continue
        mirrored = mirror_element(child, rules)
        if mirrored.tag == "asset":
            for asset_child in list(mirrored):
                if asset_child.get("name") in rules.common_names:
                    mirrored.remove(asset_child)
        root.append(mirrored)

    for xml_path in (source_tendons_xml, source_muscles_xml):
        if xml_path is None:
            continue
        for child in component_children(xml_path):
            root.append(mirror_element(child, rules))

    worldbody = ET.SubElement(root, "worldbody")
    child_root = ET.SubElement(worldbody, "body", name=root_body_name)
    ET.SubElement(child_root, "site", name=root_site_name, size="0.01")
    for child in component_children(source_chain_xml):
        child_root.append(mirror_element(child, rules))

    return ET.tostring(root, encoding="unicode")


def build_child_xml_from_components(
    *,
    model_name: str,
    compiler_meshdir: Path,
    assets_xml: Path,
    tendons_xml: Path | None,
    muscles_xml: Path | None,
    chain_xml: Path,
    root_body_name: str,
    root_site_name: str,
    extra_assets_xmls: tuple[Path, ...] = (),
    scene_xmls: tuple[Path, ...] = (),
) -> str:
    """Build a standalone child XML from asset/tendon/muscle/chain includes."""
    root = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(
        root,
        "compiler",
        angle="radian",
        meshdir=str(compiler_meshdir),
        texturedir=str(compiler_meshdir),
    )

    for asset_xml in (assets_xml, *extra_assets_xmls):
        for child in component_children(asset_xml):
            if child.tag in {"compiler", "size", "option"}:
                continue
            root.append(child)

    for xml_path in (tendons_xml, muscles_xml):
        if xml_path is None:
            continue
        for child in component_children(xml_path):
            root.append(child)

    scene_worldbody_children = []
    for scene_xml in scene_xmls:
        for child in component_children(scene_xml):
            if child.tag in {"compiler", "size", "option"}:
                continue
            if child.tag == "worldbody":
                scene_worldbody_children.extend(list(child))
                continue
            root.append(child)

    worldbody = ET.SubElement(root, "worldbody")
    for child in scene_worldbody_children:
        worldbody.append(child)
    child_root = ET.SubElement(worldbody, "body", name=root_body_name)
    ET.SubElement(child_root, "site", name=root_site_name, size="0.01")
    for child in component_children(chain_xml):
        child_root.append(child)

    return ET.tostring(root, encoding="unicode")
