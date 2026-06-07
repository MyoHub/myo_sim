"""Utilities for prototype MuJoCo MjSpec model composition."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import copy
import xml.etree.ElementTree as ET


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


def find_body(spec, body_name: str):
    """Find a body across MuJoCo Python API variants."""
    if hasattr(spec, "find_body"):
        body = spec.find_body(body_name)
    elif hasattr(spec, "find"):
        body = spec.find("body", body_name)
    else:
        try:
            body = spec.body(body_name)
        except KeyError:
            body = None

    if body is None:
        raise ValueError(f"Body not found: {body_name!r}")
    return body


def find_site(spec, site_name: str):
    """Find a site across MuJoCo Python API variants."""
    if hasattr(spec, "find_site"):
        site = spec.find_site(site_name)
    elif hasattr(spec, "find"):
        site = spec.find("site", site_name)
    else:
        try:
            site = spec.site(site_name)
        except KeyError:
            site = None

    if site is None:
        raise ValueError(f"Site not found: {site_name!r}")
    return site


def attach_to_site(parent_spec, child_spec, parent_site):
    """Attach child spec at a parent attachment site."""
    parent_spec.attach(child_spec, prefix="", suffix="", site=parent_site)


def attach_to_frame(parent_spec, child_spec, parent_frame):
    """Attach child spec at a parent frame."""
    parent_spec.attach(child_spec, prefix="", suffix="", frame=parent_frame)


def float_list(value: str):
    return [float(item) for item in value.split()]


def format_floats(values):
    return " ".join(f"{value:.12g}" for value in values)


def rename_material(spec, old_name: str, new_name: str):
    """Avoid shared material-name collisions across attached specs."""
    try:
        material = spec.material(old_name)
    except KeyError:
        return
    if material is None:
        return
    material.name = new_name


def add_contact_pairs(spec, contacts_xml: Path, include_pair=None):
    """Add contact pairs from an MJCF include file."""
    for pair in ET.parse(contacts_xml).getroot().iter("pair"):
        if include_pair is not None and not include_pair(pair):
            continue
        spec.add_pair(
            name=pair.get("name"),
            geomname1=pair.get("geom1"),
            geomname2=pair.get("geom2"),
            condim=int(pair.get("condim")) if pair.get("condim") else None,
            solref=float_list(pair.get("solref")) if pair.get("solref") else None,
            solreffriction=(float_list(pair.get("solreffriction")) if pair.get("solreffriction") else None),
            solimp=float_list(pair.get("solimp")) if pair.get("solimp") else None,
            margin=float(pair.get("margin")) if pair.get("margin") else None,
            gap=float(pair.get("gap")) if pair.get("gap") else None,
            friction=float_list(pair.get("friction")) if pair.get("friction") else None,
        )


def mirror_name(value: str, rules: MirrorRules):
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


def should_mirror_class_name(value: str, rules: MirrorRules):
    if value.endswith("_r"):
        return True
    if any(old in value for old, _ in rules.replacements):
        return True
    return any(value.startswith(old) for old, _ in rules.prefix_replacements)


def mirror_reference(element, attr_name: str, attr_value: str, rules: MirrorRules):
    mirrored = mirror_name(attr_value, rules)
    should_lowercase_geom = (attr_name == "name" and element.tag == "geom") or (attr_name == "geom")
    if should_lowercase_geom:
        for prefix in rules.lowercase_geom_prefixes:
            if mirrored.startswith(prefix):
                return f"{mirrored[0].lower()}{mirrored[1:]}"
    return mirrored


def mirror_xyz_attribute(element, attr_name: str, rules: MirrorRules):
    values = float_list(element.get(attr_name))
    if len(values) != 3:
        return
    element_name = element.get("name")
    if element.tag == "body" and element_name in rules.body_pos_x_mirror_names and attr_name == "pos":
        values[0] *= -1
    else:
        values[2] *= -1
    element.set(attr_name, format_floats(values))


def mirror_axial_attribute(element, attr_name: str):
    values = float_list(element.get(attr_name))
    if len(values) != 3:
        return
    values[0] *= -1
    values[1] *= -1
    element.set(attr_name, format_floats(values))


def mirror_quaternion(element):
    values = float_list(element.get("quat"))
    if len(values) != 4:
        return
    values[1] *= -1
    values[2] *= -1
    element.set("quat", format_floats(values))


def mirror_fromto(element):
    values = float_list(element.get("fromto"))
    if len(values) != 6:
        return
    values[2] *= -1
    values[5] *= -1
    element.set("fromto", format_floats(values))


def mirror_fullinertia(element):
    values = float_list(element.get("fullinertia"))
    if len(values) != 6:
        return
    # Ixx Iyy Izz Ixy Ixz Iyz; mirroring z flips the xz and yz products.
    values[4] *= -1
    values[5] *= -1
    element.set("fullinertia", format_floats(values))


def mirror_element(element, rules: MirrorRules):
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
    rules=None,
):
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

    source_assets = ET.parse(source_assets_xml).getroot()
    for child in list(source_assets):
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
        for child in list(ET.parse(xml_path).getroot()):
            root.append(mirror_element(child, rules))

    worldbody = ET.SubElement(root, "worldbody")
    child_root = ET.SubElement(worldbody, "body", name=root_body_name)
    ET.SubElement(child_root, "site", name=root_site_name, size="0.01")
    for child in list(ET.parse(source_chain_xml).getroot()):
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
):
    """Build a standalone child XML from asset/tendon/muscle/chain includes."""
    root = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(
        root,
        "compiler",
        angle="radian",
        meshdir=str(compiler_meshdir),
        texturedir=str(compiler_meshdir),
    )

    source_assets = ET.parse(assets_xml).getroot()
    for child in list(source_assets):
        if child.tag in {"compiler", "size", "option"}:
            continue
        root.append(copy.deepcopy(child))

    for xml_path in (tendons_xml, muscles_xml):
        if xml_path is None:
            continue
        for child in list(ET.parse(xml_path).getroot()):
            root.append(copy.deepcopy(child))

    worldbody = ET.SubElement(root, "worldbody")
    child_root = ET.SubElement(worldbody, "body", name=root_body_name)
    ET.SubElement(child_root, "site", name=root_site_name, size="0.01")
    for child in list(ET.parse(chain_xml).getroot()):
        child_root.append(copy.deepcopy(child))

    return ET.tostring(root, encoding="unicode")
