"""Build the midline cervical chain and mirror its right-side muscle paths."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import mujoco

from myo_sim import MODELS_DIR
from myo_sim.build.utils import MirrorRules, build_child_xml_from_components, mirror_element


def build_head_spec(*, standalone: bool = False) -> mujoco.MjSpec:
    """Return the HYOID neck in the existing torso's Y-up attachment frame."""
    assets = MODELS_DIR / "head" / "assets"
    xml = build_child_xml_from_components(
        model_name="myohead",
        compiler_meshdir=MODELS_DIR,
        assets_xml=assets / "myohead_assets.xml",
        tendons_xml=assets / "myohead_r_tendon.xml",
        muscles_xml=assets / "myohead_r_muscle.xml",
        chain_xml=assets / "myohead_chain.xml",
        root_body_name="neck",
        root_site_name="myohead_root_attach",
        extra_assets_xmls=(assets / "myohead_simple_assets.xml",),
        scene_xmls=(MODELS_DIR / "scene" / "myosuite_scene_musclemimic.xml",) if standalone else (),
    )
    root = ET.fromstring(xml)
    # MjSpec shares mesh names across attachments; scope the reused HAT assets.
    asset_elements = [element for asset in root.findall("asset") for element in asset]
    shared_names = {element.get("name") for element in asset_elements}
    for element in root.iter():
        for attribute in ("mesh", "material", "texture"):
            value = element.get(attribute)
            if value in shared_names:
                element.set(attribute, f"myohead_neck_{value}")
    for element in asset_elements:
        element.set("name", f"myohead_neck_{element.get('name')}")
    rules = MirrorRules()
    # The skeleton is shared. Only sites, tendons and actuators are bilateral.
    for body in root.iter("body"):
        for site in list(body.findall("site")):
            if site.get("name", "").endswith("_r"):
                body.append(mirror_element(site, rules))
    for section in (root.find("tendon"), root.find("actuator")):
        for element in list(section):
            section.append(mirror_element(element, rules))
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    # Preserve the full-body head observation / collision names.
    spec.body("myohead_skull").name = "head"
    if standalone:
        spec.body("neck").quat = (2**-0.5, 2**-0.5, 0, 0)
        spec.body("neck").pos = (0, 0, 1)
    return spec
