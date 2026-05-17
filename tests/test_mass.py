"""Mass validation checks for the local myo_sim repository.

Run from the repository root with:

    python tests/debug_test_mass.py

This intentionally avoids pytest so it can run in a plain Python environment.
"""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_ROOT = REPO_ROOT


FULL_BODY_XML = MODEL_ROOT / "body" / "myofullbody.xml"
COMPONENT_XMLS = {
    "arm_r": MODEL_ROOT / "arm" / "assets" / "myoarmR_body.xml",
    "arm_l": MODEL_ROOT / "arm" / "assets" / "myoarmL_body.xml",
    "leg": MODEL_ROOT / "leg" / "assets" / "myolegs_chain.xml",
    "trunk": MODEL_ROOT / "torso" / "assets" / "myotorso_arm_chain.xml",
    "head": MODEL_ROOT / "head" / "assets" / "myohead_rigid_chain.xml",
}
COLLISION_ASSET_XMLS = [
    MODEL_ROOT / "arm" / "assets" / "myoarmR_assets.xml",
    MODEL_ROOT / "arm" / "assets" / "myoarmL_assets.xml",
    MODEL_ROOT / "leg" / "assets" / "myolegs_assets.xml",
    MODEL_ROOT / "torso" / "assets" / "myotorso_assets.xml",
]


def get_body_names_from_xml(xml_path):
    """Return body names defined in one XML file without recursing includes."""
    root = ET.parse(str(xml_path)).getroot()
    return [elem.get("name") for elem in root.iter("body") if elem.get("name")]


def get_total_mass_for_body_list(model, body_names):
    """Calculate total mass for a list of body names in the compiled model."""
    total = 0.0
    for name in body_names:
        body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise AssertionError(f"Body {name!r} not found in compiled model")
        total += model.body_mass[body_id]
    return total


def compute_mass_by_xml(full_body_xml=FULL_BODY_XML, subfiles=COMPONENT_XMLS):
    """Compile the full model and compute mass for each component XML."""
    model = mj.MjModel.from_xml_path(str(full_body_xml))
    results = {}
    for label, xml_path in subfiles.items():
        body_names = get_body_names_from_xml(xml_path)
        results[label] = get_total_mass_for_body_list(model, body_names)
    return results, model


class TestModelMass(unittest.TestCase):
    """Mass validation checks for the full-body model."""

    def test_all_model_files_exist(self):
        self.assertTrue(FULL_BODY_XML.exists(), f"Missing {FULL_BODY_XML}")
        for component, path in COMPONENT_XMLS.items():
            self.assertTrue(path.exists(), f"Missing {component} XML: {path}")
        for path in COLLISION_ASSET_XMLS:
            self.assertTrue(path.exists(), f"Missing collision asset XML: {path}")

    def test_component_masses_within_expected_ranges(self):
        """Check component masses against broad current-repo ranges."""
        results, model = compute_mass_by_xml()
        total_mass = model.body_mass.sum()

        print("\nMass summary:")
        for component, mass in results.items():
            print(f"  {component}: {mass:.3f} kg")
        print(f"  total: {total_mass:.3f} kg")

        expected_ranges = {
            "arm_r": (4.0, 8.0),
            "arm_l": (4.0, 8.0),
            "leg": (25.0, 46.0),
            "trunk": (29.0, 39.0),
            "head": (2.0, 5.0),
        }

        for component, (min_mass, max_mass) in expected_ranges.items():
            actual_mass = results[component]
            self.assertGreaterEqual(
                actual_mass,
                min_mass,
                f"{component} mass {actual_mass:.3f} kg is below {min_mass} kg",
            )
            self.assertLessEqual(
                actual_mass,
                max_mass,
                f"{component} mass {actual_mass:.3f} kg is above {max_mass} kg",
            )

        self.assertGreater(total_mass, 0.0)

    def test_collision_defaults_have_zero_mass(self):
        """Collision defaults should set mass=0 so geoms do not add body mass."""
        for xml_file in COLLISION_ASSET_XMLS:
            root = ET.parse(str(xml_file)).getroot()
            for default_elem in root.iter("default"):
                class_name = default_elem.get("class", "")
                if "coll" not in class_name.lower():
                    continue

                for geom_elem in default_elem.iter("geom"):
                    mass_attr = geom_elem.get("mass")
                    self.assertIsNotNone(
                        mass_attr,
                        f"Collision geom default {class_name!r} in {xml_file} "
                        'is missing mass="0".',
                    )
                    self.assertEqual(
                        mass_attr,
                        "0",
                        f"Collision geom default {class_name!r} in {xml_file} "
                        f"has mass={mass_attr!r}; expected '0'.",
                    )


if __name__ == "__main__":
    unittest.main(verbosity=2)
