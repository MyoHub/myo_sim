import re
from pathlib import Path

import myo_sim
from myo_sim.build.utils import add_contact_pairs

ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = myo_sim.MODELS_DIR


def test_build_contact_sources_are_centralized():
    source = (ROOT / "myo_sim" / "build" / "compose.py").read_text()

    assert 'ROOT / "contacts" / "myoarm_contacts.xml"' in source
    assert 'ROOT / "contacts" / "myohand_contacts.xml"' in source
    assert 'ROOT / "contacts" / "myolegs_contacts.xml"' in source
    assert 'ROOT / "contacts" / "myofullbody_contacts.xml"' in source
    assert 'assets" / "myoarm_contacts.xml"' not in source
    assert 'assets" / "myohand_contacts.xml"' not in source
    assert 'assets" / "myolegs_contacts.xml"' not in source
    assert 'assets" / "myofullbody_contacts.xml"' not in source


def test_add_contact_pairs_generates_name_for_unnamed_pairs(tmp_path):
    contacts_xml = tmp_path / "contacts.xml"
    contacts_xml.write_text('<mujoco><pair geom1="pelvis" geom2="floor" /></mujoco>\n')

    class FakeSpec:
        def __init__(self):
            self.pairs = []

        def add_pair(self, **kwargs):
            if not isinstance(kwargs["name"], str):
                raise ValueError("name should be a string")
            self.pairs.append(kwargs)

    spec = FakeSpec()

    add_contact_pairs(spec, contacts_xml)

    assert spec.pairs == [{"name": "pelvis_floor_0", "geomname1": "pelvis", "geomname2": "floor"}]


def test_add_contact_pairs_generates_unique_names_across_calls(tmp_path):
    contacts_xml = tmp_path / "contacts.xml"
    contacts_xml.write_text('<mujoco><pair geom1="pelvis" geom2="floor" /></mujoco>\n')

    class FakeSpec:
        def __init__(self):
            self.pairs = []

        def add_pair(self, **kwargs):
            if any(pair["name"] == kwargs["name"] for pair in self.pairs):
                raise ValueError(f"repeated name {kwargs['name']!r} in pair")
            self.pairs.append(kwargs)

    spec = FakeSpec()

    add_contact_pairs(spec, contacts_xml)
    add_contact_pairs(spec, contacts_xml)

    assert [pair["name"] for pair in spec.pairs] == ["pelvis_floor_0", "pelvis_floor_1"]


def test_model_xml_paths_do_not_reference_old_repo_layout():
    stale_prefixes = ("../myo_sim/", "../../myo_sim/")

    for model_xml in MODELS_DIR.rglob("*.xml"):
        source = model_xml.read_text()

        for prefix in stale_prefixes:
            assert prefix not in source, f"{model_xml} contains stale path prefix {prefix!r}"


def test_model_xml_file_references_exist():
    """Every file="*.xml" include must resolve from the including file's directory.

    Matches MuJoCo native resolution and expand_component_element() in build/utils.py.
    Wrong relative paths must not rely on a parent-directory search fallback.
    """
    xml_file_reference = re.compile(r"""file=(["'])([^"']+\.xml)\1""")

    missing = []
    for model_xml in MODELS_DIR.rglob("*.xml"):
        source = model_xml.read_text()
        for match in xml_file_reference.finditer(source):
            rel_path = match.group(2)
            if not (model_xml.parent / rel_path).exists():
                missing.append(f"{model_xml.relative_to(MODELS_DIR)} -> {rel_path}")

    assert missing == []


def test_no_static_xml_combines_conflicting_asset_files():
    """Enforce policy: no static XML may combine myotorso_assets.xml and myoarm_r_assets.xml.

    Before the rename in this PR, both files declared class="wrap" and class="marker",
    causing MuJoCo 3.8 to raise "repeated default class name". Those names are now
    scoped (myotorso_wrap, myoarm_wrap, etc.) so the pair can in principle be composed,
    but multi-part assemblies must still go through build_model() in compose.py to keep
    contact pairs and bilateral symmetry centralised.
    """
    conflicting_pairs = [
        ("myotorso_assets.xml", "myoarm_r_assets.xml"),
    ]

    for model_xml in MODELS_DIR.rglob("*.xml"):
        if "assets" in model_xml.parts:
            continue
        source = model_xml.read_text()
        for a, b in conflicting_pairs:
            includes_a = a in source
            includes_b = b in source
            assert not (includes_a and includes_b), (
                f"{model_xml} includes both {a} and {b}. "
                f"Multi-part assemblies must use build_model() in compose.py "
                f"to keep contact pairs and bilateral symmetry centralised."
            )
