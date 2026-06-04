from pathlib import Path

import myo_sim


ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = myo_sim.MODELS_DIR


def test_build_contact_sources_are_centralized():
    source = (ROOT / "myo_sim" / "build" / "compose.py").read_text()

    assert 'ROOT / "contacts" / "myoarm_contacts.xml"' in source
    assert 'ROOT / "contacts" / "myolegs_contacts.xml"' in source
    assert 'ROOT / "contacts" / "myofullbody_contacts.xml"' in source
    assert 'assets" / "myoarm_contacts.xml"' not in source
    assert 'assets" / "myolegs_contacts.xml"' not in source
    assert 'assets" / "myofullbody_contacts.xml"' not in source


def test_standalone_xml_models_do_not_include_contact_pairs():
    for model_xml in (MODELS_DIR / "leg" / "myolegs.xml", MODELS_DIR / "leg" / "myolegs_abdomen.xml"):
        source = model_xml.read_text()

        assert "contacts.xml" not in source


def test_model_xml_paths_do_not_reference_old_repo_layout():
    stale_prefixes = ("../myo_sim/", "../../myo_sim/")

    for model_xml in MODELS_DIR.rglob("*.xml"):
        source = model_xml.read_text()

        for prefix in stale_prefixes:
            assert prefix not in source, f"{model_xml} contains stale path prefix {prefix!r}"
