from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_mjspec_contact_sources_are_centralized():
    source = (ROOT / "mjspec" / "prototype_mjspec_attach.py").read_text()

    assert 'ROOT / "contacts" / "myoarm_contacts.xml"' in source
    assert 'ROOT / "contacts" / "myolegs_contacts.xml"' in source
    assert 'ROOT / "contacts" / "myofullbody_contacts.xml"' in source
    assert 'assets" / "myoarm_contacts.xml"' not in source
    assert 'assets" / "myolegs_contacts.xml"' not in source
    assert 'assets" / "myofullbody_contacts.xml"' not in source


def test_standalone_xml_models_do_not_include_contact_pairs():
    for model_xml in (ROOT / "leg" / "myolegs.xml", ROOT / "leg" / "myolegs_abdomen.xml"):
        source = model_xml.read_text()

        assert "contacts.xml" not in source
