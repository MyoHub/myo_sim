import pytest
import myo_sim


def test_registry_exposed():
    assert hasattr(myo_sim, "FragmentRegistry")
    assert hasattr(myo_sim, "FragmentInfo")


def test_legacy_names_resolve():
    for name in ("elbow", "hand", "finger", "arm", "leg", "body", "torso"):
        info = myo_sim.FragmentRegistry.get(name)
        assert info.path.exists(), f"{name}: {info.path}"
        assert info.version >= 1


def test_myofullbody_matches_musclemimic():
    import mujoco

    info = myo_sim.FragmentRegistry.get("myofullbody")
    assert info.path.exists()
    m = mujoco.MjModel.from_xml_path(str(info.path))
    assert m.njnt == 123, f"Expected 123 joints, got {m.njnt}"
    assert m.nu == 416, f"Expected 416 actuators, got {m.nu}"


def test_unknown_raises_key_error():
    with pytest.raises(KeyError):
        myo_sim.FragmentRegistry.get("does_not_exist")


def test_all_names_sorted():
    names = myo_sim.FragmentRegistry.all_names()
    assert names == sorted(names)
    assert "myofullbody" in names
    assert "elbow" in names


def test_fragment_info_attributes():
    info = myo_sim.FragmentRegistry.get("myofullbody")
    assert isinstance(info, myo_sim.FragmentInfo)
    assert isinstance(info.path, __import__("pathlib").Path)
    assert isinstance(info.version, int)
    assert info.name == "myofullbody"


def test_contains():
    assert "myofullbody" in myo_sim.FragmentRegistry
    assert "does_not_exist" not in myo_sim.FragmentRegistry
