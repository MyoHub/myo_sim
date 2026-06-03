import pytest
import myo_sim


def test_registry_exposed():
    assert hasattr(myo_sim, "FragmentRegistry")
    assert hasattr(myo_sim, "FragmentInfo")


def test_registered_names_resolve():
    for name in ("arm", "leg", "myoarm_r", "myolegs", "torso"):
        info = myo_sim.FragmentRegistry.get(name)
        assert info.path.exists(), f"{name}: {info.path}"
        assert info.version >= 1


def test_registered_models_load_from_registry():
    import mujoco

    for name in myo_sim.FragmentRegistry.all_names():
        info = myo_sim.FragmentRegistry.get(name)
        m = mujoco.MjModel.from_xml_path(str(info.path))
        assert m.njnt > 0, name


def test_unknown_raises_key_error():
    with pytest.raises(KeyError):
        myo_sim.FragmentRegistry.get("does_not_exist")


def test_all_names_sorted():
    names = myo_sim.FragmentRegistry.all_names()
    assert names == sorted(names)
    assert "myoarm_r" in names
    assert "leg" in names


def test_fragment_info_attributes():
    info = myo_sim.FragmentRegistry.get("myoarm_r")
    assert isinstance(info, myo_sim.FragmentInfo)
    assert isinstance(info.path, __import__("pathlib").Path)
    assert isinstance(info.version, int)
    assert info.name == "myoarm_r"


def test_contains():
    assert "myoarm_r" in myo_sim.FragmentRegistry
    assert "does_not_exist" not in myo_sim.FragmentRegistry
