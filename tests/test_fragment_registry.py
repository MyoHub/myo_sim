import pytest
import myo_sim


def test_registry_exposed():
    assert hasattr(myo_sim, "FragmentRegistry")
    assert hasattr(myo_sim, "FragmentInfo")


def test_registered_names_resolve():
    for name in ("leg", "myolegs", "torso"):
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
    assert "leg" in names


def test_fragment_info_attributes():
    info = myo_sim.FragmentRegistry.get("myolegs")
    assert isinstance(info, myo_sim.FragmentInfo)
    assert isinstance(info.path, __import__("pathlib").Path)
    assert isinstance(info.version, int)
    assert info.name == "myolegs"


def test_myohand_r_matches_main_spec():
    """Built myohand_r must match the static myohand from main: njnt=23, nu=39."""
    import myo_sim

    model, _ = myo_sim.load("myohand_r")
    assert model.njnt == 23, f"Expected 23 joints, got {model.njnt}"
    assert model.nu == 39, f"Expected 39 actuators, got {model.nu}"


def test_load_composed_models():
    """myo_sim.load() resolves MjSpec-composed model names without a static XML."""
    import myo_sim

    for name in ("myohand_r", "myohands"):
        model, data = myo_sim.load(name)
        assert model.njnt > 0, f"{name}: no joints"
        assert model.nu > 0, f"{name}: no actuators"
