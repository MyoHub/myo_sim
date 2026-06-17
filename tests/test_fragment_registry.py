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


def test_composed_hand_models_can_forward():
    """Hand-only builds must not retain static-static arm contact pairs."""
    import mujoco
    import myo_sim

    for name in ("myohand_r", "myohands"):
        model, data = myo_sim.load(name)
        assert model.npair > 0, f"{name}: no explicit contact pairs"
        mujoco.mj_forward(model, data)


def test_composed_hand_models_default_to_raised_front_pose():
    import mujoco
    import myo_sim

    expected_body_names = {
        "myohand_r": ("capitate_r",),
        "myohands": ("capitate_r", "capitate_l"),
    }
    for name, body_names in expected_body_names.items():
        model, data = myo_sim.load(name)
        mujoco.mj_forward(model, data)
        torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        torso_y = data.xpos[torso_id, 1]
        torso_z = data.xpos[torso_id, 2]
        for body_name in body_names:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            assert data.xpos[body_id, 1] < torso_y - 0.25
            assert data.xpos[body_id, 2] > torso_z + 0.35
