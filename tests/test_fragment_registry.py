import pytest
import myo_sim


def test_registry_exposed():
    assert hasattr(myo_sim, "FragmentRegistry")
    assert hasattr(myo_sim, "FragmentInfo")


def test_static_fragment_registry_contains_legacy_models():
    assert myo_sim.FragmentRegistry.all_names() == [
        "elbow",
        "finger",
        "myoelbow",
        "myoelbow_2dof",
        "myoelbow_exo",
        "myofinger",
        "myolegs_osl",
        "osl",
    ]


def test_unknown_raises_key_error():
    with pytest.raises(KeyError):
        myo_sim.FragmentRegistry.get("does_not_exist")


@pytest.mark.parametrize("name", ["leg", "torso"])
def test_removed_static_aliases_do_not_load(name):
    with pytest.raises(ValueError):
        myo_sim.load(name)


@pytest.mark.parametrize("name", myo_sim.FragmentRegistry.all_names())
def test_static_fragment_registry_paths_exist(name):
    assert myo_sim.FragmentRegistry.get(name).path.exists()


@pytest.mark.parametrize("name", myo_sim.FragmentRegistry.all_names())
def test_static_fragment_registry_models_load(name):
    import mujoco

    model, data = myo_sim.load(name)
    assert model.njnt > 0, name
    mujoco.mj_forward(model, data)


@pytest.mark.parametrize("name", sorted(myo_sim._COMPOSED_MODELS))
def test_composed_models_load_and_forward(name):
    import mujoco

    model, data = myo_sim.load(name)
    assert model.njnt > 0, name
    mujoco.mj_forward(model, data)


def test_myotorso_loads_from_composed_registry():
    model, _ = myo_sim.load("myotorso")

    assert model.njnt > 0
    assert model.nu > 0


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
