"""The mujoco-warp version warning: when it fires, and how often."""

import warnings
from importlib.metadata import PackageNotFoundError

import pytest

from myo_sim.build import warp_version


@pytest.fixture(autouse=True)
def _fresh_state(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(warp_version, "_warned", False)


def _install(monkeypatch: pytest.MonkeyPatch, installed: str | None) -> None:
    def fake_version(_: str) -> str:
        if installed is None:
            raise PackageNotFoundError
        return installed

    monkeypatch.setattr(warp_version, "version", fake_version)


@pytest.mark.parametrize("name", ["myoarm_r", "myohands", "myofullbody"])
def test_warns_on_old_version_for_models_with_the_tendons(monkeypatch, name):
    _install(monkeypatch, "3.15.0")
    with pytest.warns(UserWarning, match="mujoco_warp#1713"):
        warp_version.warn_if_old_mujoco_warp(name)


def test_warns_once_per_process(monkeypatch):
    _install(monkeypatch, "3.15.0")
    with pytest.warns(UserWarning):
        warp_version.warn_if_old_mujoco_warp("myoarm_r")
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        warp_version.warn_if_old_mujoco_warp("myoarm_r")


@pytest.mark.parametrize(
    ("installed", "name"),
    [
        ("3.16.0", "myoarm_r"),
        ("3.17.1", "myofullbody"),
        ("4.0.0", "myofullbody"),
        (None, "myoarm_r"),
        ("dev", "myoarm_r"),
        ("3.15.0", "myolegs"),
        ("3.15.0", "myotorso"),
    ],
)
def test_silent_when_not_affected(monkeypatch, installed, name):
    _install(monkeypatch, installed)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        warp_version.warn_if_old_mujoco_warp(name)


def test_load_spec_runs_the_check(monkeypatch):
    import myo_sim

    _install(monkeypatch, "3.15.0")
    with pytest.warns(UserWarning, match="mujoco_warp#1713"):
        myo_sim.load_spec("myoarm_r")
