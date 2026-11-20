# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
#
# Adapted from MyoSuite PR #564 by Mark Colley (M-Colley), Apache-2.0
# (https://github.com/MyoHub/myosuite/pull/564).
"""``scale_spec`` on MyoLeg against OpenSim's ModelScaler on Rajagopal2016 (stored reference).

``reference.json`` comes from ``scripts/validate_opensim_scaling.py``; OpenSim is not needed here.
Values are compared as changes from each program's nominal model, so a nominal difference between
the two models does not count. See ``docs/wiki/model-scaling.md``.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim import scale_spec
from myo_sim.scaling import _scaled_inertia, assemble_qpos

DATA = Path(__file__).parent / "data" / "opensim_scaling"
REFERENCE = json.loads((DATA / "reference.json").read_text())
FIXTURE = json.loads((DATA / "fixture.json").read_text())["cases"]
# Bounds per case. The position bound is the documented convention of a child's joint offset.
BOUNDS = {
    "uniform": {"position_m": 5e-8, "muscle_ratio": 1e-5, "mass_ratio": 1e-12, "inertia_ratio": 1e-12},
    "anisotropic": {"position_m": 3.5e-4, "muscle_ratio": 6e-3, "mass_ratio": 1e-12, "inertia_ratio": None},
    "long_femur_short_tibia": {"position_m": 1.5e-3, "muscle_ratio": 5e-3, "mass_ratio": 1e-12, "inertia_ratio": 1e-12},
}


def _scale_set(path: Path, factors: dict[str, list[float]]) -> Path:
    rows = "".join(
        f"<Scale name='{name}'><scales>{' '.join(repr(float(v)) for v in values)}</scales><segment>{name}</segment><apply>true</apply></Scale>"
        for name, values in factors.items()
    )
    path.write_text(
        f"<OpenSimDocument Version='40000'><ScaleSet name='fixture'><objects>{rows}</objects></ScaleSet></OpenSimDocument>"
    )
    return path


def _positions(model: mujoco.MjModel) -> dict[str, np.ndarray]:
    """Joint centres in the pelvis frame, in the assembled default pose."""
    data = mujoco.MjData(model)
    assemble_qpos(model, data)
    pelvis = model.body("pelvis").id
    return {
        name: (data.xpos[model.body(name).id] - data.xpos[pelvis]) @ data.xmat[pelvis].reshape(3, 3)
        for name in REFERENCE["cases"]["nominal"]["positions_m"]
    }


def _case(case: str, tmp_path: Path) -> tuple[mujoco.MjModel, mujoco.MjModel]:
    spec = myo_sim.load_spec("myolegs")
    return spec.copy().compile(), scale_spec(spec, _scale_set(tmp_path / "scale.xml", FIXTURE[case])).compile()


def test_the_nominal_models_share_their_joint_centres() -> None:
    """The nominal comparison the increments below rest on (micrometres)."""
    model = myo_sim.load_spec("myolegs").compile()
    nominal = REFERENCE["cases"]["nominal"]["positions_m"]
    for name, position in _positions(model).items():
        np.testing.assert_allclose(position, nominal[name], atol=1e-6, err_msg=name)


@pytest.mark.parametrize("case", list(BOUNDS))
def test_joint_centres_follow_opensim(case: str, tmp_path: Path) -> None:
    nominal, scaled = _case(case, tmp_path)
    before, after = _positions(nominal), _positions(scaled)
    osim_before, osim_after = REFERENCE["cases"]["nominal"]["positions_m"], REFERENCE["cases"][case]["positions_m"]
    worst = max(np.abs((after[n] - before[n]) - (np.array(osim_after[n]) - np.array(osim_before[n]))).max() for n in before)
    assert worst <= BOUNDS[case]["position_m"]


@pytest.mark.parametrize("case", list(BOUNDS))
def test_muscle_lengths_follow_opensim(case: str, tmp_path: Path) -> None:
    """Optimal fibre and tendon slack length of all 80 muscles scale as OpenSim scales them."""
    nominal, scaled = _case(case, tmp_path)
    osim_before, osim_after = REFERENCE["cases"]["nominal"]["muscles"], REFERENCE["cases"][case]["muscles"]
    assert len(osim_after) == nominal.nu == 80
    for name, values in osim_after.items():
        i = nominal.actuator(name).id
        ratio = scaled.actuator_lengthrange[i] / nominal.actuator_lengthrange[i]
        for key in ("optimal_fibre_length_m", "tendon_slack_length_m"):
            np.testing.assert_allclose(
                ratio, values[key] / osim_before[name][key], atol=BOUNDS[case]["muscle_ratio"], err_msg=f"{name} {key}"
            )


@pytest.mark.parametrize("case", list(BOUNDS))
def test_mass_and_inertia_follow_opensim(case: str, tmp_path: Path) -> None:
    nominal, scaled = _case(case, tmp_path)
    osim_before, osim_after = REFERENCE["cases"]["nominal"], REFERENCE["cases"][case]
    bound = BOUNDS[case]
    for name in osim_after["mass_kg"]:
        i = nominal.body(name).id
        np.testing.assert_allclose(
            scaled.body_mass[i] / nominal.body_mass[i],
            osim_after["mass_kg"][name] / osim_before["mass_kg"][name],
            atol=bound["mass_ratio"],
        )
        if bound["inertia_ratio"] is not None:
            ratio = np.array(osim_after["inertia_moments"][name]) / np.array(osim_before["inertia_moments"][name])
            np.testing.assert_allclose(
                np.sort(scaled.body_inertia[i]) / np.sort(nominal.body_inertia[i]), ratio[0], atol=bound["inertia_ratio"]
            )


def test_inertia_rule_differs_from_opensims_as_documented() -> None:
    """OpenSim models a body as a cylinder under anisotropic factors; here the mass distribution stretches."""
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody><body name="b">
      <inertial pos="0 0 0" mass="10" diaginertia="0.1 0.2 0.25"/></body></worldbody></mujoco>""")
    factors = np.array([1.2, 0.8, 1.1])
    moments, quat = _scaled_inertia(model, 1, factors, float(np.prod(factors)))
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat)
    tensor = rot.reshape(3, 3) @ np.diag(moments) @ rot.reshape(3, 3).T
    principal = np.zeros(9)
    mujoco.mju_quat2Mat(principal, model.body_iquat[1])
    tensor = principal.reshape(3, 3).T @ tensor @ principal.reshape(3, 3)
    probe = REFERENCE["inertia_probe"]
    np.testing.assert_allclose(probe["mass_kg"], 10.56)
    np.testing.assert_allclose(probe["moments"], [0.092928, 0.27456, 0.27456], rtol=1e-9)
    np.testing.assert_allclose(np.sort(np.diag(tensor)), np.sort([0.082632, 0.298056, 0.3168]), rtol=1e-3)


def _anisotropic_errors(factors: dict[str, list[float]], tmp_path: Path) -> tuple[float, float, float]:
    """Largest position, muscle-ratio and mass-ratio differences from OpenSim's anisotropic case."""
    spec = myo_sim.load_spec("myolegs")
    nominal = spec.copy().compile()
    scaled = scale_spec(spec, _scale_set(tmp_path / "scale.xml", factors)).compile()
    before, after = _positions(nominal), _positions(scaled)
    osim_before, osim_after = REFERENCE["cases"]["nominal"], REFERENCE["cases"]["anisotropic"]
    position = max(
        np.abs(
            (after[n] - before[n]) - (np.array(osim_after["positions_m"][n]) - np.array(osim_before["positions_m"][n]))
        ).max()
        for n in before
    )
    muscle = 0.0
    for name, values in osim_after["muscles"].items():
        i = nominal.actuator(name).id
        ratio = scaled.actuator_lengthrange[i, 0] / nominal.actuator_lengthrange[i, 0]
        muscle = max(
            muscle, abs(ratio - values["optimal_fibre_length_m"] / osim_before["muscles"][name]["optimal_fibre_length_m"])
        )
    mass = max(
        abs(scaled.body(n).mass[0] / nominal.body(n).mass[0] - osim_after["mass_kg"][n] / osim_before["mass_kg"][n])
        for n in osim_after["mass_kg"]
    )
    return position, muscle, mass


@pytest.mark.parametrize(
    "segment,change,fails",
    [
        ("femur_r", [1.0, 1.05, 1.0], (True, True, True)),
        ("tibia_r", [1.05, 1.05, 1.05], (True, True, True)),
        ("pelvis", [1.0, 1.05, 1.0], (True, True, True)),
        ("talus_r", [1.05, 1.0, 1.0], (True, True, True)),
        ("femur_r", [1.05, 1.0, 1.0], (False, False, True)),
    ],
    ids=["femur_y", "tibia_xyz", "pelvis_y", "talus_x", "femur_x"],
)
def test_a_wrong_factor_fails_each_bound_it_should(
    segment: str, change: list[float], fails: tuple[bool, bool, bool], tmp_path: Path
) -> None:
    """Every tolerance must fail on a known-bad factor, and the fixture itself must pass."""
    bound = BOUNDS["anisotropic"]
    limits = (bound["position_m"], bound["muscle_ratio"], bound["mass_ratio"])
    good = _anisotropic_errors(FIXTURE["anisotropic"], tmp_path)
    assert all(value <= limit for value, limit in zip(good, limits))
    factors = {name: list(values) for name, values in FIXTURE["anisotropic"].items()}
    factors[segment] = [a * b for a, b in zip(factors[segment], change)]
    errors = _anisotropic_errors(factors, tmp_path)
    assert tuple(value > limit for value, limit in zip(errors, limits)) == fails, errors
