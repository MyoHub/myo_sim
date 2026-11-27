# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
#
# Adapted from MyoSuite PR #564 by Mark Colley (M-Colley), Apache-2.0
# (https://github.com/MyoHub/myosuite/pull/564).
"""``scale_spec`` on MyoLeg against OpenSim's ModelScaler over a pose grid (stored reference).

Six right-leg muscles over twelve hip, knee and ankle poses: moment arms, normalised fibre length,
tendon force and torque, with joint centres and sole points. The fast tests rerun the myo_sim half
and compare it with the stored OpenSim half (bounds, and the mutations each bound must reject) and
with the stored myo_sim half (a tight self-regression). Only the opt-in test runs OpenSim. See
``docs/wiki/model-scaling.md``.
"""

from __future__ import annotations

import json
import os
import subprocess
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim import scale_spec, scaling
from scripts import validate_opensim_scaling as validation

TOLERANCES = json.loads(validation.POSE_TOLERANCES.read_text())
SCALED_CASES = validation.compared_cases()[1:]
# The preserved-mass case differs from uniform_volume by its masses only.
MUTATED_CASES = ("uniform_volume", "anisotropic_volume")
WRAP = "pelvis_wrap_dimensions_m"


@pytest.fixture(scope="module")
def reference() -> dict:
    return json.loads(validation.POSE_REFERENCE.read_text())


@pytest.fixture(scope="module")
def measured() -> dict:
    return validation.sample_myo_sim()


@pytest.fixture(scope="module")
def mutants() -> dict:
    """Sampled mutants by (case, mutation), filled on first use."""
    return {}


def _mutant(cache: dict, case: str, mutation: str) -> dict[str, Any]:
    if (case, mutation) not in cache:
        recipe = validation.pose_fixture()["cases"][case]
        model = validation.build_case(recipe, mutation)
        cache[case, mutation] = validation.sample_case(model, recipe["factors"].get("calcn_r", [1.0, 1.0, 1.0]))
    return cache[case, mutation]


def _mutations(case: str) -> list[str]:
    # Swapping two equal factors changes nothing.
    uniform = validation.category(case) == "uniform"
    return [m for m in validation.MUTATIONS if not (uniform and m == "femur_r_swap_xy")]


def _scores(sample: Mapping[str, Any], measured: dict, reference: dict, case: str) -> dict[str, float]:
    """Discrepancy over bound of every bounded quantity of a sampled ``case``."""
    myo_sim = {"cases": {"nominal": measured["cases"]["nominal"], case: sample}}
    scores = {
        metric: float(np.abs(validation.discrepancy(myo_sim, reference["opensim"], case, metric)).max()) / bound
        for metric, bound in TOLERANCES[validation.category(case)].items()
    }
    scores[WRAP] = validation.wrap_increment(myo_sim, reference["opensim"], case) / TOLERANCES[WRAP]
    assert all(np.isfinite(value) for value in scores.values()), f"{case}: nonfinite discrepancy"
    return scores


def _regression_score(sample: Mapping[str, Any], stored: Mapping[str, Any]) -> float:
    """Largest deviation from the stored myo_sim half, in units of its tolerance."""
    limits = TOLERANCES["self_regression"]
    worst = 0.0
    for record, expected in stored.items():
        expected = np.asarray(expected, dtype=float)
        actual = np.asarray(sample[record], dtype=float)
        assert actual.shape == expected.shape, f"{record}: shape mismatch"
        assert np.isfinite(expected).all() and np.isfinite(actual).all(), f"{record}: nonfinite value"
        if expected.size:
            allowed = limits["atol_by_record"].get(record, limits["atol"]) + limits["rtol"] * np.abs(expected)
            worst = max(worst, float(np.max(np.abs(actual - expected) / allowed)))
    return worst


@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize("side", ["actual", "expected"])
def test_self_regression_refuses_nonfinite_records(value: float, side: str) -> None:
    sample, stored = {"root_heights_m": [1.0]}, {"root_heights_m": [1.0]}
    (sample if side == "actual" else stored)["root_heights_m"] = [value]
    with pytest.raises(AssertionError, match="nonfinite"):
        _regression_score(sample, stored)


def test_self_regression_refuses_broadcastable_shape_changes() -> None:
    with pytest.raises(AssertionError, match="shape mismatch"):
        _regression_score({"root_heights_m": [1.0]}, {"root_heights_m": [1.0, 1.0]})


@pytest.mark.parametrize("metric", ["discrepancy", "wrap_increment"])
def test_cross_model_scores_refuse_nonfinite_metrics(reference: dict, monkeypatch: pytest.MonkeyPatch, metric: str) -> None:
    monkeypatch.setattr(validation, metric, lambda *args: np.nan)
    case = MUTATED_CASES[0]
    with pytest.raises(AssertionError, match="nonfinite discrepancy"):
        _scores(reference["myo_sim"]["cases"][case], reference["myo_sim"], reference, case)


def test_the_stored_halves_match_the_fixture_and_model(reference: dict) -> None:
    assert reference["schema"] == 2
    assert reference["fixture"] == validation.pose_fixture()
    assert reference["fixture"]["muscles"] == list(validation.MUSCLES)
    assert reference["fixture"]["body_metrics_order"] == list(validation.POSE_BODIES)
    assert reference["opensim"]["provenance"]["model_sha256"] == validation.MODEL_SHA256
    assert set(reference["myo_sim"]["cases"]) == set(validation.compared_cases())


@pytest.mark.parametrize("case", SCALED_CASES)
def test_scaling_discrepancy_stays_within_bounds(case: str, reference: dict, measured: dict) -> None:
    scores = _scores(measured["cases"][case], measured, reference, case)
    assert max(scores.values()) < 1, scores


@pytest.mark.parametrize("case", MUTATED_CASES)
def test_each_bound_rejects_the_mutations_it_covers(case: str, reference: dict, measured: dict, mutants: dict) -> None:
    missed = [
        (quantity, mutation)
        for quantity, mutations in TOLERANCES["covers"][validation.category(case)].items()
        for mutation in mutations
        if _scores(_mutant(mutants, case, mutation), measured, reference, case)[quantity] <= 1
    ]
    assert not missed


@pytest.mark.parametrize("case", MUTATED_CASES)
def test_self_regression_rejects_every_mutation(case: str, reference: dict, mutants: dict) -> None:
    category = validation.category(case)
    covered = {m for ms in TOLERANCES["covers"][category].values() for m in ms}
    regression_only = set(TOLERANCES["regression_only"][category])
    # Every mutation is either seen by a cross-model bound or listed as unseen.
    assert covered | regression_only == set(_mutations(case))
    assert not covered & regression_only
    stored = reference["myo_sim"]["cases"][case]
    missed = [m for m in _mutations(case) if _regression_score(_mutant(mutants, case, m), stored) <= 1]
    assert not missed


def test_the_myo_sim_half_reproduces_its_stored_baseline(reference: dict, measured: dict) -> None:
    for case, values in reference["myo_sim"]["cases"].items():
        assert _regression_score(measured["cases"][case], values) <= 1, case


def _affine_inertia(inertia: np.ndarray, factors: np.ndarray, mass_ratio: float) -> np.ndarray:
    """Inertia after stretching the mass distribution by ``factors`` (the documented rule)."""
    second = 0.5 * np.trace(inertia) * np.eye(3) - inertia
    second = mass_ratio * np.diag(factors) @ second @ np.diag(factors)
    return np.trace(second) * np.eye(3) - second


def test_mass_scaling_matches_opensim(reference: dict, measured: dict) -> None:
    """Per-body mass ratios agree with OpenSim in every compared case, inertia ratios under one factor."""
    for case in SCALED_CASES:
        ratios = [
            np.array(half["cases"][case]["body_mass_kg"]) / np.array(half["cases"]["nominal"]["body_mass_kg"])
            for half in (measured, reference["opensim"])
        ]
        np.testing.assert_allclose(ratios[0], ratios[1], rtol=0, atol=1e-12, err_msg=case)
        if validation.category(case) == "uniform":
            inertia = [
                np.diagonal(half["cases"][case]["body_inertia_kgm2"], axis1=1, axis2=2)
                / np.diagonal(half["cases"]["nominal"]["body_inertia_kgm2"], axis1=1, axis2=2)
                for half in (measured, reference["opensim"])
            ]
            np.testing.assert_allclose(inertia[0], inertia[1], rtol=0, atol=1e-10, err_msg=case)


def test_inertia_follows_the_affine_rule(measured: dict) -> None:
    # 10 kg, diagonal inertia (0.1, 0.2, 0.25), factors (1.2, 0.8, 1.1); "keep" rescales to the nominal mass.
    factors = np.array([1.2, 0.8, 1.1])
    for total_mass, mass_ratio in ((None, float(np.prod(factors))), (10.0, 1.0)):
        spec = mujoco.MjSpec.from_string(
            '<mujoco><worldbody><body name="probe"><freejoint/><inertial mass="10" pos="0 0 0" '
            'diaginertia=".1 .2 .25"/></body></worldbody></mujoco>'
        )
        model = scale_spec(spec, {"probe": factors}, total_mass=total_mass).compile()
        assert model.body_mass[1] == pytest.approx(10.0 * mass_ratio, rel=1e-12)
        np.testing.assert_allclose(
            validation._inertia_in(model, 1, np.eye(3)),
            _affine_inertia(np.diag([0.1, 0.2, 0.25]), factors, mass_ratio),
            rtol=1e-12,
            atol=1e-15,
        )
    # MyoLeg's pelvis and right-leg bodies, in their own frames.
    recipe = validation.pose_fixture()["cases"]["anisotropic_volume"]["factors"]
    scaled, nominal = (measured["cases"][case] for case in ("anisotropic_volume", "nominal"))
    for index, body in enumerate(validation.POSE_BODIES):
        expected = _affine_inertia(
            np.array(nominal["body_inertia_kgm2"][index]),
            np.array(recipe[body]),
            scaled["body_mass_kg"][index] / nominal["body_mass_kg"][index],
        )
        np.testing.assert_allclose(scaled["body_inertia_kgm2"][index], expected, rtol=1e-10, atol=1e-14, err_msg=body)


def _axes_along_pelvis(model: mujoco.MjModel, body: str) -> np.ndarray:
    """For each axis of ``body``, the pelvis (OpenSim) axis it lies along."""
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    turn = data.xmat[model.body("pelvis").id].reshape(3, 3).T @ data.xmat[model.body(body).id].reshape(3, 3)
    return np.argmax(np.abs(turn), axis=0)


def _local_records(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Positions of the root's child bodies, then torso primitive geom positions, torso inertia and head_attach position."""
    root, torso = model.body("Full Body").id, model.body("torso").id
    primitives = (model.geom_bodyid == torso) & (model.geom_type != mujoco.mjtGeom.mjGEOM_MESH)
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, model.body_iquat[torso])
    rot = rot.reshape(3, 3)
    return (
        model.body_pos[model.body_parentid == root],
        model.geom_pos[primitives],
        rot @ np.diag(model.body_inertia[torso]) @ rot.T,
        model.body_pos[model.body("head_attach").id],
    )


def test_root_and_torso_take_opensim_axes(tmp_path: Path) -> None:
    """MyoLeg's root is rotated against the pelvis; factors per OpenSim axis must land on the axes pointing the same way."""
    factors = np.array([1.1, 1.2, 1.3])
    spec = myo_sim.load_spec("myolegs")
    nominal = spec.copy().compile()
    root_factors = factors[_axes_along_pelvis(nominal, "Full Body")]
    torso_factors = factors[_axes_along_pelvis(nominal, "torso")]
    assert not np.array_equal(root_factors, factors)
    children, geoms, inertia, head = _local_records(nominal)

    def matches(model: mujoco.MjModel) -> bool:
        scaled_children, scaled_geoms, scaled_inertia, scaled_head = _local_records(model)
        expected = _affine_inertia(inertia, torso_factors, float(np.prod(factors)))
        return (
            np.allclose(scaled_children, children * root_factors, rtol=0, atol=1e-12)
            and np.allclose(scaled_geoms, geoms * torso_factors, rtol=0, atol=1e-12)
            and np.allclose(scaled_inertia, expected, rtol=1e-10, atol=1e-14)
            and np.allclose(scaled_head, head * torso_factors, rtol=0, atol=1e-12)
        )

    path = validation.write_scale_set({"pelvis": factors, "torso": factors}, tmp_path / "scale.xml")
    assert matches(scale_spec(spec, path).compile())
    # The known-bad mapping: OpenSim factors along each body's own axes.
    assert not matches(scale_spec(spec, {"Full Body": factors, "lumbar5": factors}).compile())


def test_root_height_regression_sees_unscaled_feet(reference: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    """The feet keep their nominal collision geoms; the calcaneus landmarks cannot see this."""
    scale_geom = scaling._scale_geom

    def skip_feet(spec: Any, geom: Any, factors: np.ndarray) -> None:
        if not geom.parent.name.startswith(("calcn_", "toes_")):
            scale_geom(spec, geom, factors)

    monkeypatch.setattr(scaling, "_scale_geom", skip_feet)
    case = "anisotropic_volume"
    model = validation.build_case(validation.pose_fixture()["cases"][case])
    record = "root_heights_m"
    stored = {record: reference["myo_sim"]["cases"][case][record]}
    assert _regression_score(validation.subject_records(model), stored) > 1


def test_a_changed_model_file_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "Rajagopal2016.osim"
    path.write_text("changed upstream data")
    with pytest.raises(ValueError, match="SHA-256"):
        validation.model_path(str(path))


def test_moment_arm_finite_difference_is_stable(measured: dict) -> None:
    recipe = validation.pose_fixture()["cases"]["anisotropic_volume"]
    finer = validation.sample_case(validation.build_case(recipe), recipe["factors"]["calcn_r"], moment_arm_step=1e-6)
    np.testing.assert_allclose(
        finer["moment_arms_m"], measured["cases"]["anisotropic_volume"]["moment_arms_m"], rtol=0, atol=1e-7
    )


def test_live_opensim_reproduces_the_stored_half(reference: dict) -> None:
    """Opt-in: set OPENSIM_PYTHON to the Python of an OpenSim 4.6 environment."""
    python = os.environ.get("OPENSIM_PYTHON")
    if not python:
        pytest.skip("Set OPENSIM_PYTHON to run OpenSim's side of the comparison.")
    command = [python, validation.__file__, "--opensim-pose-half", "--model", str(validation.model_path(None))]
    actual = json.loads(subprocess.run(command, check=True, capture_output=True, text=True).stdout.splitlines()[-1])
    stored = reference["opensim"]
    for case, values in stored["cases"].items():
        for record, expected in values.items():
            np.testing.assert_allclose(
                actual["cases"][case][record], expected, rtol=1e-6, atol=1e-8, err_msg=f"{case} {record}"
            )
    for mode, values in stored["inertia_probe"].items():
        np.testing.assert_allclose(
            actual["inertia_probe"][mode]["inertia_kgm2"], values["inertia_kgm2"], rtol=1e-12, atol=1e-15
        )
