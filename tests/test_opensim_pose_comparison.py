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

import numpy as np
import pytest

from myo_sim import scaling
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
    return scores


def _regression_score(sample: Mapping[str, Any], stored: Mapping[str, Any]) -> float:
    """Largest deviation from the stored myo_sim half, in units of its tolerance."""
    limits = TOLERANCES["self_regression"]
    worst = 0.0
    for record, expected in stored.items():
        expected = np.asarray(expected, dtype=float)
        if expected.size:
            allowed = limits["atol_by_record"].get(record, limits["atol"]) + limits["rtol"] * np.abs(expected)
            worst = max(worst, float(np.max(np.abs(np.asarray(sample[record], dtype=float) - expected) / allowed)))
    return worst


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
