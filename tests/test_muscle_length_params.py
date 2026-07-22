"""Verify muscle lengthrange / operating range / FL bounds (lmin, lmax).

These tests document calibration consistency problems in the shipped models:

* ``gainprm[4]``/``[5]`` (FL ``lmin``/``lmax``) are often a shared default per model
  rather than per-muscle values (especially arm + torso).
* ``gainprm[0]``/``[1]`` (operating range) can fall outside ``[lmin, lmax]``.
* Measured musculotendon path can fall outside declared ``lengthrange``.

They are **excluded** from the default CI pytest invocation and run only from the
``muscle-params`` workflow job when model XML (or these files) change.

Recalculation helper: ``tests/recalc_from_fl_bounds.py``.
"""

from __future__ import annotations

import pytest
from recalc_from_fl_bounds import (
    MuscleParams,
    PathSample,
    fl_bound_diversity,
    operating_band_from_fl,
    propose_for_model,
    recalc_from_fl_bounds,
    verify_model,
)

# Models to audit when their assets change
MODELS = ("myoarm_r", "myolegs", "myotorso")


@pytest.mark.parametrize("model_name", MODELS)
def test_operating_range_within_fl_bounds(model_name: str):
    """r0/r1 must lie inside [lmin, lmax] (MuJoCo FL domain)."""
    violations = [v for v in verify_model(model_name) if v.kind == "operating_outside_fl"]
    if violations:
        lines = [f"{v.name}: {v.detail}" for v in violations[:30]]
        more = f"\n  ... +{len(violations) - 30} more" if len(violations) > 30 else ""
        pytest.fail(
            f"{model_name}: {len(violations)} muscles have operating range outside FL bounds:\n  " + "\n  ".join(lines) + more
        )


@pytest.mark.parametrize("model_name", MODELS)
def test_path_within_lengthrange(model_name: str):
    """Primary-joint MTU path must lie inside declared lengthrange."""
    violations = [v for v in verify_model(model_name) if v.kind == "path_outside_lengthrange"]
    if violations:
        lines = [f"{v.name}: {v.detail}" for v in violations[:30]]
        more = f"\n  ... +{len(violations) - 30} more" if len(violations) > 30 else ""
        pytest.fail(f"{model_name}: {len(violations)} muscles have path outside lengthrange:\n  " + "\n  ".join(lines) + more)


@pytest.mark.parametrize(
    "model_name,min_unique",
    [
        # Arm/torso currently share one or two FL defaults — fail to surface the issue.
        ("myoarm_r", 5),
        ("myotorso", 5),
        # Legs already vary; require at least a handful of distinct pairs.
        ("myolegs", 5),
    ],
)
def test_fl_lmin_lmax_not_a_single_shared_default(model_name: str, min_unique: int):
    """Per-muscle FL (lmin,lmax) should not collapse to one shared default.

    If myoconverter (or hand authoring) never wrote optimized FL bounds into
    ``gainprm[4]``/``[5]``, every muscle keeps the same ``(lmin, lmax)``.
    """
    summary = fl_bound_diversity(model_name)
    n_unique = summary["n_unique_fl_bounds"]
    if n_unique < min_unique:
        top = list(summary["counts"].items())[:3]
        pytest.fail(
            f"{model_name}: only {n_unique} unique (lmin,lmax) pair(s) across "
            f"{summary['n_actuators']} actuators (need >= {min_unique}). "
            f"Top counts: {top}. "
            "Use tests/recalc_from_fl_bounds.py after setting per-muscle FL bounds."
        )


def test_recalc_from_fl_bounds_places_operating_band_inside_fl():
    """Unit check: proposed r0/r1 lie in [lmin,lmax] and LR spans the path."""
    current = MuscleParams(
        name="dummy",
        lengthrange=(0.10, 0.20),
        r0=0.2,
        r1=1.9,
        lmin=0.0,
        lmax=2.0,
        fmax=100.0,
        l0_implied=0.1 / 1.7,
    )
    path = PathSample(joint="j", mtu_min=0.12, mtu_max=0.18)
    prop = recalc_from_fl_bounds(path, current, lmin=0.5, lmax=1.6, band_frac=0.15)
    assert prop.lmin == 0.5 and prop.lmax == 1.6
    assert prop.lmin - 1e-9 <= prop.r0 < prop.r1 <= prop.lmax + 1e-9
    assert prop.lengthrange_hi > prop.lengthrange_lo
    assert prop.lengthrange_lo <= path.mtu_min
    assert prop.lengthrange_hi >= path.mtu_max


def test_operating_band_from_fl_contains_one_when_possible():
    r0, r1 = operating_band_from_fl(0.5, 1.6, band_frac=0.15)
    assert r0 < 1.0 < r1
    assert 0.5 <= r0 < r1 <= 1.6


def test_propose_for_model_smoke_myoarm_single_muscle():
    props = propose_for_model("myoarm_r", muscle="DELT1", lmin=0.5, lmax=1.6)
    assert len(props) == 1
    p = props[0]
    assert p.name == "DELT1"
    assert 0.5 <= p.r0 < p.r1 <= 1.6
