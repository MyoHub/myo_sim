"""Shoulder path candidates (``scripts/fit_shoulder_paths.py``) and the literature data of the mechanics validation."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

import myo_sim

ROOT = Path(__file__).resolve().parents[1]
TABLE = json.loads((ROOT / "myo_sim" / "build" / "data" / "shoulder_paths.json").read_text())


def _script(name: str):
    if name not in sys.modules:
        sys.path.insert(0, str(ROOT / "scripts"))
        spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
        sys.modules[name] = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sys.modules[name])
    return sys.modules[name]


def test_sites_of_candidates_not_adopted_are_unchanged() -> None:
    model = myo_sim.load_model("myoarm_r")
    for c in TABLE["candidates"]:
        for name, site in c["sites"].items():
            expected = site["pos"] if c["adopted"] else site["current"]
            np.testing.assert_allclose(model.site_pos[model.site(name).id], expected, atol=1e-9)


def test_candidates_use_only_the_fit_set() -> None:
    vsm = _script("validate_shoulder_mechanics")
    assert TABLE["fit_studies"] == list(vsm.FIT_SET)
    assert not set(vsm.FIT_SET) & {"liu1997", "hughes1998", "otis1994", "axford2025"}


def test_candidate_evaluation_reproduces() -> None:
    """Stored moment arms (0.05 mm) and held-out counts of the current path and of each candidate."""
    script = _script("fit_shoulder_paths")
    arm = script.vsm.Arm()
    grid = script.Grid(arm)
    for design, stored in zip(script.CANDIDATES, TABLE["candidates"]):
        fit = script.Fit(design, arm.model, grid)
        x_current = fit.x0.copy()
        x = np.concatenate([stored["sites"][n]["pos"] for n in design["sites"]])
        for label, xs in (("current", x_current), ("candidate", x)):
            summary = fit.summary(xs)
            for plane, row in stored[label]["fit"]["planes"].items():
                np.testing.assert_allclose(
                    summary["planes"][plane]["model_mm"], row["model_mm"], atol=0.05, err_msg=f"{label} {plane}"
                )
            assert script.held_out(arm) == stored[label]["held_out"], label
        fit.curve(x_current)


def test_otis_and_axford_equations_as_published() -> None:
    """Otis 1994's supraspinatus at 60 deg as tabulated by Gatti 2007 (25.1 / 27.0 mm, anterior / posterior); Axford
    2025's maps are derivatives of one excursion, so their mixed terms agree."""
    vsm = _script("validate_shoulder_mechanics")
    for (_, _, c2, c3), expected in zip(vsm.OTIS_1994["supraspinatus"], (25.1, 27.0)):
        assert -(c2 + c3 * 0.0) * vsm.CM_PER_DEG == pytest.approx(expected, abs=0.1)
    elevation = dict(vsm.AXFORD_2025_TERMS["elevation"])
    rotation = dict(vsm.AXFORD_2025_TERMS["rotation"])
    np.testing.assert_allclose(elevation[(0, 0, 1)], rotation[(1, 0, 0)])
    assert vsm.axford("supraspinatus", 30.0, 0.0, 0.0, "elevation") == pytest.approx(22.6, abs=0.2)


def test_kuechle2000_signs_follow_the_full_text() -> None:
    """At the side, rotation 0: Kuechle 2000 Table 1 (+ = internal, cm), negated here (+ = external, mm)."""
    vsm = _script("validate_shoulder_mechanics")
    curves = vsm.LIT_CURVES["rotation"]["kuechle2000"]
    i0 = vsm.ROT.index(0)
    table1 = {"subscapularis": 2.18, "teres minor": -2.00, "infraspinatus": -2.34, "supraspinatus": -0.27,
              "latissimus dorsi": 0.82, "anterior deltoid": 0.68, "teres major": 0.67, "posterior deltoid": -0.39,
              "pectoralis major": 1.84}  # fmt: skip
    for muscle, cm in table1.items():
        assert np.sign(curves[muscle][i0]) == -np.sign(cm), muscle
        assert abs(curves[muscle][i0] + 10 * cm) <= 2.0, muscle
