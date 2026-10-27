"""Data behind the thenar and passive-moment arm fragments.

The XML fragments (``myoarm_r_thenar_*.xml``, ``myoarm_r_passive*_tendon.xml``)
are the model; the JSON tables in ``data/`` are their provenance and fitting
input (``scripts/generate_thenar_fragments.py``, ``scripts/fit_passive_moments.py``
write the fragments from them). This module reads the tables, names the
passive tendons, evaluates the reference passive moment of a table row and
applies subject scale factors to a built spec.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

DATA_DIR = Path(__file__).with_name("data")
PASSIVE_DATA_PATH = DATA_DIR / "passive_joint_moments.json"
THENAR_DATA_PATH = DATA_DIR / "thenar_muscles.json"
PASSIVE_TENDON_PREFIX = "passive_"
FRAGMENT_STATUSES = ("measured", "assumed")
SPRING_FAR = 1.0e3  # rad: "no bound" side of a one-sided deadband
_PASSIVE_TENDON = re.compile(rf"^{PASSIVE_TENDON_PREFIX}(?P<row>.+)_(?:pos|neg)\d*_[rl]$")


def load_passive_moment_table(path: Path | str | None = None) -> dict[str, Any]:
    """Read the passive-moment table (default: the packaged one)."""
    return json.loads(Path(path or PASSIVE_DATA_PATH).read_text())


def load_thenar_table(path: Path | str | None = None) -> dict[str, Any]:
    """Read the thenar-muscle table (default: the packaged one)."""
    return json.loads(Path(path or THENAR_DATA_PATH).read_text())


def rows_with_status(table: dict[str, Any], statuses: tuple[str, ...] = ("measured",)) -> list[dict[str, Any]]:
    """Rows of the passive-moment table whose status is in *statuses*."""
    return [row for row in table["joints"] if row["status"] in statuses]


def spring_segments(row: dict[str, Any], direction: str) -> list[dict[str, Any]]:
    """Spring segments of one side of a row: a dict or a list of dicts (stacked onsets)."""
    part = row.get(direction)
    if part is None:
        return []
    return part if isinstance(part, list) else [part]


def passive_tendon_name(row: str, direction: str, index: int = 0) -> str:
    """Right-side name of the fixed tendon of segment *index* of *row*'s *direction* side."""
    return f"{PASSIVE_TENDON_PREFIX}{row}_{direction[:3]}{index or ''}_r"


def spring_parameters(direction: str, onset: float, stiffness: list[float]) -> tuple[list[float], list[float]]:
    """``(springlength, stiffness)`` of a one-sided fixed tendon acting beyond *onset*.

    MuJoCo's deadband spring ``tendon_lengthspring`` with polynomial
    ``stiffness`` gives the restoring moment ``-(k0 x + k1 x^2 + k2 x^3)``.
    """
    k0, k1, k2 = stiffness
    if direction == "positive":
        return [-SPRING_FAR, onset], [k0, k1, k2]
    # Acts below onset (x < 0): the even term flips sign to stay restoring.
    return [onset, SPRING_FAR], [k0, -k1, k2]


def passive_moment(row: dict[str, Any], q: np.ndarray | float) -> np.ndarray:
    """Reference moment (Nm) of one table row at joint angles *q* (rad)."""
    q = np.asarray(q, dtype=float)
    moment = np.zeros_like(q)
    for direction, sign in (("positive", 1.0), ("negative", -1.0)):
        for part in spring_segments(row, direction):
            x = np.maximum(sign * (q - part["onset"]), 0.0)
            k0, k1, k2 = part["stiffness"]
            moment -= sign * (k0 * x + k1 * x**2 + k2 * x**3)
    return moment


def _side_base(name: str) -> str:
    return name[:-2] if name.endswith(("_r", "_l")) else name


def apply_subject_scale(spec: mujoco.MjSpec, scale: dict[str, float]) -> mujoco.MjSpec:
    """Apply subject factors to the passive-moment tendons and thenar muscles of *spec*, in place.

    Keys follow the data tables: a passive-moment row name multiplies the
    stiffness of that row's tendons on both sides; ``"<muscle>.force"``
    multiplies a thenar muscle's peak force and ``"<muscle>.length"`` its
    optimal fibre length, which moves the length range as the generator
    derives it (rigid tendon of Barry et al. 2018's slack length). Keys whose
    elements are not in *spec* are ignored.

    Args:
        spec: Built (uncompiled) spec.
        scale: Factors ``{row_name: f, "AdP.force": g, "APB.length": h, ...}``.

    Returns:
        The same spec, for chaining.
    """
    for tendon in spec.tendons:
        match = _PASSIVE_TENDON.match(tendon.name)
        if match and match["row"] in scale:
            tendon.stiffness = [k * scale[match["row"]] for k in tendon.stiffness]
    muscles = {m["name"]: m for m in load_thenar_table()["muscles"]}
    for act in spec.actuators:
        muscle = muscles.get(_side_base(act.name))
        if muscle is None:
            continue
        force = scale.get(f"{muscle['name']}.force", 1.0)
        act.gainprm[2] *= force
        act.biasprm[2] *= force
        length = scale.get(f"{muscle['name']}.length", 1.0)
        if length != 1.0:
            lo, hi = act.gainprm[0], act.gainprm[1]
            l_fo = muscle["optimal_fiber_length_m"]["value"]
            cos_alpha = np.cos(np.radians(muscle["pennation_deg"]["value"]))
            l_mt0 = 1.033 * (act.lengthrange[0] - lo * l_fo) + l_fo * cos_alpha
            l_ts = (l_mt0 - length * l_fo * cos_alpha) / 1.033
            act.lengthrange = [l_ts + lo * length * l_fo, l_ts + hi * length * l_fo]
    return spec
