"""Passive joint moments as fixed tendons with deadband polynomial springs.

Each measured direction of a joint becomes a one-joint fixed tendon whose
spring acts only beyond an onset angle: ``M = -(k0 x + k1 x^2 + k2 x^3)`` with
``x`` the displacement past the onset. MuJoCo and mujoco-warp evaluate it
natively (``tendon_lengthspring`` + ``tendon_stiffnesspoly``), so CPU and GPU
models get the same moment without callbacks.

Values come from ``data/passive_joint_moments.json``: one row per joint with a
cited source. Rows without a verified source are ``"gap"`` and add nothing.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

DATA_PATH = Path(__file__).with_name("data") / "passive_joint_moments.json"
TENDON_PREFIX = "passive_"
_FAR = 1.0e3  # rad: "no bound" side of a one-sided deadband


def load_passive_moment_table(path: Path | str | None = None) -> dict[str, Any]:
    """Read the passive-moment table (default: the packaged one)."""
    return json.loads(Path(path or DATA_PATH).read_text())


def measured_rows(table: dict[str, Any]) -> list[dict[str, Any]]:
    """Rows with a verified source (``status == "measured"``)."""
    return [row for row in table["joints"] if row["status"] == "measured"]


def _spring(direction: str, onset: float, stiffness: list[float]) -> tuple[list[float], list[float]]:
    """``(springlength, stiffness)`` of a one-sided tendon for *direction*."""
    k0, k1, k2 = stiffness
    if direction == "positive":  # acts above onset: x > 0
        return [-_FAR, onset], [k0, k1, k2]
    # Acts below -onset: x < 0, so the even term flips sign to keep restoring.
    return [-onset, _FAR], [k0, -k1, k2]


def add_passive_joint_moments(
    spec: mujoco.MjSpec,
    table: dict[str, Any] | None = None,
    sides: tuple[str, ...] = ("r", "l"),
) -> list[str]:
    """Add the measured passive joint moments of *table* to *spec*, in place.

    Joints missing from *spec* (e.g. a pruned or one-armed model) are skipped.

    Args:
        spec: Model spec to edit.
        table: Passive-moment table (default: :func:`load_passive_moment_table`).
        sides: Sides whose joint names (``row["names"][side]``) are used.

    Returns:
        Names of the added tendons.
    """
    table = table or load_passive_moment_table()
    joints = {j.name for j in spec.joints}
    added = []
    for row in measured_rows(table):
        for side in sides:
            joint = row["names"].get(side)
            if joint not in joints:
                continue
            for direction in ("positive", "negative"):
                part = row.get(direction)
                if part is None:
                    continue
                springlength, stiffness = _spring(direction, part["onset"], part["stiffness"])
                name = f"{TENDON_PREFIX}{joint}_{direction[:3]}"
                tendon = spec.add_tendon(name=name, springlength=springlength, stiffness=stiffness)
                tendon.wrap_joint(joint, 1.0)
                added.append(name)
    return added


def passive_moment(row: dict[str, Any], q: np.ndarray | float) -> np.ndarray:
    """Reference moment (Nm) of one table row at joint angles *q* (rad)."""
    q = np.asarray(q, dtype=float)
    moment = np.zeros_like(q)
    for direction, sign in (("positive", 1.0), ("negative", -1.0)):
        part = row.get(direction)
        if part is None:
            continue
        x = np.maximum(sign * q - part["onset"], 0.0)
        k0, k1, k2 = part["stiffness"]
        moment -= sign * (k0 * x + k1 * x**2 + k2 * x**3)
    return moment
