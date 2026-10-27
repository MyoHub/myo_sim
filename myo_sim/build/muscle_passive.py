"""Passive muscle tension: tendon-slack overrides that restore tenodesis.

The index finger's extrinsic muscles (FDS, FDP, EDC, EI) carry the extrinsic,
wrist-dependent part of the passive moment at the finger joints (Knutson et al.
2000), which makes the fingers flex when the wrist extends (tenodesis).
Following Binder-Markey & Murray (2017), their tendon slack lengths are fitted
to Knutson's extrinsic moment (``scripts/fit_muscle_passive.py``) and stored as
absolute values in ``data/muscle_passive.json`` with the reference values they
change. MyoHand muscles have rigid tendons: the slack length implied by a
muscle is ``lengthrange[0] - range[0] * L0`` with
``L0 = (lengthrange[1] - lengthrange[0]) / (range[1] - range[0])``, so a new
slack length is applied as a shift of ``lengthrange`` at constant ``L0``.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mujoco

DATA_PATH = Path(__file__).with_name("data") / "muscle_passive.json"


def load_muscle_passive_table(path: Path | str | None = None) -> dict[str, Any]:
    """Read the muscle passive-tension table (default: the packaged one)."""
    return json.loads(Path(path or DATA_PATH).read_text())


def implied_slack_length(act: mujoco.MjsActuator) -> float:
    """Rigid-tendon slack length implied by a muscle's ``lengthrange`` and ``range`` (m)."""
    lo, hi = act.lengthrange
    r0, r1 = act.gainprm[0], act.gainprm[1]
    return float(lo - r0 * (hi - lo) / (r1 - r0))


def _actuator(spec: mujoco.MjSpec, name: str, side: str) -> mujoco.MjsActuator | None:
    """The muscle on *side*: right-arm muscles are unsuffixed in some models (``FDS2``), ``_r`` in others."""
    for candidate in (name, f"{name}_r") if side == "r" and not name.endswith("_r") else (name,):
        act = spec.actuator(candidate)
        if act is not None:
            return act
    return None


def add_muscle_passive_tension(
    spec: mujoco.MjSpec,
    table: dict[str, Any] | None = None,
    sides: tuple[str, ...] = ("r", "l"),
    scale: dict[str, float] | None = None,
) -> list[str]:
    """Set the tendon slack lengths of the muscles in *table*, in place.

    The spec's muscles must carry explicit length ranges (myo_sim's composed
    arms do). Muscles missing from *spec* are skipped.

    Args:
        spec: Model spec to edit.
        table: Override table (default: :func:`load_muscle_passive_table`).
        sides: Sides whose muscle names (``row["names"][side]``) are used.
        scale: Subject factors ``{"<muscle>.slack": f}`` multiplying a row's
            slack length (``scales_with: length``).

    Returns:
        Names of the edited actuators.
    """
    table = table or load_muscle_passive_table()
    scale = scale or {}
    edited = []
    for row in table["muscles"]:
        if row["status"] != "measured":
            continue
        target = row["tendon_slack_m"]["value"] * scale.get(f"{row['muscle']}.slack", 1.0)
        for side in sides:
            act = _actuator(spec, row["names"][side], side)
            if act is None:
                continue
            if not any(act.lengthrange):
                raise ValueError(f"{act.name}: no explicit lengthrange to shift")
            shift = target - implied_slack_length(act)
            act.lengthrange = [v + shift for v in act.lengthrange]
            edited.append(act.name)
    return edited
