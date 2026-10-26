"""Thenar muscles (adductor pollicis, flexor and abductor pollicis brevis).

MyoHand's thumb comes from the Holzbaur et al. (2005) arm model, which has no
intrinsic hand muscles: nothing pulls the thumb into the palm plane against the
extrinsic extensors and abductor. These muscles are added on MyoHand's own
bodies from ``data/thenar_muscles.json`` (sites and forces per cited source);
every other muscle parameter is copied from a template muscle, and the length
range is computed by MuJoCo from the joint range like every MyoHand muscle.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

DATA_PATH = Path(__file__).with_name("data") / "thenar_muscles.json"
_MIRROR = np.array([1.0, 1.0, -1.0])  # myo_sim mirrors local positions in z


def load_thenar_table(path: Path | str | None = None) -> dict[str, Any]:
    """Read the thenar-muscle table (default: the packaged one)."""
    return json.loads(Path(path or DATA_PATH).read_text())


def _template(spec: mujoco.MjSpec, base: str, side: str) -> mujoco.MjsActuator | None:
    """Template muscle of *side*: right-arm muscles are unsuffixed in some models (``OP``), ``_r`` in others."""
    for name in (base, f"{base}_r") if side == "r" else (f"{base}_{side}",):
        act = spec.actuator(name)
        if act is not None:
            return act
    return None


def add_thenar_muscles(
    spec: mujoco.MjSpec,
    table: dict[str, Any] | None = None,
    sides: tuple[str, ...] = ("r", "l"),
    lengthrange: str = "computed",
) -> list[str]:
    """Add the thenar muscles of *table* to every hand of *spec*, in place.

    A side is skipped when its template muscle (``table["template_muscle"]``,
    e.g. ``OP`` / ``OP_l``) is missing.

    Args:
        spec: Model spec to edit.
        table: Thenar-muscle table (default: :func:`load_thenar_table`).
        sides: Hands to edit (``"r"``, ``"l"``).
        lengthrange: ``"computed"`` (MuJoCo computes it at compile time, as for
            the other MyoHand muscles) or ``"source"`` (the source's fixed
            ``source_lengthrange``).

    Returns:
        Names of the added actuators.
    """
    table = table or load_thenar_table()
    base = table["template_muscle"]
    added = []
    for side in sides:
        template = _template(spec, base, side)
        if template is None:
            continue
        ref_tendon = spec.tendon(template.target)
        ref_site = next((s for s in spec.sites if s.name.startswith(f"{base}-P") and s.name.endswith(f"_{side}")), None)
        for muscle in table["muscles"]:
            # Same naming as the template: OP -> AdP, OP_tendon_l -> AdP_tendon_l.
            name = template.name.replace(base, muscle["name"], 1)
            tendon = spec.add_tendon(name=template.target.replace(base, muscle["name"], 1))
            tendon.width = ref_tendon.width
            tendon.rgba = ref_tendon.rgba
            for i, (body, pos) in enumerate(muscle["sites"], start=1):
                local = np.asarray(pos) * (1.0 if side == "r" else _MIRROR)
                site = spec.body(f"{body}_{side}").add_site(name=f"{muscle['name']}-P{i}_{side}", pos=local.tolist())
                if ref_site is not None:
                    site.size, site.rgba, site.group = ref_site.size, ref_site.rgba, ref_site.group
                tendon.wrap_site(site.name)
            act = spec.add_actuator(name=name, target=tendon.name, trntype=mujoco.mjtTrn.mjTRN_TENDON)
            for attr in (
                "gaintype",
                "biastype",
                "dyntype",
                "dynprm",
                "gainprm",
                "biasprm",
                "ctrlrange",
                "ctrllimited",
                "forcerange",
                "forcelimited",
                "actrange",
                "actlimited",
                "actearly",
                "group",
            ):
                setattr(act, attr, getattr(template, attr))
            act.gainprm[2] = act.biasprm[2] = muscle["force"]
            act.lengthrange = muscle["source_lengthrange"] if lengthrange == "source" else [0.0, 0.0]
            added.append(name)
    return added
