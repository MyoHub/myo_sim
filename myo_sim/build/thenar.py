"""Thenar muscles (adductor pollicis, flexor and abductor pollicis brevis).

MyoHand's thumb comes from the Holzbaur et al. (2005) arm model, which has no
intrinsic hand muscles: nothing pulls the thumb into the palm plane against the
extrinsic extensors and abductor. These muscles are added on MyoHand's own
bodies from ``data/thenar_muscles.json``: attachment sites from MUSIC (Xu et
al. 2026, MIT licence), architecture (PCSA, optimal fibre length, pennation)
from Jacobson et al. (1992). Activation dynamics and force-length shape are
copied from a template muscle (OP).

Length ranges (``lengthrange=``):

* ``"architecture"`` (default): rigid tendon of slack length
  ``(L_MT(q = 0) - L_fo cos(alpha)) / 1.033`` (Barry et al. 2018, Eq. 1), so the
  normalised fibre length ``range`` 0.75-1.05 spans ``[0.75, 1.05] L_fo``.
* ``"computed"``: MuJoCo maps the joint-range excursion onto ``range``, the
  convention of the other MyoHand muscles.
* ``"source"``: MUSIC's fixed values.

Subject scaling: ``scale={"<muscle>.force": f, "<muscle>.length": g}`` multiplies
the peak force and the optimal fibre length (see ``scales_with`` in the data).
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


def _architecture_lengthranges(spec: mujoco.MjSpec, rows: list[tuple[str, dict[str, Any], float]]) -> None:
    """Set ``[L_ts + 0.75 L_fo, L_ts + 1.05 L_fo]`` from the muscle-tendon lengths at q = 0."""
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_tendon(model, data)
    mujoco.mj_transmission(model, data)
    for name, muscle, l_fo in rows:
        act = spec.actuator(name)
        lo_hi = act.gainprm[:2]
        l_mt0 = float(data.actuator_length[model.actuator(name).id])
        l_ts = (l_mt0 - l_fo * np.cos(np.radians(muscle["pennation_deg"]["value"]))) / 1.033
        act.lengthrange = [l_ts + lo_hi[0] * l_fo, l_ts + lo_hi[1] * l_fo]


def add_thenar_muscles(
    spec: mujoco.MjSpec,
    table: dict[str, Any] | None = None,
    sides: tuple[str, ...] = ("r", "l"),
    lengthrange: str = "architecture",
    scale: dict[str, float] | None = None,
) -> list[str]:
    """Add the thenar muscles of *table* to every hand of *spec*, in place.

    A side is skipped when its template muscle (``table["template_muscle"]``,
    e.g. ``OP`` / ``OP_l``) is missing.

    Args:
        spec: Model spec to edit.
        table: Thenar-muscle table (default: :func:`load_thenar_table`).
        sides: Hands to edit (``"r"``, ``"l"``).
        lengthrange: ``"architecture"``, ``"computed"`` or ``"source"`` (see
            the module docstring).
        scale: Subject factors, ``{"<muscle>.force": f, "<muscle>.length": g}``.

    Returns:
        Names of the added actuators.
    """
    table = table or load_thenar_table()
    scale = scale or {}
    base = table["template_muscle"]
    added, architecture = [], []
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
            force = muscle["force_N"]["value"] * scale.get(f"{muscle['name']}.force", 1.0)
            act.gainprm[2] = act.biasprm[2] = force
            if lengthrange == "source":
                act.lengthrange = muscle["source_lengthrange_m"]
            else:
                act.lengthrange = [0.0, 0.0]  # computed at compile time unless set below
                if lengthrange == "architecture":
                    l_fo = muscle["optimal_fiber_length_m"]["value"] * scale.get(f"{muscle['name']}.length", 1.0)
                    architecture.append((name, muscle, l_fo))
            added.append(name)
    if architecture:
        _architecture_lengthranges(spec, architecture)
    return added
