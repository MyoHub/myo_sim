"""Standard motion-capture marker sets as sites on a composed model.

A marker set is a YAML file in ``data/``. Each marker is placed either relative to an existing site of the model
(``anchor`` plus an ``offset`` in that site's body frame, or ``between`` two sites of one body at fraction ``t``) or
directly in a body frame (``body`` plus ``pos``). Body frames follow OpenSim: x anterior, y up, z right.
``add_markerset`` adds the markers as sites of an uncompiled ``MjSpec``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from importlib import resources
from typing import Any

import mujoco
import numpy as np
import yaml

KINDS = ("anatomical", "technical", "cluster", "virtual")
CONFIDENCE = ("anatomical", "fitted", "reference", "nominal")


@dataclass(frozen=True)
class Marker:
    """One marker. ``confidence``: "anatomical" (on a model landmark), "fitted" (fitted to recorded data, see the set's
    description), "reference" (copied from a published model with matching body frames) or "nominal" (placed from the written placement description)."""

    name: str
    kind: str
    description: str
    anchor: str | None = None
    body: str | None = None
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    confidence: str = "nominal"
    between: tuple[str, str] | None = None
    t: float = 0.0


@dataclass(frozen=True)
class MarkerSet:
    name: str
    description: str
    sources: tuple[str, ...]
    markers: tuple[Marker, ...] = field(default_factory=tuple)

    def names(self) -> list[str]:
        return [m.name for m in self.markers]


def list_markersets() -> list[str]:
    """Names of the packaged marker sets."""
    files = resources.files("myo_sim.markersets").joinpath("data").iterdir()
    return sorted(f.name[: -len(".yaml")] for f in files if f.name.endswith(".yaml"))


def _marker(name: str, raw: dict[str, Any]) -> Marker:
    if sum(raw.get(k) is not None for k in ("anchor", "body", "between")) != 1:
        raise ValueError(f"marker {name!r} needs exactly one of 'anchor', 'between' or 'body'")
    if raw["kind"] not in KINDS or raw.get("confidence", "nominal") not in CONFIDENCE:
        raise ValueError(f"marker {name!r}: bad kind or confidence")
    pos = tuple(float(x) for x in raw.get("offset", raw.get("pos", (0, 0, 0))))
    if len(pos) != 3:
        raise ValueError(f"marker {name!r}: offset/pos needs three values")
    return Marker(
        name,
        raw["kind"],
        raw["description"],
        raw.get("anchor"),
        raw.get("body"),
        pos,
        raw.get("confidence", "nominal"),
        tuple(raw["between"]) if raw.get("between") else None,
        float(raw.get("t", 0.5)),
    )


def load_markerset(name: str) -> MarkerSet:
    """Load a packaged marker set by name (see ``list_markersets``)."""
    if name not in list_markersets():
        raise KeyError(f"unknown marker set {name!r}; available: {list_markersets()}")
    raw = yaml.safe_load(resources.files("myo_sim.markersets").joinpath("data", f"{name}.yaml").read_text())
    return MarkerSet(
        raw["name"], raw["description"], tuple(raw["sources"]), tuple(_marker(k, v) for k, v in raw["markers"].items())
    )


def _resolve(spec: mujoco.MjSpec, m: Marker):
    """Parent body and body-frame position of a marker, or (None, None) if its anchors are not in the model."""
    if m.between is not None:
        a, b = (spec.site(n) for n in m.between)
        if a is None or b is None:
            return None, None
        if a.parent.name != b.parent.name:
            raise ValueError(f"marker {m.name!r}: 'between' sites must share a body")
        return a.parent, np.asarray(a.pos) + m.t * (np.asarray(b.pos) - np.asarray(a.pos)) + m.pos
    if m.anchor is not None:
        site = spec.site(m.anchor)
        return (site.parent, np.asarray(site.pos) + m.pos) if site is not None else (None, None)
    body = spec.body(m.body)
    return body, np.asarray(m.pos)


def add_markerset(
    spec: mujoco.MjSpec, markerset: str | MarkerSet, prefix: str = "mk_", skip_missing: bool = False
) -> list[str]:
    """Add the markers of a set as sites named ``prefix + marker name``; returns the names of skipped markers.

    Args:
        spec: Uncompiled spec of the model.
        markerset: Set name or object.
        prefix: Prefix of the site names.
        skip_missing: Skip markers whose anchor site or body is absent (partial models) instead of raising.
    """
    ms = load_markerset(markerset) if isinstance(markerset, str) else markerset
    skipped = []
    for m in ms.markers:
        body, pos = _resolve(spec, m)
        if body is None:
            if not skip_missing:
                raise KeyError(f"marker {m.name!r}: {m.anchor or m.between or m.body!r} is not in the model")
            skipped.append(m.name)
            continue
        body.add_site(name=prefix + m.name, pos=pos, size=[0.008, 0.0, 0.0], rgba=[1.0, 0.5, 0.0, 0.0])
    return skipped
