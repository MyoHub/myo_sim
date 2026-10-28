"""Generic post-processing functions for loaded MjSpecs.

Model policy (docs/wiki/model-authoring.md, policy 4): simplifications that apply
to many models are functions the user runs on a loaded ``MjSpec``, not per-model
variants or build options. They never change the reference model.
"""

from __future__ import annotations

import re
from collections.abc import Iterable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import mujoco


def _compile_patterns(patterns: str | Iterable[str] | None) -> list[re.Pattern[str]]:
    if patterns is None:
        return []
    if isinstance(patterns, str):
        patterns = [patterns]
    return [re.compile(pattern) for pattern in patterns]


def disable_collisions(
    spec: mujoco.MjSpec,
    bodies: str | Iterable[str] | None = None,
    geoms: str | Iterable[str] | None = None,
) -> mujoco.MjSpec:
    """Disable collisions on all geoms of a spec, or on a selected subset, in place.

    With neither ``bodies`` nor ``geoms`` given, every geom is made
    non-collidable, including the floor, and every explicit contact pair is
    removed. Otherwise a geom is selected if its own name matches a ``geoms``
    pattern or the name of the body it belongs to matches a ``bodies`` pattern
    (union of both). Patterns are regular expressions matched against the whole
    name (``re.fullmatch``); selections that match nothing are skipped
    silently, so one pattern list can serve right-only and bilateral models.

    Selected geoms get ``contype = conaffinity = 0``. Explicit ``<pair>``
    contacts that reference a selected geom are deleted as well, because pairs
    bypass contype/conaffinity filtering.

    Example:
        Disable the finger and thumb capsules of both hands, keeping the palm::

            spec = myo_sim.load_spec("myohands")
            myo_sim.disable_collisions(spec, bodies=r"(\\d?proxph\\d?|midph\\d|distph\\d|.*_thumb)_[rl]")
            model = spec.compile()

    Args:
        spec: A loaded, uncompiled MjSpec (e.g. from ``myo_sim.load_spec()``).
        bodies: Regular expression(s) for body names; all geoms attached
            directly to a matching body are selected.
        geoms: Regular expression(s) for geom names.

    Returns:
        The same spec, mutated in place, for chaining.
    """
    body_patterns = _compile_patterns(bodies)
    geom_patterns = _compile_patterns(geoms)
    select_all = not body_patterns and not geom_patterns

    def selected(geom: mujoco.MjsGeom) -> bool:
        if select_all:
            return True
        if any(pattern.fullmatch(geom.name) for pattern in geom_patterns):
            return True
        return any(pattern.fullmatch(geom.parent.name) for pattern in body_patterns)

    disabled_names = set()
    for geom in spec.geoms:
        if selected(geom):
            geom.contype = 0
            geom.conaffinity = 0
            disabled_names.add(geom.name)
    disabled_names.discard("")
    for pair in list(spec.pairs):
        if select_all or pair.geomname1 in disabled_names or pair.geomname2 in disabled_names:
            spec.delete(pair)
    return spec
