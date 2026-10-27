"""Standard motion-capture marker sets as optional site overlays.

Each set is an MJCF file in ``models/markersets/`` with one ``<site>`` per marker inside a ``<body>`` of the same name as
the model body it belongs to, positioned in that body's frame. ``add_markerset`` copies those sites onto a composed,
uncompiled spec; the registered models themselves never include them.
"""

from __future__ import annotations

from pathlib import Path

import mujoco

MARKERSET_DIR = Path(__file__).resolve().parent.parent / "models" / "markersets"


def list_markersets() -> list[str]:
    """Names of the packaged marker sets."""
    return sorted(p.stem for p in MARKERSET_DIR.glob("*.xml"))


def markerset_path(name: str) -> Path:
    """Path of a packaged marker-set overlay."""
    path = MARKERSET_DIR / f"{name}.xml"
    if not path.exists():
        raise KeyError(f"unknown marker set {name!r}; available: {list_markersets()}")
    return path


def add_markerset(spec: mujoco.MjSpec, name: str | Path, prefix: str = "", skip_missing: bool = False) -> list[str]:
    """Add the sites of a marker-set overlay to the bodies of ``spec`` with the same names.

    Args:
        spec: Uncompiled spec of the model.
        name: Packaged set name or path to an overlay file.
        prefix: Prefix for the added site names (e.g. to combine two sets).
        skip_missing: Skip markers whose body is not in the model (partial models) instead of raising.

    Returns:
        Names of the skipped markers.
    """
    path = Path(name) if Path(name).suffix == ".xml" else markerset_path(str(name))
    overlay = mujoco.MjSpec.from_file(str(path))
    skipped = []
    for site in overlay.sites:
        body = spec.body(site.parent.name)
        if body is None:
            if not skip_missing:
                raise KeyError(f"marker {site.name!r}: body {site.parent.name!r} is not in the model")
            skipped.append(site.name)
            continue
        body.add_site(name=prefix + site.name, type=site.type, pos=site.pos, size=site.size, rgba=site.rgba, group=site.group)
    return skipped
