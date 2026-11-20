"""Warn when an installed mujoco-warp gets the arm and hand tendon paths wrong."""

from __future__ import annotations

import warnings
from importlib.metadata import PackageNotFoundError, version

# google-deepmind/mujoco_warp#1713, fixed in 3.16. See models/arm/README.md.
MIN_MUJOCO_WARP = (3, 16)
_AFFECTED_MODEL_PARTS = ("arm", "hand", "fullbody")
_warned = False


def _installed_version() -> tuple[int, int] | None:
    """Return mujoco-warp's (major, minor), or None if it is absent or not numeric."""
    try:
        major, minor = version("mujoco-warp").split(".")[:2]
        return int(major), int(minor)
    except (PackageNotFoundError, ValueError):
        return None


def warn_if_old_mujoco_warp(model_name: str) -> None:
    """Warn once per process if *model_name* has tendons that mujoco-warp < 3.16 misplaces.

    Args:
        model_name: Registry name of the model being loaded.
    """
    global _warned
    if _warned or not any(part in model_name for part in _AFFECTED_MODEL_PARTS):
        return
    installed = _installed_version()
    if installed is None or installed >= MIN_MUJOCO_WARP:
        return
    _warned = True
    warnings.warn(
        f"mujoco-warp {installed[0]}.{installed[1]} is installed. Before "
        f"{MIN_MUJOCO_WARP[0]}.{MIN_MUJOCO_WARP[1]}, arm and hand tendons with a sidesite inside "
        "their wrap geom can be up to about 3 mm longer on the GPU than in CPU MuJoCo "
        "(google-deepmind/mujoco_warp#1713). Upgrade if you train on mujoco-warp.",
        stacklevel=3,
    )
