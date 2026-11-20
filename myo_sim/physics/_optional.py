"""Optional array backends (torch, jax) of :mod:`myo_sim.physics`, imported on first use."""

from __future__ import annotations

import importlib
from types import ModuleType


def require(module: str, extra: str) -> ModuleType:
    """Import *module*, or raise an ImportError that names the myo-sim extra providing it.

    Raises:
        ImportError: If *module* is not installed.
    """
    try:
        return importlib.import_module(module)
    except ImportError as err:
        raise ImportError(f'{module} is needed here; install it with pip install "myo-sim[{extra}]"') from err
