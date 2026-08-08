from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class FragmentInfo:
    """Metadata for a registered MJCF fragment."""

    name: str
    path: Path
    version: int
    description: str = ""


class _FragmentRegistry:
    """Registry mapping fragment names to their XML paths and versions.

    myosuite's ModelBuilder calls ``FragmentRegistry.get(name)`` as its first
    resolution step before falling back to the simhive git submodule.
    """

    def __init__(self) -> None:
        self._store: dict[str, FragmentInfo] = {}

    def get(self, name: str) -> FragmentInfo:
        """Return FragmentInfo for *name*, raising KeyError if unknown."""
        if name not in self._store:
            raise KeyError(f"Fragment {name!r} not registered. Available: {self.all_names()}")
        return self._store[name]

    def all_names(self) -> list[str]:
        """Return sorted list of all registered fragment names."""
        return sorted(self._store)

    def __contains__(self, name: str) -> bool:
        return name in self._store

    def __repr__(self) -> str:
        return f"FragmentRegistry({self.all_names()})"


FragmentRegistry = _FragmentRegistry()
