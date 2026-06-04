from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from myo_sim.fragments import FragmentInfo, FragmentRegistry

try:
    __version__ = version("myo-sim")
except PackageNotFoundError:
    __version__ = "0.0.0+unknown"

# Root of the packaged XML model tree.
MODELS_DIR = Path(__file__).resolve().parent / "models"

# Catalog of fragments: (name, rel_path, version).
# Legacy names match myosuite's _FALLBACK_PATHS so ModelBuilder resolves them
# via FragmentRegistry before falling back to the simhive submodule.
_FRAGMENT_CATALOG: list[tuple[str, str, int]] = [
    # Legacy part aliases.
    ("arm", "arm/myoarm_r.xml", 1),
    ("leg", "leg/myolegs.xml", 1),
    ("torso", "torso/myotorso.xml", 1),
    # Current static models.
    ("myoarm_r", "arm/myoarm_r.xml", 1),
    ("myolegs", "leg/myolegs.xml", 1),
    ("myotorso", "torso/myotorso.xml", 1),
]

for _name, _rel, _ver in _FRAGMENT_CATALOG:
    _p = MODELS_DIR / _rel
    if _p.exists():
        FragmentRegistry._store[_name] = FragmentInfo(name=_name, path=_p, version=_ver)

REGISTRY = {
    "myoarm_r": "arm/myoarm_r.xml",
    "myolegs": "leg/myolegs.xml",
    "myotorso": "torso/myotorso.xml",
}


def get_xml_path(name: str) -> Path:
    """Return the absolute Path to a named model XML.

    Raises ValueError for unknown names.
    """
    if name not in REGISTRY:
        raise ValueError(f"Unknown model {name!r}. Available: {sorted(REGISTRY)}")
    return MODELS_DIR / REGISTRY[name]


def load(name: str):
    """Load a MuJoCo model by registry name. Returns (MjModel, MjData)."""
    import mujoco

    xml_path = get_xml_path(name)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    return model, mujoco.MjData(model)


def print_path(name: str | None = None) -> None:
    """Print model path(s). Pass name=None to list all."""
    if name:
        print(get_xml_path(name))
    else:
        for k in sorted(REGISTRY):
            print(f"{k}: {MODELS_DIR / REGISTRY[k]}")
