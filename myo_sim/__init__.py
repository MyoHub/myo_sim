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
    # Legacy part aliases — names match myosuite's _FALLBACK_PATHS.
    ("elbow", "elbow/myoelbow_2dof6muscles.xml", 1),
    ("finger", "finger/myofinger_v0.xml", 1),
    ("hand", "hand/myohand.xml", 1),
    ("shoulder", "arm/myoarm_r.xml", 1),
    ("arm", "arm/myoarm_r.xml", 1),
    ("leg", "leg/myolegs.xml", 1),
    ("osl", "osl/myolegs_osl.xml", 1),
    ("body", "body/myobody.xml", 1),
    ("torso", "torso/myotorso.xml", 1),
    # Current static models (canonical names).
    ("myoelbow", "elbow/myoelbow_2dof6muscles.xml", 1),
    ("myofinger", "finger/myofinger_v0.xml", 1),
    ("myohand", "hand/myohand.xml", 1),
    ("myoarm", "arm/myoarm_r.xml", 1),
    ("myolegs", "leg/myolegs.xml", 1),
    ("myobody", "body/myobody.xml", 1),
    ("myotorso", "torso/myotorso.xml", 1),
]

for _name, _rel, _ver in _FRAGMENT_CATALOG:
    _p = MODELS_DIR / _rel
    if _p.exists():
        FragmentRegistry._store[_name] = FragmentInfo(name=_name, path=_p, version=_ver)

# Derived from _FRAGMENT_CATALOG so adding a model requires editing one place.
REGISTRY = {name: rel for name, rel, _ in _FRAGMENT_CATALOG}


def get_xml_path(name: str) -> Path:
    """Return the absolute Path to a named model XML.

    Raises ValueError for unknown names.
    """
    if name not in REGISTRY:
        raise ValueError(f"Unknown model {name!r}. Available: {sorted(REGISTRY)}")
    return MODELS_DIR / REGISTRY[name]


def load(name: str) -> tuple:
    """Load a MuJoCo model by registry name. Returns (MjModel, MjData)."""
    import mujoco

    xml_path = get_xml_path(name)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    return model, mujoco.MjData(model)


def get_path(rel: str) -> Path:
    """Return the absolute Path to a model file by relative path within MODELS_DIR.

    This is a convenience wrapper for scripts and tutorials that prefer to
    reference models by relative path (e.g. ``"arm/myoarm.xml"``) rather than
    registry name.

    Args:
        rel: Relative path inside MODELS_DIR, e.g. ``"arm/myoarm.xml"``.

    Returns:
        Absolute Path to the file.

    Raises:
        FileNotFoundError: If the path does not exist.
    """
    p = MODELS_DIR / rel
    if not p.exists():
        raise FileNotFoundError(f"Model file not found: {p}")
    return p


def print_path(name: str | None = None) -> None:
    """Print model path(s). Pass name=None to list all."""
    if name:
        print(get_xml_path(name))
    else:
        for k in sorted(REGISTRY):
            print(f"{k}: {MODELS_DIR / REGISTRY[k]}")
