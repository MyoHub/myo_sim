from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from myo_sim.fragments import FragmentInfo, FragmentRegistry

try:
    __version__ = version("myo-sim")
except PackageNotFoundError:
    __version__ = "0.0.0+unknown"

# Root of the packaged XML model tree.
MODELS_DIR = Path(__file__).resolve().parent / "models"

# Catalog of static XML fragments: (name, rel_path, version).
# Only list models whose XML files are actually present in myo_sim/models/.
# Legacy names match myosuite's _FALLBACK_PATHS so ModelBuilder resolves them
# via FragmentRegistry before falling back to the simhive submodule.
_FRAGMENT_CATALOG: list[tuple[str, str, int]] = [
    # Static models present in this package.
    ("leg", "leg/myolegs.xml", 1),
    ("torso", "torso/myotorso.xml", 1),
    ("myolegs", "leg/myolegs.xml", 1),
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


# MjSpec-composed models: built via build_model() rather than a static XML path.
_COMPOSED_MODELS: frozenset[str] = frozenset({"hand", "myohand", "myohand_r", "myohands", "myoarm_r", "myoarms", "myofullbody"})


def load(name: str) -> tuple:
    """Load a MuJoCo model by registry name. Returns (MjModel, MjData).

    Supports both static XML models (resolved via REGISTRY) and MjSpec-composed
    models (myohand_r, myohands, myoarms, myofullbody).
    """
    import mujoco

    if name in _COMPOSED_MODELS:
        from myo_sim.build.compose import build_model

        # Legacy aliases: myohand and hand resolve to the composed right-hand model.
        composed_name = {"hand": "myohand_r", "myohand": "myohand_r"}.get(name, name)
        model = build_model(composed_name)
        return model, mujoco.MjData(model)

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
