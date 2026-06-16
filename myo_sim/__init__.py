from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from myo_sim.fragments import FragmentInfo as FragmentInfo, FragmentRegistry as FragmentRegistry

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
    # Legacy models — pending proper migration (see legacy/README.md).
    ("elbow", "legacy/elbow/myoelbow_1dof6muscles.xml", 1),
    ("myoelbow", "legacy/elbow/myoelbow_1dof6muscles.xml", 1),
    ("myoelbow_2dof", "legacy/elbow/myoelbow_2dof6muscles.xml", 1),
    ("myoelbow_exo", "legacy/elbow/myoelbow_1dof6muscles_1dofexo.xml", 1),
    ("finger", "legacy/finger/myofinger_v0.xml", 1),
    ("myofinger", "legacy/finger/myofinger_v0.xml", 1),
    ("osl", "legacy/osl/myolegs_osl.xml", 1),
    ("myolegs_osl", "legacy/osl/myolegs_osl.xml", 1),
]

REGISTRY: dict[str, str] = {}

for _name, _rel, _ver in _FRAGMENT_CATALOG:
    _p = MODELS_DIR / _rel
    if _p.exists():
        FragmentRegistry._store[_name] = FragmentInfo(name=_name, path=_p, version=_ver)
        REGISTRY[_name] = _rel


def get_xml_path(name: str) -> Path:
    """Return the absolute Path to a named model XML.

    Raises ValueError for unknown names.
    """
    if name not in REGISTRY:
        raise ValueError(f"Unknown model {name!r}. Available: {sorted(REGISTRY)}")
    return MODELS_DIR / REGISTRY[name]


# MjSpec-composed models: built via build_model() rather than a static XML path.
_COMPOSED_MODELS: frozenset[str] = frozenset(
    {
        "hand",
        "myohand",
        "myohand_r",
        "myohands",
        "myoarm_r",
        "myoarms",
        "myofullbody",
        "myolegs",
        "myolegs_abdomen",
        "myotorso",
        "myotorso_abdomen",
        "myotorso_arm_r",
        "myotorso_arms",
    }
)


def _right_hand_spec():
    from myo_sim.build.compose import load_right_hand_from_arm_spec

    return load_right_hand_from_arm_spec()


def _left_hand_spec():
    from myo_sim.build.compose import load_left_hand_from_arm_spec

    return load_left_hand_from_arm_spec()


# Maps fragment names to zero-arg callables returning MjSpec (hand-only, no torso
# scaffold).  Used by downstream consumers (e.g. myosuite ModelBuilder) so that
# attach_fragment("hand") transparently routes through the compose pipeline instead
# of falling back to a bundled static XML.
_FRAGMENT_SPEC_BUILDERS: dict[str, object] = {
    "hand": _right_hand_spec,
    "myohand": _right_hand_spec,
    "myohand_r": _right_hand_spec,
    "myohand_l": _left_hand_spec,
}


def load(name: str) -> tuple:
    """Load a MuJoCo model by registry name. Returns (MjModel, MjData).

    Supports MjSpec-composed models such as myohand_r, myohands, myoarms,
    myofullbody, and myolegs, plus packaged legacy static XMLs.
    """
    import mujoco

    if name in _COMPOSED_MODELS:
        from myo_sim.build.compose import build_model

        # Legacy aliases: myohand and hand resolve to the composed right-hand model.
        composed_name = {"hand": "myohand_r", "myohand": "myohand_r"}.get(name, name)
        model = build_model(composed_name)
        return model, mujoco.MjData(model)

    if name not in REGISTRY:
        available = sorted(_COMPOSED_MODELS | frozenset(REGISTRY))
        raise ValueError(f"Unknown model {name!r}. Available: {available}")

    model = mujoco.MjModel.from_xml_path(str(get_xml_path(name)))
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
    """Print packaged model file paths."""
    if name:
        print(get_path(name))
    else:
        print(MODELS_DIR)
