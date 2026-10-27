from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import TYPE_CHECKING, Any, Optional

from myo_sim.fragments import FragmentInfo as FragmentInfo
from myo_sim.fragments import FragmentRegistry as FragmentRegistry

if TYPE_CHECKING:
    import mujoco

    from myo_sim.build.compose import CollisionMode as CollisionMode

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


def _composed_models() -> frozenset[str]:
    """Names loadable via the MjSpec compose pipeline (registry keys + aliases).

    Derived from ``MODEL_REGISTRY`` and ``ALIASES`` so there is a single source
    of truth; importing compose here (rather than at module top) keeps
    ``import myo_sim`` free of a MuJoCo dependency.
    """
    from myo_sim.build.compose import ALIASES, MODEL_REGISTRY

    return frozenset(MODEL_REGISTRY) | frozenset(ALIASES)


def __getattr__(name: str):
    # Preserve the historical ``myo_sim._COMPOSED_MODELS`` attribute while keeping
    # its contents derived from the compose registry instead of a hand-kept copy.
    if name == "_COMPOSED_MODELS":
        return _composed_models()
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def _right_hand_spec():
    from myo_sim.build.compose import build_right_hand_from_arm_spec

    return build_right_hand_from_arm_spec()


def _left_hand_spec():
    from myo_sim.build.compose import build_left_hand_from_arm_spec

    return build_left_hand_from_arm_spec()


def _legs_spec():
    from myo_sim.build.compose import build_legs_spec

    return build_legs_spec()


def _legs26_spec():
    from myo_sim.build.compose import build_legs26_spec

    return build_legs26_spec()


# TODO: there should be a myohand_l alias, which should work with load. Also a given name (myolegs) should refer to the
#       same model consistently.
# Maps fragment names to zero-arg callables returning an editable (uncompiled)
# MjSpec, with no torso scaffold.  Used by downstream consumers (e.g. myosuite
# ModelBuilder, assist_sim) so that attaching a fragment routes through the compose
# pipeline instead of falling back to a bundled static XML.  Includes myohand_l
# (left hand only); there is no myo_sim.load("myohand_l") alias — use this dict or
# myohands instead.  Note "myolegs" here is the bare legs fragment; myo_sim.load(
# "myolegs") differs in that it attaches the legs to a passive torso scaffold.
FRAGMENT_SPEC_BUILDERS: dict[str, object] = {
    "hand": _right_hand_spec,
    "myohand": _right_hand_spec,
    "myohand_r": _right_hand_spec,
    "myohand_l": _left_hand_spec,
    "myolegs": _legs_spec,
    "myolegs26": _legs26_spec,
}


def load(name: str) -> tuple:
    """Load a MuJoCo model by registry name. Returns (MjModel, MjData).

    Supports MjSpec-composed models such as myohand_r, myohands, myoarms,
    myofullbody, and myolegs, plus packaged legacy static XMLs. For a
    left-hand-only MjSpec builder, use FRAGMENT_SPEC_BUILDERS["myohand_l"].
    """
    import mujoco

    model = load_model(name)
    return model, mujoco.MjData(model)


def load_model(name: str) -> tuple:
    spec = load_spec(name)
    model = spec.compile()
    return model


def load_spec(
    name: str,
    build_kwargs: Optional[dict[str, Any]] = None,
    inertia_floor: float | None = None,
) -> "mujoco.MjSpec":
    """Build and return the uncompiled MjSpec for a named model.

    Unlike load(), this stops at the editable MjSpec so downstream consumers
    (e.g. assist_sim) can edit the model -- attach devices, delete bodies, add
    actuators -- before compiling it themselves.

    Args:
        name: Registry name or legacy alias (e.g. "myotorso_arms", "hand").
        build_kwargs: Passed along to builder function.
                      TODO: Raise error on unused kwarg.
        inertia_floor: Optional numerical-conditioning floor for
            auto/mesh-derived body inertia (sets the compiler's
            ``boundinertia``/``boundmass``). None (default, unchanged
            behavior) applies no floor. Pass ``0.0001`` to restore the
            legacy static-XML convention that several small wrist/finger
            bones in the right-hand fragment relied on for a
            well-conditioned mass matrix -- see
            myo_sim.build.compose.build_spec() and
            https://github.com/MyoHub/myo_sim/issues/128. Only supported for
            MjSpec-composed models; ignored for packaged static-XML
            fragments.
    """
    import mujoco

    from myo_sim.build.compose import ALIASES, build_spec

    build_kwargs = build_kwargs or {}
    composed_models = _composed_models()
    if name in composed_models:
        # Legacy aliases (e.g. hand, myohand, myoarm) resolve to a registry entry.
        composed_name = ALIASES.get(name, name)
        spec = build_spec(
            composed_name,
            inertia_floor=inertia_floor,
            build_kwargs=build_kwargs,
        )
        return spec

    if name not in REGISTRY:
        available = sorted(composed_models | frozenset(REGISTRY))
        raise ValueError(f"Unknown model {name!r}. Available: {available}")

    spec = mujoco.MjSpec.from_file(str(get_xml_path(name)))
    return spec


def get_path(rel: str) -> Path:
    """Return the absolute Path to a model file by relative path within MODELS_DIR.

    This is a convenience wrapper for scripts and tutorials that prefer to
    reference models by relative path (e.g. ``"scene/myosuite_scene.xml"``)
    rather than registry name.

    Args:
        rel: Relative path inside MODELS_DIR, e.g. ``"scene/myosuite_scene.xml"``.

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
