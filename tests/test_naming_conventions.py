"""Enforce the fragment and default-class naming rules from the wiki.

See ``docs/wiki/engineering-standards.md``. These checks keep the naming
convention self-policing so it does not drift again (it has been corrected by
hand more than once). Legacy models under ``models/legacy/`` are intentionally
grandfathered and excluded.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import myo_sim

# Singular role suffixes a fragment file may end with.
ASSET_ROLES = {"chain", "muscle", "tendon", "assets"}


def _non_legacy_model_xml() -> list[Path]:
    return sorted(p for p in myo_sim.MODELS_DIR.rglob("*.xml") if "legacy" not in p.parts)


def _fragment_asset_files() -> list[Path]:
    """Part fragment files: ``models/<part>/assets/*.xml``, excluding legacy."""
    return sorted(p for p in myo_sim.MODELS_DIR.rglob("assets/*.xml") if "legacy" not in p.parts and p.name.startswith("myo"))


@pytest.mark.parametrize("path", _fragment_asset_files(), ids=lambda p: p.name)
def test_fragment_role_suffix_is_singular(path: Path):
    """Fragment files end in a singular role: chain/muscle/tendon/assets.

    Catches plural drift such as ``_muscles.xml`` or ``_tendons.xml``.
    """
    role = path.stem.rsplit("_", 1)[-1]
    assert role in ASSET_ROLES, (
        f"{path.name}: role suffix {role!r} is not one of {sorted(ASSET_ROLES)} "
        f"(use the singular form, e.g. _muscle not _muscles)"
    )


@pytest.mark.parametrize("path", _non_legacy_model_xml(), ids=lambda p: str(p.relative_to(myo_sim.MODELS_DIR)))
def test_default_class_names_are_part_scoped(path: Path):
    """Every ``<default class="...">`` is ``main`` or a ``myo<part>_...`` name.

    Rejects generic, unscoped class names (e.g. ``motor``, ``sidesite``,
    ``wrap``, ``marker``, ``muscle``) and alternate stems (e.g. ``myoBack_*``).
    """
    root = ET.parse(path).getroot()
    offenders = sorted(
        {
            cls
            for default in root.iter("default")
            if (cls := default.get("class")) is not None and cls != "main" and not cls.startswith("myo")
        }
    )
    assert not offenders, (
        f"{path.name}: unscoped default class name(s) {offenders}; scope every nested default to the part as myo<part>_<role>"
    )
