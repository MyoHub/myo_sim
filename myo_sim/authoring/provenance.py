"""Canonical chain ownership for explicitly verified registered model exports."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any

import mujoco


def _provenance(
    model: mujoco.MjModel, source_model: str | None, fingerprint: str
) -> tuple[dict[str, Any], dict[str, dict[str, dict[str, Any]]]]:
    """Describe ownership only when the supplied model matches a registered source."""
    from myo_sim import MODELS_DIR, load_spec
    from myo_sim.build.compose import ALIASES, MODEL_REGISTRY
    from myo_sim.build.utils import MirrorRules, mirror_element

    from .interchange import model_fingerprint

    provenance = dict(
        scope="instance",
        status="unknown",
        source_model=None,
        reason="No registered source_model was supplied",
        guidance=(
            "Selected patches customize this instance. For a canonical contribution, edit the reported source chain "
            "and rebuild through composition and mirroring; generated-left edits are not written back automatically."
        ),
    )
    owners: dict[str, dict[str, dict[str, Any]]] = {"body": {}, "site": {}}
    if source_model is None:
        return provenance, owners
    if not isinstance(source_model, str) or not source_model:
        raise ValueError("source_model must be a nonempty registered model name")
    canonical = ALIASES.get(source_model, source_model)
    reference = load_spec(canonical).compile()
    if fingerprint != model_fingerprint(reference):
        provenance["reason"] = "The model fingerprint differs from the registered source_model"
        return provenance, owners
    provenance.update(status="verified", source_model=canonical, reason=None)
    registration = MODEL_REGISTRY.get(canonical)
    rules = registration.mirror_rules if registration is not None else MirrorRules()
    available = {
        "body": {model.body(i).name for i in range(model.nbody)},
        "site": {model.site(i).name for i in range(model.nsite)},
    }
    candidates: dict[str, dict[str, list[dict[str, Any]]]] = {"body": {}, "site": {}}
    # Read declarations themselves, rather than maintaining another model/part registry.
    for path in sorted(MODELS_DIR.rglob("*_chain.xml")):
        if "legacy" in path.relative_to(MODELS_DIR).parts:
            continue
        source = ET.parse(path).getroot()
        variants = [(source, False)]
        if path.name.endswith("_r_chain.xml"):
            variants.append((mirror_element(source, rules), True))
        for tree, generated_left in variants:
            declared = [(e.tag, e.get("name")) for e in tree.iter() if e.tag in available and e.get("name")]
            if not declared or not all(name in available[kind] for kind, name in declared):
                continue
            originals = [e for e in source.iter() if e.tag in available and e.get("name")]
            for (kind, name), original in zip(declared, originals):
                candidates[kind].setdefault(name, []).append(
                    dict(
                        status="canonical",
                        chain_fragment=path.relative_to(MODELS_DIR).as_posix(),
                        canonical_name=original.get("name"),
                        generated_left=generated_left,
                    )
                )
    for kind, entries in candidates.items():
        owners[kind] = {name: matches[0] for name, matches in entries.items() if len(matches) == 1}
    return provenance, owners


def _unknown_origin() -> dict[str, Any]:
    return dict(status="unknown", chain_fragment=None, canonical_name=None, generated_left=None)
