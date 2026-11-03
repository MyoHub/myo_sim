"""Mass audit helpers for compiled models."""

from __future__ import annotations

import mujoco
import numpy as np

# De Leva head segment root: the muscular neck chain (skull + cervical segments), else the rigid head.
HEAD_ROOTS = ("myohead_base", "head")


def subtree_mass(model: mujoco.MjModel, root: str) -> float:
    """Total mass (kg) of ``root`` and all its descendant bodies."""
    rid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root)
    if rid < 0:
        raise KeyError(f"unknown body {root!r}")
    total = 0.0
    for b in range(model.nbody):
        c = b
        while c > 0 and c != rid:
            c = int(model.body_parentid[c])
        if c == rid:
            total += float(model.body_mass[b])
    return total


def audit_mass(model: mujoco.MjModel) -> dict[str, float | list[str]]:
    """Summarise total mass, head fraction and inertia-triangle violations of a compiled model."""
    total = float(model.body_mass.sum())
    violations = []
    for b in range(1, model.nbody):
        i = np.sort(model.body_inertia[b])
        if model.body_mass[b] > 0 and i[2] > i[0] + i[1] + 1e-12:
            violations.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b))
    out: dict[str, float | list[str]] = {"total_mass": total, "triangle_violations": violations}
    root = next((r for r in HEAD_ROOTS if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, r) >= 0), None)
    if root is not None:
        out["head_fraction"] = subtree_mass(model, root) / total
    return out
