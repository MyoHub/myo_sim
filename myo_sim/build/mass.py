"""Mass audit and an opt-in head/trunk mass correction for composed models."""

from __future__ import annotations

import mujoco
import numpy as np

# de Leva (1996), mean of the male and female tables: head and neck as a fraction of total body mass.
HEAD_NECK_FRACTION = 0.0694
# Rajagopal/Lai torso is the first body below the lumbar vertebrae that carries the (head-inclusive) trunk mass.
TRUNK_BODY = "torso"
HEAD_BODY = "head"


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
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, HEAD_BODY) >= 0:
        out["head_fraction"] = subtree_mass(model, HEAD_BODY) / total
    return out


def apply_head_mass_correction(spec: mujoco.MjSpec, head_neck_fraction: float = HEAD_NECK_FRACTION) -> mujoco.MjSpec:
    """Move head-and-neck mass from the trunk into the ``head`` body, in place.

    The OpenSim torso mass already contains the head, and the head body adds a second, mesh-density
    mass on top, so the head is too light (about 2.5 % instead of about 6.9 % of body weight) and
    the trunk too heavy. This sets the head to ``head_neck_fraction`` of the total mass (inertia scaled
    with the mass, centre of mass kept) and removes the same mass from the trunk body (inertia scaled
    likewise), so the total mass is unchanged.

    Args:
        spec: Composed, uncompiled MjSpec containing the ``head`` and ``torso`` bodies.
        head_neck_fraction: Target head mass as a fraction of total body mass.

    Returns:
        The same spec.
    """
    model = spec.copy().compile()
    total = float(model.body_mass.sum())
    head_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, HEAD_BODY)
    head, trunk = spec.body(HEAD_BODY), spec.body(TRUNK_BODY)
    if head is None or trunk is None or head_id < 0:
        raise KeyError(f"spec needs bodies {HEAD_BODY!r} and {TRUNK_BODY!r}")
    head_mass = float(model.body_mass[head_id])
    delta = head_neck_fraction * total - head_mass
    trunk_mass = float(trunk.mass)
    if trunk_mass - delta <= 0:
        raise ValueError("trunk would lose all its mass")

    k_head = (head_mass + delta) / head_mass
    head.explicitinertial = 1
    head.mass = head_mass + delta
    head.ipos = model.body_ipos[head_id].copy()
    head.iquat = model.body_iquat[head_id].copy()
    head.inertia = model.body_inertia[head_id] * k_head
    k_trunk = (trunk_mass - delta) / trunk_mass
    trunk.mass = trunk_mass - delta
    trunk.inertia = np.asarray(trunk.inertia, dtype=float) * k_trunk
    return spec
