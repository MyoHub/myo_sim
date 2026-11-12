# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Versioned, editor-independent interchange of compiled MuJoCo anatomy."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings

from .provenance import _provenance, _unknown_origin

SCHEMA_VERSION = 2


def _peak_force(model: mujoco.MjModel) -> np.ndarray:
    """Resolve MuJoCo's automatic muscle peak force from scale and acceleration."""
    force = np.array(model.actuator_gainprm[:, 2], dtype=np.float64)
    auto = (force < 0) & (model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE)
    force[auto] = model.actuator_gainprm[auto, 3] / np.maximum(model.actuator_acc0[auto], mujoco.mjMINVAL)
    return force


def model_fingerprint(model: mujoco.MjModel) -> str:
    """Hash the complete compiled model, including embedded meshes and options.

    Args:
        model: Source model.

    Returns:
        SHA-256 of its MuJoCo-version-specific binary representation.
    """
    buffer = np.empty(mujoco.mj_sizeModel(model), dtype=np.uint8)
    mujoco.mj_saveModel(model, buffer=buffer)
    return hashlib.sha256(buffer.tobytes()).hexdigest()


def _names(model: mujoco.MjModel, kind: str, count: int) -> list[str]:
    return [getattr(model, kind)(i).name or f"{kind}_{i}" for i in range(count)]


def _quat(matrix: np.ndarray) -> list[float]:
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, matrix)
    return quaternion.tolist()


def _inertia(diagonal: np.ndarray, quaternion: np.ndarray) -> list[list[float]]:
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quaternion)
    rotation = rotation.reshape(3, 3)
    return (rotation @ np.diag(diagonal) @ rotation.T).tolist()


def _wrap_flags(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    """Report inside sidesites in native pose coordinates, independent of any editor."""
    flags = []
    for tendon in range(model.ntendon):
        start = int(model.tendon_adr[tendon])
        for wrap in range(start, start + int(model.tendon_num[tendon])):
            if model.wrap_type[wrap] not in (mujoco.mjtWrap.mjWRAP_SPHERE, mujoco.mjtWrap.mjWRAP_CYLINDER):
                continue
            side = int(model.wrap_prm[wrap])
            if side < 0:
                continue
            geom = int(model.wrap_objid[wrap])
            local = data.geom_xmat[geom].reshape(3, 3).T @ (data.site_xpos[side] - data.geom_xpos[geom])
            radial = local[:2] if model.wrap_type[wrap] == mujoco.mjtWrap.mjWRAP_CYLINDER else local
            flags.append(
                dict(
                    tendon=tendon,
                    element=wrap - start,
                    geom=geom,
                    sidesite=side,
                    sidesite_inside=bool(radial @ radial < model.geom_size[geom, 0] ** 2),
                )
            )
    return flags


def export_model(
    model: mujoco.MjModel,
    output: str | Path,
    *,
    qpos: np.ndarray | None = None,
    source_model: str | None = None,
) -> dict[str, Any]:
    """Export resolved anatomy, original paths and MuJoCo-evaluated poses to JSON.

    Args:
        model: Compiled source model; it is never modified.
        output: Destination JSON file. Mesh vertices are embedded.
        qpos: Optional ``(frames, nq)`` poses. Joint equalities are projected with
            the shared coupling implementation before evaluating each pose.
        source_model: Optional registered model name. Canonical chain ownership
            is reported only when that source's complete fingerprint matches.
            Custom or unmatched models retain explicitly unknown origins.

    Returns:
        The JSON-compatible document. Coordinates are metres and radians,
        quaternions are wxyz, and inertias are about the COM in body axes.

    Raises:
        ValueError: Invalid poses or unsupported flex/skin/heightfield/SDF geometry.
    """
    if model.nflex or model.nskin:
        raise ValueError("Flex and skin geometry cannot be represented by this anatomy format")
    poses = np.asarray(model.qpos0[None] if qpos is None else qpos, dtype=float)
    if poses.ndim != 2 or poses.shape[1] != model.nq or not len(poses):
        raise ValueError(f"qpos must have nonempty shape (frames, {model.nq})")
    if not np.isfinite(poses).all():
        raise ValueError("qpos must be finite")
    names = {
        kind: _names(model, kind, count)
        for kind, count in (
            ("body", model.nbody),
            ("joint", model.njnt),
            ("site", model.nsite),
            ("geom", model.ngeom),
            ("actuator", model.nu),
            ("tendon", model.ntendon),
        )
    }
    muscles = np.flatnonzero(model.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE)
    if any(model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_TENDON for a in muscles):
        raise ValueError("Muscles without tendon transmission are unsupported")
    force = _peak_force(model)
    fingerprint = model_fingerprint(model)
    provenance, owners = _provenance(model, source_model, fingerprint)
    bodies = [
        dict(
            id=i,
            name=names["body"][i],
            source_name=model.body(i).name,
            origin=owners["body"].get(model.body(i).name, _unknown_origin()),
            parent=int(model.body_parentid[i]),
            mass=float(model.body_mass[i]),
            com=model.body_ipos[i].tolist(),
            inertia=_inertia(model.body_inertia[i], model.body_iquat[i]),
        )
        for i in range(model.nbody)
    ]
    joints = [
        dict(
            id=i,
            name=names["joint"][i],
            body=int(model.jnt_bodyid[i]),
            type=mujoco.mjtJoint(int(model.jnt_type[i])).name.removeprefix("mjJNT_"),
            pos=model.jnt_pos[i].tolist(),
            axis=model.jnt_axis[i].tolist(),
            qpos_address=int(model.jnt_qposadr[i]),
            limited=bool(model.jnt_limited[i]),
            range=model.jnt_range[i].tolist(),
        )
        for i in range(model.njnt)
    ]
    sites = [
        dict(
            id=i,
            name=names["site"][i],
            source_name=model.site(i).name,
            origin=owners["site"].get(model.site(i).name, _unknown_origin()),
            body=int(model.site_bodyid[i]),
            pos=model.site_pos[i].tolist(),
        )
        for i in range(model.nsite)
    ]
    tendons = []
    for t in range(model.ntendon):
        elements = []
        for w in range(model.tendon_adr[t], model.tendon_adr[t] + model.tendon_num[t]):
            element = dict(
                type=mujoco.mjtWrap(int(model.wrap_type[w])).name.removeprefix("mjWRAP_"),
                object=int(model.wrap_objid[w]),
                parameter=float(model.wrap_prm[w]),
            )
            if model.wrap_type[w] in (mujoco.mjtWrap.mjWRAP_SPHERE, mujoco.mjtWrap.mjWRAP_CYLINDER):
                element["sidesite"] = int(model.wrap_prm[w]) if model.wrap_prm[w] >= 0 else None
            elements.append(element)
        tendons.append(dict(id=t, name=names["tendon"][t], elements=elements))
    sidesite_tendons = {t["id"] for t in tendons if any(e.get("sidesite") is not None for e in t["elements"])}
    actuators = [
        dict(
            id=int(a),
            name=names["actuator"][a],
            tendon=int(model.actuator_trnid[a, 0]),
            has_sidesite_constraints=int(model.actuator_trnid[a, 0]) in sidesite_tendons,
            force=float(force[a]),
            gear=model.actuator_gear[a].tolist(),
            gain=model.actuator_gainprm[a].tolist(),
            bias=model.actuator_biasprm[a].tolist(),
            dynamics=model.actuator_dynprm[a].tolist(),
            lengthrange=model.actuator_lengthrange[a].tolist(),
        )
        for a in muscles
    ]
    colors = model.geom_rgba.copy()
    # MuJoCo uses the material RGBA only when all four geom values equal its default.
    uses_material = (model.geom_matid >= 0) & np.all(colors == [0.5, 0.5, 0.5, 1.0], axis=1)
    colors[uses_material] = model.mat_rgba[model.geom_matid[uses_material]]
    geoms = []
    for i in range(model.ngeom):
        geom = dict(
            id=i,
            name=names["geom"][i],
            body=int(model.geom_bodyid[i]),
            type=mujoco.mjtGeom(int(model.geom_type[i])).name.removeprefix("mjGEOM_"),
            size=model.geom_size[i].tolist(),
            rgba=colors[i].tolist(),
        )
        if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[i]
            va, vn = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            fa, fn = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            geom.update(
                vertices=model.mesh_vert[va : va + vn].tolist(),
                faces=model.mesh_face[fa : fa + fn].tolist(),
            )
        elif model.geom_type[i] == mujoco.mjtGeom.mjGEOM_HFIELD:
            raise ValueError("Heightfields are unsupported; export an anatomy-only model")
        elif model.geom_type[i] == mujoco.mjtGeom.mjGEOM_SDF:
            raise ValueError("SDF geometry is unsupported; export an anatomy-only model")
        geoms.append(geom)
    equalities = [
        dict(
            id=i,
            type=mujoco.mjtEq(int(model.eq_type[i])).name.removeprefix("mjEQ_"),
            object_type=mujoco.mjtObj(int(model.eq_objtype[i])).name.removeprefix("mjOBJ_"),
            active=bool(model.eq_active0[i]),
            object1=int(model.eq_obj1id[i]),
            object2=int(model.eq_obj2id[i]),
            data=model.eq_data[i].tolist(),
        )
        for i in range(model.neq)
    ]
    data, coupling = mujoco.MjData(model), JointCouplings.from_model(model)
    moment = np.empty((len(muscles), model.nv))
    frames = []
    for pose in poses:
        data.qpos[:] = pose
        mujoco.mj_normalizeQuat(model, data.qpos)
        project_joint_couplings(coupling, data.qpos)
        mujoco.mj_forward(model, data)
        if len(muscles):
            # Selected row addresses still refer to the full native sparse storage.
            mujoco.mju_sparse2dense(
                moment,
                data.actuator_moment,
                data.moment_rownnz[muscles],
                data.moment_rowadr[muscles],
                data.moment_colind,
            )
        frames.append(
            dict(
                qpos=data.qpos.tolist(),
                body_pos=data.xpos.tolist(),
                body_quat=data.xquat.tolist(),
                geom_pos=data.geom_xpos.tolist(),
                geom_quat=[_quat(r) for r in data.geom_xmat],
                site_pos=data.site_xpos.tolist(),
                muscle_length=data.actuator_length[muscles].tolist(),
                muscle_tendon_length=data.ten_length[model.actuator_trnid[muscles, 0]].tolist(),
                moment_arms=moment.tolist(),
                wrap_flags=_wrap_flags(model, data),
            )
        )
    warnings: dict[tuple[int, ...], dict[str, Any]] = {}
    for index, frame in enumerate(frames):
        for flag in frame["wrap_flags"]:
            if flag["sidesite_inside"]:
                key = tuple(flag[field] for field in ("tendon", "element", "geom", "sidesite"))
                if key not in warnings:
                    warnings[key] = dict(
                        code="inside_sidesite",
                        **{field: flag[field] for field in ("tendon", "element", "geom", "sidesite")},
                        frames=[],
                        message=(
                            "A sidesite lies inside its wrap geometry; external previews may choose a different path. "
                            "MuJoCo tendon lengths and moment arms remain authoritative."
                        ),
                    )
                warnings[key]["frames"].append(index)
    document = dict(
        schema=SCHEMA_VERSION,
        patch_schema=1,
        fingerprint=fingerprint,
        mujoco_version=mujoco.__version__,
        provenance=provenance,
        warnings=list(warnings.values()),
        units="metres-radians-kilograms",
        bodies=bodies,
        joints=joints,
        sites=sites,
        tendons=tendons,
        muscles=actuators,
        geoms=geoms,
        equalities=equalities,
        qpos0=model.qpos0.tolist(),
        frames=frames,
    )
    Path(output).write_text(json.dumps(document, allow_nan=False) + "\n", encoding="utf-8")
    return document


def joint_sweep(model: mujoco.MjModel, joint: str, samples: int = 21) -> np.ndarray:
    """Return poses spanning a limited scalar joint's range, for geometry checks.

    Args:
        model: Source model.
        joint: Name of a limited hinge or slide joint.
        samples: Number of evenly spaced poses, at least two.

    Returns:
        ``(samples, nq)`` poses; export projects dependent coordinates.
    """
    j = model.joint(joint)
    if j.type[0] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
        raise ValueError("A sweep requires a scalar hinge or slide joint")
    if not j.limited[0] or samples < 2:
        raise ValueError("A sweep requires a limited joint and at least two samples")
    if any(e["object1"] == j.id and e["active"] for e in _joint_equalities(model)):
        raise ValueError("Sweep an independent joint, not a dependent coordinate")
    qpos = np.tile(model.qpos0, (samples, 1))
    qpos[:, j.qposadr[0]] = np.linspace(*j.range, samples)
    return qpos


def _joint_equalities(model: mujoco.MjModel) -> list[dict[str, Any]]:
    return [
        dict(object1=int(model.eq_obj1id[i]), active=bool(model.eq_active0[i]))
        for i in range(model.neq)
        if model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT
    ]
