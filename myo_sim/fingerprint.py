"""Stable hash of the musculoskeletal content of a compiled model."""

from __future__ import annotations

import hashlib
import json
from typing import Any

import mujoco
import numpy as np

from myo_sim.physics.joint_coupling import project_tendon_nodes
from myo_sim.physics.muscle_strength import peak_force


def musculoskeletal_fingerprint(model: mujoco.MjModel) -> str:
    """Return a 16-hex SHA-256 of the musculoskeletal content of *model*.

    Hashes kinematics, inertias, active equalities, contact pairs, tendons, actuators
    and the actuator moment arms at ``qpos0`` (tendon-node slides projected). Visual
    properties (geom rgba, materials) do not enter.

    Args:
        model: Compiled MuJoCo model.

    Returns:
        First 16 hex digits of the SHA-256 of the rounded (1e-6) model document.
    """

    def r(x: Any) -> list[int]:
        arr = np.clip(
            np.nan_to_num(np.asarray(x, np.float64), nan=0.0, posinf=1e9, neginf=-1e9),
            -1e9,
            1e9,
        )
        return np.rint(arr * 1e6).astype(np.int64).ravel().tolist()

    def obj_name(obj: mujoco.mjtObj, idx: int) -> str | None:
        return mujoco.mj_id2name(model, obj, int(idx)) if idx >= 0 else None

    j_obj, b_obj = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY
    t_obj, a_obj = mujoco.mjtObj.mjOBJ_TENDON, mujoco.mjtObj.mjOBJ_ACTUATOR
    g_obj = mujoco.mjtObj.mjOBJ_GEOM

    data = mujoco.MjData(model)
    data.qpos[:] = project_tendon_nodes(model, model.qpos0)
    mujoco.mj_fwdPosition(model, data)
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(
        moment,
        data.actuator_moment,
        data.moment_rownnz,
        data.moment_rowadr,
        data.moment_colind,
    )
    peaks = peak_force(model)
    joints = sorted(range(model.njnt), key=lambda j: int(model.jnt_qposadr[j]))
    doc = {
        "joints": [
            [
                obj_name(j_obj, j),
                int(model.jnt_type[j]),
                int(model.jnt_qposadr[j]),
                int(model.jnt_dofadr[j]),
                obj_name(b_obj, model.jnt_bodyid[j]),
                r(model.jnt_pos[j]),
                r(model.jnt_axis[j]),
                int(model.jnt_limited[j]),
                r(model.jnt_range[j]),
                r(model.jnt_stiffness[j]),
            ]
            for j in joints
        ],
        "dofs": [
            r(model.dof_damping),
            r(model.dof_armature),
            r(model.dof_frictionloss),
        ],
        "qpos0": r(model.qpos0),
        "qpos_spring": r(model.qpos_spring),
        "bodies": [
            [
                obj_name(b_obj, b),
                int(model.body_parentid[b]),
                r(model.body_pos[b]),
                r(model.body_quat[b]),
                r(model.body_mass[b]),
                r(model.body_ipos[b]),
                r(model.body_iquat[b]),
                r(model.body_inertia[b]),
            ]
            for b in range(model.nbody)
        ],
        "couplings": [
            [
                int(model.eq_type[e]),
                int(model.eq_obj1id[e]),
                int(model.eq_obj2id[e]),
                r(model.eq_data[e, :5]),
            ]
            for e in range(model.neq)
            if model.eq_active0[e]
        ],
        "pairs": [
            [
                obj_name(g_obj, model.pair_geom1[p]),
                obj_name(g_obj, model.pair_geom2[p]),
                r(model.pair_margin[p]),
            ]
            for p in range(model.npair)
        ],
        "tendons": [
            [
                obj_name(t_obj, t),
                int(model.tendon_limited[t]),
                r(model.tendon_range[t]),
                r(model.tendon_stiffness[t]),
                r(model.tendon_damping[t]),
                r(model.tendon_lengthspring[t]),
                r(model.tendon_length0[t]),
            ]
            for t in range(model.ntendon)
        ],
        "actuators": [
            [
                obj_name(a_obj, a),
                int(model.actuator_trntype[a]),
                int(model.actuator_dyntype[a]),
                int(model.actuator_gaintype[a]),
                int(model.actuator_biastype[a]),
                model.actuator_trnid[a].tolist(),
                r(model.actuator_dynprm[a]),
                r(model.actuator_gainprm[a]),
                r(model.actuator_biasprm[a]),
                r(model.actuator_lengthrange[a]),
                r(model.actuator_acc0[a]),
                r(model.actuator_gear[a]),
                r(peaks[a]),
            ]
            for a in range(model.nu)
        ],
        "moment_qpos0": r(moment),
    }
    blob = json.dumps(doc, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()[:16]
