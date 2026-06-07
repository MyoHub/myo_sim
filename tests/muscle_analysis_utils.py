"""Utilities for analyzing muscle symmetry via moment arm and force-length curves."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import matplotlib.pyplot as plt


def parse_joint_equalities(expanded_xml_path, model):
    """Parse joint equality constraints from an expanded MuJoCo XML."""
    tree = ET.parse(expanded_xml_path)
    root = tree.getroot()

    eq = {}
    for elem in root.iter():
        if not elem.tag.endswith("joint"):
            continue

        slave_name = elem.get("joint1")
        master_name = elem.get("joint2")
        if slave_name is None or master_name is None:
            continue

        slave_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, slave_name)
        master_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, master_name)
        if slave_id < 0 or master_id < 0:
            continue

        poly = elem.get("polycoef")
        coeffs = [0.0, 1.0] if poly is None else [float(x) for x in poly.split()]
        eq[slave_id] = (master_id, coeffs)

    return eq


def parse_model_joint_equalities(model):
    """Save the compiled model XML and parse resolved joint equalities from it."""
    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            expanded_xml_path = Path(tmpdir) / "expanded_model.xml"
            mujoco.mj_saveLastXML(str(expanded_xml_path), model)
            return parse_joint_equalities(expanded_xml_path, model)
    except mujoco.FatalError:
        return parse_compiled_joint_equalities(model)


def parse_compiled_joint_equalities(model):
    """Parse joint equality constraints directly from a compiled model."""
    eq = {}
    for eq_id in range(model.neq):
        if model.eq_type[eq_id] != mujoco.mjtEq.mjEQ_JOINT:
            continue

        slave_id = int(model.eq_obj1id[eq_id])
        master_id = int(model.eq_obj2id[eq_id])
        if slave_id < 0 or master_id < 0:
            continue

        coeffs = [float(value) for value in model.eq_data[eq_id, :5]]
        eq[slave_id] = (master_id, coeffs)
    return eq


def apply_eq_constraints(data, model, eq_map):
    """Evaluate joint equalities into ``data.qpos`` before ``mj_forward``."""
    for slave_id, (master_id, coeffs) in eq_map.items():
        slave_q = model.jnt_qposadr[slave_id]
        master_q = model.jnt_qposadr[master_id]
        master_value = data.qpos[master_q]

        slave_value = 0.0
        for power, coeff in enumerate(coeffs):
            slave_value += coeff * (master_value**power)

        data.qpos[slave_q] = slave_value


def compute_moment_arm_curve(model, data, tendon_id, jnt_id, eps=1e-5, n=100, eq_map=None):
    """Compute moment arm curve for a tendon across a joint's range using finite differences."""
    qpos_id = model.jnt_qposadr[jnt_id]
    q0, q1 = model.jnt_range[jnt_id]
    if q0 == q1:
        return None, None

    qs = np.linspace(q0, q1, n)
    ma = np.zeros_like(qs)
    eq_map = eq_map or {}

    data.qpos[:] = 0.0
    for i, q in enumerate(qs):
        data.qpos[qpos_id] = q - eps
        apply_eq_constraints(data, model, eq_map)
        mujoco.mj_forward(model, data)
        L1 = data.ten_length[tendon_id]

        data.qpos[qpos_id] = q + eps
        apply_eq_constraints(data, model, eq_map)
        mujoco.mj_forward(model, data)
        L2 = data.ten_length[tendon_id]

        ma[i] = -(L2 - L1) / (2 * eps)

    return qs, ma


def compute_force_length_curve(model, data, act_id, jnt_id, activation=1.0, n=100, eq_map=None):
    """Compute MTU force-length curve for an actuator across a joint's range."""
    qpos_id = model.jnt_qposadr[jnt_id]
    q0, q1 = model.jnt_range[jnt_id]
    if q0 == q1:
        return None, None

    qs = np.linspace(q0, q1, n)
    lengths, forces = [], []
    eq_map = eq_map or {}

    for q in qs:
        data.qpos[:] = 0.0
        data.qpos[qpos_id] = q
        apply_eq_constraints(data, model, eq_map)
        data.act[:] = 0.0
        data.act[act_id] = activation
        mujoco.mj_forward(model, data)

        lengths.append(data.actuator_length[act_id])
        forces.append(-data.actuator_force[act_id])

    return np.asarray(lengths), np.asarray(forces)


def pair_discrepancy_summary(
    curves,
    moment_arm_tol=2e-5,
    moment_arm_rtol=5e-4,
    force_rtol=1e-2,
    force_atol=0.1,
):
    """Compare left/right curves and return pass/fail flags plus max discrepancies."""
    right, left = curves["right"], curves["left"]

    moment_arm_diff = np.abs(right["moment_arms"] - left["moment_arms"])
    moment_arm_ok = np.allclose(
        right["moment_arms"],
        left["moment_arms"],
        atol=moment_arm_tol,
        rtol=moment_arm_rtol,
    )

    force_diff = np.abs(right["forces"] - left["forces"])
    force_scale = np.maximum(np.abs(right["forces"]), np.abs(left["forces"]))
    force_ok = np.all((force_diff < force_atol) | (force_diff < force_rtol * force_scale))
    nonzero_force = force_scale > 0.0
    force_pct = float(np.max(force_diff[nonzero_force] / force_scale[nonzero_force]) * 100.0) if np.any(nonzero_force) else 0.0

    return {
        "ok": bool(moment_arm_ok and force_ok),
        "moment_arm_ok": bool(moment_arm_ok),
        "force_ok": bool(force_ok),
        "moment_arm": float(np.max(moment_arm_diff)),
        "force": float(np.max(force_diff)),
        "force_pct": force_pct,
    }


def plot_pair(
    curves,
    title,
    out_path=None,
    moment_arm_tol=2e-5,
    moment_arm_rtol=5e-4,
    force_rtol=1e-2,
    force_atol=0.1,
):
    """Plot left/right muscle comparison. Saves plot only if discrepancy found."""
    summary = pair_discrepancy_summary(
        curves,
        moment_arm_tol=moment_arm_tol,
        moment_arm_rtol=moment_arm_rtol,
        force_rtol=force_rtol,
        force_atol=force_atol,
    )
    discrep = not summary["ok"]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    for side, style in [("right", "-"), ("left", "dotted")]:
        c = curves[side]
        axes[0].plot(
            c["jnt_range"],
            c["moment_arms"],
            linestyle=style,
            label=c["muscle"],
        )
        axes[1].plot(
            c["mtu_lengths"],
            c["forces"],
            linestyle=style,
            label=c["muscle"],
        )

    # ---- Titles ----
    axes[0].set_title("Moment arm")
    axes[1].set_title("Force–length")

    # ---- Axis labels ----
    axes[0].set_xlabel("Joint angle (rad)")
    axes[0].set_ylabel("Moment arm (m)")

    axes[1].set_xlabel("MTU length (m)")
    axes[1].set_ylabel("Muscle force (N)")

    for ax in axes:
        ax.legend(fontsize=8)
        ax.grid(True)

    fig.suptitle(("✓ OK" if not discrep else "✗ DISCREPANCY") + " – " + title)
    plt.tight_layout()

    if discrep and out_path is not None:
        plt.savefig(out_path, dpi=200, bbox_inches="tight")

    plt.close(fig)
    return not discrep
