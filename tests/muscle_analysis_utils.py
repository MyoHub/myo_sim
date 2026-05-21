"""Utilities for analyzing muscle symmetry via moment arm and force-length curves."""

import mujoco
import numpy as np
import matplotlib.pyplot as plt


def compute_moment_arm_curve(model, data, tendon_id, jnt_id, eps=1e-5, n=100):
    """Compute moment arm curve for a tendon across a joint's range using finite differences."""
    qpos_id = model.jnt_qposadr[jnt_id]
    q0, q1 = model.jnt_range[jnt_id]
    if q0 == q1:
        return None, None

    qs = np.linspace(q0, q1, n)
    ma = np.zeros_like(qs)

    data.qpos[:] = 0.0
    for i, q in enumerate(qs):
        data.qpos[qpos_id] = q - eps
        mujoco.mj_forward(model, data)
        L1 = data.ten_length[tendon_id]

        data.qpos[qpos_id] = q + eps
        mujoco.mj_forward(model, data)
        L2 = data.ten_length[tendon_id]

        ma[i] = -(L2 - L1) / (2 * eps)

    return qs, ma


def compute_force_length_curve(model, data, act_id, jnt_id, activation=1.0, n=100):
    """Compute MTU force-length curve for an actuator across a joint's range."""
    qpos_id = model.jnt_qposadr[jnt_id]
    q0, q1 = model.jnt_range[jnt_id]
    if q0 == q1:
        return None, None

    qs = np.linspace(q0, q1, n)
    lengths, forces = [], []

    for q in qs:
        data.qpos[:] = 0.0
        data.qpos[qpos_id] = q
        data.act[:] = 0.0
        data.act[act_id] = activation
        mujoco.mj_forward(model, data)

        lengths.append(data.actuator_length[act_id])
        forces.append(-data.actuator_force[act_id])

    return np.asarray(lengths), np.asarray(forces)


def plot_pair(curves, title, out_path=None, tol=1e-6):
    """Plot left/right muscle comparison. Saves plot only if discrepancy found."""
    r, l = curves["right"], curves["left"]

    discrep = (
        not np.allclose(r["moment_arms"], l["moment_arms"], atol=tol, rtol=1e-5)
        or not np.allclose(r["forces"], l["forces"], atol=tol, rtol=1e-5)
    )

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
