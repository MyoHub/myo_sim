"""Automatic discovery of muscle *redundancy groups* from a compiled model.

A redundancy group is a set of actuators that are mechanically interchangeable:
their moment arms about every degree of freedom point in nearly the same
direction across posture, so the same joint torque can be produced by many
different force splits between them. Which split the controller happens to pick
is unconstrained by the task, which is why learned policies routinely settle on
physiologically implausible recruitment (for example loading a small lateral
gastrocnemius instead of a large soleus).

Nothing here uses muscle names, hand-written anatomy tables, or EMG data. Groups
fall out of the model's own geometry, so the same code works on any MyoSuite
model and on new models as they are authored.

Typical use when building an RL reward or observation::

    from myo_sim.analysis.muscle_groups import group_for_actuator

    group = group_for_actuator(model, "soleus_r")
    #   group.actuators      -> ('gaslat_r', 'gasmed_r', 'soleus_r')
    #   group.target_share   -> array([0.149, 0.252, 0.599])   (fmax-proportional)

    obs = group.observation(data)          # per-muscle stress + signed share error
    cost = group.effort(data)              # Crowninshield-Brand recruitment cost

Before targeting a specific share split, check that the split is actually
identifiable from the dynamics::

    group.identifiable_rank      # 2 for the triceps surae: only 2 of 3 dof matter
    group.identifiable_targets() # merges the directions the task cannot resolve

Targeting a share along a direction the model cannot distinguish wastes
optimisation budget and yields an arbitrary answer -- see
`docs/muscle-groups-fullbody.md` for the full worked example.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

__all__ = [
    "MuscleGroup",
    "SignatureResult",
    "discover_muscle_groups",
    "group_for_actuator",
    "moment_arm_signatures",
]

# Muscle actuators report negative `actuator_force` when pulling. Forces are
# converted to positive magnitudes and clipped at zero so a slack muscle cannot
# contribute negatively and push another muscle's share above 1.
_FORCE_EPS = 1e-6

# Default cosine similarity above which two actuators are considered
# mechanically interchangeable. Calibrated on myofullbody: the triceps surae
# sits at 0.87-0.98 internally while the nearest non-member is at 0.73.
DEFAULT_COS_THRESHOLD = 0.85

# A singular direction carrying less than this fraction of the leading singular
# value is treated as not identifiable from joint torque.
DEFAULT_RANK_TOL = 0.05


def _dense_moment(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Return the (nu, nv) dense actuator moment-arm matrix."""
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
    return moment


def _dependent_joint_ids(model: mujoco.MjModel) -> set[int]:
    """Joints whose value is slaved to another joint by an equality constraint.

    Knees in this model lineage are built from a driving `knee_angle_*` hinge
    plus several coupled DOFs pinned to it. Those coupled DOFs are not
    independent degrees of freedom, so counting them would weight the knee
    several times over in a moment-arm signature.
    """
    dependent = set()
    for eq_id in range(model.neq):
        if model.eq_type[eq_id] == mujoco.mjtEq.mjEQ_JOINT:
            dependent.add(int(model.eq_obj1id[eq_id]))
    return dependent


def _hinge_joints(model: mujoco.MjModel) -> list[str]:
    """Names of every independent hinge joint -- those a moment arm is properly defined about.

    Two exclusions, both of which matter in practice on `myofullbody`:

    * Slide joints. Their `actuator_moment` entry is a force-transmission ratio
      in metres, not a moment arm, so pooling the two into one vector is
      dimensionally unsound -- and the coupled translational DOFs of a modelled
      knee carry entries around 0.8, swamping every genuine rotational moment
      arm (order 0.04).
    * Equality-constrained joints, which would count one physical joint many times.
    """
    dependent = _dependent_joint_ids(model)
    names = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE and joint_id not in dependent:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if name is not None:
                names.append(name)
    return names


def joints_spanned_by(model: mujoco.MjModel, actuator: str, tol: float = 1e-4) -> list[str]:
    """Joints the actuator has a non-negligible moment arm about, at the default posture."""
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
    if actuator_id < 0:
        raise ValueError(f"Unknown actuator: {actuator}")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    moment = _dense_moment(model, data)
    names = _hinge_joints(model)
    dof_adr = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in names]
    return [name for name, dof in zip(names, dof_adr) if abs(moment[actuator_id, dof]) > tol]


def auto_scope_joints(model: mujoco.MjModel, actuator: str, tol: float = 1e-4) -> list[str]:
    """Joints forming the mechanical neighbourhood of `actuator`.

    Starts from the joints the actuator itself spans, then adds every joint
    spanned by any *other* actuator that shares one of them. That one-hop
    expansion is what pulls in the joints which distinguish otherwise-similar
    muscles -- for the soleus it adds the knee, which is exactly what separates
    the mono-articular soleus from the bi-articular gastrocnemii.

    Scoping matters. Sampling postures over an entire body randomises joints
    that have nothing to do with the muscle under study and distorts its
    moment-arm signature; on `myofullbody` the whole-body scope fails to group
    the triceps surae at all, while this scope recovers it cleanly.
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    moment = _dense_moment(model, data)
    names = _hinge_joints(model)
    dof_adr = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in names])
    spans = np.abs(moment[:, dof_adr]) > tol

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
    if actuator_id < 0:
        raise ValueError(f"Unknown actuator: {actuator}")
    seed_joints = np.where(spans[actuator_id])[0]
    if seed_joints.size == 0:
        return []
    neighbours = np.where(spans[:, seed_joints].any(axis=1))[0]
    expanded = np.where(spans[neighbours].any(axis=0))[0]
    return [names[i] for i in expanded]


@dataclass(frozen=True)
class SignatureResult:
    """Posture-sampled moment-arm signatures for the actuators that span `joints`."""

    actuators: tuple[str, ...]
    actuator_ids: np.ndarray
    joints: tuple[str, ...]
    signatures: np.ndarray
    """(n_actuators, n_postures * n_joints) unit-norm moment-arm signature."""
    mean_moment_arm: np.ndarray
    """(n_actuators, n_joints) posture-averaged moment arm, in metres."""


def moment_arm_signatures(
    model: mujoco.MjModel,
    joints: list[str] | None = None,
    n_postures: int = 60,
    seed: int = 0,
) -> SignatureResult:
    """Sample each actuator's moment arm about `joints` over random postures.

    Postures are drawn uniformly from each joint's `jnt_range`, so the signature
    describes an actuator's mechanical action across its whole working range
    rather than at one arbitrary pose. Signatures are L2-normalised, making the
    cosine between two of them a scale-free measure of how similarly the two
    actuators load the skeleton.

    Actuators whose moment arm about every requested joint is numerically zero
    are dropped: they do not act on this joint set.
    """
    joint_names = joints if joints is not None else _hinge_joints(model)
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
    if any(joint_id < 0 for joint_id in joint_ids):
        missing = [name for name, joint_id in zip(joint_names, joint_ids) if joint_id < 0]
        raise ValueError(f"Unknown joint(s): {missing}")

    dof_adr = np.array([model.jnt_dofadr[joint_id] for joint_id in joint_ids])
    qpos_adr = np.array([model.jnt_qposadr[joint_id] for joint_id in joint_ids])
    ranges = np.array([model.jnt_range[joint_id] for joint_id in joint_ids])

    reference = mujoco.MjData(model)
    mujoco.mj_forward(model, reference)
    qpos0 = np.array(reference.qpos)

    rng = np.random.default_rng(seed)
    samples = np.empty((n_postures, model.nu, len(dof_adr)))
    data = mujoco.MjData(model)
    for index in range(n_postures):
        data.qpos[:] = qpos0
        data.qpos[qpos_adr] = ranges[:, 0] + (ranges[:, 1] - ranges[:, 0]) * rng.random(len(qpos_adr))
        mujoco.mj_forward(model, data)
        samples[index] = _dense_moment(model, data)[:, dof_adr]

    spanning = np.where(np.abs(samples).max(axis=(0, 2)) > 1e-4)[0]
    flat = samples[:, spanning, :].transpose(1, 0, 2).reshape(len(spanning), -1)
    norms = np.linalg.norm(flat, axis=1, keepdims=True)
    names = tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(i)) for i in spanning)
    return SignatureResult(
        actuators=names,
        actuator_ids=spanning,
        joints=tuple(joint_names),
        signatures=flat / np.maximum(norms, _FORCE_EPS),
        mean_moment_arm=samples[:, spanning, :].mean(axis=0),
    )


def _average_linkage_clusters(distance: np.ndarray, cut: float) -> list[list[int]]:
    """Agglomerative average-linkage clustering, cut at `cut`.

    Implemented directly against numpy so this module keeps the package's
    runtime dependencies at mujoco + numpy (scipy is a dev-only dependency).
    Uses the Lance-Williams update for average linkage.
    """
    n = distance.shape[0]
    dist = distance.astype(float).copy()
    np.fill_diagonal(dist, np.inf)
    members: list[list[int]] = [[i] for i in range(n)]
    active = np.ones(n, dtype=bool)

    while active.sum() > 1:
        masked = np.where(active[:, None] & active[None, :], dist, np.inf)
        flat_index = int(np.argmin(masked))
        i, j = divmod(flat_index, n)
        if not np.isfinite(masked[i, j]) or masked[i, j] > cut:
            break
        size_i, size_j = len(members[i]), len(members[j])
        dist[i, :] = (size_i * dist[i, :] + size_j * dist[j, :]) / (size_i + size_j)
        dist[:, i] = dist[i, :]
        dist[i, i] = np.inf
        members[i] = members[i] + members[j]
        members[j] = []
        active[j] = False

    return [sorted(m) for i, m in enumerate(members) if active[i] and m]


@dataclass(frozen=True)
class MuscleGroup:
    """A set of mechanically interchangeable actuators, plus recruitment priors.

    All arrays are ordered to match `actuators`.
    """

    actuators: tuple[str, ...]
    actuator_ids: np.ndarray
    fmax: np.ndarray
    """Peak isometric force per actuator, in newtons."""
    joints: tuple[str, ...]
    mean_moment_arm: np.ndarray
    """(n_actuators, n_joints) posture-averaged moment arm, in metres."""
    dominant_joint: str
    """Joint about which the group has the largest combined torque capacity."""
    _singular_values: np.ndarray = field(repr=False)

    # ---- priors -----------------------------------------------------------

    @property
    def target_share(self) -> np.ndarray:
        """Force share proportional to each actuator's fmax.

        In the Rajagopal-derived MyoSuite lineage fmax is PCSA times a uniform
        specific tension, so this is equivalently a PCSA-proportional prior. It
        is a *capacity* prior: it says a muscle should carry load in proportion
        to how much load it is built to carry. It is not a measured force share.
        """
        return self.fmax / self.fmax.sum()

    @property
    def torque_capacity(self) -> np.ndarray:
        """(n_actuators, n_joints) fmax-scaled moment arm, in newton-metres."""
        return self.fmax[:, None] * self.mean_moment_arm

    @property
    def singular_values(self) -> np.ndarray:
        """Singular values of the torque-capacity matrix, descending."""
        return self._singular_values

    @property
    def conditioning(self) -> np.ndarray:
        """Singular values normalised by the leading one."""
        leading = self._singular_values[0]
        return self._singular_values / leading if leading > 0 else self._singular_values

    def identifiable_rank(self, tol: float = DEFAULT_RANK_TOL) -> int:
        """How many independent force-split directions the task can actually resolve.

        If this is less than `len(actuators)`, some share targets are not
        identifiable: the dynamics are nearly unchanged when force moves between
        those muscles, so any optimiser will drift along that direction and the
        resulting split is arbitrary.
        """
        return int((self.conditioning >= tol).sum())

    def identifiable_targets(self, tol: float = DEFAULT_RANK_TOL) -> list[tuple[tuple[str, ...], float]]:
        """Target shares merged down to directions the model can distinguish.

        Actuators whose torque-capacity rows are nearly collinear are pooled and
        given a single combined target. Use these as reward targets instead of
        the raw per-muscle `target_share` when `identifiable_rank` is deficient.
        """
        capacity = self.torque_capacity
        norms = np.linalg.norm(capacity, axis=1, keepdims=True)
        unit = capacity / np.maximum(norms, _FORCE_EPS)
        similarity = np.clip(unit @ unit.T, -1.0, 1.0)
        # Pool rows that are collinear to within the same tolerance used for rank.
        clusters = _average_linkage_clusters(1.0 - similarity, tol)
        share = self.target_share
        merged = [(tuple(self.actuators[i] for i in cluster), float(share[list(cluster)].sum())) for cluster in clusters]
        return sorted(merged, key=lambda item: -item[1])

    # ---- runtime quantities ----------------------------------------------

    def forces(self, data: mujoco.MjData) -> np.ndarray:
        """Per-actuator tensile force magnitude, clipped at zero."""
        return np.maximum(-np.asarray(data.actuator_force)[self.actuator_ids], 0.0)

    def relative_stress(self, data: mujoco.MjData) -> np.ndarray:
        """Force normalised by each actuator's own fmax.

        This is the quantity a controller needs in order to tell *which* member
        of a redundant group it is currently leaning on. A policy that cannot
        observe it cannot represent a load-redistribution strategy at all.
        """
        return self.forces(data) / self.fmax

    def shares(self, data: mujoco.MjData) -> np.ndarray:
        """Each actuator's fraction of the group's combined force.

        Falls back to an equal split when the group is unloaded.
        """
        force = self.forces(data)
        total = float(force.sum())
        if total <= _FORCE_EPS:
            return np.full(len(self.actuators), 1.0 / len(self.actuators))
        return force / total

    def share_error(self, data: mujoco.MjData) -> np.ndarray:
        """Signed difference between current share and `target_share`, zero when unloaded."""
        force = self.forces(data)
        if float(force.sum()) <= _FORCE_EPS:
            return np.zeros(len(self.actuators))
        return self.shares(data) - self.target_share

    def observation(self, data: mujoco.MjData) -> np.ndarray:
        """Concatenated `relative_stress` and `share_error`, for use as policy input.

        Length is `2 * len(actuators)`.
        """
        return np.concatenate([self.relative_stress(data), self.share_error(data)])

    def effort(self, data: mujoco.MjData, exponent: float = 3.0) -> float:
        """Crowninshield-Brand recruitment cost, ``sum (F / fmax) ** exponent``.

        Minimising this is the standard way to resolve muscular redundancy. Note
        it is only weakly identifying near its optimum: most of its reduction is
        achieved well before a capacity-proportional split is reached, so it is
        best combined with an explicit target-share term rather than used alone.
        """
        return float(np.sum(self.relative_stress(data) ** exponent))

    def summary(self) -> str:
        """One-line-per-actuator human-readable description."""
        lines = [
            f"group of {len(self.actuators)} about {self.dominant_joint} "
            f"(identifiable rank {self.identifiable_rank()}/{len(self.actuators)})"
        ]
        share = self.target_share
        joint_index = self.joints.index(self.dominant_joint)
        for i, name in enumerate(self.actuators):
            lines.append(
                f"  {name:<16s} fmax={self.fmax[i]:8.1f} N  "
                f"moment_arm={self.mean_moment_arm[i, joint_index]:+.4f} m  target_share={share[i]:.3f}"
            )
        return "\n".join(lines)


def _build_group(
    model: mujoco.MjModel,
    signature: SignatureResult,
    indices: list[int],
) -> MuscleGroup:
    actuator_ids = signature.actuator_ids[indices]
    fmax = np.asarray(model.actuator_gainprm[actuator_ids, 2], dtype=float)
    mean_moment_arm = signature.mean_moment_arm[indices]
    capacity = fmax[:, None] * mean_moment_arm
    dominant = signature.joints[int(np.argmax(np.abs(capacity).sum(axis=0)))]
    singular_values = np.linalg.svd(capacity, compute_uv=False)
    return MuscleGroup(
        actuators=tuple(signature.actuators[i] for i in indices),
        actuator_ids=actuator_ids,
        fmax=fmax,
        joints=signature.joints,
        mean_moment_arm=mean_moment_arm,
        dominant_joint=dominant,
        _singular_values=singular_values,
    )


def discover_muscle_groups(
    model: mujoco.MjModel,
    joints: list[str] | None = None,
    cos_threshold: float = DEFAULT_COS_THRESHOLD,
    min_size: int = 2,
    n_postures: int = 60,
    seed: int = 0,
) -> list[MuscleGroup]:
    """Find every redundancy group in `model`, using geometry alone.

    Args:
        model: a compiled model.
        joints: restrict the analysis to these joints, e.g. one limb. Defaults to
            every hinge and slide joint, which finds groups across the whole body.
        cos_threshold: minimum cosine similarity between moment-arm signatures
            for two actuators to be pooled. Higher is stricter.
        min_size: ignore groups smaller than this. A group of one is not redundant.
        n_postures: random postures used to build each signature.
        seed: posture sampling seed.

    Returns:
        Groups sorted by descending size, then by leading actuator name.
    """
    signature = moment_arm_signatures(model, joints=joints, n_postures=n_postures, seed=seed)
    similarity = np.clip(signature.signatures @ signature.signatures.T, -1.0, 1.0)
    clusters = _average_linkage_clusters(1.0 - similarity, 1.0 - cos_threshold)
    groups = [_build_group(model, signature, cluster) for cluster in clusters if len(cluster) >= min_size]
    return sorted(groups, key=lambda g: (-len(g.actuators), g.actuators[0]))


def group_for_actuator(
    model: mujoco.MjModel,
    actuator: str,
    joints: list[str] | None = None,
    cos_threshold: float = DEFAULT_COS_THRESHOLD,
    n_postures: int = 60,
    seed: int = 0,
) -> MuscleGroup | None:
    """Return the redundancy group containing `actuator`, or None if it is alone.

    Convenience wrapper around `discover_muscle_groups` for the common case of
    caring about one muscle -- "what else can do this muscle's job?".

    When `joints` is None the analysis is scoped automatically to the actuator's
    mechanical neighbourhood via `auto_scope_joints`, which is almost always what
    you want on a whole-body model.
    """
    groups = discover_muscle_groups(
        model,
        joints=joints if joints is not None else auto_scope_joints(model, actuator),
        cos_threshold=cos_threshold,
        min_size=1,
        n_postures=n_postures,
        seed=seed,
    )
    for group in groups:
        if actuator in group.actuators:
            return group if len(group.actuators) > 1 else None
    return None
