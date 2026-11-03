"""Check a scaled model against its reference, and refit its muscles' operating curves.

:func:`fit_length_ranges` is an optional step after :func:`myo_sim.scaling.scale_spec`. It adapts
the idea of Modenese et al. (2016, doi:10.1016/j.jbiomech.2015.11.006), which preserves each
muscle's normalised operating curve over the joints' ranges, to MuJoCo's ``lengthrange``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import mujoco
import numpy as np

from .scaling import _same_paths, _tendon_states, assemble_qpos, free_joints, matched_poses


@dataclass
class CheckResult:
    """One check of a scaled model.

    Attributes:
        name: Name of the check.
        passed: Whether ``value`` is within ``threshold``.
        value: The measured quantity.
        threshold: Its limit.
        detail: What was measured, and the worst element.
    """

    name: str
    passed: bool
    value: float
    threshold: float
    detail: str = ""


@dataclass
class ScaledModelChecks:
    """The checks of a scaled model against its reference."""

    results: list[CheckResult] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        """Whether every check passed."""
        return all(r.passed for r in self.results)

    def __getitem__(self, name: str) -> CheckResult:
        return next(r for r in self.results if r.name == name)

    def to_dict(self) -> dict:
        """The results as plain data."""
        return {"passed": self.passed, "checks": [asdict(r) for r in self.results]}


def fit_length_ranges(
    spec: mujoco.MjSpec,
    reference: mujoco.MjModel,
    *,
    n: int = 300,
    seed: int = 0,
    min_r2: float = 0.9,
) -> tuple[mujoco.MjSpec, dict[str, float]]:
    """Return a copy of a scaled spec whose muscles keep the reference's normalised lengths.

    Over shared random poses where both paths run the same way, each muscle's reference
    length is regressed on its scaled length, ``L_ref = a + b L_scaled``, and its range becomes
    ``(range_ref - a) / b``. That is exact whenever the path scaled affinely. MuJoCo derives
    optimal fibre and tendon slack length from ``lengthrange``, so this refits both. A muscle
    whose fit is worse than ``min_r2``, or whose length does not vary, keeps the range it has.

    Args:
        spec: The scaled spec; it is not changed.
        reference: The compiled unscaled model, with the same actuators.
        n: Number of sampled poses.
        seed: Seed of the pose sampler.
        min_r2: Smallest R² at which a fit is used.

    Returns:
        The refitted spec, and muscle name -> R² of its fit.

    Raises:
        ValueError: If the spec and the reference differ in their actuators.
    """
    fitted = spec.copy()
    scaled = fitted.copy().compile()
    if scaled.nu != reference.nu:
        raise ValueError(f"The spec has {scaled.nu} actuators, the reference {reference.nu}.")
    muscles = _muscles(reference)
    ref_poses, poses = matched_poses(reference, scaled, n, seed)
    lengths_ref, lengths = _actuator_lengths(reference, ref_poses), _actuator_lengths(scaled, poses)
    same_path = _same_wrap_points(reference, scaled, ref_poses, poses)
    r2: dict[str, float] = {}
    for position, actuator in enumerate(fitted.actuators):
        i = reference.actuator(actuator.name).id if actuator.name else position
        if i not in muscles:
            continue
        x, y = lengths[same_path[:, i], i], lengths_ref[same_path[:, i], i]
        if len(x) < 2:
            r2[reference.actuator(i).name] = 0.0
            continue
        if np.ptp(x) < 1e-9 or np.ptp(y) < 1e-9:
            r2[reference.actuator(i).name] = 1.0
            continue
        b, a = np.polyfit(x, y, 1)
        total = float(np.sum((y - y.mean()) ** 2))
        fit_r2 = 1.0 - float(np.sum((y - (a + b * x)) ** 2)) / total
        r2[reference.actuator(i).name] = fit_r2
        if fit_r2 >= min_r2 and b > 0:
            actuator.lengthrange = (reference.actuator_lengthrange[i] - a) / b
    return fitted, r2


def check_scaled_model(
    model: mujoco.MjModel,
    reference: mujoco.MjModel,
    *,
    n: int = 200,
    seed: int = 0,
    steps: int = 300,
    operating_tol: float = 0.05,
    min_sign_agreement: float = 0.99,
    max_speed: float = 100.0,
    max_constraint: float = 0.01,
) -> ScaledModelChecks:
    """Check a scaled model against its unscaled reference.

    The checks are ``operating_range`` (the normalised length of every muscle over shared random
    poses matches the reference), ``moment_arm_signs`` (where the reference's moment arm exceeds
    5 mm, the scaled model's has the same sign), ``stability`` (a passive and lightly activated
    rollout stays finite and slow) and ``constraints`` (the equality residuals in that rollout,
    joint couplings and tendon ties, stay small).

    Args:
        model: The compiled scaled model.
        reference: The compiled unscaled model.
        n: Number of sampled poses.
        seed: Seed of the pose sampler and the rollout.
        steps: Number of steps of the rollout.
        operating_tol: Limit on the 99th percentile of the normalised length difference.
        min_sign_agreement: Smallest fraction of moment arms with the reference's sign.
        max_speed: Limit on the largest joint speed in the rollout (rad/s or m/s).
        max_constraint: Limit on the largest equality residual in the rollout's second half,
            beyond the reference's (m or rad).

    Returns:
        The results of the four checks.
    """
    stability, constraints = _check_rollouts(model, reference, seed, steps, max_speed, max_constraint)
    return ScaledModelChecks(
        [
            _check_operating_range(model, reference, n, seed, operating_tol),
            _check_moment_arm_signs(model, reference, 0.005, min_sign_agreement),
            stability,
            constraints,
        ]
    )


def _muscles(model: mujoco.MjModel) -> set[int]:
    return {i for i in range(model.nu) if model.actuator_gaintype[i] == mujoco.mjtGain.mjGAIN_MUSCLE}


def _actuator_lengths(model: mujoco.MjModel, poses: np.ndarray) -> np.ndarray:
    data = mujoco.MjData(model)
    out = np.empty((len(poses), model.nu))
    for k, q in enumerate(poses):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_tendon(model, data)
        mujoco.mj_transmission(model, data)
        out[k] = data.actuator_length
    return out


def _same_wrap_points(reference: mujoco.MjModel, model: mujoco.MjModel, ref_poses: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """Per pose and actuator, whether a muscle's path runs the same way in both models."""
    paths = _same_paths(_tendon_states(reference, ref_poses)[1], _tendon_states(model, poses)[1])
    same = np.ones((len(poses), reference.nu), dtype=bool)
    on_tendon = np.flatnonzero(reference.actuator_trntype == mujoco.mjtTrn.mjTRN_TENDON)
    same[:, on_tendon] = paths[:, reference.actuator_trnid[on_tendon, 0]]
    return same


def _normalised(model: mujoco.MjModel, lengths: np.ndarray, ids: list[int]) -> np.ndarray:
    ranges = model.actuator_lengthrange[ids]
    return (lengths[:, ids] - ranges[:, 0]) / (ranges[:, 1] - ranges[:, 0])


def _check_operating_range(model: mujoco.MjModel, reference: mujoco.MjModel, n: int, seed: int, tol: float) -> CheckResult:
    ids = sorted(_muscles(reference))
    if not ids:
        return CheckResult("operating_range", True, 0.0, tol, "no muscles")
    q_ref, q_model = matched_poses(reference, model, n, seed)
    difference = np.abs(
        _normalised(model, _actuator_lengths(model, q_model), ids)
        - _normalised(reference, _actuator_lengths(reference, q_ref), ids)
    )
    value = float(np.quantile(difference, 0.99))
    worst = ids[int(np.argmax(difference.max(axis=0)))]
    detail = f"99th percentile of |normalised length difference|; worst muscle {reference.actuator(worst).name} ({difference.max():.3f})"
    return CheckResult("operating_range", value <= tol, value, tol, detail)


def _moment_arms(model: mujoco.MjModel, eps: float = 1e-6) -> np.ndarray:
    """Actuator length change per unit of each free hinge, couplings followed: ``(nu, n_hinges)``."""
    data = mujoco.MjData(model)
    base = assemble_qpos(model, data)
    columns = []
    for j in free_joints(model):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        adr, lengths = model.jnt_qposadr[j], []
        for sign in (1, -1):
            q = base.copy()
            q[adr] += sign * eps
            lengths.append(_actuator_lengths(model, assemble_qpos(model, data, q)[None])[0])
        columns.append((lengths[0] - lengths[1]) / (2 * eps))
    return np.stack(columns, axis=1) if columns else np.empty((model.nu, 0))


def _check_moment_arm_signs(
    model: mujoco.MjModel, reference: mujoco.MjModel, min_arm: float, min_fraction: float
) -> CheckResult:
    ids = sorted(_muscles(reference))
    arms, arms_ref = _moment_arms(model)[ids], _moment_arms(reference)[ids]
    large = np.abs(arms_ref) > min_arm
    agree = np.sign(arms) == np.sign(arms_ref)
    fraction = float(np.mean(agree[large])) if large.any() else 1.0
    flipped = sorted({reference.actuator(ids[i]).name for i in np.flatnonzero(np.any(large & ~agree, axis=1))})
    detail = f"{int(large.sum())} muscle-joint pairs above {min_arm * 1000:.0f} mm; flipped: {flipped or 'none'}"
    return CheckResult("moment_arm_signs", fraction >= min_fraction, fraction, min_fraction, detail)


def _rollout(model: mujoco.MjModel, seed: int, steps: int) -> tuple[float, float, str]:
    """Largest joint speed and late equality residual of a rollout; a note on failure."""
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(model)
    assemble_qpos(model, data)
    speed, residual = 0.0, 0.0
    for k in range(steps):
        data.ctrl[:] = 0.0 if k < steps // 2 else rng.uniform(0, 0.2, model.nu)
        mujoco.mj_step(model, data)
        if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
            return float("inf"), float("inf"), f"non-finite state at step {k}"
        speed = max(speed, float(np.abs(data.qvel).max()) if model.nv else 0.0)
        if k >= steps // 2:
            equality = data.efc_pos[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
            residual = max(residual, float(np.abs(equality).max()) if equality.size else 0.0)
    return speed, residual, ""


def _check_rollouts(
    model: mujoco.MjModel,
    reference: mujoco.MjModel,
    seed: int,
    steps: int,
    max_speed: float,
    max_constraint: float,
) -> tuple[CheckResult, CheckResult]:
    speed, residual, note = _rollout(model, seed, steps)
    _, residual_ref, note_ref = _rollout(reference, seed, steps)
    baseline = residual_ref if not note_ref else 0.0
    limit = baseline + max_constraint
    return (
        CheckResult("stability", speed < max_speed, speed, max_speed, note or "largest joint speed (rad/s or m/s)"),
        CheckResult(
            "constraints", residual <= limit, residual, limit, note or f"largest equality residual; reference {baseline:.2e}"
        ),
    )
