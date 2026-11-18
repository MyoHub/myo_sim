"""Runtime physics of the myo_sim models that MuJoCo does not provide on its own.

- :mod:`~myo_sim.physics.joint_coupling`: put coupled joints (``mjEQ_JOINT``) on their polynomials.
- :mod:`~myo_sim.physics.fatigue`: 3CC-r cumulative muscle fatigue (NumPy; batched torch).
- :mod:`~myo_sim.physics.fatigue_jax`: the same fatigue step in JAX.
- :mod:`~myo_sim.physics.muscle_groups`: functional muscle groups, the keys of the fatigue parameters.
- :mod:`~myo_sim.physics.muscle_strength`: native muscle peak-force scaling and isometric MVC torque calibration.
- :mod:`~myo_sim.physics.activation_dynamics`: Hill-type muscle activation and deactivation time-constant transforms.
- :mod:`~myo_sim.physics.joint_mechanics`: passive joint damping, stiffness, and range-of-motion copy transforms.

NumPy and MuJoCo suffice to import this package; torch and JAX are imported only where used
(``pip install "myo-sim[torch]"`` / ``"myo-sim[jax]"``).
"""

from myo_sim.physics.fatigue import (
    FATIGUE_REST_THRESHOLD,
    MUSCLE_FATIGUE_PARAMS,
    CumulativeFatigue,
    TorchFatigueState,
    three_cc_r_step,
    torch_three_cc_r_step,
)
from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings, projected_joint_couplings

__all__ = [
    "FATIGUE_REST_THRESHOLD",
    "MUSCLE_FATIGUE_PARAMS",
    "CumulativeFatigue",
    "JointCouplings",
    "TorchFatigueState",
    "project_joint_couplings",
    "projected_joint_couplings",
    "three_cc_r_step",
    "torch_three_cc_r_step",
]
