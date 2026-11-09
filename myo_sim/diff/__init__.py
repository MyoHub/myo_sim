"""Differentiable (PyTorch) muscle-tendon model generated from a compiled myo_sim ``mujoco.MjModel``.

Batched, device-agnostic replicas of MuJoCo's forward kinematics, spatial and fixed tendons (wrapping,
side sites, pulleys), moment arms, muscle forces and activation dynamics; see
``docs/wiki/differentiable-muscle-tendon.md``. Needs PyTorch (``pip install "myo-sim[torch]"``);
``import myo_sim`` itself never imports torch.

Parts of this package are ported from muscle-supermimic (Apache-2.0); see ``NOTICE`` in this directory.
"""

try:
    import torch  # noqa: F401
except ImportError as e:  # pragma: no cover - exercised only without torch
    raise ImportError('myo_sim.diff needs PyTorch; install it with pip install "myo-sim[torch]"') from e

from .coupling import JointCouplings
from .kinematics import Kinematics, KinematicsState
from .model import MuscleTendonModel, MuscleTendonState, default_side_rule
from .tendon import Tendons, TendonState
from .wrap import SIDE_RULES, wrap

__all__ = [
    "SIDE_RULES",
    "JointCouplings",
    "Kinematics",
    "KinematicsState",
    "MuscleTendonModel",
    "MuscleTendonState",
    "Tendons",
    "TendonState",
    "default_side_rule",
    "wrap",
]
