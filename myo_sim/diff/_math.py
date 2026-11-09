"""Small batched vector and rotation helpers shared by the differentiable model.

All functions are elementwise over leading dimensions, avoid host synchronisation and keep finite
gradients where MuJoCo clamps (``mjMINVAL``) or where a ``torch.where`` branch is not selected.
"""

from __future__ import annotations

import torch

MINVAL = 1e-15  # mjMINVAL


def safe_sqrt(x: torch.Tensor) -> torch.Tensor:
    """sqrt with a finite gradient at and below zero (unselected branches would otherwise give NaN)."""
    return x.clamp(min=1e-30).sqrt()


def norm(x: torch.Tensor) -> torch.Tensor:
    """Euclidean norm over the last dimension, with a finite gradient at zero."""
    return safe_sqrt((x * x).sum(-1))


def dot(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Dot product over the last dimension."""
    return (a * b).sum(-1)


def cross(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Cross product of (..., 3) vectors, written out so it broadcasts and compiles to one kernel."""
    a0, a1, a2 = a.unbind(-1)
    b0, b1, b2 = b.unbind(-1)
    return torch.stack((a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0), -1)


def normalize(x: torch.Tensor) -> torch.Tensor:
    """``mju_normalize``: unit vector, or the first basis vector if the norm is below mjMINVAL."""
    n = norm(x)
    small = (n < MINVAL).unsqueeze(-1)
    e0 = torch.zeros_like(x)
    e0[..., 0] = 1.0
    return torch.where(small, e0, x / n.clamp(min=MINVAL).unsqueeze(-1))


def mv(R: torch.Tensor, x: torch.Tensor) -> torch.Tensor:
    """R x for (..., 3, 3) matrices and (..., 3) vectors, as a product and a sum (fuses with its neighbours)."""
    return (R * x.unsqueeze(-2)).sum(-1)


def mtv(R: torch.Tensor, x: torch.Tensor) -> torch.Tensor:
    """R^T x for (..., 3, 3) matrices and (..., 3) vectors."""
    return (R * x.unsqueeze(-1)).sum(-2)


def mm(A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
    """A B for (..., 3, 3) matrices.

    A batched matmul: the product-and-sum form of :func:`mv` crashes inductor's CPU backward (torch 2.13).
    """
    return A @ B


def quat_to_mat(q: torch.Tensor) -> torch.Tensor:
    """Rotation matrix of a (..., 4) wxyz quaternion, normalised first like ``mju_normalize4``."""
    n = norm(q)
    unit = torch.zeros_like(q)
    unit[..., 0] = 1.0
    q = torch.where((n < MINVAL).unsqueeze(-1), unit, q / n.clamp(min=MINVAL).unsqueeze(-1))
    w, x, y, z = q.unbind(-1)
    return torch.stack(
        (
            1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
        ),
        -1,
    ).reshape(q.shape[:-1] + (3, 3))  # fmt: skip


def axis_angle_to_mat(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    """Rodrigues rotation for unit axes (..., 3) and angles (...)."""
    x, y, z = axis.unbind(-1)
    c, s = torch.cos(angle), torch.sin(angle)
    C = 1 - c
    return torch.stack(
        (
            c + x * x * C, x * y * C - z * s, x * z * C + y * s,
            y * x * C + z * s, c + y * y * C, y * z * C - x * s,
            z * x * C - y * s, z * y * C + x * s, c + z * z * C,
        ),
        -1,
    ).reshape(angle.shape + (3, 3))  # fmt: skip
