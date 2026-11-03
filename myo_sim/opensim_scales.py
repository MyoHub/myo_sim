# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
#
# Scaling rules and parts of this module are adapted from MyoSuite PR #525 by
# Mark Colley (M-Colley), Apache-2.0 (https://github.com/MyoHub/myosuite/pull/525).
"""OpenSim scale files and the Rajagopal segments of the myo_sim models.

:func:`myo_sim.scaling.scale_spec` reads an OpenSim ScaleSet or a scaled ``.osim`` model with
these helpers. During parsing the factors are converted between the axis conventions of OpenSim
(y-up) and of the model, body by body: most bodies kept OpenSim's frames, a few are z-up.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .scaling import _frame_rot, _quat, _rot

_LEG_SEGMENTS = ("femur", "tibia", "patella", "talus", "calcn", "toes")

# A segment's first bodies and, for each of their axes, the OpenSim axis it lies along (OpenSim
# frames: x anterior, y up, z to the right). Every other body takes its nearest mapped ancestor's
# factors, turned into its own frame.
_Segments = dict[str, tuple[tuple[str, ...], tuple[int, int, int]]]
_OPENSIM_AXES = (0, 1, 2)
_Z_UP = (0, 2, 1)  # x anterior, y lateral, z up

# The composed models (myofullbody, myolegs, myoarm_r, ...), whose root is z-up with a lateral x.
# The lumbar spine, thorax, shoulder girdle and head follow ``torso``; the hand bones follow ``hand``.
_COMPOSED_SEGMENTS: _Segments = {
    "pelvis": (("Full Body",), (2, 0, 1)),
    "torso": (("lumbar5",), _OPENSIM_AXES),
    **{
        f"{segment}_{side}": ((f"{segment}_{side}",), _OPENSIM_AXES)
        for segment in (*_LEG_SEGMENTS, "humerus", "ulna", "radius")
        for side in "rl"
    },
    "hand_r": (("lunate_r",), _OPENSIM_AXES),
    "hand_l": (("lunate_l",), _OPENSIM_AXES),
}

# The stand-alone leg model with a free ``root`` and ``torso`` (z-up); its pelvis and legs use
# OpenSim's frames.
_STANDALONE_LEG_SEGMENTS: _Segments = {
    "pelvis": (("root",), _Z_UP),
    "torso": (("torso",), _Z_UP),
    **{f"{segment}_{side}": ((f"{segment}_{side}",), _OPENSIM_AXES) for segment in _LEG_SEGMENTS for side in "rl"},
}


def read_opensim_scales(path: str | os.PathLike[str]) -> dict[str, np.ndarray]:
    """Read segment name -> ``(sx, sy, sz)`` from a ScaleSet or a scaled ``.osim`` model.

    A scaled model carries the applied factors as its geometry's ``scale_factors``.

    Args:
        path: The OpenSim XML file.

    Returns:
        The factors of each segment, along OpenSim's axes.
    """
    root = ET.parse(path).getroot()
    return _scale_set(root) or _model_scales(root)


def opensim_body_factors(
    spec: mujoco.MjSpec, path: str | os.PathLike[str]
) -> tuple[dict[str, np.ndarray], set[str], list[str]]:
    """Map an OpenSim scale file onto the bodies of a model.

    Args:
        spec: The model.
        path: The OpenSim ScaleSet or scaled ``.osim`` model.

    Returns:
        Body name -> factors along that body's axes, the segments' first bodies (the subject, even
        those left unscaled), and the file's segments that this model does not have.

    Raises:
        ValueError: If no segment mapping fits this model.
    """
    segments = _segments_for(spec)
    scales = read_opensim_scales(path)
    ignored = sorted(set(scales) - set(segments))
    return _body_factors(spec, scales, segments), {b for bodies, _ in segments.values() for b in bodies}, ignored


def _segments_for(spec: mujoco.MjSpec) -> _Segments:
    """The segments of a model: those of its table whose first bodies are in the model."""
    names = {b.name for b in spec.bodies[1:]}
    if "Full Body" in names:
        table = _COMPOSED_SEGMENTS
    elif {"root", "torso"} <= names:
        table = _STANDALONE_LEG_SEGMENTS
    else:
        raise ValueError("No OpenSim segment mapping fits this model; pass per-body factors instead.")
    return {segment: (bodies, axes) for segment, (bodies, axes) in table.items() if set(bodies) <= names}


def _scale_set(root: ET.Element) -> dict[str, np.ndarray]:
    scales = {}
    for scale in root.iter("Scale"):
        if (scale.findtext("apply") or "true").strip().lower() == "false":
            continue
        segment, values = scale.findtext("segment"), scale.findtext("scales")
        if segment and values:
            scales[segment.strip()] = np.array(values.split(), dtype=float)
    return scales


def _model_scales(root: ET.Element) -> dict[str, np.ndarray]:
    scales = {}
    for body in root.iter("Body"):
        name, factors = body.get("name"), body.find(".//scale_factors")
        if name and factors is not None and factors.text:
            scales[name] = np.array(factors.text.split(), dtype=float)
    return scales


def _body_factors(spec: mujoco.MjSpec, scales: dict[str, np.ndarray], segments: _Segments) -> dict[str, np.ndarray]:
    """Permute each segment's factors onto its first bodies and hand them down the tree.

    A segment missing from ``scales`` stays unscaled, as in OpenSim. Every body takes the factors
    of its nearest first body, turned into its own frame at ``qpos0``.
    """
    # Validate even unused segments, so that malformed values cannot pass silently.
    for values in scales.values():
        if np.shape(values) != (3,) or not np.all(np.isfinite(values)) or np.any(np.asarray(values) <= 0):
            raise ValueError(f"Scale factors must be finite and positive, got {values}.")
    starts: dict[str, np.ndarray] = {}
    for segment, (first_bodies, axes) in segments.items():
        factors = np.asarray(scales.get(segment, np.ones(3)), dtype=float)[list(axes)]
        for body in first_bodies:
            if body in starts:
                raise ValueError(f"Body {body!r} starts more than one segment.")
            starts[body] = factors
    per_body: dict[str, np.ndarray] = {}
    for body in spec.bodies[1:]:
        ancestor, rot = body, np.eye(3)
        while ancestor.name not in starts and ancestor.parent is not None:
            rot = _frame_rot(spec, ancestor.frame) @ _rot(_quat(spec, ancestor)) @ rot
            ancestor = ancestor.parent
        if ancestor.name in starts:
            if not body.name:
                raise ValueError(f"An unnamed body below {ancestor.name!r} cannot be scaled.")
            per_body[body.name] = _turned_factors(starts[ancestor.name], rot)
    return per_body


def _turned_factors(s: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Factors along the axes of a frame rotated by ``rot`` from the frame of ``s``.

    Frames that differ by a signed axis permutation, up to a tilt below about 0.8 degrees, swap
    the factors exactly; other rotations take the scale along each axis, dropping shear.
    """
    if np.ptp(s) == 0:
        return s.copy()
    axes = np.argmax(np.abs(rot), axis=0)
    if np.allclose(np.abs(rot[axes, np.arange(3)]), 1.0, rtol=0, atol=1e-4):
        return s[axes]
    return np.linalg.norm(np.diag(s) @ rot, axis=0)
