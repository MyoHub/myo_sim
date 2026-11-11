# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Shared source models and numerical checks for the anatomy API and CLI."""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

import myo_sim
from myo_sim.muskemo import model_fingerprint


def _elbow_model() -> tuple[mujoco.MjSpec, mujoco.MjModel]:
    spec = myo_sim.load_spec("myoelbow")
    return spec, spec.compile()


def _patch(model: mujoco.MjModel) -> dict[str, Any]:
    return dict(
        schema=1,
        fingerprint=model_fingerprint(model),
        mujoco_version=mujoco.__version__,
        sites=[],
        inertias=[],
    )


def _edit(model: mujoco.MjModel) -> dict[str, Any]:
    patch = _patch(model)
    position = (model.site("BRA-P2").pos + [0.004, 0.002, -0.003]).tolist()
    patch["sites"] = [dict(name="BRA-P2", pos=position)]
    patch["inertias"] = [
        dict(
            name="r_ulna_radius_hand",
            mass=1.7,
            com=[0.01, -0.17, 0.01],
            inertia=[[0.02, 0.001, 0.002], [0.001, 0.03, 0.003], [0.002, 0.003, 0.04]],
        )
    ]
    return patch


def _source_state(spec: mujoco.MjSpec) -> dict[str, Any]:
    fields = {
        "sites": ("pos",),
        "geoms": ("pos", "quat"),
        "bodies": (
            "mass",
            "ipos",
            "iquat",
            "inertia",
            "fullinertia",
            "explicitinertial",
        ),
        "meshes": ("file", "content_type"),
    }
    state = {
        f"{kind}[{index}].{field}": np.array(getattr(item, field), copy=True)
        for kind, attributes in fields.items()
        for index, item in enumerate(getattr(spec, kind))
        for field in attributes
    }
    return {**state, "assets": dict(spec.assets)}


def _assert_source_unchanged(spec: mujoco.MjSpec, before: dict[str, Any]) -> None:
    for field, value in _source_state(spec).items():
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(value, before[field], err_msg=field)
        else:
            assert value == before[field], field


def _fusion_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string(
        '<mujoco><compiler fusestatic="true"/><worldbody>'
        '<body name="moving"><joint/><geom size=".1"/>'
        '<body name="fixed" pos=".1 .2 .3" euler="0 0 90">'
        '<geom name="fixed_geom" size=".01"/>'
        '<site name="s" pos=".1 0 0"/><site name="untouched" pos=".2 0 0"/>'
        "</body></body></worldbody></mujoco>"
    )
