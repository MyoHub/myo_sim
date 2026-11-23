# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""``default_scale_groups`` and ``inherited_scales`` on synthetic specs and composed models."""

from __future__ import annotations

import mujoco
import pytest

import myo_sim
from myo_sim import default_scale_groups, inherited_scales, scale_spec

TREE_XML = """
<mujoco>
  <worldbody>
    <body name="table" pos="2 0 0"><geom type="box" size=".1 .1 .1"/></body>
    <body name="root" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size=".05"/>
      <body name="arm_l" pos=".1 0 0"><joint axis="0 1 0"/><geom type="capsule" fromto="0 0 0 .2 0 0" size=".02"/>
        <body name="hand_l" pos=".2 0 0"><joint axis="0 1 0"/><geom type="sphere" size=".02"/></body>
      </body>
      <body name="arm_r" pos="-.1 0 0"><joint axis="0 1 0"/><geom type="capsule" fromto="0 0 0 -.2 0 0" size=".02"/></body>
      <body name="head" pos="0 0 .2"><joint axis="0 1 0"/><geom type="sphere" size=".05"/></body>
    </body>
  </worldbody>
</mujoco>
"""


def _bodies(spec: mujoco.MjSpec) -> set[str]:
    return {b.name for b in spec.bodies}


@pytest.mark.parametrize("model", ["myolegs", "myofullbody"])
def test_groups_of_a_composed_model_pair_sides(model: str) -> None:
    spec = myo_sim.load_spec(model)
    groups = default_scale_groups(spec)
    assert groups
    assert {b for g in groups for b in g} <= _bodies(spec)
    assert all(len(g) == 1 or {n[:-2] for n in g} == {g[0][:-2]} and {n[-2:] for n in g} == {"_l", "_r"} for g in groups)
    assert any(len(g) == 2 for g in groups)


@pytest.mark.parametrize("model", ["myolegs", "myofullbody"])
def test_groups_without_symmetry_are_singletons(model: str) -> None:
    spec = myo_sim.load_spec(model)
    split = default_scale_groups(spec, bilateral_symmetry=False)
    assert all(len(g) == 1 for g in split)
    assert {b for g in split for b in g} == {b for g in default_scale_groups(spec) for b in g}


def test_groups_fall_back_to_pairing_lr_bodies() -> None:
    spec = mujoco.MjSpec.from_string(TREE_XML)
    assert default_scale_groups(spec) == (("table",), ("root",), ("arm_l", "arm_r"), ("hand_l",), ("head",))
    assert all(len(g) == 1 for g in default_scale_groups(spec, bilateral_symmetry=False))


def test_inheritance_uses_the_nearest_listed_ancestor() -> None:
    spec = mujoco.MjSpec.from_string(TREE_XML)
    out = inherited_scales(spec, {"root": 1.1, "arm_l": 0.9})
    assert out == {"root": 1.1, "arm_l": 0.9, "hand_l": 0.9, "arm_r": 1.1, "head": 1.1}
    assert "table" not in out


def test_unlisted_free_root_defaults_to_one() -> None:
    spec = mujoco.MjSpec.from_string(TREE_XML)
    assert inherited_scales(spec, {"head": 1.2})["arm_l"] == 1.0


def test_without_a_free_joint_all_worldbody_children_are_roots() -> None:
    spec = mujoco.MjSpec.from_string(TREE_XML.replace("<freejoint/>", ""))
    out = inherited_scales(spec, {"table": 2.0})
    assert out["table"] == 2.0 and out["root"] == 1.0 and "head" in out


def test_scaling_with_inherited_factors_compiles() -> None:
    spec = myo_sim.load_spec("myolegs")
    groups = default_scale_groups(spec)
    factors = {b: 1.05 for g in groups for b in g}
    full = inherited_scales(spec, factors)
    assert set(full) <= _bodies(spec)
    scale_spec(spec, full).compile()
