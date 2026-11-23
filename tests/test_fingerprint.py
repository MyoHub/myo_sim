"""Musculoskeletal fingerprint and tendon-node projection."""

from __future__ import annotations

import copy

import mujoco
import numpy as np

from myo_sim.fingerprint import musculoskeletal_fingerprint
from myo_sim.physics.joint_coupling import project_tendon_nodes

_TWO_MUSCLE_ARM_XML = """
<mujoco model="two_muscle_hinge">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="upper" pos="0 0 0.5">
      <site name="flex_orig" pos="0.04 0 0.12"/>
      <site name="ext_orig" pos="-0.04 0 0.12"/>
      <site name="lig_orig" pos="0.02 0 0.04"/>
      <site name="elec_flex" pos="0.04 0 0.06"/>
      <site name="elec_ext" pos="-0.04 0 0.06"/>
      <body name="forearm" pos="0 0 0">
        <inertial pos="0 0 -0.15" mass="1.2" diaginertia="0.012 0.012 0.002"/>
        <joint name="elbow_flexion" type="hinge" axis="0 -1 0" range="-1.5 1.5" damping="0.8"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.30" size="0.025"/>
        <site name="flex_ins" pos="0.035 0 -0.05"/>
        <site name="ext_ins" pos="-0.035 0 -0.05"/>
        <site name="lig_ins" pos="0.02 0 -0.04"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="bic_tendon"><site site="flex_orig"/><site site="flex_ins"/></spatial>
    <spatial name="tri_tendon"><site site="ext_orig"/><site site="ext_ins"/></spatial>
    <spatial name="capsule_ligament" stiffness="25.0" damping="0.5" springlength="0.075 0.095">
      <site site="lig_orig"/><site site="lig_ins"/>
    </spatial>
  </tendon>
  <actuator>
    <muscle name="biceps_r" tendon="bic_tendon" force="120" lengthrange="0.09 0.20"/>
    <muscle name="triceps_r" tendon="tri_tendon" force="120" lengthrange="0.09 0.20"/>
  </actuator>
</mujoco>
"""

_SLIDE_TENDON_NODE_XML = """
<mujoco model="slide_tendon_node">
  <compiler angle="radian"/>
  <worldbody>
    <body name="palm" pos="0 0 0">
      <site name="fdp_orig" pos="0.02 0 0.08"/>
      <body name="node_lum" pos="0.02 0 0.02">
        <inertial pos="0 0 0" mass="0.001" diaginertia="1e-6 1e-6 1e-6"/>
        <joint name="node_LUM1_r" type="slide" axis="0 0 1" range="-0.05 0.05"/>
      </body>
      <body name="finger" pos="0 0 0">
        <inertial pos="0 0 -0.04" mass="0.03" diaginertia="1e-5 1e-5 1e-5"/>
        <joint name="mcp2_flexion_r" type="hinge" axis="0 -1 0" range="-0.2 1.4"/>
        <site name="fdp_ins" pos="0.015 0 -0.03"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="node_LUM1_t"><joint joint="node_LUM1_r" coef="1.0"/></fixed>
    <spatial name="fdp2_tendon"><site site="fdp_orig"/><site site="fdp_ins"/></spatial>
  </tendon>
  <equality>
    <tendon name="lum_tie" tendon1="node_LUM1_t" tendon2="fdp2_tendon" polycoef="0 1.0 0 0 0"/>
  </equality>
</mujoco>
"""


def _arm() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(_TWO_MUSCLE_ARM_XML)


def test_identical_models_share_fingerprint() -> None:
    fp = musculoskeletal_fingerprint(_arm())
    assert len(fp) == 16
    assert fp == musculoskeletal_fingerprint(_arm())


def test_musculoskeletal_changes_alter_fingerprint() -> None:
    base = _arm()
    fp = musculoskeletal_fingerprint(base)

    ligament = copy.deepcopy(base)
    ligament.tendon_stiffness[ligament.tendon("capsule_ligament").id] = 60.0
    force = copy.deepcopy(base)
    force.actuator_gainprm[0, 2] *= 1.25
    site = mujoco.MjModel.from_xml_string(_TWO_MUSCLE_ARM_XML.replace('pos="0.035 0 -0.05"', 'pos="0.055 0 -0.05"'))

    fps = {musculoskeletal_fingerprint(m) for m in (ligament, force, site)}
    assert len(fps) == 3 and fp not in fps


def test_visual_change_keeps_fingerprint() -> None:
    model = _arm()
    fp = musculoskeletal_fingerprint(model)
    model.geom_rgba[:] = [1.0, 0.0, 0.0, 1.0]
    assert musculoskeletal_fingerprint(model) == fp


def test_project_tendon_nodes_satisfies_tendon_equality() -> None:
    model = mujoco.MjModel.from_xml_string(_SLIDE_TENDON_NODE_XML)
    q = np.zeros((5, model.nq))
    q[:, 1] = np.linspace(0.0, 1.1, 5)
    q_proj = project_tendon_nodes(model, q)
    assert q_proj.shape == q.shape and not np.allclose(q_proj[:, 0], q[:, 0])

    data = mujoco.MjData(model)
    for row in q_proj:
        data.qpos[:] = row
        mujoco.mj_forward(model, data)
        assert abs(data.efc_pos[0]) < 1e-10

    single = project_tendon_nodes(model, q[3])
    assert single.shape == (model.nq,)
    np.testing.assert_array_equal(single, q_proj[3])
    assert np.all(q[:, 0] == 0.0)


def test_project_tendon_nodes_without_ties_returns_copy() -> None:
    model = _arm()
    q = model.qpos0.copy()
    out = project_tendon_nodes(model, q)
    np.testing.assert_array_equal(out, q)
    assert out is not q
