# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
#
# Scaling rules and parts of this module are adapted from MyoSuite PR #525 by
# Mark Colley (M-Colley), Apache-2.0 (https://github.com/MyoHub/myosuite/pull/525).
"""``scale_spec`` on synthetic specs, MyoLeg and MyoFullBody.

A uniform factor ``s`` must be an exact similarity: in a matched pose every site, spatial tendon and
muscle length is ``s`` times longer and every muscle force is the same. Per-axis factors must move
everything a body carries with that body's frame.
"""

from __future__ import annotations

import subprocess
import sys
from collections.abc import Callable
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim import check_scaled_model, fit_length_ranges, model_digest, scale_report, scale_spec, segment_factors
from myo_sim.opensim_scales import (
    _COMPOSED_SEGMENTS,
    _STANDALONE_LEG_SEGMENTS,
    _body_factors,
    _segments_for,
    read_opensim_scales,
)
from myo_sim.scaling import _fit_points, _frame_rot, _geom_bottom, _peak_force, _quat, _rot, assemble_qpos, matched_poses

LEG_XML = """
<mujoco>
  <compiler eulerseq="xyz"/>
  <asset>
    <mesh name="cube" vertex="-.1 -.1 -.1 .1 -.1 -.1 .1 .1 -.1 -.1 .1 -.1 -.1 -.1 .1 .1 -.1 .1 .1 .1 .1 -.1 .1 .1"/>
  </asset>
  <worldbody>
    <body name="thigh" pos="0 0 1" euler="0 0 30">
      <joint name="hip" axis="0 1 0"/>
      <geom name="thigh_cap" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.05"/>
      <geom name="thigh_mesh" type="mesh" mesh="cube" pos="0 0 -0.2"/>
      <site name="origin" pos="0.03 0.01 -0.1"/>
      <frame name="fr" pos="0.02 0 -0.15" euler="0 45 0">
        <frame name="fr2" pos="0 0.01 0" axisangle="1 0 0 30">
          <site name="side" pos="0.04 0 0"/>
          <geom name="wrap" type="cylinder" size="0.02 0.03" zaxis="1 1 0"/>
        </frame>
        <geom name="pad" type="box" size="0.01 0.02 0.03" xyaxes="0 1 0 -1 0 0"/>
      </frame>
      <body name="shank" pos="0 0 -0.4">
        <joint name="knee" axis="0 1 0" range="-120 0"/>
        <joint name="knee_tx" type="slide" axis="1 0 0" range="-0.01 0.01"/>
        <geom name="shank_cap" type="capsule" pos="0 0 -0.2" size="0.04 0.2"/>
        <site name="insert" pos="0.03 0 -0.1"/>
        <site name="marker" type="capsule" fromto="0 0 -0.05 0 0 -0.15" size="0.005"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="ten" springlength="0.3" range="0.1 0.6" limited="true">
      <site site="origin"/>
      <geom geom="wrap" sidesite="side"/>
      <site site="insert"/>
    </spatial>
  </tendon>
  <equality>
    <joint joint1="knee_tx" joint2="knee" polycoef="0 0.01 0.002 0 0"/>
  </equality>
  <actuator>
    <muscle name="vas" tendon="ten" force="-1" scale="200"/>
  </actuator>
  <keyframe>
    <key name="bent" qpos="0.1 -0.5 0.005"/>
  </keyframe>
</mujoco>
"""

# A digit with a slide node tied to a tendon, a ligament on two hinges (coefficients are moment
# arms), a spring on a joint angle, a spring tendon between two sites and a muscle that starts on the node.
DIGIT_XML = """
<mujoco>
  <compiler angle="radian"/>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <geom type="box" size="0.02 0.02 0.05" mass="0.1"/>
      <site name="flexor_origin" pos="0.02 0 0.2"/>
      <body name="proximal" pos="0 0 -0.05">
        <joint name="mcp" axis="0 1 0" range="-1 1"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.04" size="0.008" mass="0.02"/>
        <site name="flexor_via" pos="0.01 0 -0.02"/>
        <site name="ligament_a" pos="-0.008 0 -0.01"/>
        <body name="node" pos="0.01 0 -0.03">
          <joint name="node_slide" type="slide" axis="0 0 1" damping="5" stiffness="2"/>
          <geom type="sphere" size="0.002" mass="0.001"/>
          <site name="lumbrical_origin"/>
        </body>
        <body name="middle" pos="0 0 -0.04">
          <joint name="pip" axis="0 1 0" range="-1.5 0.2"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.03" size="0.007" mass="0.01"/>
          <site name="flexor_insert" pos="0.008 0 -0.01"/>
          <site name="ligament_b" pos="-0.007 0 -0.01"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="node_coord"><joint joint="node_slide" coef="1"/></fixed>
    <spatial name="flexor">
      <site site="flexor_origin"/><site site="flexor_via"/><site site="flexor_insert"/>
    </spatial>
    <spatial name="lumbrical">
      <site site="lumbrical_origin"/><site site="flexor_insert"/>
    </spatial>
    <fixed name="retinacular" stiffness="1000" damping="4" springlength="-1000 0.002">
      <joint joint="mcp" coef="0.004"/><joint joint="pip" coef="-0.004"/>
    </fixed>
    <fixed name="angle_spring" stiffness="0.5" springlength="-1000 0.3"><joint joint="pip" coef="1"/></fixed>
    <spatial name="ligament" stiffness="800" springlength="0 0.04">
      <site site="ligament_a"/><site site="ligament_b"/>
    </spatial>
  </tendon>
  <equality>
    <tendon tendon1="node_coord" tendon2="flexor" polycoef="0 1 0 0 0"/>
  </equality>
  <actuator>
    <muscle name="fds" tendon="flexor" force="100"/>
    <muscle name="lum" tendon="lumbrical" force="10"/>
  </actuator>
</mujoco>
"""

# qpos0 floats 5 cm, "stand" touches the floor, "tilted" sinks a tilted sole 8.5 cm.
GROUND_XML = """<mujoco><worldbody>
<geom name="floor" type="plane" size="2 2 .01"/>
<body name="root" pos="0 0 1.1"><freejoint name="root_free"/>
<geom type="sphere" size=".1" mass="2"/>
<body name="foot" pos="0 0 -1"><joint name="ankle" axis="0 1 0"/>
<geom name="sole" type="box" size=".1 .2 .05" mass="1"/>
</body></body></worldbody>
<keyframe>
<key name="stand" qpos="0 0 1.05 1 0 0 0 0"/>
<key name="tilted" qpos="0.3 0 1.0 1 0 0 0 0.4"/>
</keyframe></mujoco>"""


def _leg() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string(LEG_XML)


def _digit() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string(DIGIT_XML)


def _myolegs() -> mujoco.MjSpec:
    return myo_sim.load_spec("myolegs")


def _fullbody() -> mujoco.MjSpec:
    return myo_sim.load_spec("myofullbody")


MODELS: dict[str, Callable[[], mujoco.MjSpec]] = {"leg": _leg, "digit": _digit, "myolegs": _myolegs, "fullbody": _fullbody}


def _names(spec: mujoco.MjSpec) -> list[str]:
    return [b.name for b in spec.bodies[1:]]


def _model_bytes(model: mujoco.MjModel) -> bytes:
    buffer = np.zeros(mujoco.mj_sizeModel(model), dtype=np.uint8)
    mujoco.mj_saveModel(model, None, buffer)
    return buffer.tobytes()


def _scale_set(path: Path, scales: dict[str, tuple[float, float, float]]) -> Path:
    """Write an OpenSim ScaleSet with one ``<Scale>`` per segment."""
    rows = "".join(
        f"<Scale name='{name}'><scales>{' '.join(map(str, values))}</scales><segment>{name}</segment><apply>true</apply></Scale>"
        for name, values in scales.items()
    )
    path.write_text(
        f"<OpenSimDocument Version='40000'><ScaleSet name='subject'><objects>{rows}</objects></ScaleSet></OpenSimDocument>"
    )
    return path


def _angle_tendons(model: mujoco.MjModel) -> np.ndarray:
    """Fixed tendons on hinges: their lengths stay in joint coordinates."""
    return np.array(
        [
            t
            for t in range(model.ntendon)
            if model.wrap_type[model.tendon_adr[t]] == mujoco.mjtWrap.mjWRAP_JOINT
            and model.jnt_type[model.wrap_objid[model.tendon_adr[t]]] == mujoco.mjtJoint.mjJNT_HINGE
        ],
        dtype=int,
    )


@pytest.mark.parametrize("name", ["leg", "digit", "myolegs"])
def test_factors_of_one_leave_the_model_bit_identical(name: str) -> None:
    spec = MODELS[name]()
    before = _model_bytes(spec.copy().compile())
    assert _model_bytes(scale_spec(spec, {}).compile()) == before
    assert _model_bytes(scale_spec(spec, {b: 1.0 for b in _names(spec)}).compile()) == before


@pytest.mark.parametrize("name", ["leg", "digit", "myolegs"])
def test_the_callers_spec_is_not_changed(name: str) -> None:
    spec = MODELS[name]()
    before = _model_bytes(spec.copy().compile())
    xml = spec.to_xml()
    scaled = scale_spec(spec, {b: 1.1 for b in _names(spec)}, total_mass=50)
    assert scaled is not spec
    assert spec.to_xml() == xml
    assert _model_bytes(spec.copy().compile()) == before


@pytest.mark.parametrize("name", list(MODELS))
def test_resolved_orientations_match_the_compiler(name: str) -> None:
    """Euler, axis-angle, z-axis, xy-axes and nested frames resolve as MuJoCo does."""
    spec = MODELS[name]()
    model = spec.copy().compile()

    def same(q1: np.ndarray, q2: np.ndarray) -> bool:
        return abs(abs(float(np.dot(q1, q2))) - 1.0) < 1e-9

    for body in spec.bodies[1:]:
        if body.name:
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, (_frame_rot(spec, body.frame) @ _rot(_quat(spec, body))).ravel())
            assert same(quat, model.body(body.name).quat), body.name
    for i, site in enumerate(spec.sites):
        if np.isnan(site.fromto[0]):
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, (_frame_rot(spec, site.frame) @ _rot(_quat(spec, site))).ravel())
            assert same(quat, model.site_quat[i]), site.name


@pytest.mark.parametrize("name", list(MODELS))
@pytest.mark.parametrize("s", [1.1, 0.85])
def test_one_factor_is_an_exact_similarity(name: str, s: float) -> None:
    spec = MODELS[name]()
    ref = spec.copy().compile()
    scaled = scale_spec(spec, {b: s for b in _names(spec)}).compile()
    for attr in ("nq", "nv", "nu", "nsite", "ntendon", "nbody", "neq"):
        assert getattr(scaled, attr) == getattr(ref, attr), attr

    np.testing.assert_allclose(scaled.body_mass.sum(), s**3 * ref.body_mass.sum(), rtol=1e-12)
    massive = ref.body_mass > 0
    np.testing.assert_allclose(
        np.sort(scaled.body_inertia[massive], axis=1), s**5 * np.sort(ref.body_inertia[massive], axis=1), rtol=1e-9
    )
    muscles = ref.actuator_gaintype == mujoco.mjtGain.mjGAIN_MUSCLE
    np.testing.assert_allclose(scaled.actuator_lengthrange[muscles], s * ref.actuator_lengthrange[muscles], rtol=1e-12)

    angles = _angle_tendons(ref)
    lengths = np.setdiff1d(np.arange(ref.ntendon), angles)
    data, scaled_data = mujoco.MjData(ref), mujoco.MjData(scaled)
    poses, scaled_poses = matched_poses(ref, scaled, 4, seed=0)
    for q, q_scaled in zip(poses, scaled_poses):
        data.qpos[:], scaled_data.qpos[:] = q, q_scaled
        data.act[:] = scaled_data.act[:] = 0.5
        mujoco.mj_forward(ref, data)
        mujoco.mj_forward(scaled, scaled_data)
        # The world is not scaled: body sites scale about the root body.
        on_body = ref.site_bodyid > 0
        np.testing.assert_allclose(
            scaled_data.site_xpos[on_body] - scaled_data.xpos[1],
            s * (data.site_xpos[on_body] - data.xpos[1]),
            rtol=1e-9,
            atol=1e-12,
        )
        np.testing.assert_allclose(scaled_data.ten_length[lengths], s * data.ten_length[lengths], rtol=1e-9, atol=1e-12)
        np.testing.assert_allclose(scaled_data.ten_length[angles], data.ten_length[angles], rtol=1e-9, atol=1e-12)
        np.testing.assert_allclose(scaled_data.actuator_length, s * data.actuator_length, rtol=1e-9)
        np.testing.assert_allclose(scaled_data.actuator_force[muscles], data.actuator_force[muscles], rtol=1e-9, atol=1e-9)
        equality = scaled_data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        np.testing.assert_allclose(scaled_data.efc_pos[equality], 0.0, atol=1e-9)


def test_passive_forces_of_a_similarity() -> None:
    """Spring tendons keep their force, moment-arm springs and muscles grow their torque with ``s``."""
    s = 1.2
    ref = _digit().compile()
    scaled = scale_spec(_digit(), {b: s for b in _names(_digit())}).compile()
    ligament, retinacular, angle = (ref.tendon(n).id for n in ("ligament", "retinacular", "angle_spring"))
    np.testing.assert_allclose(scaled.tendon_lengthspring[ligament], s * ref.tendon_lengthspring[ligament])
    np.testing.assert_allclose(scaled.tendon_stiffness[ligament], ref.tendon_stiffness[ligament] / s)
    # Coefficients cannot be edited in a spec: the spring carries the moment arm's factor.
    np.testing.assert_allclose(scaled.tendon_stiffness[retinacular], s * ref.tendon_stiffness[retinacular])
    np.testing.assert_allclose(scaled.tendon_damping[retinacular], s * ref.tendon_damping[retinacular])
    np.testing.assert_allclose(scaled.tendon_lengthspring[retinacular], ref.tendon_lengthspring[retinacular])
    np.testing.assert_allclose(scaled.tendon_stiffness[angle], ref.tendon_stiffness[angle])
    slide = ref.joint("node_slide")
    np.testing.assert_allclose(scaled.jnt_stiffness[slide.id], ref.jnt_stiffness[slide.id] / s)
    np.testing.assert_allclose(scaled.dof_damping[slide.dofadr[0]], ref.dof_damping[slide.dofadr[0]] / s)


def test_a_ties_slope_stays_when_only_part_of_its_path_scales() -> None:
    """A node follows its tendon's excursion, however much of the tendon was scaled."""
    ref = _digit().compile()
    scaled = scale_spec(_digit(), {"base": 1.4}).compile()
    assert scaled.tendon_length0[ref.tendon("flexor").id] > 1.2 * ref.tendon_length0[ref.tendon("flexor").id]
    np.testing.assert_allclose(scaled.eq_data[0, :5], ref.eq_data[0, :5])
    data = mujoco.MjData(scaled)
    q = scaled.qpos0.copy()
    q[scaled.joint("mcp").qposadr[0]] = 0.5
    assemble_qpos(scaled, data, q)
    mujoco.mj_forward(scaled, data)
    np.testing.assert_allclose(data.efc_pos[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY], 0.0, atol=1e-12)


def test_slide_joints_couplings_and_keyframes_scale_with_the_parent() -> None:
    ref = _leg().compile()
    scaled = scale_spec(_leg(), {"thigh": (1.2, 1.0, 1.0)}).compile()  # knee_tx slides along thigh x
    knee_tx = ref.joint("knee_tx")
    np.testing.assert_allclose(scaled.jnt_range[knee_tx.id], 1.2 * ref.jnt_range[knee_tx.id])
    np.testing.assert_allclose(scaled.eq_data[0, :5], 1.2 * ref.eq_data[0, :5])
    np.testing.assert_allclose(scaled.key_qpos[0, knee_tx.qposadr[0]], 1.2 * ref.key_qpos[0, knee_tx.qposadr[0]])
    np.testing.assert_allclose(
        scaled.tendon_lengthspring[0] / ref.tendon_lengthspring[0], scaled.tendon_range[0] / ref.tendon_range[0]
    )


def test_slide_joint_axis_resolves_nested_frames() -> None:
    xml = """<mujoco><worldbody><body name="parent"><geom type="sphere" size="0.1"/>
      <body name="child" euler="10 25 -15"><frame euler="0 0 35"><frame euler="15 -20 45">
        <joint name="slide" type="slide" axis="1 0 0" range="-0.1 0.2" ref="0.01" springref="0.02"/>
      </frame></frame><geom type="sphere" size="0.1"/></body>
    </body></worldbody></mujoco>"""
    ref = mujoco.MjSpec.from_string(xml).compile()
    factors = np.array([1.7, 1.1, 0.8])
    expected = np.linalg.norm(factors * (_rot(ref.body("child").quat) @ ref.joint("slide").axis))
    scaled = scale_spec(mujoco.MjSpec.from_string(xml), {"parent": factors}).compile()
    np.testing.assert_allclose(scaled.jnt_range, ref.jnt_range * expected)
    np.testing.assert_allclose(scaled.qpos0, ref.qpos0 * expected)
    np.testing.assert_allclose(scaled.qpos_spring, ref.qpos_spring * expected)


def test_keyframe_speeds_of_slide_joints_scale() -> None:
    xml = """<mujoco><worldbody><body name="parent"><geom size=".1"/>
      <body name="child"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><geom size=".1"/></body>
    </body></worldbody><keyframe><key qpos=".2 .3" qvel=".4 .5"/></keyframe></mujoco>"""
    scaled = scale_spec(mujoco.MjSpec.from_string(xml), {"parent": [2, 3, 1]}).compile()
    np.testing.assert_allclose(scaled.key_qpos, [[0.4, 0.9]])
    np.testing.assert_allclose(scaled.key_qvel, [[0.8, 1.5]])


def test_partial_keyframe_scales_slide_values() -> None:
    """A keyframe shorter than nq is padded with qpos0 and still scales its slides."""
    xml = """<mujoco><worldbody><body name="parent"><geom size=".1"/>
      <body name="child"><joint name="slide" type="slide" axis="1 0 0"/><joint name="hinge"/><geom size=".1"/></body>
    </body></worldbody><keyframe><key qpos=".2"/></keyframe></mujoco>"""
    np.testing.assert_allclose(scale_spec(mujoco.MjSpec.from_string(xml), {"parent": 2}).compile().key_qpos, [[0.4, 0]])


@pytest.mark.parametrize("euler", ["0 0 0", "0 0 45", "20 35 10"])
def test_sphere_radius_uses_opensim_mean_independent_of_orientation(euler: str) -> None:
    spec = mujoco.MjSpec.from_string(f"""<mujoco><worldbody><body name="body"><joint/>
      <geom name="sphere" type="sphere" size="0.1" euler="{euler}"/></body></worldbody></mujoco>""")
    factors = np.array([2.0, 1.0, 0.8])
    np.testing.assert_allclose(scale_spec(spec, {"body": factors}).compile().geom("sphere").size[0], 0.1 * factors.mean())


@pytest.mark.parametrize("kind", ["cylinder", "capsule"])
@pytest.mark.parametrize("placement", ['euler="20 35 10" size="0.1 0.2"', 'fromto="0 0 0 0.2 0.3 0.4" size="0.1"'])
def test_cylinder_radius_uses_opensim_transverse_mean(kind: str, placement: str) -> None:
    spec = mujoco.MjSpec.from_string(f"""<mujoco><worldbody><body name="body"><joint/>
      <frame euler="10 25 30"><geom name="geom" type="{kind}" {placement}/></frame>
    </body></worldbody></mujoco>""")
    ref = spec.copy().compile()
    factors = np.array([2.0, 1.0, 0.8])
    local = np.linalg.norm(factors[:, None] * _rot(ref.geom("geom").quat), axis=0)
    model = scale_spec(spec, {"body": factors}).compile()
    np.testing.assert_allclose(model.geom("geom").size[0], ref.geom("geom").size[0] * local[:2].mean())
    np.testing.assert_allclose(model.geom("geom").size[1], ref.geom("geom").size[1] * local[2])


@pytest.mark.parametrize("name", list(MODELS))
def test_per_axis_factors_move_everything_with_its_body(name: str) -> None:
    """Sites, anchors and geoms take their body's factors, a child body's position its parent's; axes stay."""
    spec = MODELS[name]()
    ref = spec.copy().compile()
    rng = np.random.default_rng(1)
    factors = {b: rng.uniform(0.8, 1.25, 3) for b in _names(spec)}
    scaled = scale_spec(spec, factors).compile()

    def s(body_id: int) -> np.ndarray:
        return factors.get(ref.body(body_id).name, np.ones(3))

    for i in range(ref.nsite):
        np.testing.assert_allclose(scaled.site_pos[i], s(ref.site_bodyid[i]) * ref.site_pos[i], atol=1e-12)
    for j in range(ref.njnt):
        np.testing.assert_allclose(scaled.jnt_pos[j], s(ref.jnt_bodyid[j]) * ref.jnt_pos[j], atol=1e-12)
    np.testing.assert_array_equal(scaled.jnt_axis, ref.jnt_axis)
    for b in range(1, ref.nbody):
        # A free root only moves up or down, to keep its ground clearance.
        free = ref.body_jntnum[b] > 0 and int(ref.jnt_type[ref.body_jntadr[b]]) == int(mujoco.mjtJoint.mjJNT_FREE)
        axes = slice(0, 2) if free else slice(0, 3)
        np.testing.assert_allclose(scaled.body_pos[b, axes], (s(ref.body_parentid[b]) * ref.body_pos[b])[axes], atol=1e-12)
    skipped = {int(mujoco.mjtGeom.mjGEOM_MESH), int(mujoco.mjtGeom.mjGEOM_PLANE), int(mujoco.mjtGeom.mjGEOM_HFIELD)}
    for g in range(ref.ngeom):
        if int(ref.geom_type[g]) not in skipped:
            np.testing.assert_allclose(scaled.geom_pos[g], s(ref.geom_bodyid[g]) * ref.geom_pos[g], atol=1e-12)


def test_inertia_is_the_exact_affine_transform() -> None:
    """A box's inertia after stretching is that of the stretched box."""
    xml = """<mujoco><worldbody><body name="b"><freejoint/>
      <geom type="box" size="{} {} {}" density="500"/></body></worldbody></mujoco>"""
    factors = np.array([1.3, 0.8, 1.1])
    scaled = scale_spec(mujoco.MjSpec.from_string(xml.format(0.1, 0.2, 0.3)), {"b": factors}).compile()
    direct = mujoco.MjSpec.from_string(xml.format(*(np.array([0.1, 0.2, 0.3]) * factors))).compile()
    np.testing.assert_allclose(scaled.body_mass, direct.body_mass, rtol=1e-12)
    np.testing.assert_allclose(np.sort(scaled.body_inertia[1]), np.sort(direct.body_inertia[1]), rtol=1e-12)


def test_rotated_mesh_scale_is_exact_for_an_axis_permutation() -> None:
    """A mesh with a reference frame and a geom turned by an axis permutation follows the body's axes."""
    vertices = "0 0 0 1 0 0 0 2 0 0 0 3"
    xml = f"""<mujoco><asset><mesh name="tetra" vertex="{vertices}" refquat="0.7071068 0.7071068 0 0"/></asset>
      <worldbody><body name="b"><joint/><geom type="mesh" mesh="tetra" euler="0 90 0"/></body></worldbody></mujoco>"""
    factors = np.array([1.5, 0.7, 1.2])

    def extent(model: mujoco.MjModel) -> np.ndarray:
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        points = model.mesh_vert @ data.geom_xmat[0].reshape(3, 3).T + data.geom_xpos[0]
        return np.ptp(points, axis=0)

    ref = mujoco.MjSpec.from_string(xml).compile()
    scaled = scale_spec(mujoco.MjSpec.from_string(xml), {"b": factors}).compile()
    np.testing.assert_allclose(extent(scaled), factors * extent(ref), rtol=1e-5)


def test_a_recreated_mesh_keeps_its_authored_fields() -> None:
    spec = _leg()
    mesh = spec.mesh("cube")
    mesh.refpos = [0.01, -0.02, 0.03]
    mesh.refquat = [0.9238795325112867, 0.0, 0.0, 0.3826834323650898]
    mesh.scale = [1.1, 0.9, 1.2]
    mesh.smoothnormal = False
    mesh.maxhullvert = 6
    mesh.info = "authored mesh provenance"
    fields = ("file", "content_type", "refpos", "refquat", "inertia", "smoothnormal", "maxhullvert", "uservert", "usernormal")
    fields += ("userface", "usertexcoord", "userfacenormal", "userfacetexcoord", "material", "info")
    before = {name: np.array(getattr(mesh, name), copy=True) for name in fields}
    spec.compile()
    scaled = scale_spec(spec, {"thigh": (0.8, 0.9, 1.1)})
    copy = scaled.mesh("cube")
    assert copy.classname.name == mesh.classname.name
    assert scaled.geom("thigh_mesh").meshname == "cube"
    for name in fields:
        np.testing.assert_array_equal(getattr(copy, name), before[name], err_msg=name)
    np.testing.assert_allclose(copy.scale, np.array([1.1, 0.9, 1.2]) * [0.8, 0.9, 1.1])
    scaled.compile()


@pytest.mark.parametrize("name", ["leg", "myolegs", "fullbody"])
def test_a_compiled_spec_scales_as_a_fresh_one(name: str) -> None:
    """Attached components stay scaled: a compiled spec freezes frames, which scaling leaves alone."""
    fresh, compiled = MODELS[name](), MODELS[name]()
    compiled.compile()
    rng = np.random.default_rng(7)
    factors = {b: rng.uniform(0.9, 1.1, 3) for b in _names(fresh)}
    expected = scale_spec(fresh, factors).compile()
    actual = scale_spec(compiled, factors).compile()
    for attr in ("body_pos", "site_pos", "geom_size", "mesh_vert", "geom_aabb", "bvh_aabb", "actuator_lengthrange"):
        np.testing.assert_array_equal(getattr(actual, attr), getattr(expected, attr), err_msg=attr)
    assert model_digest(actual) == model_digest(expected)


def test_attached_components_move_with_their_parent() -> None:
    spec = _fullbody()
    ref = spec.copy().compile()
    scaled = scale_spec(spec, {b: 1.2 for b in _names(spec)}).compile()
    for body in ("myoarm_r_root", "myoarm_l_root", "myolegs_root", "femur_r", "humerus_l", "head"):
        np.testing.assert_allclose(scaled.body(body).pos, 1.2 * ref.body(body).pos, atol=1e-12, err_msg=body)


TETRA_OBJ = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n"

# Radians, a zyx sequence and a meshdir of its own, unlike the empty MjSpec it is attached to.
ATTACHED_XML = """<mujoco><compiler angle="radian" eulerseq="zyx" meshdir="{meshdir}"/>
  <asset><mesh name="tetra" file="tetra.obj"/></asset>
  <worldbody><body name="b" euler="0.1 0.2 0.3"><joint name="j"/>
    <geom name="box" type="box" size=".1 .2 .3" euler="0.3 0 0.2"/><geom name="tetra" type="mesh" mesh="tetra"/>
    <frame euler="0 0.4 0"><site name="s" pos=".1 0 0"/></frame>
  </body></worldbody></mujoco>"""


def _in_scene(child: mujoco.MjSpec) -> mujoco.MjSpec:
    """``child`` attached to an empty MjSpec (degrees, xyz), as MyoSuite composes its scenes."""
    scene = mujoco.MjSpec()
    scene.attach(child, prefix="", suffix="", frame=scene.worldbody.add_frame())
    return scene


def test_an_attached_spec_scales_with_its_own_compiler(tmp_path: Path) -> None:
    """Angle unit, euler sequence and meshdir are those of the attached spec, as MuJoCo compiles it."""
    (tmp_path / "tetra.obj").write_text(TETRA_OBJ)
    xml = ATTACHED_XML.format(meshdir=tmp_path.as_posix())
    factors = {"b": (2.0, 1.0, 0.5)}
    scene = _in_scene(mujoco.MjSpec.from_string(xml))
    if not hasattr(scene.body("b"), "compiler"):  # MuJoCo < 3.6 hides it: refuse rather than misread
        with pytest.raises(NotImplementedError, match="compiled orientation"):
            scale_spec(scene, factors)
        return
    alone = scale_spec(mujoco.MjSpec.from_string(xml), factors).compile()
    scaled = scale_spec(scene, factors).compile()
    for attr in ("geom_size", "geom_pos", "geom_quat", "site_pos", "mesh_vert", "body_mass", "body_inertia"):
        np.testing.assert_allclose(getattr(scaled, attr), getattr(alone, attr), atol=1e-9, err_msg=attr)


def test_opensim_factors_land_on_the_same_axes_in_a_scene(tmp_path: Path) -> None:
    spec = _in_scene(_myolegs())
    if not hasattr(spec.body("pelvis"), "compiler"):
        pytest.skip("MuJoCo < 3.6 refuses an attached spec with its own angle unit")
    path = _scale_set(tmp_path / "scales.xml", {"pelvis": (1.1, 1.2, 1.3), "femur_r": (1.04, 1.08, 1.02)})
    factors = scale_report(spec, path).factors
    np.testing.assert_array_equal(factors["pelvis"], [1.1, 1.2, 1.3])
    np.testing.assert_array_equal(factors["femur_r"], [1.04, 1.08, 1.02])
    assert np.isfinite(scale_spec(spec, path).compile().body_mass).all()  # the legs' meshes are found


def test_an_attached_spec_taking_inertia_from_geoms_is_refused() -> None:
    scene = _in_scene(
        mujoco.MjSpec.from_string("""<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="a">
        <inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/><geom size=".1" mass="1"/></body></worldbody></mujoco>""")
    )
    if not hasattr(scene.body("a"), "compiler"):
        pytest.skip("MuJoCo < 3.6 does not expose an attached spec's compiler")
    with pytest.raises(NotImplementedError, match="inertiafromgeom"):
        scale_spec(scene, {"a": 2})


def _coupled_leg() -> mujoco.MjSpec:
    """The leg with a knee coupling that ``qpos0`` violates, like the patella of MyoLeg."""
    spec = _leg()
    spec.equalities[0].data[0] = 0.02
    return spec


@pytest.mark.parametrize("name", [*MODELS, "coupled_leg"])
def test_muscles_keep_their_force_in_the_assembled_default_pose(name: str) -> None:
    """OpenSim rule: fibre and tendon slack length follow the muscle-tendon length in the default pose."""
    spec = _coupled_leg() if name == "coupled_leg" else MODELS[name]()
    ref = spec.copy().compile()
    rng = np.random.default_rng(2)
    factors = {b: rng.uniform(0.85, 1.2, 3) for b in _names(spec)}
    scaled = scale_spec(spec, factors).compile()
    forces = []
    for model in (ref, scaled):
        data = mujoco.MjData(model)
        assemble_qpos(model, data)
        data.act[:] = 0.5
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.efc_pos[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY], 0, atol=1e-12)
        forces.append(data.actuator_force.copy())
    # A wrap that changes its path in this pose takes another ratio, and the report says so.
    rerouted = {item.split()[1].rstrip(":") for item in scale_report(spec, factors).skipped if "changes its path" in item}
    keep = [i for i in range(ref.nu) if ref.tendon(int(ref.actuator_trnid[i, 0])).name not in rerouted]
    assert len(rerouted) <= 0.1 * ref.ntendon
    np.testing.assert_allclose(forces[1][keep], forces[0][keep], rtol=1e-9, atol=1e-9)


def test_rectus_femoris_length_ratio_is_measured_with_the_knee_assembled() -> None:
    """The patella of MyoLeg sits 5 cm off its knee coupling at qpos0, which skews the raw ratio."""
    spec = _myolegs()
    ref = spec.copy().compile()
    scaled = scale_spec(spec, {"femur_r": 1.12, "tibia_r": 0.95}).compile()
    assert abs(ref.eq("knee_angle_beta_translation1_constraint_r").data[0]) > 0.05
    muscle = ref.actuator("recfem_r").id
    np.testing.assert_allclose(scaled.actuator_lengthrange[muscle] / ref.actuator_lengthrange[muscle], 1.0949, atol=5e-5)
    data, scaled_data = mujoco.MjData(ref), mujoco.MjData(scaled)
    mujoco.mj_forward(ref, data)
    mujoco.mj_forward(scaled, scaled_data)
    np.testing.assert_allclose(scaled_data.actuator_length[muscle] / data.actuator_length[muscle], 1.0794, atol=5e-5)


def test_a_wrap_that_changes_its_path_does_not_set_the_length_ratio() -> None:
    """A 1 % wider pelvis sends glmax3 the long way around its wrap in the default pose."""
    spec = _myolegs()
    ref = spec.copy().compile()
    factors = {"pelvis": (1.01, 1.0, 1.0)}
    model = scale_spec(spec, factors).compile()
    data, scaled_data = mujoco.MjData(ref), mujoco.MjData(model)
    assemble_qpos(ref, data)
    assemble_qpos(model, scaled_data)
    tendon = ref.tendon("glmax3_r_tendon").id
    assert scaled_data.ten_length[tendon] / data.ten_length[tendon] > 1.5
    muscle = ref.actuator("glmax3_r").id
    np.testing.assert_allclose(model.actuator_lengthrange[muscle] / ref.actuator_lengthrange[muscle], 1.0, atol=0.02)
    assert any("glmax3_r_tendon" in item for item in scale_report(spec, factors).skipped)


def test_mass_scales_with_volume_and_strength_is_kept() -> None:
    ref = _leg().compile()
    model = scale_spec(_leg(), {"thigh": 1.2}).compile()
    thigh = ref.body("thigh").id
    np.testing.assert_allclose(model.body_mass[thigh], 1.2**3 * ref.body_mass[thigh])
    total = scale_spec(_leg(), {"thigh": 1.2}, total_mass=7.5).compile()
    np.testing.assert_allclose(total.body_mass.sum(), 7.5)
    # force="-1": the peak force comes from the mass, so scaling must freeze it first.
    same = scale_spec(_leg(), {"thigh": 1.2, "shank": 1.2}).compile()
    np.testing.assert_allclose(_peak_force(same), _peak_force(ref))


@pytest.mark.parametrize("total,expected", [(None, [80, 20]), (50, [40, 10])])
def test_compiler_total_mass_is_resolved_before_scaling(total: float | None, expected: list[float]) -> None:
    spec = mujoco.MjSpec.from_string("""<mujoco><compiler settotalmass="30"/>
        <worldbody><body name="a"><geom size=".1" mass="1"/>
        <body name="b"><geom size=".1" mass="2"/></body></body></worldbody></mujoco>""")
    np.testing.assert_allclose(scale_spec(spec, {"a": 2}, total_mass=total).compile().body_mass[1:], expected)


def test_inertia_from_geoms_cannot_overrule_the_scaled_inertia() -> None:
    xml = """<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="a"><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
      <geom size=".1" mass="1"/></body></worldbody></mujoco>"""
    ref = mujoco.MjSpec.from_string(xml).compile()
    scaled = scale_spec(mujoco.MjSpec.from_string(xml), {"a": 2}).compile()
    np.testing.assert_allclose(scaled.body_mass[1], 8 * ref.body_mass[1])
    np.testing.assert_allclose(scaled.body_inertia[1], 32 * ref.body_inertia[1])


def _clearance(model: mujoco.MjModel, qpos: np.ndarray, floor: str) -> float:
    """Signed distance of the subject's collision geoms to the floor (MuJoCo's collider)."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    plane = model.geom(floor).id
    return min(
        mujoco.mj_geomDistance(model, data, g, plane, 10.0, None)
        for g in range(model.ngeom)
        if model.geom_bodyid[g] > 0 and (model.geom_contype[g] or model.geom_conaffinity[g])
    )


@pytest.mark.parametrize("scales", [{"root": 1.2, "foot": (1.1, 1.3, 1.25)}, {"foot": 0.8}])
def test_every_pose_keeps_its_ground_clearance(scales: dict) -> None:
    """qpos0 and each keyframe are aligned on their own, not by qpos0's offset."""
    spec = mujoco.MjSpec.from_string(GROUND_XML)
    ref = spec.copy().compile()
    model = scale_spec(spec, scales).compile()
    for nominal, scaled in [(ref.qpos0, model.qpos0), *zip(ref.key_qpos, model.key_qpos)]:
        assert _clearance(model, scaled, "floor") == pytest.approx(_clearance(ref, nominal, "floor"), abs=1e-9)
        np.testing.assert_array_equal(np.delete(scaled, 2), np.delete(nominal, 2))  # only the root height changes


def test_root_alignment_works_in_nested_rotated_frames() -> None:
    spec = mujoco.MjSpec.from_string("""<mujoco><worldbody>
        <geom name="floor" type="plane" size="2 2 .01"/>
        <frame pos=".1 .2 .3" euler="20 30 40"><frame pos=".1 .1 .1" euler="-10 20 30">
        <body name="root" pos="0 0 1"><freejoint/><geom name="sole" type="box" size=".1 .2 .3"/></body>
        </frame></frame></worldbody></mujoco>""")
    ref = spec.copy().compile()
    model = scale_spec(spec, {"root": (1.2, 0.9, 1.1)}).compile()
    assert _clearance(model, model.qpos0, "floor") == pytest.approx(_clearance(ref, ref.qpos0, "floor"), abs=1e-9)
    np.testing.assert_allclose(model.qpos0[:2], ref.qpos0[:2], atol=1e-12)


def test_full_body_keeps_its_clearance() -> None:
    spec = _fullbody()
    ref = spec.copy().compile()
    model = scale_spec(spec, {b: 1.1 for b in _names(spec)}).compile()
    assert _clearance(model, model.qpos0, "floor") == pytest.approx(_clearance(ref, ref.qpos0, "floor"), abs=1e-6)


@pytest.mark.parametrize(
    "kind,size,euler,extent",
    [
        ("sphere", ".1", "0 0 0", 0.1),
        ("capsule", ".1 .2", "0 0 0", 0.3),
        ("cylinder", ".1 .2", "0 0 0", 0.2),
        ("ellipsoid", ".1 .2 .3", "0 0 0", 0.3),
        ("box", ".1 .2 .3", "0 0 0", 0.3),
        ("capsule", ".1 .2", "0 45 0", 0.1 + 0.2 / np.sqrt(2)),
        ("cylinder", ".1 .2", "0 45 0", 0.3 / np.sqrt(2)),
        ("ellipsoid", ".1 .2 .3", "0 45 0", np.sqrt(0.05)),
        ("box", ".1 .2 .3", "0 45 0", 0.4 / np.sqrt(2)),
    ],
)
def test_geom_bottom_is_the_support_of_each_shape(kind: str, size: str, euler: str, extent: float) -> None:
    model = mujoco.MjModel.from_xml_string(f"""<mujoco><worldbody><body pos="0 0 1"><freejoint/>
        <geom type="{kind}" size="{size}" euler="{euler}"/></body></worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    assert _geom_bottom(model, data, 0) == pytest.approx(1 - extent)


def test_geom_bottom_uses_the_transformed_mesh_vertices() -> None:
    model = mujoco.MjModel.from_xml_string("""<mujoco><asset><mesh name="tetra" vertex="0 0 0 1 0 0 0 2 0 0 0 3"/></asset>
        <worldbody><body pos="0 0 1"><freejoint/><geom type="mesh" mesh="tetra" pos="0 0 -.2" euler="0 45 0"/></body>
        </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    # The authored vertex (1, 0, 0) is lowest after the turn about y.
    assert _geom_bottom(model, data, 0) == pytest.approx(1 - 0.2 - 1 / np.sqrt(2), abs=2e-7)


def _leg_in_scene(with_agent: bool) -> mujoco.MjSpec:
    """The leg next to a heavy static prop and, optionally, a free-floating agent."""
    spec = _leg()
    spec.worldbody.add_body(name="prop", pos=[1, 0, 0]).add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.2, 0.2], mass=40.0
    )
    if with_agent:
        agent = spec.worldbody.add_body(name="agent", pos=[-1, 0, 1])
        agent.add_freejoint()
        agent.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1, 0, 0], mass=60.0)
    return spec


@pytest.mark.parametrize("scales,with_agent", [({"thigh": 1.2}, True), ({"thigh": 1.0}, True), ({}, False)])
def test_total_mass_rescales_only_the_subject(scales: dict[str, float], with_agent: bool) -> None:
    """Static scenery and other agents in a composed scene keep their mass."""
    spec = _leg_in_scene(with_agent)
    ref = spec.copy().compile()
    model = scale_spec(spec, scales, total_mass=7.5).compile()
    subject = [model.body(name).id for name in ("thigh", "shank")]
    np.testing.assert_allclose(model.body_mass[subject].sum(), 7.5)
    for name in ("prop", "agent") if with_agent else ("prop",):
        np.testing.assert_allclose(model.body(name).mass, ref.body(name).mass)


def test_total_mass_without_a_named_subject_must_be_unambiguous() -> None:
    with pytest.raises(ValueError, match="ambiguous"):
        scale_spec(_leg_in_scene(with_agent=True), {}, total_mass=7.5)


def _shared_mesh_leg() -> mujoco.MjSpec:
    spec = _leg()
    spec.body("shank").add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="cube")
    return spec


def _welded_leg() -> mujoco.MjSpec:
    spec = _leg()
    spec.add_equality(type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_BODY, name1="shank", name2="world")
    return spec


HFIELD_XML = """<mujoco><asset><hfield name="h" nrow="3" ncol="3" size="1 1 .1 .1"/></asset>
<worldbody><body name="a"><geom size=".1"/></body>
<body name="b" pos="0 0 1"><geom type="hfield" hfield="h"/></body></worldbody></mujoco>"""

TWO_AGENTS_XML = """<mujoco><compiler settotalmass="30"/><worldbody>
<body name="a"><freejoint/><geom size=".1" mass="1"/></body>
<body name="b" pos="1 0 0"><freejoint/><geom size=".1" mass="2"/></body>
</worldbody></mujoco>"""

FUSED_XML = """<mujoco><compiler fusestatic="true"/><worldbody>
<body name="a"><joint/><geom size=".1"/><body name="b"><geom size=".1"/></body></body></worldbody></mujoco>"""

FLEX_XML = """<mujoco><worldbody><body name="a"><geom size=".1"/></body><body name="b" pos="1 0 0"><geom size=".1"/></body>
<body name="c" pos="0 1 0"><geom size=".1"/></body></worldbody>
<deformable><flex name="f" dim="2" body="a b c" vertex="0 0 0 0 0 0 0 0 0" element="0 1 2"><edge damping="1"/></flex></deformable></mujoco>"""


@pytest.mark.parametrize(
    "make,scales,total_mass,error",
    [
        (_shared_mesh_leg, {"thigh": 1.1}, None, NotImplementedError),
        (_shared_mesh_leg, {"thigh": 1.1, "shank": 1.2}, None, NotImplementedError),
        (_welded_leg, {"shank": 1.1}, None, NotImplementedError),
        (lambda: mujoco.MjSpec.from_string(HFIELD_XML), {"a": 1.1, "b": 1.1}, None, NotImplementedError),
        (lambda: mujoco.MjSpec.from_string(FUSED_XML), {"a": 1.1}, None, NotImplementedError),
        (lambda: mujoco.MjSpec.from_string(FUSED_XML), {}, 5.0, NotImplementedError),
        (lambda: mujoco.MjSpec.from_string(FLEX_XML), {"a": 1.1}, None, NotImplementedError),
        (lambda: mujoco.MjSpec.from_string(TWO_AGENTS_XML), {}, 50.0, ValueError),
    ],
    ids=["mesh-also-unscaled", "mesh-scaled-twice", "connect", "height-field", "fusestatic", "fusestatic-total-mass", "flex", "ambiguous-mass"],
)
def test_refused_scaling_leaves_the_spec_unchanged(
    make: Callable[[], mujoco.MjSpec], scales: dict, total_mass: float | None, error: type[Exception]
) -> None:
    """Everything that can refuse a scaling is checked before the first edit."""
    spec = make()
    xml, before = spec.to_xml(), _model_bytes(spec.copy().compile())
    with pytest.raises(error):
        scale_spec(spec, scales, total_mass=total_mass)
    assert spec.to_xml() == xml
    assert _model_bytes(spec.copy().compile()) == before


def test_refusals_name_what_they_refuse() -> None:
    with pytest.raises(NotImplementedError, match="cube"):
        scale_spec(_shared_mesh_leg(), {"thigh": 1.1})
    with pytest.raises(NotImplementedError, match="connect/weld"):
        scale_spec(_welded_leg(), {"shank": 1.1})


def test_site_anchored_weld_follows_its_sites() -> None:
    xml = """<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><geom size=".1"/><site name="s" pos=".1 0 0"/></body>
      <site name="w" pos=".1 0 0"/></worldbody><equality><weld site1="s" site2="w"/></equality></mujoco>"""
    model = scale_spec(mujoco.MjSpec.from_string(xml), {"a": 2}).compile()
    np.testing.assert_allclose(model.site("s").pos, [0.2, 0, 0])


def test_unknown_bodies_and_bad_values_are_refused() -> None:
    with pytest.raises(KeyError, match="nope"):
        scale_spec(_leg(), {"nope": 1.1})
    with pytest.raises(KeyError, match="world"):
        scale_spec(_leg(), {"world": 1.0})


@pytest.mark.parametrize("factor", [0.0, -1.0, np.nan, np.inf, -np.inf])
def test_invalid_factors_and_masses_are_refused(factor: float) -> None:
    with pytest.raises(ValueError, match="finite and positive"):
        scale_spec(_leg(), {"thigh": [1.0, factor, 1.0]})
    with pytest.raises(ValueError, match="total_mass must be finite and positive"):
        scale_spec(_leg(), {"thigh": 1.2}, total_mass=factor)


def test_segment_factors() -> None:
    np.testing.assert_array_equal(segment_factors("y", 1.2, 0.9), [0.9, 1.2, 0.9])
    with pytest.raises(ValueError, match="long_axis"):
        segment_factors("w", 1.0, 1.0)
    ref = _myolegs().compile()
    scaled = scale_spec(_myolegs(), {"femur_r": segment_factors("y", 1.2, 1.0)}).compile()
    tibia = ref.body("tibia_r").id
    np.testing.assert_allclose(scaled.body_pos[tibia, 1], 1.2 * ref.body_pos[tibia, 1])
    femur_l = ref.body("femur_l").id
    np.testing.assert_allclose(scaled.body_pos[femur_l], ref.body_pos[femur_l])


def test_report_lists_what_is_approximated_and_skipped(tmp_path: Path) -> None:
    spec = _digit()
    report = scale_report(spec, {"middle": (1.1, 1.0, 0.9), "proximal": 1.2})
    assert report.bodies_scaled == 2
    assert any("angle_spring" in item for item in report.skipped)
    assert not any("retinacular" in item for item in report.skipped)
    uniform = scale_report(_leg(), {"thigh": 1.2, "shank": 1.2})
    assert not uniform.approximated_geoms
    rotated = scale_report(_leg(), {"thigh": (1.2, 1.0, 0.9)})
    assert "wrap" in rotated.approximated_geoms and "thigh_cap" in rotated.approximated_geoms
    aligned = mujoco.MjSpec.from_string("""<mujoco><worldbody><body name="b"><joint/><geom name="box" type="box" size=".1 .2 .3" euler="0 0 90"/>
        </body></worldbody></mujoco>""")
    assert not scale_report(aligned, {"b": (1.2, 1.0, 0.9)}).approximated_geoms
    tilted = mujoco.MjSpec.from_string("""<mujoco><worldbody><body name="b"><joint/><geom name="box" type="box" size=".1 .2 .3" euler="0 0 30"/>
        </body></worldbody></mujoco>""")
    assert scale_report(tilted, {"b": (1.2, 1.0, 0.9)}).approximated_geoms == ["box"]
    path = _scale_set(tmp_path / "scales.xml", {"femur_r": (1.1,) * 3, "unknown_segment": (1.1,) * 3})
    assert any("unknown_segment" in item for item in scale_report(_myolegs(), path).skipped)


@pytest.mark.parametrize(
    "euler,factors,approximate",
    [
        ("0 0 45", (1.2, 1.0, 0.9), True),  # both radial factors are equal, but the section is an ellipse
        ("0 0 30", (1.2, 1.0, 0.9), True),
        ("0 0 0", (1.2, 1.0, 0.9), True),
        ("0 0 45", (1.1, 1.1, 0.9), False),  # a circle stays a circle
        ("0 30 0", (1.1, 1.1, 0.9), True),
    ],
)
def test_report_lists_a_cylinder_whose_section_stops_being_a_circle(euler: str, factors: tuple, approximate: bool) -> None:
    spec = mujoco.MjSpec.from_string(f"""<mujoco><worldbody><body name="b"><joint/>
        <geom name="g" type="cylinder" size=".1 .2" euler="{euler}"/></body></worldbody></mujoco>""")
    assert scale_report(spec, {"b": factors}).approximated_geoms == (["g"] if approximate else [])


def test_rajagopal_segments_map_onto_myofullbody(tmp_path: Path) -> None:
    spec = _fullbody()
    assert _segments_for(spec).keys() == _COMPOSED_SEGMENTS.keys()
    scales = {
        "torso": np.full(3, 1.1),
        "hand_r": np.full(3, 0.9),
        "femur_r": np.array([1.0, 1.2, 1.0]),
        "humerus_l": np.full(3, 1.05),
    }
    resolved = _body_factors(spec, scales, _COMPOSED_SEGMENTS)
    resolved = {name: s for name, s in resolved.items() if not np.array_equal(s, np.ones(3))}
    np.testing.assert_array_equal(resolved["lumbar5"], [1.1] * 3)
    np.testing.assert_array_equal(resolved["humphant_r"], [1.1] * 3)  # shoulder helpers follow the torso
    np.testing.assert_array_equal(resolved["head"], [1.1] * 3)
    np.testing.assert_array_equal(resolved["femur_r"], [1.0, 1.2, 1.0])
    assert "tibia_r" not in resolved and "sacrum" not in resolved and "pelvis" not in resolved  # missing segments stay
    hand = [name for name, s in resolved.items() if np.array_equal(s, [0.9] * 3)]
    assert len(hand) == 27 and "lunate_r" in hand
    model = scale_spec(spec, _scale_set(tmp_path / "scales.xml", {"torso": (1.1,) * 3, "hand_r": (0.9,) * 3})).compile()
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))


def test_opensim_file_scales_the_mapped_bodies(tmp_path: Path) -> None:
    spec = _myolegs()
    segments = {"femur_r": (1.04, 1.08, 1.02), "tibia_l": (0.97,) * 3}
    by_file = scale_spec(spec, _scale_set(tmp_path / "scales.xml", segments), total_mass=70).compile()
    by_body = scale_spec(spec, {"Full Body": 1.0, "femur_r": segments["femur_r"], "tibia_l": 0.97}, total_mass=70).compile()
    np.testing.assert_allclose(by_file.body_mass, by_body.body_mass, rtol=1e-12)
    np.testing.assert_array_equal(by_file.site_pos, by_body.site_pos)
    np.testing.assert_allclose(by_file.body_mass.sum(), 70)
    with pytest.raises(ValueError, match="No OpenSim segment mapping"):
        scale_spec(_leg(), tmp_path / "scales.xml")


def test_opensim_factors_land_on_the_right_axes_of_z_up_bodies() -> None:
    """OpenSim's frames are y-up; the root of the composed models is z-up with a lateral x."""
    spec = _fullbody()
    pelvis = np.array([1.1, 1.2, 1.3])
    resolved = _body_factors(spec, {"pelvis": pelvis}, _COMPOSED_SEGMENTS)
    np.testing.assert_array_equal(resolved["Full Body"], [1.3, 1.1, 1.2])
    for body in ("sacrum", "pelvis"):
        np.testing.assert_array_equal(resolved[body], pelvis)
    np.testing.assert_array_equal(resolved["myolegs_root"], [1.3, 1.1, 1.2])


def test_segment_axis_maps_match_the_model_frames() -> None:
    """Each first body's axes lie along the OpenSim axes its map names; the femur frame is OpenSim's."""
    model = _fullbody().compile()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    opensim = data.xmat[model.body("femur_r").id].reshape(3, 3)
    for segment, (bodies, axes) in _COMPOSED_SEGMENTS.items():
        for body in bodies:
            rot = opensim.T @ data.xmat[model.body(body).id].reshape(3, 3)
            np.testing.assert_allclose(np.abs(rot).max(axis=0), 1.0, atol=1e-5, err_msg=segment)
            assert tuple(np.argmax(np.abs(rot), axis=0)) == axes, segment


@pytest.mark.parametrize("name", ["myolegs", "myoarm_r", "myohand_r", "myohands", "myotorso_arms"])
def test_partial_models_take_the_segments_they_have(name: str, tmp_path: Path) -> None:
    spec = myo_sim.load_spec(name)
    path = _scale_set(tmp_path / "scales.xml", {segment: (1.05, 1.1, 0.95) for segment in _COMPOSED_SEGMENTS})
    report = scale_report(spec, path)
    assert report.bodies_scaled > 0
    assert np.isfinite(scale_spec(spec, path).compile().body_mass).all()


def test_standalone_leg_model_takes_its_own_axis_map(tmp_path: Path) -> None:
    spec = myo_sim.load_spec("myolegs_osl")
    if not {"root", "torso"} <= set(_names(spec)):
        pytest.skip("this tree's stand-alone leg has other body names")
    assert _segments_for(spec).keys() <= _STANDALONE_LEG_SEGMENTS.keys()
    resolved = _body_factors(spec, {"torso": np.array([1.0, 1.2, 1.0])}, _segments_for(spec))
    np.testing.assert_array_equal(resolved["torso"], [1.0, 1.0, 1.2])


def test_read_opensim_scales(tmp_path: Path) -> None:
    scale_set = tmp_path / "scale_set.xml"
    scale_set.write_text("""<OpenSimDocument Version="40000"><ScaleSet name="subject"><objects>
      <Scale name="s1"><scales> 1.1 1.2 1.3</scales><segment>femur_r</segment><apply>true</apply></Scale>
      <Scale name="s2"><scales> 0.9 0.9 0.9</scales><segment>torso</segment><apply>false</apply></Scale>
    </objects></ScaleSet></OpenSimDocument>""")
    scales = read_opensim_scales(scale_set)
    assert set(scales) == {"femur_r"}  # apply=false is skipped
    np.testing.assert_allclose(scales["femur_r"], [1.1, 1.2, 1.3])
    scaled = tmp_path / "scaled.osim"
    scaled.write_text("""<OpenSimDocument Version="40000"><Model name="m"><BodySet><objects>
      <Body name="femur_r"><attached_geometry><Mesh name="g"><scale_factors>1.05 1.05 1.05</scale_factors></Mesh></attached_geometry></Body>
    </objects></BodySet></Model></OpenSimDocument>""")
    np.testing.assert_allclose(read_opensim_scales(scaled)["femur_r"], [1.05] * 3)


def test_a_body_cannot_start_two_segments() -> None:
    segments = {"a": (("thigh",), (0, 1, 2)), "b": (("thigh",), (0, 1, 2))}
    with pytest.raises(ValueError, match="more than one segment"):
        _body_factors(_leg(), {"a": np.full(3, 1.1), "b": np.full(3, 1.2)}, segments)


@pytest.mark.parametrize("factor", [0.0, -1.0, np.nan, np.inf])
def test_malformed_factors_are_refused_even_for_missing_segments(factor: float, tmp_path: Path) -> None:
    path = _scale_set(tmp_path / "scales.xml", {"femur_r": (1.1, 1.1, 1.1), "humerus_r": (1.0, factor, 1.0)})
    with pytest.raises(ValueError, match="finite and positive"):
        scale_spec(_myolegs(), path)


def _measured(points: np.ndarray, factors: float | np.ndarray) -> np.ndarray:
    """Points scaled along the body axes, then seen from a rotated, shifted lab."""
    rot = np.zeros(9)
    quat = np.zeros(4)
    mujoco.mju_axisAngle2Quat(
        quat, np.array([0.3, -0.5, 1.2]) / np.linalg.norm([0.3, -0.5, 1.2]), np.linalg.norm([0.3, -0.5, 1.2])
    )
    mujoco.mju_quat2Mat(rot, quat)
    return (np.asarray(points) * factors) @ rot.reshape(3, 3).T + [0.4, -0.2, 1.1]


def test_points_spanning_three_directions_give_three_factors() -> None:
    points = np.random.default_rng(3).uniform(-0.1, 0.1, (6, 3))
    factors = np.array([1.08, 0.95, 1.12])
    by_points = scale_spec(_leg(), {"thigh": (points, _measured(points, factors))}).compile()
    by_factors = scale_spec(_leg(), {"thigh": factors}).compile()
    np.testing.assert_allclose(by_points.site_pos, by_factors.site_pos, atol=1e-9)
    np.testing.assert_allclose(by_points.geom_size, by_factors.geom_size, atol=1e-9)
    np.testing.assert_allclose(by_points.actuator_lengthrange, by_factors.actuator_lengthrange, rtol=1e-8)


def test_a_marker_pair_or_planar_points_give_one_factor() -> None:
    points = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, -0.4], [0.05, 0.0, -0.2]])
    measured = _measured(points, 1.07)
    expected = scale_spec(_leg(), {"thigh": 1.07}).compile()
    for count in (2, 3):  # two points: the ratio of their distances, as OpenSim's marker pairs
        model = scale_spec(_leg(), {"thigh": (points[:count], measured[:count])}).compile()
        np.testing.assert_allclose(model.site_pos, expected.site_pos, atol=1e-12)
    np.testing.assert_allclose(_fit_points("thigh", points[:2], measured[:2] + [0.0, 0.0, 0.01]), np.full(3, 1.07))


def test_measured_points_may_be_y_up_or_z_up() -> None:
    """The fit includes a rotation, so a y-up and a z-up lab agree."""
    points = np.random.default_rng(6).uniform(-0.1, 0.1, (6, 3))
    y_up = _measured(points, [1.08, 0.95, 1.12])
    z_up = y_up @ np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]).T
    np.testing.assert_allclose(_fit_points("thigh", points, z_up), [1.08, 0.95, 1.12])
    np.testing.assert_allclose(
        _fit_points("thigh", points[:2], z_up[:2]), _fit_points("thigh", points[:2], y_up[:2]), rtol=1e-12
    )


@pytest.mark.parametrize("seed", range(5))
def test_points_recover_known_factors_with_noise_free_data(seed: int) -> None:
    rng = np.random.default_rng(seed)
    points = rng.uniform(-0.1, 0.1, (8, 3))
    factors = rng.uniform(0.7, 1.4, 3)
    np.testing.assert_allclose(_fit_points("b", points, _measured(points, factors)), factors, rtol=1e-8)


def test_point_and_factor_entries_mix() -> None:
    points = np.random.default_rng(4).uniform(-0.1, 0.1, (5, 3))
    measured = _measured(points, [1.1, 1.0, 0.9])
    fitted = _fit_points("thigh", points, measured)
    np.testing.assert_allclose(fitted, [1.1, 1.0, 0.9])
    mixed = scale_spec(_leg(), {"thigh": (points, measured), "shank": 1.05}).compile()
    direct = scale_spec(_leg(), {"thigh": fitted, "shank": 1.05}).compile()
    assert _model_bytes(mixed) == _model_bytes(direct)


def test_missing_points_are_skipped_and_unfit_points_refused() -> None:
    points = np.random.default_rng(5).uniform(-0.1, 0.1, (6, 3))
    measured = _measured(points, [1.1, 1.0, 0.9])
    missing = measured.copy()
    missing[0] = np.nan  # an occluded marker
    np.testing.assert_allclose(_fit_points("thigh", points, missing), [1.1, 1.0, 0.9])
    with pytest.raises(ValueError, match="metres"):  # measured in millimetres
        scale_spec(_leg(), {"thigh": (points, 1000 * measured)})
    with pytest.raises(ValueError, match="shape"):
        scale_spec(_leg(), {"thigh": (points, measured[:3])})
    with pytest.raises(ValueError, match="two or more"):
        scale_spec(_leg(), {"thigh": (points[:1], measured[:1])})
    with pytest.raises(ValueError, match="NaN"):
        scale_spec(_leg(), {"thigh": (points, np.where(missing > 0, np.inf, 0.0))})


def test_model_digest_tracks_the_model() -> None:
    source = mujoco.MjSpec.from_string(GROUND_XML)
    scaled = model_digest(scale_spec(source, {"root": 1.1, "foot": 1.2}).compile())
    assert scaled == model_digest(scale_spec(source, {"foot": [1.2] * 3, "root": [1.1] * 3}).compile())
    changed = mujoco.MjSpec.from_string(GROUND_XML)
    changed.geom("sole").size[0] *= 2
    assert model_digest(scale_spec(changed, {"root": 1.1, "foot": 1.2}).compile()) != scaled


def test_model_digest_does_not_depend_on_where_the_assets_are(tmp_path: Path) -> None:
    tetra = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n"
    models = []
    for folder in ("a", "a/much/deeper/folder"):
        root = tmp_path / folder
        root.mkdir(parents=True)
        (root / "tetra.obj").write_text(tetra)
        spec = mujoco.MjSpec.from_string(f"""<mujoco><asset><mesh name="tetra" file="{(root / "tetra").as_posix()}.obj"/></asset>
            <worldbody><body name="root" pos="0 0 1"><freejoint/><geom type="mesh" mesh="tetra"/></body></worldbody></mujoco>""")
        models.append(scale_spec(spec, {"root": 1.1}).compile())
    assert models[0].paths != models[1].paths
    assert model_digest(models[0]) == model_digest(models[1])


def test_model_digest_is_stable_across_processes() -> None:
    code = f"import mujoco; from myo_sim import model_digest; print(model_digest(mujoco.MjSpec.from_string({GROUND_XML!r}).compile()))"
    result = subprocess.run([sys.executable, "-c", code], text=True, capture_output=True, check=True)
    assert result.stdout.splitlines()[-1] == model_digest(mujoco.MjSpec.from_string(GROUND_XML).compile())


FEMUR_TIBIA = {
    "femur_r": segment_factors("y", 1.15, 0.97),
    "tibia_r": segment_factors("y", 0.92, 1.03),
    "femur_l": segment_factors("y", 1.05, 1.01),
    "tibia_l": segment_factors("y", 1.1, 0.96),
    "pelvis": (1.04, 0.98, 1.06),
    "calcn_r": segment_factors("x", 1.08, 0.98),
}


def test_refit_improves_the_normalised_length_match() -> None:
    spec = _myolegs()
    reference = spec.copy().compile()
    scaled = scale_spec(spec, FEMUR_TIBIA)
    before = check_scaled_model(scaled.copy().compile(), reference)["operating_range"].value
    refitted, r2 = fit_length_ranges(scaled, reference)
    after = check_scaled_model(refitted.compile(), reference)["operating_range"].value
    assert after < 0.6 * before
    assert after < 0.05
    assert set(r2) == {reference.actuator(i).name for i in range(reference.nu)}
    assert np.median(list(r2.values())) > 0.99


def test_refit_leaves_a_similarity_alone_and_the_input_unchanged() -> None:
    spec = _myolegs()
    reference = spec.copy().compile()
    scaled = scale_spec(spec, {b: 1.1 for b in _names(spec)})
    before = scaled.to_xml()
    refitted, r2 = fit_length_ranges(scaled, reference)
    assert scaled.to_xml() == before
    np.testing.assert_allclose(refitted.compile().actuator_lengthrange, 1.1 * reference.actuator_lengthrange, rtol=1e-9)
    assert min(r2.values()) > 0.999


def test_refit_keeps_the_range_of_a_muscle_that_does_not_fit() -> None:
    spec = _digit()
    reference = spec.copy().compile()
    scaled = scale_spec(spec, {"proximal": 1.1})
    refitted, r2 = fit_length_ranges(scaled, reference, min_r2=1.1)
    assert all(value <= 1.0 for value in r2.values())
    np.testing.assert_array_equal(refitted.compile().actuator_lengthrange, scaled.compile().actuator_lengthrange)


def test_checks_pass_on_the_reference_and_on_a_scaled_model() -> None:
    reference = _myolegs().compile()
    assert check_scaled_model(reference, reference).passed
    spec = _myolegs()
    assert check_scaled_model(scale_spec(spec, {b: 1.1 for b in _names(spec)}).compile(), reference).passed


def test_checks_catch_a_bad_model() -> None:
    """Each deliberate error fails its own check and only that check."""
    spec = _myolegs()
    reference = spec.copy().compile()
    good = scale_spec(spec, {b: 1.1 for b in _names(spec)})

    stale = good.copy().compile()
    stale.actuator_lengthrange[:] = reference.actuator_lengthrange  # the range of the old size
    result = check_scaled_model(stale, reference)
    assert not result["operating_range"].passed and result["moment_arm_signs"].passed and result["stability"].passed

    flipped = good.copy()
    for site in flipped.sites:
        if site.parent.name in ("pelvis", "femur_r", "femur_l", "tibia_r", "tibia_l"):
            site.pos = -np.asarray(site.pos)  # path points on the far side of each joint
    result = check_scaled_model(flipped.compile(), reference)
    assert not result["moment_arm_signs"].passed

    unstable = good.copy().compile()
    unstable.opt.timestep = 0.5
    unstable.dof_armature[:] = 0.0
    result = check_scaled_model(unstable, reference)
    assert not result["stability"].passed

    loose = good.copy()
    loose.equalities[0].solref = [10.0, 1.0]
    for eq in loose.equalities:
        eq.solref = [2.0, 1.0]
    result = check_scaled_model(loose.compile(), reference, max_constraint=1e-4)
    assert not result["constraints"].passed


def test_checks_of_a_model_without_a_limited_undriven_hinge() -> None:
    """Moment arms have no joint to be measured on, so that check holds vacuously."""
    spec = mujoco.MjSpec.from_string("""<mujoco><worldbody><site name="fixed" pos="-1 0 0"/>
        <body name="b"><joint type="slide" axis="1 0 0" range="-.1 .1"/><geom size=".1"/><site name="s"/>
        <body name="c" pos="0 0 1"><joint axis="0 1 0"/><geom size=".1"/></body></body></worldbody>
        <tendon><spatial name="t"><site site="fixed"/><site site="s"/></spatial></tendon>
        <actuator><muscle tendon="t"/></actuator></mujoco>""")
    reference = spec.copy().compile()
    result = check_scaled_model(scale_spec(spec, {"b": 1.1}).compile(), reference, steps=4)
    assert result["moment_arm_signs"].passed and result["moment_arm_signs"].value == 1.0


def test_imports_are_lazy() -> None:
    code = "import sys, myo_sim; print('myo_sim.scaling' in sys.modules, 'mujoco' in sys.modules)"
    result = subprocess.run([sys.executable, "-c", code], text=True, capture_output=True, check=True)
    assert result.stdout.split()[0] == "False"
