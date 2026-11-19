# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Pose normalization and inside-wrap predicates remain valid at finite extremes."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import pytest

from myo_sim.authoring import export_model, model_fingerprint


@pytest.fixture(scope="module")
def articulated_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><joint name="angle"/><geom size=".05"/></body>'
        '<body name="root" pos=".2 .3 .4"><freejoint name="free_root"/><geom size=".1"/>'
        '<body name="child" pos=".1 -.2 .3"><joint name="ball" type="ball"/>'
        '<geom size=".1"/><site name="tip" pos=".4 .1 -.2"/></body></body></worldbody></mujoco>'
    )


@pytest.mark.parametrize("joint_name", ["free_root", "ball"])
@pytest.mark.parametrize(
    "scale",
    [
        pytest.param(1.0, id="unit-scale"),
        pytest.param(-3.0, id="negative-nonunit"),
        pytest.param(np.finfo(float).max, id="maximum-finite"),
        pytest.param(1e155, id="overflowing-native-norm"),
        pytest.param(1e-200, id="underflowing-native-norm"),
        pytest.param(np.nextafter(0.0, 1.0), id="smallest-subnormal"),
        pytest.param(0.0, id="zero-native-fallback"),
    ],
)
def test_scaled_quaternion_poses_match_native_unit_reference(
    tmp_path: Path, articulated_model: mujoco.MjModel, joint_name: str, scale: float
) -> None:
    model = articulated_model
    free_address = model.joint("free_root").qposadr[0]
    ball_address = model.joint("ball").qposadr[0]
    quaternion_addresses = {"free_root": free_address + 3, "ball": ball_address}
    components = {"free_root": np.ones(4), "ball": np.array([1.0, -1.0, 1.0, -1.0])}
    pose = model.qpos0.copy()
    pose[model.joint("angle").qposadr[0]] = 0.37
    pose[free_address : free_address + 3] = [0.6, -0.4, 1.2]
    for name, address in quaternion_addresses.items():
        pose[address : address + 4] = components[name] * 0.5
    reference = pose.copy()
    address = quaternion_addresses[joint_name]
    pose[address : address + 4] = components[joint_name] * scale
    reference[address : address + 4] = 0.0 if scale == 0 else components[joint_name] * (0.5 * np.sign(scale))
    data = mujoco.MjData(model)
    data.qpos[:] = reference
    mujoco.mj_normalizeQuat(model, data.qpos)
    mujoco.mj_forward(model, data)
    poses = pose[None]
    original = poses.copy()
    poses.setflags(write=False)
    fingerprint = model_fingerprint(model)

    frame = export_model(model, tmp_path / "pose.json", qpos=poses)["frames"][0]

    np.testing.assert_allclose(frame["qpos"], data.qpos, rtol=1e-15, atol=1e-15)
    np.testing.assert_allclose(frame["body_quat"], data.xquat, rtol=1e-15, atol=1e-15)
    np.testing.assert_allclose(frame["geom_pos"], data.geom_xpos, atol=1e-15)
    np.testing.assert_allclose(frame["site_pos"], data.site_xpos, atol=1e-15)
    np.testing.assert_array_equal(poses, original)
    assert model_fingerprint(model) == fingerprint


@pytest.mark.parametrize(
    "poses",
    [
        np.array([[0.2 + 0.8j]]),
        np.array([[0.2 + 0.0j]]),
        [[0.2 + 0.8j]],
        np.array([[True]]),
        np.array([["0.2"]]),
        np.array([[b"0.2"]]),
        np.array([[0.2]], dtype=object),
    ],
)
@pytest.mark.filterwarnings("error")
def test_nonreal_pose_dtypes_rejected_without_changing_source_or_output(tmp_path: Path, poses: Any) -> None:
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint/><geom size=".1"/></body></worldbody></mujoco>')
    output = tmp_path / "pose.json"
    previous = b'{"previous": "pose"}\n'
    output.write_bytes(previous)
    original = np.asarray(poses).copy()
    fingerprint = model_fingerprint(model)

    with pytest.raises(ValueError, match="qpos.*real"):
        export_model(model, output, qpos=poses)

    assert output.read_bytes() == previous
    np.testing.assert_array_equal(np.asarray(poses), original)
    assert model_fingerprint(model) == fingerprint


@pytest.mark.parametrize("dtype", [np.int32, np.uint64, np.float32, np.float64])
def test_real_numeric_pose_dtypes_remain_supported(tmp_path: Path, dtype: type[np.generic]) -> None:
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint/><geom size=".1"/></body></worldbody></mujoco>')
    poses = np.array([[1]], dtype=dtype)

    frame = export_model(model, tmp_path / "real.json", qpos=poses)["frames"][0]

    assert frame["qpos"] == [1.0]
    np.testing.assert_array_equal(poses, [[1]])


@pytest.mark.parametrize("radius", [0.1, 1e-180, 1e180])
@pytest.mark.parametrize("shape", ["sphere", "cylinder"])
@pytest.mark.parametrize("inside", [False, True])
@pytest.mark.filterwarnings("error")
def test_inside_wrap_predicate_is_scale_invariant_in_native_geom_frame(
    tmp_path: Path, radius: float, shape: str, inside: bool
) -> None:
    radial = radius * (0.5 if inside else 2.0)
    # Cylinder wrapping uses radial distance even beyond the displayed end caps.
    axial = 3 * radius if shape == "cylinder" else 0.0
    size = str(radius) if shape == "sphere" else f"{radius} {radius}"
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><worldbody><frame euler="90 0 0"><geom name="wrap" type="{shape}" size="{size}"/>'
        f'<site name="start" pos="{-2 * radius} 0 0"/><site name="end" pos="{2 * radius} 0 0"/>'
        f'<site name="side" pos="{radial} 0 {axial}"/></frame></worldbody>'
        '<tendon><spatial><site site="start"/><geom geom="wrap" sidesite="side"/>'
        '<site site="end"/></spatial></tendon></mujoco>'
    )
    fingerprint = model_fingerprint(model)

    document = export_model(model, tmp_path / "wrap.json")

    assert document["frames"][0]["wrap_flags"][0]["sidesite_inside"] is inside
    assert bool(document["warnings"]) is inside
    assert model_fingerprint(model) == fingerprint
