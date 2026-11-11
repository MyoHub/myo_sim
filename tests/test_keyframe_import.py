"""Imported keyframes reset the full simulation state just like native MJCF."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from myo_sim.build.utils import add_keyframes


@pytest.mark.parametrize(
    "key",
    [
        '<key name="complete" time=".7" qpos=".2" qvel=".3" act=".4 .6" ctrl=".5 .7" '
        'mpos="1 2 3" mquat=".9238795325 0 0 .3826834324"/>',
        '<key name="pose" qpos=".25"/>',
        "<key/>",
        '<key name=""/>',
        '<key name="partial" act=".4" ctrl=".5"/>',
    ],
    ids=["complete-state", "native-defaults", "unnamed-defaults", "empty-name", "partial-state"],
)
def test_imported_keyframe_matches_native_reset(key: str, tmp_path: Path) -> None:
    """Activation, control and mocap state must survive the spec-level import."""
    body = (
        '<worldbody><body><joint name="j" ref=".2"/><geom size=".1"/></body>'
        '<body name="marker" mocap="true" pos=".1 .2 .3" quat=".8 0 0 .6">'
        '<geom size=".01"/></body></worldbody>'
        '<actuator><muscle name="muscle" joint="j" lengthrange="0 1" force="1"/>'
        '<muscle name="other" joint="j" lengthrange="0 1" force="1"/></actuator>'
    )
    keyframes = f"<keyframe>{key}</keyframe>"
    path = tmp_path / "keyframes.xml"
    path.write_text(keyframes, encoding="utf-8")
    spec = mujoco.MjSpec.from_string(f"<mujoco>{body}</mujoco>")
    add_keyframes(spec, path)
    imported = spec.compile()
    native = mujoco.MjModel.from_xml_string(f"<mujoco>{body}{keyframes}</mujoco>")

    for field in ("qpos", "qvel", "act", "ctrl", "mpos", "mquat", "time"):
        np.testing.assert_array_equal(getattr(imported, f"key_{field}"), getattr(native, f"key_{field}"), err_msg=field)

    imported_data, native_data = mujoco.MjData(imported), mujoco.MjData(native)
    mujoco.mj_resetDataKeyframe(imported, imported_data, 0)
    mujoco.mj_resetDataKeyframe(native, native_data, 0)
    for field in ("qpos", "qvel", "act", "ctrl", "mocap_pos", "mocap_quat", "time"):
        np.testing.assert_array_equal(getattr(imported_data, field), getattr(native_data, field), err_msg=field)
