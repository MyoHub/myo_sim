"""Standing keyframes of the free-standing models.

``qpos0`` of ``myofullbody``, ``myolegs`` and ``myolegs_abdomen`` is not a posture
the model can start from (see ``docs/wiki/standing-keyframe.md``). The pose lives in
``data/standing_keyframes.json``, written by ``scripts/generate_standing_keyframes.py``.

Specs stay free of keyframes unless asked for: on MuJoCo 3 a keyframe's ``qpos`` has a fixed
size, so deleting a joint from a spec that holds one makes ``compile()`` fail. MuJoCo 4 keeps
the keyframe's values by joint name across the edit.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    import mujoco

STANDING_KEYFRAME_PATH = Path(__file__).with_name("data") / "standing_keyframes.json"
STANDING_KEYFRAME_NAME = "standing"
ROOT_JOINT = "root"


def load_standing_table() -> dict[str, Any]:
    """Read the standing keyframe table."""
    return json.loads(STANDING_KEYFRAME_PATH.read_text())


def has_standing_keyframe(model_name: str) -> bool:
    """Whether *model_name* has an entry in the standing keyframe table."""
    return model_name in load_standing_table()["models"]


def add_standing_keyframe(spec: "mujoco.MjSpec", model_name: str = "myofullbody") -> "mujoco.MjsKey":
    """Add the ``standing`` keyframe of *model_name* to an uncompiled spec, in place.

    Values are matched by joint name on the spec as it is now, so call this after
    deleting or adding joints: joints the spec no longer has are skipped, joints the
    table does not list stay at ``qpos0``. The root keeps the spec's own x, y and
    orientation and takes its height from the table, plus the entry's ``root_y_offset`` and
    ``root_quat`` in the root's frame where it has them. The keyframe survives
    ``MjSpec.attach`` (with the attach prefix in its name), and the other bodies of the
    composed model are at their ``qpos0`` in it.

    Args:
        spec: Uncompiled spec of the model, optionally edited.
        model_name: Entry of the table to read.

    Returns:
        The new keyframe.

    Raises:
        KeyError: If the table has no entry for *model_name*.
    """
    import mujoco

    entry = load_standing_table()["models"][model_name]
    for key in list(spec.keys):
        if key.name == STANDING_KEYFRAME_NAME:
            spec.delete(key)
    model = spec.compile()
    qpos = model.qpos0.copy()
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name in entry["joints"]:
            qpos[model.jnt_qposadr[joint_id]] = entry["joints"][name]
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ROOT_JOINT)
    if root_id >= 0:
        adr = model.jnt_qposadr[root_id]
        qpos[adr + 2] = entry["root_z"]
        if "root_quat" in entry:
            own = qpos[adr + 3 : adr + 7].copy()
            offset = np.zeros(3)
            mujoco.mju_rotVecQuat(offset, [0.0, entry["root_y_offset"], 0.0], own)
            qpos[adr : adr + 2] += offset[:2]
            mujoco.mju_mulQuat(qpos[adr + 3 : adr + 7], own, np.array(entry["root_quat"]))
    return spec.add_key(name=STANDING_KEYFRAME_NAME, qpos=qpos.tolist())
