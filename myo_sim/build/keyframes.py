"""Named starting postures of the composed models, as keyframes.

``qpos0`` is the reference of the joint couplings and the zero of the tendon lengths, not
a posture to start from (see ``docs/wiki/postures.md``). The postures live in
``data/postures.json``, written by ``scripts/generate_postures.py``.

Specs stay free of keyframes unless asked for: on MuJoCo 3 a keyframe's ``qpos`` has a fixed
size, so deleting a joint from a spec that holds one makes ``compile()`` fail. MuJoCo 4 keeps
the keyframe's values by joint name across the edit.
"""

from __future__ import annotations

import json
from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    import mujoco

POSTURES_PATH = Path(__file__).with_name("data") / "postures.json"
ROOT_JOINT = "root"


def load_posture_table() -> dict[str, Any]:
    """Read the posture table."""
    return json.loads(POSTURES_PATH.read_text())


def _model_postures(model_name: str) -> dict[str, Any]:
    from myo_sim.build.compose import ALIASES

    return load_posture_table()["models"].get(ALIASES.get(model_name, model_name), {})


def list_postures(model_name: str) -> list[str]:
    """Names of the postures of a model, in keyframe order.

    Args:
        model_name: Registry name or alias (e.g. "myofullbody").

    Returns:
        The posture names, empty if the model has none.
    """
    return list(_model_postures(model_name))


def posture_qpos(model: "mujoco.MjModel", entry: dict[str, Any]) -> np.ndarray:
    """``qpos`` of a table entry on a compiled model, matched by joint name.

    Joints the model does not have are skipped and joints the entry does not list stay
    at ``qpos0``. The root keeps its x, y and orientation and takes its height from the entry,
    plus the entry's ``root_y_offset`` and ``root_quat`` in the root's frame where it has them.

    Args:
        model: Compiled model.
        entry: One posture of the table.

    Returns:
        The ``qpos`` array.
    """
    import mujoco

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
    return qpos


def add_posture_keyframes(
    spec: "mujoco.MjSpec", model_name: str, postures: Sequence[str] | None = None
) -> list["mujoco.MjsKey"]:
    """Add postures of *model_name* to an uncompiled spec as keyframes, in place.

    Values are matched by joint name on the spec as it is now, so call this after
    deleting or adding joints: joints the spec no longer has are skipped, joints the
    table does not list stay at ``qpos0``. The keyframes survive ``MjSpec.attach`` (with
    the attach prefix in their names), and the other bodies of the composed model are at
    their ``qpos0`` in them. A keyframe of the same name is replaced.

    Args:
        spec: Uncompiled spec of the model, optionally edited.
        model_name: Entry of the table to read (registry name or alias).
        postures: Names to add, in this order. None adds all of the model's postures.

    Returns:
        The new keyframes.

    Raises:
        KeyError: If the table does not have one of the postures for *model_name*.
    """
    entries = _model_postures(model_name)
    names = list(entries) if postures is None else list(postures)
    for name in names:
        if name not in entries:
            raise KeyError(f"{model_name!r} has no posture {name!r}. Available: {list(entries)}")
    for key in list(spec.keys):
        if key.name in names:
            spec.delete(key)
    model = spec.compile()
    return [spec.add_key(name=name, qpos=posture_qpos(model, entries[name]).tolist()) for name in names]
