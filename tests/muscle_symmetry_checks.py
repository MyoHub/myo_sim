from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from muscle_analysis_utils import (  # noqa: E402
    compute_force_length_curve,
    compute_moment_arm_curve,
    pair_discrepancy_summary,
)


@dataclass(frozen=True)
class MuscleSymmetryPair:
    right_muscle: str
    left_muscle: str
    right_joint: str
    left_joint: str

    @property
    def label(self) -> str:
        return f"{self.right_muscle}/{self.left_muscle} @ {self.right_joint}/{self.left_joint}"


def actuator_id(model, name: str) -> int:
    actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert actuator >= 0, name
    return actuator


def joint_id(model, name: str) -> int:
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    assert joint >= 0, name
    return joint


def has_nonzero_moment_arm(model, data, eq_map, actuator: int, joint: int, *, atol=5e-5) -> bool:
    tendon = model.actuator_trnid[actuator, 0]
    if tendon < 0:
        return False

    _, moment_arms = compute_moment_arm_curve(
        model,
        data,
        tendon,
        joint,
        n=25,
        eq_map=eq_map,
    )
    return moment_arms is not None and not np.allclose(moment_arms, 0.0, atol=atol)


def compare_muscle_pair(model, data, eq_map, pair: MuscleSymmetryPair):
    curves = {}
    for side, muscle_name, joint_name in (
        ("right", pair.right_muscle, pair.right_joint),
        ("left", pair.left_muscle, pair.left_joint),
    ):
        actuator = actuator_id(model, muscle_name)
        joint = joint_id(model, joint_name)
        tendon = model.actuator_trnid[actuator, 0]
        assert tendon >= 0, muscle_name

        joint_range, moment_arms = compute_moment_arm_curve(
            model,
            data,
            tendon,
            joint,
            n=50,
            eq_map=eq_map,
        )
        mtu_lengths, forces = compute_force_length_curve(
            model,
            data,
            actuator,
            joint,
            n=50,
            eq_map=eq_map,
        )

        assert joint_range is not None, pair.label
        assert moment_arms is not None, pair.label
        assert mtu_lengths is not None, pair.label
        assert forces is not None, pair.label

        curves[side] = {
            "muscle": muscle_name,
            "jnt_range": joint_range,
            "moment_arms": moment_arms,
            "mtu_lengths": mtu_lengths,
            "forces": forces,
        }

    return pair_discrepancy_summary(curves)


def discover_bilateral_suffix_pairs(
    model,
    data,
    eq_map,
    *,
    right_muscle_suffix: str,
    left_muscle_suffix: str,
    right_joint_suffix: str,
    left_joint_suffix: str,
    limit: int | None = None,
) -> list[MuscleSymmetryPair]:
    pairs = []
    for actuator in range(model.nu):
        muscle_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
        if muscle_name is None:
            continue
        if right_muscle_suffix:
            if not muscle_name.endswith(right_muscle_suffix):
                continue
            muscle_base = muscle_name[: -len(right_muscle_suffix)]
        elif muscle_name.endswith((left_muscle_suffix, "_left")):
            continue
        else:
            muscle_base = muscle_name

        left_muscle = f"{muscle_base}{left_muscle_suffix}"
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, left_muscle) < 0:
            continue

        for joint in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if joint_name is None or not joint_name.endswith(right_joint_suffix):
                continue
            if not has_nonzero_moment_arm(model, data, eq_map, actuator, joint):
                continue

            joint_base = joint_name[: -len(right_joint_suffix)]
            left_joint = f"{joint_base}{left_joint_suffix}"
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, left_joint) < 0:
                continue

            pairs.append(
                MuscleSymmetryPair(
                    right_muscle=muscle_name,
                    left_muscle=left_muscle,
                    right_joint=joint_name,
                    left_joint=left_joint,
                )
            )
            if limit is not None and len(pairs) >= limit:
                return pairs
    return pairs


def discover_shared_joint_pairs(
    model,
    data,
    eq_map,
    *,
    skip_joints: set[str],
    limit: int | None = None,
) -> list[MuscleSymmetryPair]:
    pairs = []
    for actuator in range(model.nu):
        muscle_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
        if muscle_name is None or muscle_name.endswith("_left"):
            continue

        if muscle_name.endswith("_r"):
            left_muscle = f"{muscle_name[:-2]}_l"
        else:
            left_muscle = f"{muscle_name}_left"
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, left_muscle) < 0:
            continue

        for joint in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if joint_name is None or joint_name in skip_joints:
                continue
            if not has_nonzero_moment_arm(model, data, eq_map, actuator, joint):
                continue

            pairs.append(
                MuscleSymmetryPair(
                    right_muscle=muscle_name,
                    left_muscle=left_muscle,
                    right_joint=joint_name,
                    left_joint=joint_name,
                )
            )
            if limit is not None and len(pairs) >= limit:
                return pairs
    return pairs
