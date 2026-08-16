"""Regenerate `docs/muscle-groups-fullbody.md` from the current myofullbody model.

Manual script, not a pytest test. Run with:

    uv run python tests/generate_muscle_group_report.py
"""

from __future__ import annotations

import numpy as np

from myo_sim.analysis.muscle_groups import discover_muscle_groups
from myo_sim.build.compose import build_model

REGIONS: dict[str, list[str]] = {
    "Right leg": [
        "hip_flexion_r",
        "hip_adduction_r",
        "hip_rotation_r",
        "knee_angle_r",
        "ankle_angle_r",
        "subtalar_angle_r",
        "mtp_angle_r",
    ],
    "Right arm": [
        "elv_angle_r",
        "shoulder_elv_r",
        "shoulder_rot_r",
        "elbow_flexion_r",
        "pro_sup_r",
        "deviation_r",
        "flexion_r",
    ],
    "Lumbar / torso": ["flex_extension", "lat_bending", "axial_rotation"],
}


def main() -> None:
    model = build_model("myofullbody")
    for region, joints in REGIONS.items():
        available = [j for j in joints if _has_joint(model, j)]
        if not available:
            print(f"\n## {region}\n(no joints found)")
            continue
        groups = discover_muscle_groups(model, joints=available)
        print(f"\n## {region}   ({len(groups)} groups over {len(available)} joints)")
        for group in groups:
            rank = group.identifiable_rank()
            print(f"\n### {' + '.join(group.actuators)}")
            print(f"dominant joint: {group.dominant_joint}   identifiable rank: {rank}/{len(group.actuators)}")
            share = group.target_share
            index = group.joints.index(group.dominant_joint)
            for i, name in enumerate(group.actuators):
                print(
                    f"  {name:<16s} fmax={group.fmax[i]:8.1f} N  "
                    f"MA={group.mean_moment_arm[i, index]:+.4f} m  share={share[i]:.3f}"
                )
            print(f"  conditioning: {np.round(group.conditioning, 4).tolist()}")
            if rank < len(group.actuators):
                merged = ", ".join(f"({'+'.join(n)})={s:.3f}" for n, s in group.identifiable_targets())
                print(f"  identifiable targets: {merged}")


def _has_joint(model, name: str) -> bool:
    import mujoco

    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0


if __name__ == "__main__":
    main()
