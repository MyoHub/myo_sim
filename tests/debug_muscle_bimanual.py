"""Verify left/right symmetry of bimanual arm muscles.

Naming convention: muscles use ""/_left, joints use _r/_l suffixes.
"""

from pathlib import Path

import mujoco
import numpy as np

from muscle_analysis_utils import (
    compute_moment_arm_curve,
    compute_force_length_curve,
    pair_discrepancy_summary,
    parse_model_joint_equalities,
    plot_pair,
)

OUT_DIR = Path(__file__).resolve().parent / "output" / "muscle_analysis"
OUT_DIR.mkdir(parents=True, exist_ok=True)

XML_PATH = Path(__file__).resolve().parent.parent
XML_PATH = XML_PATH / "arm" / "myoarm_bimanual.xml"

model = mujoco.MjModel.from_xml_path(str(XML_PATH))
data = mujoco.MjData(model)
EQ_MAP = parse_model_joint_equalities(model)

ACTIVATION = 1.0
EPS = 1e-5

MUSCLE_SUFFIX = {"right": "", "left": "_l"}
JOINT_SUFFIX = {"right": "_r", "left": "_l"}


def analyze_pair(base_muscle: str, base_joint: str):
    """Compare left/right muscle pair for a given joint."""
    curves = {}

    for side in ("right", "left"):
        muscle = base_muscle + MUSCLE_SUFFIX[side]
        joint = base_joint + JOINT_SUFFIX[side]

        act_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle
        )
        jnt_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint
        )

        if act_id < 0 or jnt_id < 0:
            return None

        tendon_id = model.actuator_trnid[act_id, 0]
        if tendon_id < 0:
            return None

        jnt_range, ma = compute_moment_arm_curve(
            model, data, tendon_id, jnt_id, eps=EPS, eq_map=EQ_MAP
        )
        if jnt_range is None or np.allclose(ma, 0, atol=5e-5):
            return None

        mtu_len, forces = compute_force_length_curve(
            model, data, act_id, jnt_id, activation=ACTIVATION, eq_map=EQ_MAP
        )

        curves[side] = dict(
            muscle=muscle,
            jnt_range=jnt_range,
            moment_arms=ma,
            mtu_lengths=mtu_len,
            forces=forces,
        )

    summary = pair_discrepancy_summary(curves)
    ok = plot_pair(
        curves,
        title=f"{base_muscle} @ {base_joint}",
        out_path=OUT_DIR / f"{base_muscle}_{base_joint}.png",
    )
    return ok, summary


print("\nRunning BIMANUAL muscle symmetry analysis...\n")

ok_cnt = 0
bad_cnt = 0
skip_cnt = 0

for act_id in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
    if name is None or name.endswith("_left"):
        continue

    base_muscle = name
    tendon_id = model.actuator_trnid[act_id, 0]
    if tendon_id < 0:
        continue

    for jnt_id in range(model.njnt):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        if jnt_name is None or not jnt_name.endswith("_r"):
            continue

        q0, q1 = model.jnt_range[jnt_id]
        if q0 == q1:
            continue

        jnt_range, ma = compute_moment_arm_curve(
            model, data, tendon_id, jnt_id, eps=EPS, eq_map=EQ_MAP
        )
        if jnt_range is None or np.allclose(ma, 0, atol=5e-5):
            continue

        base_joint = jnt_name[:-2]

        res = analyze_pair(base_muscle, base_joint)
        pair = f"{base_muscle}_{base_joint}"

        if res is None:
            skip_cnt += 1
            print(f"skip: {pair}", flush=True)
            continue

        ok, diff = res
        diff_msg = (
            f"max |dMA|={diff['moment_arm']:.6g} m, "
            f"max |dF|={diff['force']:.6g} N "
            f"({diff['force_pct']:.6g}%), "
            f"MA {'ok' if diff['moment_arm_ok'] else 'fail'}, "
            f"F {'ok' if diff['force_ok'] else 'fail'}"
        )
        if ok:
            ok_cnt += 1
            print(f"ok:   {pair} ({diff_msg})", flush=True)
        else:
            bad_cnt += 1
            print(f"x:    {pair} ({diff_msg})", flush=True)

print("\n" + "=" * 60)
print(f"ok           : {ok_cnt}")
print(f"discrepancy  : {bad_cnt}")
print(f"skipped      : {skip_cnt}")
print(f"output dir   : {OUT_DIR.resolve()}")
print("=" * 60)