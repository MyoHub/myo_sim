"""Verify left/right symmetry of leg muscles.

Naming convention: muscles and joints use _r/_l suffixes.
"""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import myo_sim  # noqa: E402

from muscle_analysis_utils import (  # noqa: E402
    compute_moment_arm_curve,
    compute_force_length_curve,
    parse_model_joint_equalities,
    plot_pair,
)

OUT_DIR = Path(__file__).resolve().parent / "output" / "muscle_analysis"
OUT_DIR.mkdir(parents=True, exist_ok=True)

EPS = 1e-5
ACTIVATION = 1.0

model, data = myo_sim.load("myolegs")
EQ_MAP = parse_model_joint_equalities(model)


def analyze_pair(base_muscle: str, base_joint: str):
    """Compare left/right muscle pair for a given joint."""
    curves = {}

    for side in ("right", "left"):
        suf = "_r" if side == "right" else "_l"
        muscle = base_muscle + suf
        joint = base_joint + suf

        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
        jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if act_id < 0 or jnt_id < 0:
            return None

        tendon_id = model.actuator_trnid[act_id, 0]
        if tendon_id < 0:
            return None

        jnt_range, ma = compute_moment_arm_curve(model, data, tendon_id, jnt_id, eps=EPS, eq_map=EQ_MAP)
        if jnt_range is None or ma is None or np.allclose(ma, 0, atol=1e-6):
            return None

        mtu_len, forces = compute_force_length_curve(model, data, act_id, jnt_id, activation=ACTIVATION, eq_map=EQ_MAP)

        curves[side] = dict(
            muscle=muscle,
            jnt_range=jnt_range,
            moment_arms=ma,
            mtu_lengths=mtu_len,
            forces=forces,
        )

    out_path = OUT_DIR / f"{base_muscle}_{base_joint}.png"
    return plot_pair(curves, title=f"{base_muscle} @ {base_joint}", out_path=out_path)


print("\nRunning LEG muscle symmetry analysis...\n", flush=True)

ok_cnt = 0
bad_cnt = 0
skip_cnt = 0
total = 0


for act_id in range(model.nu):
    muscle_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
    if muscle_name is None or not muscle_name.endswith("_r"):
        continue

    base_muscle = muscle_name[:-2]
    tendon_id = model.actuator_trnid[act_id, 0]
    if tendon_id < 0:
        continue

    for jnt_id in range(model.njnt):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        if jnt_name is None:
            continue

        q0, q1 = model.jnt_range[jnt_id]
        if q0 == q1:
            continue

        jnt_range, ma = compute_moment_arm_curve(model, data, tendon_id, jnt_id, eps=EPS, eq_map=EQ_MAP)
        if jnt_range is None or np.allclose(ma, 0, atol=1e-6):
            continue

        if jnt_name.endswith(("_r", "_l")):
            base_joint = jnt_name[:-2]
        else:
            base_joint = jnt_name

        res = analyze_pair(base_muscle, base_joint)
        pair = f"{base_muscle}_{base_joint}"

        if res is None:
            skip_cnt += 1
            print(f"skip: {pair}", flush=True)
        elif res:
            ok_cnt += 1
            print(f"ok:   {pair}", flush=True)
        else:
            bad_cnt += 1
            print(f"x:    {pair}", flush=True)

        total += 1

print("\n" + "=" * 60, flush=True)
print(f"checked      : {total}", flush=True)
print(f"ok           : {ok_cnt}", flush=True)
print(f"discrepancy  : {bad_cnt}", flush=True)
print(f"skipped      : {skip_cnt}", flush=True)
print(f"output dir   : {OUT_DIR.resolve()}", flush=True)
print("=" * 60, flush=True)
