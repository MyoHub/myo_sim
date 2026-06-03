"""Verify left/right symmetry of bimanual arm muscles.

Naming convention: right muscles have no suffix, left muscles use _l, and
joints use _r/_l suffixes.
"""

import argparse
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

DEFAULT_MODEL = "myotorso_arms"

ACTIVATION = 1.0
EPS = 1e-5

MUSCLE_SUFFIX = {"right": "", "left": "_l"}
JOINT_SUFFIX = {"right": "_r", "left": "_l"}


def load_model(model_name: str):
    from myo_sim.mjspec.prototype_mjspec_attach import build_model

    return build_model(model_name)


def load_equality_map(model, model_name: str):
    return parse_model_joint_equalities(model)


def model_choices():
    from myo_sim.mjspec.prototype_mjspec_attach import MODEL_REGISTRY

    return tuple(sorted(MODEL_REGISTRY))


def analyze_pair(model, data, eq_map, base_muscle: str, base_joint: str, *, plot=False):
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
            model, data, tendon_id, jnt_id, eps=EPS, eq_map=eq_map
        )
        if jnt_range is None or np.allclose(ma, 0, atol=5e-5):
            return None

        mtu_len, forces = compute_force_length_curve(
            model, data, act_id, jnt_id, activation=ACTIVATION, eq_map=eq_map
        )

        curves[side] = dict(
            muscle=muscle,
            jnt_range=jnt_range,
            moment_arms=ma,
            mtu_lengths=mtu_len,
            forces=forces,
        )

    summary = pair_discrepancy_summary(curves)
    if plot:
        ok = plot_pair(
            curves,
            title=f"{base_muscle} @ {base_joint}",
            out_path=OUT_DIR / f"{base_muscle}_{base_joint}.png",
            save_all=True,
        )
    else:
        ok = summary["ok"]
    return ok, summary


def print_result(pair, ok, diff):
    diff_msg = (
        f"max |dMA|={diff['moment_arm']:.6g} m, "
        f"max |dF|={diff['force']:.6g} N "
        f"({diff['force_pct']:.6g}%), "
        f"MA {'ok' if diff['moment_arm_ok'] else 'fail'}, "
        f"F {'ok' if diff['force_ok'] else 'fail'}"
    )
    print(f"{'ok' if ok else 'x':<5} {pair} ({diff_msg})", flush=True)


def run_single(model, data, eq_map, base_muscle, base_joint, *, plot=False):
    res = analyze_pair(model, data, eq_map, base_muscle, base_joint, plot=plot)
    pair = f"{base_muscle}_{base_joint}"
    if res is None:
        print(f"skip: {pair}", flush=True)
        return None
    ok, diff = res
    print_result(pair, ok, diff)
    return ok


def run_all(model, data, eq_map, *, plot=False):
    ok_cnt = 0
    bad_cnt = 0
    skip_cnt = 0

    for act_id in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
        if name is None or name.endswith(("_l", "_left")):
            continue

        left_name = name + MUSCLE_SUFFIX["left"]
        left_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, left_name)
        if left_act_id < 0:
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
                model, data, tendon_id, jnt_id, eps=EPS, eq_map=eq_map
            )
            if jnt_range is None or np.allclose(ma, 0, atol=5e-5):
                continue

            base_joint = jnt_name[:-2]

            res = analyze_pair(model, data, eq_map, base_muscle, base_joint, plot=plot)
            pair = f"{base_muscle}_{base_joint}"

            if res is None:
                skip_cnt += 1
                print(f"skip: {pair}", flush=True)
                continue

            ok, diff = res
            if ok:
                ok_cnt += 1
            else:
                bad_cnt += 1
            print_result(pair, ok, diff)

    print("\n" + "=" * 60)
    print(f"ok           : {ok_cnt}")
    print(f"discrepancy  : {bad_cnt}")
    print(f"skipped      : {skip_cnt}")
    print(f"output dir   : {OUT_DIR.resolve()}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        choices=model_choices(),
        default=DEFAULT_MODEL,
        help="Model source to analyze",
    )
    parser.add_argument("--muscle", help="Base/right muscle name, e.g. DELT1")
    parser.add_argument("--joint", help="Base joint name, e.g. shoulder_elv")
    parser.add_argument("--plot", action="store_true", help="Save plots")
    args = parser.parse_args()

    if bool(args.muscle) != bool(args.joint):
        parser.error("--muscle and --joint must be provided together")

    model = load_model(args.model)
    data = mujoco.MjData(model)
    eq_map = load_equality_map(model, args.model)

    print(f"\nRunning BIMANUAL muscle symmetry analysis on {args.model}...\n")

    if args.muscle:
        run_single(model, data, eq_map, args.muscle, args.joint, plot=args.plot)
    else:
        run_all(model, data, eq_map, plot=args.plot)


if __name__ == "__main__":
    main()