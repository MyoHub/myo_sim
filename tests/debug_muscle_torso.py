"""Verify left/right symmetry of torso muscles.

Naming convention: muscles use _r/_l or ""/_left suffixes.
Joints use base names without side suffixes.
"""

from pathlib import Path

import mujoco
import numpy as np

from muscle_analysis_utils import (
    compute_moment_arm_curve,
    compute_force_length_curve,
    plot_pair,
)

OUT_DIR = Path(__file__).resolve().parent / "output" / "muscle_analysis"
OUT_DIR.mkdir(parents=True, exist_ok=True)

BASE_DIR = Path(__file__).resolve().parent.parent / "torso"
CHAIN_SRC = BASE_DIR / "assets/myotorso_bimanual_chain.xml"
CHAIN_TMP = BASE_DIR / "assets/_tmp_myotorso_bimanual_chain__comment503_505.xml"
TMP_XML = BASE_DIR / "_tmp_myotorso.xml"

# Lines to comment out in chain XML (problematic includes)
COMMENT_START_LINE = 503
COMMENT_END_LINE = 505

EPS = 1e-5
ACTIVATION = 1.0

# Joints that are inherently non-symmetric (lateral bending, axial rotation)
SKIP_JOINTS = {
    "lat_bending",
    "axial_rotation",
    "L4_L5_LB",
    "L4_L5_AR",
    "Abs_t1",
    "L3_L4_LB",
    "L3_L4_AR",
    "Abs_t2",
    "L2_L3_LB",
    "L2_L3_AR",
    "Abs_r3",
    "L1_L2_LB",
    "L1_L2_AR",
}


def comment_out_lines(src: Path, dst: Path, start: int, end: int):
    """Comment out lines [start, end] in src and write to dst."""
    lines = src.read_text().splitlines(True)
    chunk = "".join(lines[start - 1 : end])
    lines[start - 1 : end] = [f"<!--\n{chunk}-->\n"]
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text("".join(lines))


def build_tmp_wrapper_xml(chain_include: str) -> str:
    return rf"""<mujoco model="MyoTorso_v1.0">
  <include file="assets/myotorso_assets.xml"/>
  <include file="../arm/assets/myoarm_bimanual_assets.xml"/>
  <include file="../head/assets/myohead_simple_assets.xml"/>
  <compiler meshdir=".." texturedir=".."/>

  <worldbody>
    <body name="Torso" pos="-.025 0.1 0.9">
      <include file="{chain_include}"/>
    </body>
  </worldbody>
</mujoco>
"""


comment_out_lines(CHAIN_SRC, CHAIN_TMP, COMMENT_START_LINE, COMMENT_END_LINE)
TMP_XML.write_text(build_tmp_wrapper_xml(f"assets/{CHAIN_TMP.name}"))

model = mujoco.MjModel.from_xml_path(str(TMP_XML))
data = mujoco.MjData(model)


def muscle_pair_sides(base_muscle: str):
    """Determine suffix pairing: _r/_l or ""/_left."""
    if base_muscle.endswith(("_r", "_l")):
        return [("_r", "right"), ("_l", "left")]
    return [("", "right"), ("_left", "left")]


def build_muscle_name(base_muscle: str, suffix: str) -> str:
    """Build full muscle name from base and suffix."""
    if suffix in ("_r", "_l") and base_muscle.endswith(("_r", "_l")):
        return base_muscle[:-2] + suffix
    return base_muscle + suffix


def analyze_pair(base_muscle: str, base_joint: str):
    """Compare left/right muscle pair for a given joint."""
    curves = {}

    for suffix, side in muscle_pair_sides(base_muscle):
        muscle = build_muscle_name(base_muscle, suffix)

        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
        if act_id < 0:
            continue

        jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, base_joint)
        if jnt_id < 0:
            continue

        tendon_id = model.actuator_trnid[act_id, 0]
        if tendon_id < 0:
            continue

        jnt_range, ma = compute_moment_arm_curve(model, data, tendon_id, jnt_id, eps=EPS)
        if jnt_range is None or ma is None or np.allclose(ma, 0, atol=1e-6):
            return None

        mtu_len, forces = compute_force_length_curve(model, data, act_id, jnt_id, activation=ACTIVATION)

        curves[side] = dict(
            muscle=muscle,
            jnt_range=jnt_range,
            moment_arms=ma,
            mtu_lengths=mtu_len,
            forces=forces,
        )

    if "right" not in curves or "left" not in curves:
        return None

    out_path = OUT_DIR / f"{base_muscle}_{base_joint}.png"
    return plot_pair(curves, title=f"{base_muscle} @ {base_joint}", out_path=out_path)


print("\nRunning TORSO muscle symmetry analysis...\n", flush=True)

ok_cnt = 0
bad_cnt = 0
skip_cnt = 0
total = 0

try:
    for act_id in range(model.nu):
        base_muscle = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
        if base_muscle is None:
            continue

        tendon_id = model.actuator_trnid[act_id, 0]
        if tendon_id < 0:
            continue

        for jnt_id in range(model.njnt):
            jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
            if jnt_name is None or jnt_name in SKIP_JOINTS:
                continue

            q0, q1 = model.jnt_range[jnt_id]
            if q0 == q1:
                continue

            jnt_range, ma = compute_moment_arm_curve(model, data, tendon_id, jnt_id, eps=EPS)
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

finally:
    TMP_XML.unlink(missing_ok=True)
    CHAIN_TMP.unlink(missing_ok=True)

print("\n" + "=" * 60, flush=True)
print(f"checked      : {total}", flush=True)
print(f"ok           : {ok_cnt}", flush=True)
print(f"discrepancy  : {bad_cnt}", flush=True)
print(f"skipped      : {skip_cnt}", flush=True)
print(f"output dir   : {OUT_DIR.resolve()}", flush=True)
print("=" * 60, flush=True)
