# Standing keyframe

`myofullbody`, `myolegs` and `myolegs_abdomen` have a keyframe named `standing`. `qpos0` and every model parameter are unchanged, so it only matters if you start from it.

## Why

`qpos0` keeps its role as the reference of every joint coupling and the zero of the tendon lengths, but it is not a posture to start from:

- The knee couplings have non-zero constant terms, so at `qpos0` they are up to 52.4 mm off and the constraint solver pulls them back within milliseconds.
- In `myofullbody` the humerus capsules overlap `thorax_coll1` by 11.4 mm, at 688 N per arm at t = 0.
- The root is at z = 1.0, so the feet are 5.5 cm above the floor in `myofullbody`. In the default scene of `myolegs`, the feet are 4 cm above the stage.

`myolegs26` has no such coupling error and `myoarms`, `myoarm_r`, `myotorso_arms` and `myotorso_arm_r` are fixed-base, so they get no keyframe. The fixed-base arm models do have the same humerus overlap at `qpos0`.

## How the pose is made

`scripts/generate_standing_keyframes.py` writes `myo_sim/build/data/standing_keyframes.json` from `qpos0` of each composed model, in up to six steps:

1. Raise `shoulder_elv_r` and `shoulder_elv_l` to 0.10 rad (5.7 deg), the plane of elevation staying at 0. The humerus then clears the thorax by 6.2 mm (0.05 rad leaves 2 mm of overlap, 0.08 rad clears it by 3 mm). This is a clearance value, not a measured arm posture. The scapular joints follow through their couplings.
2. Evaluate every joint coupling from its polynomial, so that couplings hold to rounding.
3. Bisect the ankle angle until heel and forefoot are level (0.012 rad, 0.7 deg of dorsiflexion).
4. Lower the root until the feet are 0.1 mm above the floor. That is inside the 1 mm contact margin of the foot geoms, so the feet carry the weight from t = 0.
5. In `myofullbody`, solve each thumb's four joints for the equilibrium of the passive moments and gravity at zero activation: CMC flexion -7.5 deg, CMC abduction 21.2 deg, MP flexion -15.3 deg, IP flexion -9.7 deg, on both hands.
6. In `myolegs_abdomen`, tilt the pelvis forward by 15.6 deg, the angle that puts the trunk's hinge above the hip axis (4.2 cm behind and 14.9 cm above it when upright). Hip flexion and trunk extension of the same angle keep the legs and trunk upright. This is the only model whose root turns.

The script also prints the tables below (`--report`). To change the pose, edit the constants in the script and rerun it. `tests/test_standing_keyframe.py` checks that the JSON is what the script writes. The table holds only kinematics: the body masses do not enter it and the knee hinges only matter away from knee 0, so the knee and mass changes of v0.3.0 left it as it was, to the last digit. The contact forces, the speeds and the centre of mass below did change.

## Using it

- `myo_sim.load(name)` and `myo_sim.load_model(name)` include the keyframe; use `mujoco.mj_resetDataKeyframe(model, data, model.key("standing").id)`.
- `myo_sim.load_spec(name)` and `build_spec(name)` leave it out unless you pass `build_kwargs={"standing_keyframe": True}`. A keyframe's `qpos` has a fixed size, so `compile()` fails after deleting a joint from a spec that holds one (`keyframe 'standing': invalid qpos size`), and specs from `load_spec` are meant to be edited. Adding a joint is worse: MuJoCo pads the keyframe at the end, which is right for a new body at the end of the tree, but a hinge added under `femur_r` silently shifts 10 joints of the left leg by one.
- `myo_sim.add_standing_keyframe(spec, model_name)` adds it to an edited spec, matched by joint name: joints the spec no longer has are skipped and joints the table does not list stay at `qpos0`. The root keeps the spec's x, y and orientation, apart from the table's `root_y_offset` and `root_quat` for the tilt of `myolegs_abdomen`, which apply in the root's frame.
- `MjSpec.attach` keeps the keyframe under the attach prefix: two bodies attached with `a_` and `b_` give `a_standing` and `b_standing`, each with the other body at its `qpos0`. Tested on MuJoCo 3.17.

## Measurements

MuJoCo 3.17, zero activation, the model's own timestep (2 ms).

| Model | Pose | Root z (m) | Coupling error (mm or mrad) | Contacts at t = 0 | Largest force (N) | Sum / weight | Deepest (mm) |
|---|---|---|---|---|---|---|---|
| myofullbody | `qpos0` | 1.0000 | 52.4 | 2 | 688 | 1.83 | 11.38 |
| myofullbody | `standing` | 0.9455 | 0.0 | 8 | 115 | 0.69 | -0.10 |
| myolegs | `qpos0` | 1.0000 | 52.4 | 0 | 0 | 0.00 | -0.00 |
| myolegs | `standing` | 0.9605 | 0.0 | 8 | 131 | 0.88 | -0.10 |
| myolegs_abdomen | `qpos0` | 1.0000 | 52.4 | 0 | 0 | 0.00 | -0.00 |
| myolegs_abdomen | `standing` | 0.9425 | 0.0 | 8 | 125 | 0.86 | -0.10 |

"Deepest" is the largest overlap of any contact (negative: the closest contact is still apart). At t = 0 only the feet touch in `standing`. Their contact forces add up to at most body weight because the body is already accelerating.

| Model | Pose | Peak speed, 10 ms | Peak speed, 50 ms | Peak speed, 50 ms, muscles off | Deepest in 50 ms (mm) |
|---|---|---|---|---|---|
| myofullbody | `qpos0` | 6.65 (cmc_abduction_l) | 9.86 (knee_angle_l) | 8.24 (cmc_abduction_r) | 11.38 |
| myofullbody | `standing` | 1.39 (pro_sup_l) | 3.35 (hip_flexion_l) | 1.97 (hip_flexion_l) | 0.64 |
| myolegs | `qpos0` | 4.70 (knee_angle_l) | 9.98 (knee_angle_l) | 0.98 (knee_angle_beta_translation1_l) | 8.02 |
| myolegs | `standing` | 0.72 (subtalar_angle_l) | 2.56 (knee_angle_l) | 0.65 (mtp_angle_l) | 0.68 |
| myolegs_abdomen | `qpos0` | 6.82 (hip_flexion_l) | 14.62 (hip_flexion_r) | 0.98 (knee_angle_beta_translation1_l) | 8.49 |
| myolegs_abdomen | `standing` | 0.81 (hip_flexion_r) | 2.92 (hip_flexion_r) | 0.66 (mtp_angle_l) | 0.69 |

Speeds are the largest joint speed, without the free root joint. The leg speeds at `qpos0` come from the violated couplings and the overlap.

The thumb is why step 5 exists. The passive spring of `cmc_abduction` holds 0.59 N m at `qpos0` on a very small inertia, so from there the joint reaches 8 rad/s in 50 ms, with or without the muscles. The thumb's equilibrium is well defined: eight random starts reach the same angles with residuals below 1e-13 N m, it is stable, and the left hand balances at the right hand's angles.

The finger joints stay at `qpos0`. They start away from equilibrium too, but by at most 0.07 N m (`mcp3_flexion`), and reach 2.5 rad/s in 50 ms. The same solve has no single answer for them: without contacts, starts end up to 51 deg apart with fingers overlapping by up to 13 mm, because the passive moments are small and finger abduction has almost no stiffness.

Step 6 comes from the first 50 ms of `myolegs_abdomen` with the muscles off, where the fastest joint is the trunk hinge. Without the tilt the 29.7 kg above the hinge load the pelvis 4.2 cm behind the hips, the hinge and the hips have no stiffness, and so the pelvis turns between them: 3.05 rad/s in 50 ms. When the sacrum became massless (#155), this rose from 1.88 rad/s: the pelvis link lost 7.486 kg under the same load, and putting that mass back on the current model gives 1.89 rad/s. With the tilt it is 0.66 rad/s (`mtp_angle_l`). The tilt also takes 115 N off `psoas` (307 -> 192 N), so the peak with the muscles falls from 4.58 to 2.92 rad/s. `myofullbody` does not take it: its torso muscles hold the trunk, and the same tilt raises its peak with the muscles from 3.35 to 5.0 rad/s. `myolegs` has no trunk hinge.

What remains in the legs and trunk comes from the passive force of the muscles at zero activation: the pelvis pitches at about 85 rad/s^2 (the largest passive forces on the trunk and hip are 308 N in `rect_abd` and 307 N in `psoas`). A keyframe cannot remove this without bending the body: 0.17 rad of trunk flexion with 0.04 rad of knee flexion (hips and ankles following half the knee angle) lowers the 50 ms peak from 3.4 to 2.0 rad/s. The keyframe stays upright instead.

| Model | Pose | Shortest | Longest | Outside [lmin, lmax] | Over 1.2 | Largest passive force (N), muscle | Largest change from `qpos0` |
|---|---|---|---|---|---|---|---|
| myofullbody | `qpos0` | 0.130 | 1.663 | 4 | 58 | 555, recfem_l | 0.000, Ps_L1_VB_r |
| myofullbody | `standing` | 0.130 | 1.603 | 4 | 56 | 308, rect_abd_r | 1.044, recfem_r |
| myolegs | `qpos0` | 0.653 | 1.663 | 0 | 10 | 555, recfem_l | 0.000, addbrev_r |
| myolegs | `standing` | 0.614 | 1.378 | 0 | 8 | 307, psoas_r | 1.044, recfem_r |
| myolegs_abdomen | `qpos0` | 0.653 | 1.663 | 0 | 10 | 555, recfem_l | 0.000, ercspn_r |
| myolegs_abdomen | `standing` | 0.557 | 1.378 | 0 | 8 | 192, psoas_r | 1.106, recfem_r |

Normalised lengths are in units of optimal fibre length, and `[lmin, lmax]` is the range of the MuJoCo muscle force curve, `[0, 2]` here. The four muscles outside it (`FDS3` and `FDP3` of each hand) are outside at `qpos0` as well. The largest change, `recfem`, is the coupling correction: putting the patella where its coupling says shortens rectus femoris and takes its passive force from 555 N to 0.

## Against the literature

| Model | Heel share of foot load at t = 0 | Forefoot share | Centre of mass ahead of the ankle (cm) |
|---|---|---|---|
| myofullbody | 64% | 36% | -0.9 |
| myolegs | 66% | 34% | -0.7 |
| myolegs_abdomen | 65% | 35% | 1.3 |

| Quantity | Model | Literature | Reference and protocol |
|---|---|---|---|
| Heel share of the load under the foot | 64 % (`myofullbody`), 66 % (`myolegs`), 65 % (`myolegs_abdomen`) | 60 % | Cavanagh, Rodgers and Iiboshi 1987, Foot Ankle 7(5):262-276; barefoot standing, 107 feet |
| Everything ahead of the heel | 36 %, 34 %, 35 % (the model has no separate midfoot) | midfoot 8 %, forefoot 28 %, the rest under the toes | same |
| Centre of mass ahead of the ankle | -0.9 cm (`myofullbody`), -0.7 cm (`myolegs`), 1.3 cm (`myolegs_abdomen`) | "a few centimetres in front of the ankle joint"; "several centimetres" | Loram and Lakie 2002, J Physiol 545:1041-1053; Loram, Maganaris and Lakie 2007, J Physiol 584:661-675; quiet standing |
| Ankle angle | 0.7 deg of dorsiflexion | mean ankle angles of 1.5 to 4 deg | Loram and Lakie 2002; quiet standing |

The foot load is close. `myofullbody` and `myolegs` stand more upright than people do: their centre of mass is 0.7-0.9 cm behind the ankle rather than a few centimetres ahead of it; only the tilted `myolegs_abdomen` has it ahead, by 1.3 cm. Leaning forward would need a larger dorsiflexion than the level foot of step 3 gives, and the keyframe does not attempt it. It is a clean start, not a balanced posture: at zero activation the body collapses within about 0.5 s.
