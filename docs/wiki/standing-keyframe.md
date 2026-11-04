# Standing keyframe

`myofullbody`, `myolegs` and `myolegs_abdomen` have a keyframe named `standing`. `qpos0` and every model parameter are unchanged, so it only matters if you start from it.

## Why

`qpos0` keeps its role as the reference of every joint coupling and the zero of the tendon lengths, but it is not a posture to start from:

- The knee couplings have non-zero constant terms, so at `qpos0` they are up to 52.4 mm off and the constraint solver pulls them back within milliseconds.
- In `myofullbody` the humerus capsules overlap `thorax_coll1` by 11.4 mm, at 688 N per arm at t = 0.
- The root is at z = 1.0, so the feet are 5.5 cm above the floor in `myofullbody`. In the default scene of `myolegs`, the feet are 4 cm above the stage.

`myolegs26` has no such coupling error and `myoarms`, `myoarm_r`, `myotorso_arms` and `myotorso_arm_r` are fixed-base, so they get no keyframe. The fixed-base arm models do have the same humerus overlap at `qpos0`.

## How the pose is made

`scripts/generate_standing_keyframes.py` writes `myo_sim/build/data/standing_keyframes.json` from `qpos0` of each composed model, in four steps:

1. Raise `shoulder_elv_r` and `shoulder_elv_l` to 0.10 rad (5.7 deg), the plane of elevation staying at 0. The humerus then clears the thorax by 6.2 mm (0.05 rad leaves 2 mm of overlap, 0.08 rad clears it by 3 mm). This is a clearance value, not a measured arm posture. The scapular joints follow through their couplings.
2. Evaluate every joint coupling from its polynomial, so that couplings hold to rounding.
3. Bisect the ankle angle until heel and forefoot are level (0.012 rad, 0.7 deg of dorsiflexion).
4. Lower the root until the feet are 0.1 mm above the floor. That is inside the 1 mm contact margin of the foot geoms, so the feet carry the weight from t = 0.

The script also prints the tables below (`--report`). To change the pose, edit the constants in the script and rerun it. `tests/test_standing_keyframe.py` checks that the JSON is what the script writes. The table holds only kinematics: the body masses do not enter it and the knee hinges only matter away from knee 0, so the knee and mass changes of v0.3.0 left it as it was, to the last digit. The contact forces, the speeds and the centre of mass below did change.

## Using it

- `myo_sim.load(name)` and `myo_sim.load_model(name)` include the keyframe; use `mujoco.mj_resetDataKeyframe(model, data, model.key("standing").id)`.
- `myo_sim.load_spec(name)` and `build_spec(name)` leave it out unless you pass `build_kwargs={"standing_keyframe": True}`. A keyframe's `qpos` has a fixed size, so `compile()` fails after deleting a joint from a spec that holds one (`keyframe 'standing': invalid qpos size`), and specs from `load_spec` are meant to be edited. Adding a joint is worse: MuJoCo pads the keyframe at the end, which is right for a new body at the end of the tree, but a hinge added under `femur_r` silently shifts 10 joints of the left leg by one.
- `myo_sim.add_standing_keyframe(spec, model_name)` adds it to an edited spec, matched by joint name: joints the spec no longer has are skipped and joints the table does not list stay at `qpos0`. The root keeps the spec's x, y and orientation.
- `MjSpec.attach` keeps the keyframe under the attach prefix: two bodies attached with `a_` and `b_` give `a_standing` and `b_standing`, each with the other body at its `qpos0`. Tested on MuJoCo 3.17.

## Measurements

MuJoCo 3.17, zero activation, the model's own timestep (2 ms).

| Model | Pose | Root z (m) | Coupling error (mm or mrad) | Contacts at t = 0 | Largest force (N) | Sum / weight | Deepest (mm) |
|---|---|---|---|---|---|---|---|
| myofullbody | `qpos0` | 1.0000 | 52.4 | 2 | 688 | 1.83 | 11.38 |
| myofullbody | `standing` | 0.9455 | 0.0 | 8 | 114 | 0.69 | -0.10 |
| myolegs | `qpos0` | 1.0000 | 52.4 | 0 | 0 | 0.00 | -0.00 |
| myolegs | `standing` | 0.9605 | 0.0 | 8 | 130 | 0.88 | -0.10 |
| myolegs_abdomen | `qpos0` | 1.0000 | 52.4 | 0 | 0 | 0.00 | -0.00 |
| myolegs_abdomen | `standing` | 0.9605 | 0.0 | 8 | 148 | 1.00 | -0.10 |

"Deepest" is the largest overlap of any contact (negative: the closest contact is still apart). At t = 0 only the feet touch in `standing`. Their contact forces add up to at most body weight because the body is already accelerating.

| Model | Pose | Peak speed, 10 ms | Peak speed, 50 ms | Peak speed, 50 ms, muscles off | Deepest in 50 ms (mm) |
|---|---|---|---|---|---|
| myofullbody | `qpos0` | 6.65 (cmc_abduction_l) | 9.81 (knee_angle_l) | 8.24 (cmc_abduction_r) | 11.38 |
| myofullbody | `standing` | 6.43 (cmc_abduction_r) | 8.04 (cmc_abduction_r) | 8.00 (cmc_abduction_r) | 0.66 |
| myolegs | `qpos0` | 4.67 (knee_angle_l) | 9.93 (knee_angle_l) | 0.98 (knee_angle_beta_translation1_l) | 8.39 |
| myolegs | `standing` | 0.73 (subtalar_angle_l) | 2.46 (knee_angle_l) | 0.65 (mtp_angle_l) | 0.71 |
| myolegs_abdomen | `qpos0` | 6.82 (hip_flexion_l) | 14.62 (hip_flexion_r) | 0.98 (knee_angle_beta_translation1_l) | 8.87 |
| myolegs_abdomen | `standing` | 1.18 (hip_flexion_l) | 4.54 (hip_flexion_r) | 3.05 (flex_extension) | 0.68 |

Speeds are the largest joint speed, without the free root joint. The leg speeds at `qpos0` come from the violated couplings and the overlap.

In `myofullbody` the largest speed in `standing` is the thumb: the keyframe leaves the hand at `qpos0`, where the passive spring of `cmc_abduction` holds 0.58 N m on a very small inertia, so the joint reaches 8 rad/s in 50 ms, with or without the muscles. That is above the 50 ms limits of `tests/test_standing_keyframe.py`, so `test_first_50_ms_stay_slow[myofullbody]` fails and stays open. The arm models show the same speed from `qpos0`. Without the hand joints the 50 ms peak is 3.39 rad/s (`hip_flexion_l`), 1.97 rad/s with the muscles off.

In `myolegs_abdomen` `test_first_50_ms_stay_slow` fails as well and stays open: 4.54 rad/s (`hip_flexion_r`, limit 3.0), 3.05 rad/s with the muscles off (`flex_extension`, limit 2.0). The peak with the muscles was already 5.46 rad/s before the knee and pelvis changes; the peak with the muscles off rose from 1.88 rad/s when the sacrum became massless.

What remains in the legs and trunk comes from the passive force of the muscles at zero activation: the pelvis pitches at about 85 rad/s^2 (the largest passive forces on the trunk and hip are 308 N in `rect_abd` and 294 N in `psoas`). A keyframe cannot remove this without bending the body: 0.17 rad of trunk flexion with 0.04 rad of knee flexion (hips and ankles following half the knee angle) lowers the 50 ms peak without the hand joints from 3.4 to 1.9 rad/s. The keyframe stays upright instead.

| Model | Pose | Shortest | Longest | Outside [lmin, lmax] | Over 1.2 | Largest passive force (N), muscle | Largest change from `qpos0` |
|---|---|---|---|---|---|---|---|
| myofullbody | `qpos0` | 0.130 | 1.663 | 4 | 58 | 555, recfem_l | 0.000, Ps_L1_VB_r |
| myofullbody | `standing` | 0.130 | 1.603 | 4 | 58 | 308, rect_abd_r | 1.044, recfem_r |
| myolegs | `qpos0` | 0.656 | 1.663 | 0 | 10 | 555, recfem_l | 0.000, addbrev_r |
| myolegs | `standing` | 0.617 | 1.364 | 0 | 8 | 294, psoas_r | 1.044, recfem_r |
| myolegs_abdomen | `qpos0` | 0.656 | 1.663 | 0 | 10 | 555, recfem_l | 0.000, ercspn_r |
| myolegs_abdomen | `standing` | 0.617 | 1.364 | 0 | 8 | 294, psoas_r | 1.044, recfem_r |

Normalised lengths are in units of optimal fibre length, and `[lmin, lmax]` is the range of the MuJoCo muscle force curve, `[0, 2]` here. The four muscles outside it (`FDS3` and `FDP3` of each hand) are outside at `qpos0` as well. The largest change, `recfem`, is the coupling correction: putting the patella where its coupling says shortens rectus femoris and takes its passive force from 555 N to 0.

## Against the literature

| Model | Heel share of foot load at t = 0 | Forefoot share | Centre of mass ahead of the ankle (cm) |
|---|---|---|---|
| myofullbody | 63% | 37% | -0.9 |
| myolegs | 65% | 35% | -0.7 |
| myolegs_abdomen | 67% | 33% | -0.7 |

| Quantity | Model | Literature | Reference and protocol |
|---|---|---|---|
| Heel share of the load under the foot | 63 % (`myofullbody`), 65 % (`myolegs`), 67 % (`myolegs_abdomen`) | 60 % | Cavanagh, Rodgers and Iiboshi 1987, Foot Ankle 7(5):262-276; barefoot standing, 107 feet |
| Everything ahead of the heel | 37 %, 35 %, 33 % (the model has no separate midfoot) | midfoot 8 %, forefoot 28 %, the rest under the toes | same |
| Centre of mass ahead of the ankle | -0.9 cm (`myofullbody`), -0.7 cm (leg models) | "a few centimetres in front of the ankle joint"; "several centimetres" | Loram and Lakie 2002, J Physiol 545:1041-1053; Loram, Maganaris and Lakie 2007, J Physiol 584:661-675; quiet standing |
| Ankle angle | 0.7 deg of dorsiflexion | mean ankle angles of 1.5 to 4 deg | Loram and Lakie 2002; quiet standing |

The foot load is close. The body stands more upright than people do: its centre of mass is 0.7-0.9 cm behind the ankle rather than a few centimetres ahead of it. Leaning forward would need a larger dorsiflexion than the level foot of step 3 gives, and the keyframe does not attempt it. It is a clean start, not a balanced posture: at zero activation the body collapses within about 0.5 s.
