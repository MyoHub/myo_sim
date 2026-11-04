# Postures

The composed models have a set of named starting postures, stored as keyframes. `qpos0` and every model parameter are unchanged, so they only matter if you start from them. `standing` is the same pose as in 0.2.4, value for value.

| Family | Models | Postures, in keyframe order |
|---|---|---|
| Whole body and legs | `myofullbody`, `myolegs`, `myolegs_abdomen` | `standing`, `ready`, `staggered`, `seat_off`, `squat` |
| 26-muscle legs | `myolegs26` | `standing`, `ready`, `staggered` |
| Arms | `myoarm_r`, `myoarms`, `myotorso_arm_r`, `myotorso_arms` | `arms_at_side`, `forearm_raised`, `reach` |
| Hands | `myohand_r`, `myohands` | `relaxed`, `open` |

`myotorso` and `myotorso_abdomen` have no postures, and no other model gets a keyframe.

## Why

`qpos0` keeps its role as the reference of every joint coupling and the zero of the tendon lengths, but it is not a posture to start from:

- The knee couplings have non-zero constant terms, so at `qpos0` they are up to 52.4 mm off and the constraint solver pulls them back within milliseconds.
- In `myofullbody` and the fixed-base arm models the humerus capsules overlap `thorax_coll1` by 11.4 mm, at 709 N per arm at t = 0 in `myofullbody`.
- The root is at z = 1.0, so the feet are 5.5 cm above the floor in `myofullbody`. In the default scene of `myolegs`, the feet are 4 cm above the stage.
- The hand has its fingers on their extension stops.

One posture is also not enough to learn from. A policy that always starts upright sees one narrow set of muscle lengths and joint angles. These postures spread the start over states a person actually passes through, so a task can start near where it will be used.

## What each posture is for

Balance margins are for `myofullbody`. None of these reasons has been tested in a training run yet.

| Posture | What it is | Why it can help learning | Statically balanced, or holding? |
|---|---|---|---|
| `standing` | Upright, feet flat, arms just clear of the thorax | The reference start | Yes, 3.6 cm inside the support |
| `ready` | Hip 25, knee 30 deg, feet apart, forearms forward | Twice the balance margin of `standing`, joints away from their stops | Yes, 7.1 cm |
| `staggered` | Right foot 0.28 m ahead, both flat | Breaks the left-right symmetry that gait and turning must leave | Yes, 4.8 cm |
| `seat_off` | Hip 93, knee 83 deg, as when leaving a chair | Sit-to-stand start, with a seat in the scene | **No**, 9.5 cm outside the support, behind the heels; without a seat it falls back |
| `squat` | Same joint depth with the centre of mass over the feet | Start for rising or jumping, on its feet | Yes, 7.1 cm |
| `arms_at_side` | Arm down, elbow 30 deg, forearm neutral, relaxed hand | Near the model's passive rest | Holds for 2 s in `myoarm_r` and `myoarms`; lost at 0.2 s in the torso variants, where the trunk moves too |
| `forearm_raised` | Upper arm down, elbow 80 deg | Middle of the elbow range | Falls within 0.2 s |
| `reach` | Upper arm 45 deg forward, elbow 70 deg, hand in front of the chest | Start for reaching and manipulation | Falls within 0.2 s |
| `relaxed` | Resting hand | Measured rest posture; nearly still at zero activation | Holds for 2 s |
| `open` | Fingers just off their stops, thumb abducted | Start for reaching to grasp | Lost at 1.9 s, by the thumb |

No free-standing posture stays up at zero activation, and none is claimed to: "balanced" above is only the centre of mass over the feet at t = 0. All of them lose 5 cm of root height within 0.1 to 0.4 s, and something other than the feet reaches the floor within 0.4 to 1.0 s. Without active muscles the knees buckle.

## How the postures are made

`scripts/generate_postures.py` writes `myo_sim/build/data/postures.json`. Each posture is a short declaration: target joint angles in degrees, whether the feet are the support, and its source. A key names one joint or both sides of one, and keys a model has no joint for are skipped, so one declaration serves every model of a family. From `qpos0` of each composed model the script:

1. Sets the targets. In `standing` and the arm postures the shoulder is elevated by 0.10 rad, which clears the thorax by 6.2 mm (0.05 rad leaves 2 mm of overlap). This is a clearance value, not a measured arm posture.
2. Evaluates every joint coupling from its polynomial, so that couplings hold to rounding.
3. Where the feet are the support, solves together: the ankle angles (heel and forefoot level), the subtalar angles (medial and lateral rail level, where declared and where the model has the joint), a knee (both feet at the same height, in `staggered`), and the pelvis pitch (centre of mass at a declared place along the foot, where declared).
4. Lowers the root until the feet are 0.1 mm above the floor. That is inside the 1 mm contact margin of the foot geoms, so the feet carry the weight from t = 0.
5. Refuses to write the table if a check fails: couplings off, a joint out of range, more than 1 mm of penetration, or, with the feet as support, a foot tilted by more than 2 mm or off the floor, a foot contact over 250 N, or anything else touching.

The result does not depend on hand-edited numbers. To change a posture, edit its declaration and rerun the script; `--report` prints the tables below and `tests/test_postures.py` checks that the JSON is what the script writes. The script fails when a target matches no joint of any model, so a joint rename shows up there. Rerun it after any change to joint names, knee hinges, couplings, foot geoms or masses; the angle targets stay, and the solved values follow.

`myolegs26` has no subtalar joint, so `ready` skips the hip abduction that would roll its feet. Its knee flexes towards negative angles, which the script handles with a sign. It has no `seat_off` or `squat`: at 82.7 deg of knee flexion its couplings leave the range of `knee_r_translation1` (7.9 mm against 7.7 mm) and three via-point joints, and the ankle needs 28.6 deg against a 20 deg limit.

## Using them

- `myo_sim.list_postures(name)` gives the names of a model's postures in keyframe order.
- `myo_sim.load(name)` and `myo_sim.load_model(name)` include all of them; use `mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)`.
- `myo_sim.load_spec(name)` and `build_spec(name)` leave them out unless you pass `build_kwargs={"postures": True}` or a list of names, as in `{"postures": ["standing", "ready"]}`. A keyframe's `qpos` has a fixed size, so `compile()` fails after deleting a joint from a spec that holds one (`invalid qpos size`), and specs from `load_spec` are meant to be edited. Adding a joint is worse: MuJoCo pads the keyframe at the end, which is right for a new body at the end of the tree, but a hinge added under `femur_r` silently shifts 10 joints of the left leg by one.
- `myo_sim.add_posture_keyframes(spec, model_name, postures=None)` and `add_posture_keyframe(spec, model_name, posture)` add them to an edited spec, matched by joint name: joints the spec no longer has are skipped and joints the table does not list stay at `qpos0`. The root keeps the spec's x and y, takes its height from the table and is pitched by the table's pelvis pitch about the world x axis, on top of the spec's orientation. Adding a posture again replaces it and moves it to the end.
- `myo_sim.add_standing_keyframe(spec, model_name)` and `build_kwargs={"standing_keyframe": True}` from 0.2.4 still work and add `standing` alone.
- `MjSpec.attach` keeps the keyframes under the attach prefix: two bodies attached with `a_` and `b_` give `a_standing`, `a_ready`, ... and `b_standing`, `b_ready`, ..., each with the other body at its `qpos0`.

## What changes downstream

`nkey` is now 5 in `myofullbody`, `myolegs` and `myolegs_abdomen` (1 in 0.2.4), 3 in `myolegs26` and the arm models, and 2 in the hand models (0 in 0.2.4).

- Code that reads `key_qpos[0]` when `nkey > 0` keeps getting `standing` in the models that had it. It now also gets a keyframe in `myolegs26` (`standing`), the arm models (`arms_at_side`) and the hand models (`relaxed`), where it used to fall back to `qpos0`. Environments and policies on those models then start from a different state.
- Code that assumes `nkey == 1`, or iterates over all keyframes as a set of resets, needs a look.
- Specs from `load_spec` and `build_spec` are unchanged, and so are compiled arrays other than the keyframes.

## Measurements

MuJoCo 3.17, zero activation, the model's own timestep (2 ms), from `scripts/generate_postures.py --report`.

| Model | Posture | Peak speed, 50 ms | Muscles off, 50 ms | Deepest in 50 ms (mm) | Lost at (s) | Other than feet on the floor at (s) | COM inside support (cm) |
|---|---|---|---|---|---|---|---|
| myofullbody | `qpos0` | 9.92 (knee_angle_l) | 1.45 (flexion_l) | 11.38 | 0.12 | 0.66 | - |
| myofullbody | `standing` | 5.06 (hip_flexion_l) | 0.94 (hip_flexion_r) | 0.44 | 0.20 | 0.50 | 3.6 |
| myofullbody | `ready` | 4.37 (hip_flexion_r) | 2.87 (knee_angle_r) | 0.27 | 0.19 | 0.50 | 7.1 |
| myofullbody | `staggered` | 4.86 (hip_flexion_l) | 2.88 (knee_angle_r) | 0.89 | 0.18 | 0.54 | 4.8 |
| myofullbody | `seat_off` | 5.02 (hip_flexion_r) | 1.49 (knee_angle_r) | -0.00 | 0.20 | 0.50 | -9.5 |
| myofullbody | `squat` | 5.25 (hip_flexion_l) | 1.69 (knee_angle_r) | 0.37 | 0.21 | 0.67 | 7.1 |
| myolegs | `qpos0` | 10.18 (knee_angle_l) | 0.98 (knee_angle_beta_translation1_l) | 7.78 | 0.09 | 0.89 | - |
| myolegs | `standing` | 2.42 (knee_angle_l) | 0.75 (mtp_angle_l) | 0.63 | 0.18 | 0.82 | 3.8 |
| myolegs | `ready` | 2.41 (knee_angle_r) | 2.80 (knee_angle_r) | 0.43 | 0.12 | 0.86 | 7.1 |
| myolegs | `staggered` | 3.19 (knee_angle_l) | 2.60 (knee_angle_l) | 1.03 | 0.15 | 0.93 | 4.8 |
| myolegs | `seat_off` | 1.43 (knee_angle_r) | 1.48 (knee_angle_r) | -0.00 | 0.11 | 0.51 | -13.3 |
| myolegs | `squat` | 1.80 (mtp_angle_r) | 1.70 (knee_angle_r) | 0.57 | 0.11 | 0.84 | 7.0 |
| myolegs26 | `qpos0` | 0.78 (knee_angle_l) | 0.00 (iliopsoas_l_psoas_l-P3_y) | -0.00 | 0.10 | 0.79 | - |
| myolegs26 | `standing` | 3.29 (mtp_angle_r) | 3.31 (mtp_angle_r) | 2.02 | 0.38 | 0.94 | 3.8 |
| myolegs26 | `ready` | 2.94 (knee_angle_r) | 3.39 (knee_angle_r) | 0.80 | 0.13 | 0.69 | 6.6 |
| myolegs26 | `staggered` | 2.64 (mtp_angle_l) | 2.87 (mtp_angle_l) | 1.33 | 0.17 | 0.50 | 3.6 |
| myolegs_abdomen | `qpos0` | 10.50 (knee_angle_r) | 0.98 (knee_angle_beta_translation1_l) | 7.85 | 0.08 | 0.91 | - |
| myolegs_abdomen | `standing` | 2.37 (knee_angle_l) | 1.79 (flex_extension) | 0.62 | 0.19 | 0.74 | 3.8 |
| myolegs_abdomen | `ready` | 2.28 (knee_angle_r) | 2.78 (knee_angle_r) | 0.40 | 0.15 | 0.70 | 7.1 |
| myolegs_abdomen | `staggered` | 2.80 (knee_angle_r) | 2.75 (knee_angle_r) | 0.99 | 0.16 | 0.90 | 4.8 |
| myolegs_abdomen | `seat_off` | 4.38 (flex_extension) | 1.49 (knee_angle_r) | -0.00 | 0.17 | 0.43 | -12.0 |
| myolegs_abdomen | `squat` | 4.10 (flex_extension) | 1.71 (knee_angle_r) | 0.54 | 0.18 | 0.61 | 7.0 |
| myoarm_r | `qpos0` | 1.49 (flexion_r) | 1.47 (flexion_r) | 11.38 | 0.41 (elbow_flexion_r) | - | - |
| myoarm_r | `arms_at_side` | 1.37 (elbow_flexion_r) | 0.92 (elbow_flexion_r) | -0.00 | holds 2 s | - | - |
| myoarm_r | `forearm_raised` | 1.79 (elbow_flexion_r) | 1.59 (elbow_flexion_r) | -0.00 | 0.16 (elbow_flexion_r) | - | - |
| myoarm_r | `reach` | 1.95 (elbow_flexion_r) | 1.14 (elbow_flexion_r) | -0.00 | 0.16 (elbow_flexion_r) | - | - |
| myoarms | `qpos0` | 1.49 (flexion_l) | 1.47 (flexion_l) | 11.38 | 0.41 (elbow_flexion_r) | - | - |
| myoarms | `arms_at_side` | 1.37 (elbow_flexion_r) | 0.92 (elbow_flexion_r) | -0.00 | holds 2 s | - | - |
| myoarms | `forearm_raised` | 1.79 (elbow_flexion_r) | 1.59 (elbow_flexion_r) | -0.00 | 0.16 (elbow_flexion_r) | - | - |
| myoarms | `reach` | 1.95 (elbow_flexion_l) | 1.14 (elbow_flexion_l) | -0.00 | 0.16 (elbow_flexion_r) | - | - |
| myotorso_arm_r | `qpos0` | 1.38 (flexion_r) | 1.35 (flexion_r) | 11.38 | 0.28 (elbow_flexion_r) | - | - |
| myotorso_arm_r | `arms_at_side` | 1.60 (elbow_flexion_r) | 0.91 (elbow_flexion_r) | -0.00 | 0.21 (elbow_flexion_r) | - | - |
| myotorso_arm_r | `forearm_raised` | 1.53 (elbow_flexion_r) | 1.56 (elbow_flexion_r) | -0.00 | 0.19 (elbow_flexion_r) | - | - |
| myotorso_arm_r | `reach` | 1.69 (elbow_flexion_r) | 1.12 (elbow_flexion_r) | -0.00 | 0.18 (elbow_flexion_r) | - | - |
| myotorso_arms | `qpos0` | 1.46 (flexion_l) | 1.44 (flexion_l) | 11.38 | 0.32 (elbow_flexion_r) | - | - |
| myotorso_arms | `arms_at_side` | 1.50 (elbow_flexion_r) | 0.92 (elbow_flexion_l) | -0.00 | 0.24 (elbow_flexion_r) | - | - |
| myotorso_arms | `forearm_raised` | 1.63 (elbow_flexion_l) | 1.59 (elbow_flexion_l) | -0.00 | 0.18 (elbow_flexion_l) | - | - |
| myotorso_arms | `reach` | 1.78 (elbow_flexion_l) | 1.14 (elbow_flexion_l) | -0.00 | 0.17 (elbow_flexion_l) | - | - |
| myohand_r | `qpos0` | 1.29 (mcp2_abduction_r) | 0.51 (deviation_r) | -0.00 | holds 2 s | - | - |
| myohand_r | `relaxed` | 0.48 (deviation_r) | 0.50 (deviation_r) | -0.00 | holds 2 s | - | - |
| myohand_r | `open` | 1.58 (mcp2_flexion_r) | 0.50 (deviation_r) | -0.00 | 1.94 (cmc_flexion_r) | - | - |
| myohands | `qpos0` | 1.29 (mcp2_abduction_l) | 0.51 (deviation_l) | -0.00 | holds 2 s | - | - |
| myohands | `relaxed` | 0.48 (deviation_l) | 0.50 (deviation_l) | -0.00 | holds 2 s | - | - |
| myohands | `open` | 1.58 (mcp2_flexion_r) | 0.50 (deviation_l) | -0.00 | 1.94 (cmc_flexion_l) | - | - |

Speeds are in rad/s (or m/s for a slide joint), the largest joint speed without the free root joint. "Lost at" is when the root has dropped by 5 cm in a free-standing model, and in a fixed-base model when a joint other than a coupled one has moved by 20 deg, or "holds" if that does not happen within 2 s. "Deepest" is the largest overlap of any contact over the 50 ms, with -0.00 for no contact.

What this shows:

- The couplings and the arm overlap are what make `qpos0` fast: 10 rad/s against 2 to 5 rad/s in `standing`. `myolegs26` has no coupling error and starts in the air, so its `qpos0` is slow for the 50 ms it spends falling.
- The speeds that remain in the leg models come from the passive force of the muscles at zero activation: `standing` in `myofullbody` runs at 5.1 rad/s with the muscles and 0.9 rad/s without them.
- Flexing the legs does not remove this. `ready` is only 14 % slower than `standing` in `myofullbody` (4.4 against 5.1 rad/s), and the same in the leg models. Without muscles it is faster than `standing`, because the bent knee has nothing to hold it. The gain of `ready` is its balance margin, not a quieter start.
- `relaxed` is the clearest gain: 0.48 rad/s against 1.29 at `qpos0`, in a hand whose fingers rest on their stops at `qpos0`.

On MuJoCo 3.4 and 3.9 the table is the same and `tests/test_postures.py` passes. Outside `myolegs26` the peak speeds stay within 0.09 rad/s of the values above, the speeds with muscles off within 0.32 rad/s, the deepest contact within 1.2 mm (against the cylinder of the default scene) and the "lost at" times are the same. The mesh feet of `myolegs26` differ more: on 3.4 and 3.9 `standing` reaches 5.7 rad/s at the toes and sinks 9 to 10 mm, against 3.3 rad/s and 2 mm on 3.17.

| Model | Posture | Root z (m) | Pelvis pitch (deg) | Trunk rotation (deg) | Ankle r, l (deg) | Knee r, l (deg) | Stance width (m) | Step length (m) | Contacts at t = 0 | Largest force (N) | Sum / weight | Heel share of foot load | COM ahead of the ankle (cm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| myofullbody | `standing` | 0.946 | 0.0 | 0.0 | 0.7, 0.7 | 0.0, 0.0 | 0.152 | 0.000 | 8 | 111 | 0.69 | 64% | -0.3 |
| myofullbody | `ready` | 0.901 | 7.0 | 16.4 | 14.5, 14.5 | 30.0, 30.0 | 0.294 | 0.000 | 8 | 70 | 0.51 | 46% | 3.3 |
| myofullbody | `staggered` | 0.921 | 1.9 | 1.9 | 3.2, 16.3 | 20.3, 5.0 | 0.146 | 0.281 | 8 | 98 | 0.59 | 48% | 4.2 |
| myofullbody | `seat_off` | 0.617 | 19.4 | 32.5 | 9.3, 9.3 | 82.7, 82.7 | 0.119 | 0.000 | 8 | 40 | 0.31 | 56% | -13.4 |
| myofullbody | `squat` | 0.660 | 35.6 | 35.6 | 26.0, 26.0 | 82.7, 82.7 | 0.119 | 0.000 | 8 | 80 | 0.35 | 9% | 3.2 |
| myolegs | `standing` | 0.961 | 0.0 | 0.0 | 0.7, 0.7 | 0.0, 0.0 | 0.152 | 0.000 | 8 | 121 | 0.82 | 65% | -0.0 |
| myolegs | `ready` | 0.915 | 8.8 | 8.8 | 16.4, 16.4 | 30.0, 30.0 | 0.294 | 0.000 | 8 | 73 | 0.48 | 31% | 3.3 |
| myolegs | `staggered` | 0.936 | 2.0 | 2.0 | 3.4, 16.3 | 20.5, 5.0 | 0.146 | 0.280 | 8 | 97 | 0.68 | 50% | 4.2 |
| myolegs | `seat_off` | 0.632 | 19.4 | 19.4 | 9.3, 9.3 | 82.7, 82.7 | 0.119 | 0.000 | 8 | 39 | 0.33 | 56% | -17.1 |
| myolegs | `squat` | 0.677 | 37.6 | 37.6 | 28.0, 28.0 | 82.7, 82.7 | 0.119 | 0.000 | 8 | 83 | 0.40 | 6% | 3.2 |
| myolegs26 | `standing` | 0.928 | 0.0 | 0.0 | 0.6, 0.6 | 0.0, 0.0 | 0.167 | 0.000 | 4 | 71 | 0.19 | 97% | 0.5 |
| myolegs26 | `ready` | 0.893 | 8.7 | 8.7 | 14.3, 14.3 | 30.0, 30.0 | 0.167 | 0.000 | 4 | 96 | 0.26 | 97% | 3.2 |
| myolegs26 | `staggered` | 0.908 | 1.6 | 1.6 | 3.4, 15.2 | 21.2, 5.0 | 0.167 | 0.274 | 4 | 125 | 0.26 | 97% | 4.1 |
| myolegs_abdomen | `standing` | 0.961 | 0.0 | 0.0 | 0.7, 0.7 | 0.0, 0.0 | 0.152 | 0.000 | 8 | 121 | 0.82 | 65% | -0.0 |
| myolegs_abdomen | `ready` | 0.916 | 8.1 | 13.1 | 15.7, 15.7 | 30.0, 30.0 | 0.294 | 0.000 | 8 | 71 | 0.48 | 34% | 3.3 |
| myolegs_abdomen | `staggered` | 0.936 | 2.0 | 2.0 | 3.4, 16.3 | 20.5, 5.0 | 0.146 | 0.280 | 8 | 93 | 0.64 | 47% | 4.2 |
| myolegs_abdomen | `seat_off` | 0.632 | 19.4 | 26.4 | 9.3, 9.3 | 82.7, 82.7 | 0.119 | 0.000 | 8 | 38 | 0.34 | 56% | -15.8 |
| myolegs_abdomen | `squat` | 0.677 | 37.6 | 37.6 | 28.0, 28.0 | 82.7, 82.7 | 0.119 | 0.000 | 8 | 83 | 0.40 | 6% | 3.2 |

Trunk rotation is the angle of the torso body from `qpos0`. The heel share is of the contact force at t = 0, when passive muscle force is already acting. In `squat` the passive calf pull moves it onto the forefoot (6 to 9 %) even though the centre of mass is mid-foot. The mesh feet of `myolegs26` have their contact points on the heel at t = 0 (97 %).

| Model | Posture | Shortest | Longest | Outside [lmin, lmax] | Over 1.2 | Largest passive force (N), muscle |
|---|---|---|---|---|---|---|
| myofullbody | `standing` | -0.977 | 1.603 | 2 | 34 | 583, DELT2 |
| myofullbody | `ready` | -0.889 | 1.590 | 4 | 34 | 443, DELT2 |
| myofullbody | `staggered` | -0.977 | 1.603 | 4 | 36 | 583, DELT2_l |
| myofullbody | `seat_off` | -0.727 | 1.583 | 4 | 46 | 476, glmax1_l |
| myofullbody | `squat` | -0.727 | 1.603 | 4 | 44 | 542, soleus_r |
| myolegs | `standing` | 0.619 | 1.308 | 0 | 6 | 204, bfsh_l |
| myolegs | `ready` | 0.632 | 1.322 | 0 | 8 | 240, soleus_r |
| myolegs | `staggered` | 0.625 | 1.440 | 0 | 8 | 249, soleus_l |
| myolegs | `seat_off` | 0.436 | 1.474 | 0 | 20 | 476, glmax1_l |
| myolegs | `squat` | 0.436 | 1.474 | 0 | 20 | 605, soleus_r |
| myolegs26 | `standing` | 0.491 | 1.209 | 0 | 2 | 65, bifemsh_r |
| myolegs26 | `ready` | 0.515 | 1.149 | 0 | 0 | 177, glutmax_r |
| myolegs26 | `staggered` | 0.513 | 1.190 | 0 | 0 | 121, glutmax_r |
| myolegs_abdomen | `standing` | 0.619 | 1.308 | 0 | 6 | 204, bfsh_l |
| myolegs_abdomen | `ready` | 0.632 | 1.316 | 0 | 8 | 224, soleus_r |
| myolegs_abdomen | `staggered` | 0.625 | 1.440 | 0 | 8 | 249, soleus_l |
| myolegs_abdomen | `seat_off` | 0.436 | 1.474 | 0 | 20 | 476, glmax1_l |
| myolegs_abdomen | `squat` | 0.436 | 1.474 | 0 | 20 | 605, soleus_r |
| myoarm_r | `arms_at_side` | -0.977 | 1.344 | 2 | 3 | 583, DELT2 |
| myoarm_r | `forearm_raised` | -0.977 | 1.344 | 2 | 2 | 583, DELT2 |
| myoarm_r | `reach` | -0.727 | 1.262 | 2 | 1 | 136, DELT2 |
| myoarms | `arms_at_side` | -0.977 | 1.344 | 4 | 6 | 583, DELT2 |
| myoarms | `forearm_raised` | -0.977 | 1.344 | 4 | 4 | 583, DELT2 |
| myoarms | `reach` | -0.727 | 1.262 | 4 | 2 | 136, DELT2_l |
| myotorso_arm_r | `arms_at_side` | -0.977 | 1.603 | 2 | 25 | 583, DELT2 |
| myotorso_arm_r | `forearm_raised` | -0.977 | 1.603 | 2 | 24 | 583, DELT2 |
| myotorso_arm_r | `reach` | -0.727 | 1.603 | 2 | 23 | 308, rect_abd_r |
| myotorso_arms | `arms_at_side` | -0.977 | 1.603 | 4 | 28 | 583, DELT2 |
| myotorso_arms | `forearm_raised` | -0.977 | 1.603 | 4 | 26 | 583, DELT2 |
| myotorso_arms | `reach` | -0.727 | 1.603 | 4 | 24 | 308, rect_abd_r |
| myohand_r | `relaxed` | 0.362 | 1.020 | 1 | 0 | 0, EPB_r |
| myohand_r | `open` | 0.553 | 1.188 | 0 | 0 | 16, RI2_r |
| myohands | `relaxed` | 0.362 | 1.020 | 2 | 0 | 0, EPB_r |
| myohands | `open` | 0.553 | 1.188 | 0 | 0 | 16, RI2_l |

Normalised lengths are in units of optimal fibre length, and `[lmin, lmax]` is the range of the MuJoCo muscle force curve. Two muscles outside it (`LAT1`, `LAT1_l`) are outside at `qpos0` as well. The others are `RI5` and `RI5_l`, which at 0.36 sit below their `lmin` of 0.5 in the postures with the relaxed hand; they are short in the model's own rest state too.

Where the fixed-base models settle at zero activation, in degrees, after 4 s from their first posture:

- myoarm_r, peak speed at the end 0.179 rad/s: elv_angle_r 0.0 to 32.8, shoulder_elv_r 5.7 to 4.5, shoulder_rot_r 0.0 to -18.5, elbow_flexion_r 30.0 to 44.6, pro_sup_r 0.0 to -20.5, deviation_r -6.4 to -10.1, flexion_r -15.6 to -18.4, cmc_flexion -30.0 to -35.4, cmc_abduction 5.0 to -4.6, mp_flexion_r -42.8 to -31.3, ip_flexion_r -9.4 to -9.1, mcp2_flexion_r 27.2 to 24.4, mcp2_abduction_r 0.0 to -7.5, pm2_flexion_r 26.4 to 25.4, md2_flexion_r 9.6 to 9.6, mcp3_flexion_r 31.8 to 27.0, mcp3_abduction_r 0.0 to -6.0, pm3_flexion_r 27.3 to 25.8, md3_flexion 9.9 to 9.9, mcp4_flexion_r 23.8 to 18.1, mcp4_abduction_r 0.0 to -5.2, pm4_flexion 32.1 to 30.5, md4_flexion_r 8.2 to 8.2, mcp5_flexion_r 28.5 to 25.0, mcp5_abduction_r 0.0 to -2.6, pm5_flexion_r 33.9 to 33.0, md5_flexion_r 11.6 to 11.6
- myohand_r, peak speed at the end 0.043 rad/s: pro_sup_r 0.0 to 3.3, deviation_r -6.4 to -10.1, flexion_r -15.6 to -19.5, cmc_flexion_r -30.0 to -44.7, cmc_abduction_r 5.0 to 4.0, mp_flexion_r -42.8 to -32.2, ip_flexion_r -9.4 to -10.1, mcp2_flexion_r 27.2 to 26.8, mcp2_abduction_r 0.0 to -11.1, pm2_flexion_r 26.4 to 26.4, md2_flexion_r 9.6 to 9.6, mcp3_flexion_r 31.8 to 29.0, mcp3_abduction_r 0.0 to -9.9, pm3_flexion_r 27.3 to 27.0, md3_flexion_r 9.9 to 9.9, mcp4_flexion_r 23.8 to 18.7, mcp4_abduction_r 0.0 to -9.4, pm4_flexion_r 32.1 to 31.2, md4_flexion_r 8.2 to 8.2, mcp5_flexion_r 28.5 to 25.4, mcp5_abduction_r 0.0 to -4.7, pm5_flexion_r 33.9 to 33.4, md5_flexion_r 11.6 to 11.6

## Against the literature

Where a target is taken from a paper, the table gives the model next to the paper. Angles are not always defined the same way: marker sets and radiographs differ from the joint axes of this model, so treat these as data points, not validation.

| Quantity | Model | Literature | Reference and protocol |
|---|---|---|---|
| `standing`: heel share of the load under the foot | 64 % (`myofullbody`), 65 % (`myolegs`, `myolegs_abdomen`) | 60 % | Cavanagh, Rodgers and Iiboshi 1987, Foot Ankle 7(5):262-276; barefoot standing, 107 feet |
| `standing`: centre of mass ahead of the ankle | -0.3 cm, 0.0 cm | "a few centimetres in front of the ankle joint"; "several centimetres" | Loram and Lakie 2002, J Physiol 545:1041-1053; Loram, Maganaris and Lakie 2007, J Physiol 584:661-675; quiet standing |
| `standing`: ankle dorsiflexion | 0.7 deg | mean 1.5 to 4 deg; 4.0 deg (95 % CI 3.6 to 4.4) as shank inclination | Loram and Lakie 2002; Hasegawa et al. 2017, J Anat, doi:10.1111/joa.12586; radiographs of 136 adults standing with hands on their cheeks |
| `standing`: knee flexion | 0 deg | -1.6 deg (95 % CI -2.3 to -0.8) | Hasegawa et al. 2017 |
| `ready`: knee flexion | 30 deg, chosen | 58 +- 3 deg (mean, SE), football goalkeepers before a dive | Ibrahim et al. 2019, Front Sports Act Living, doi:10.3389/fspor.2019.00015; 10 elite goalkeepers |
| `ready`: stance width | 0.29 m between ankle centres | 39 to 50 % of leg length between the medial malleoli | Ibrahim et al. 2019 |
| `staggered`: hip flexion, front and rear | 20, -8 deg, chosen | peak +33.3 and -13.0 deg over the walking cycle | Winiarski, Pietraszewska and Pietraszewski 2019, Biomed Res Int 2019:9232430, doi:10.1155/2019/9232430; 20 young women at 1.32 m/s, cycle peaks, not a gait event |
| `staggered`: knee, front and rear | 20, 5 deg | cycle range 1.4 to 61.5 deg | same |
| `staggered`: ankle, front and rear | 3, 16 deg | cycle range -19.3 to 14.5 deg | same |
| `staggered`: step length | 0.28 m | 0.61 +- 0.08 m | same; the model keeps both feet flat, so it stays much shorter than a walking step |
| `seat_off`: hip flexion | 93.3 deg, target | 93.3 +- 9.0 deg | Kinoshita, Kiyama and Yoshimoto 2015, PLoS One, doi:10.1371/journal.pone.0133747, Table 1; 16 healthy young adults at seat-off, no handrail |
| `seat_off`: knee flexion | 82.7 deg, target | 82.7 +- 7.4 deg | same |
| `seat_off`: ankle dorsiflexion | 9.3 deg, from the pelvis pitch | 9.0 +- 4.6 deg | same |
| `seat_off`: trunk forward tilt | 32.5 deg (`myofullbody`), 26.4 deg (`myolegs_abdomen`), 19.4 deg (`myolegs`, no lumbar joints) | 32.2 +- 5.3 deg, care-needing older adults; young adults did not differ significantly | same |
| `squat`: ankle dorsiflexion | 26.0 to 28.0 deg, a result | 25.9 +- 5.1, 23.4 +- 5.9 deg, at a deeper squat | Endo, Miura and Sakamoto 2020, J Phys Ther Sci, doi:10.1589/jpts.32.391, Table 1; 9 healthy males at maximum depth, hip 124 and knee 125 deg |
| `squat`: trunk tilt | 35.6 to 37.6 deg, a result | 33.2 +- 8.3, 32.6 +- 8.0 deg | same |
| `relaxed`: wrist | 15.6 deg extension, 6.4 deg ulnar, targets; rests at 19.5 deg and 10.1 deg in the model | -15.6 +- 5.6 deg, 6.4 +- 5.3 deg | Lee and Jung 2010, J Ergon Soc Korea 29(5):727-734, doi:10.5143/JESK.2010.29.5.727, neutral forearm, 40 students standing, palm towards the leg; about 16 deg also in Lee and Jung 2014, Int J Occup Saf Ergon 20:91-101 |
| `relaxed`: index MCP, PIP, DIP | 27.2, 26.4, 9.6 deg, targets; rest at 26.8, 26.4, 9.6 in the model | 27.2 +- 4.4, 26.4 +- 7.7, 9.6 +- 5.0 deg | Lee and Jung 2010 |
| `relaxed`: middle MCP, PIP, DIP | 31.8, 27.3, 9.9 deg, targets; MCP rests at 29.0 | 31.8 +- 4.3, 27.3 +- 10.8, 9.9 +- 5.3 deg | same |
| `relaxed`: ring MCP, PIP, DIP | 23.8, 32.1, 8.2 deg, targets | 23.8 +- 4.8, 32.1 +- 8.5, 8.2 +- 4.4 deg | same |
| `relaxed`: little MCP, PIP, DIP | 28.5, 33.9, 11.6 deg, targets | 28.5 +- 7.2, 33.9 +- 10.6, 11.6 +- 6.6 deg | same |
| `relaxed`: thumb MCP, IP | 42.8, 9.4 deg, targets; 42.8 is 2 deg inside the model's limit | 42.8 +- 9.5, 9.4 +- 5.5 deg | same |
| `forearm_raised`: elbow | 80 deg, the middle of the arc | functional arc 30 to 130 deg | Morrey, Askew and Chao 1981, J Bone Joint Surg Am 63:872-877; 33 normal subjects, 15 daily activities |
| `reach`: shoulder elevation | 45 deg, chosen | maximum 49.7 +- 11.1 deg eating with a spoon, 39.2 +- 10.9 deg with chopsticks | Inagaki et al. 2025, Front Sports Act Living, doi:10.3389/fspor.2025.1646326, Table 1; 31 healthy adults, seated, maxima over each movement |
| `reach`: elbow | 70 deg, chosen | 48.2 +- 10.3 to 121.4 +- 4.2 deg eating with a spoon | same, Table 2 |

The body stands more upright than people do: its centre of mass is over the ankle rather than a few centimetres ahead of it. Leaning forward would need a larger dorsiflexion than the level foot gives, and `standing` does not attempt it.

### Chosen, not measured

These values are ours, with the reason given:

- **`ready`.** Hip 25, knee 30, trunk -5 deg of `flex_extension`, 6 deg of hip abduction, and the arm values. We found no measured general ready stance, only the goalkeeper data above, which is much deeper. Values were chosen to keep the joints mid-range with the centre of mass mid-foot.
- **`staggered`.** All hip and knee targets. They are inside the peak ranges of walking but are not a gait event, and we have no measured static staggered stance.
- **`squat`.** Hip and knee are borrowed from seat-off; the ankle and trunk are results of balancing, not targets.
- **Arm postures.** Shoulder elevation 0.10 rad is a clearance value. Elbow 30 deg in `arms_at_side` lies between a hanging arm and the model's passive rest of about 45 deg. We found no measured value for a hanging arm's elbow or forearm rotation, so the forearm is at neutral, the middle of its range. The arm in `ready`, `seat_off` and `squat` is chosen too.
- **Pelvis pitch in `seat_off`** (19.4 deg), set so that the ankle comes out near the measured 9.0 deg; lumbar flexion in `seat_off` (-7 deg) brings the trunk of `myofullbody` near the measured tilt.
- **Thumb CMC and finger abduction in `relaxed`.** The published CMC angles are between marker-defined segments and do not map onto the axes of this model. CMC is set close to where the model's thumb rests, and abduction to zero.
- **`open`.** All values except the wrist: small flexion just off the joint limits.

In this model the wrist deviation joint is positive towards the radial side and ranges from -10 to +25 deg, so the measured 6.4 deg of ulnar deviation is -6.4 deg, and the wrist rests on its -10 deg stop.
