# Model scaling

Scale a model to known factors, with the rules of OpenSim's Scale Tool (Delp et al. 2007). The code is in `myo_sim/scaling.py` (rules), `myo_sim/opensim_scales.py` (OpenSim scale files and segment maps) and `myo_sim/scaling_checks.py` (operating-curve refit and checks). Nothing runs at load time: `import myo_sim` does not import these modules, and a scaled model steps as fast as the original (MyoFullBody, 1000 steps: 0.30 s and 0.31 s).

Scaling rules and parts of this module are adapted from MyoSuite PR #525 by Mark Colley (M-Colley), Apache-2.0 ([MyoHub/myosuite#525](https://github.com/MyoHub/myosuite/pull/525)). The OpenSim comparison follows MyoSuite PR #564, by the same author.

## API

```python
import myo_sim
from myo_sim import check_scaled_model, fit_length_ranges, scale_report, scale_spec, segment_factors

spec = myo_sim.load_spec("myofullbody")

scaled = scale_spec(spec, {"femur_r": 1.08, "tibia_r": (1.0, 0.96, 1.0)})  # one factor, or three along the body's axes
scaled = scale_spec(spec, {"femur_r": segment_factors("y", long=1.12, trans=0.98)})
scaled = scale_spec(spec, "subject_scale.xml", total_mass=72.0)             # OpenSim ScaleSet or scaled .osim
scaled = scale_spec(spec, {"femur_r": (model_points, measured_points)})     # point correspondences
model = scaled.compile()

report = scale_report(spec, scales)                # what would be approximated or skipped
refitted, r2 = fit_length_ranges(scaled, spec.copy().compile())   # optional operating-curve refit, R² per muscle
checks = check_scaled_model(model, spec.copy().compile())         # checks.passed, checks["operating_range"], ...
```

| Name | What it does |
|---|---|
| `scale_spec(spec, scales, *, total_mass=None)` | Returns a new scaled `MjSpec`; `spec` stays as it is. A compiled `spec` is fine. |
| `segment_factors(long_axis, long, trans)` | `(sx, sy, sz)` with one factor along a segment's long axis and one across it. |
| `scale_report(spec, scales)` | `ScaleReport`: per-body factors, `approximated_geoms`, `skipped`. |
| `fit_length_ranges(spec, reference, ...)` | Returns a new spec with refitted muscle ranges, and R² per muscle. |
| `check_scaled_model(model, reference, ...)` | `operating_range`, `moment_arm_signs`, `stability`, `constraints`. |
| `model_digest(model)` | SHA-256 of a compiled model, independent of where its assets are. It changes with the MuJoCo version. |

A refused scaling (`KeyError`, `ValueError`, `NotImplementedError`) raises before any edit. Bodies, joints, tendons and actuators stay in their order, so an environment's observation and action spaces keep their shape.

## Rules

Each body has factors `(sx, sy, sz)` along its own axes, the convention of OpenSim's Scale Tool.

- **Bodies.** Everything a body carries scales with its factors: geoms, meshes, sites, joint anchors, cameras, lights and its inertial frame. A child body's position scales with its parent's factors. Frames of any nesting and every orientation type are resolved as MuJoCo resolves them. Joint axes do not change. The world is not scaled.
- **Shapes.** Sites and primitive geoms take the factors along their own axes, with OpenSim's arithmetic mean across a cylinder's axis and over all axes for a sphere. A primitive fitted to a mesh (`type="box" mesh=...`) is refitted to the scaled mesh, and counts as a user of that mesh.
- **Slide joints.** Translations, ranges, `ref`, `springref`, keyframe values and speeds scale with the parent's factor along the joint axis. Polynomial joint couplings `y1 = sum c_k y2^k` become `a c_k / b^k`. Stiffness and damping divide by the factor, so the same relative motion gives the same force.
- **Muscles.** Optimal fibre and tendon slack length follow the muscle-tendon length in the default pose: `qpos0` with every joint coupling and slide tie assembled, as OpenSim assembles it. MuJoCo derives both from `lengthrange`, which takes the ratio. Peak force is kept, automatic peak forces included.
- **Spatial tendons.** Range, margin and spring lengths take the tendon's own length ratio. Stiffness and damping divide by it, so a ligament keeps its force at the same strain.
- **Fixed tendons.** A slide coordinate scales with its joint. A hinge coefficient below 0.2 is taken as a moment arm in metres. `MjSpec` cannot edit the coefficients of a fixed tendon, so the spring's stiffness and damping take the moment arm's factor (the mean factor across the hinge axis in the joint's body): the tendon's length stays in joint coordinates and its joint torque grows with the moment arm. A larger coefficient is a plain joint angle; such a spring keeps its torque and is listed in the report.
- **Tendon equalities.** `L1 = sum c_k L2^k` becomes `a c_k / b^k` for `k != 1`, with `a` and `b` the tendons' ratios. The slope `c_1` is a kinematic ratio and stays: a slide node tied to a tendon follows that tendon's excursion, however much of the tendon was scaled.
- **Wraps.** A small change of a path can send it the long way around a wrap geom. Two paths are the same if they run over the same objects and pass each wrap geom on the same side: the middle of the arc, seen from the geom's centre, lies on the same side in both. If a path differs from the unscaled one in the default pose, and its length ratio there is more than 10 % from the median over 32 other poses where the paths are the same, the median is used and the report names the tendon. If they differ in every one of those poses, the ratio stays and the report names the tendon too.
- **Mass.** Mass scales with volume, and inertia by the exact affine transform of the mass distribution. An `alignfree` body scales from its authored inertial frame, and is aligned again when compiled. `total_mass` then rescales the subject's kinematic trees (those holding a body named in `scales`, or else the one articulated tree); scenery and other agents keep their mass.
- **Ground.** A free-floating subject is raised or lowered in `qpos0` and in each keyframe separately, so that its lowest collision point keeps its unscaled height in that pose.
- **OpenSim files.** A ScaleSet or a scaled `.osim`. The file's Rajagopal segments are mapped onto the composed models (`Full Body` root) or the stand-alone leg (`root`, `torso`). A model takes the segments it has; a body takes the factors of its nearest mapped ancestor. During parsing the factors are converted between the axis conventions of OpenSim (y-up) and of each body, since most bodies kept OpenSim's frames and a few are z-up.
- **Point correspondences.** `(model_points, measured_points)`: points on the body in its frame and the same points measured in metres in any right-handed frame, NaN rows ignored. Points spanning all three directions (at least four) give three factors by least squares with a nuisance rotation; two points or points on a plane give one factor, the ratio of their spreads. A fit outside 0.5 to 2 is refused, since it usually means millimetres or mismatched points.

## Operating-curve refit

The rule above sets each muscle's lengths from one pose. Under per-axis or per-segment factors a muscle's excursion does not change as its length does, so its normalised operating curve drifts. `fit_length_ranges` is a separate step, adapted from the idea of Modenese et al. (2016) of preserving the normalised operating curve over the joint ranges: over random poses, `L_ref = a + b L_scaled`, and the muscle's `lengthrange` becomes `(range_ref - a) / b`. MuJoCo derives optimal fibre length and tendon slack length from that range, so both are refitted. Poses where a wrap changes its path are left out. A muscle with R² below `min_r2` (default 0.9) keeps its range.

## Checks

`check_scaled_model` measures a scaled model against its reference. `operating_range` and `moment_arm_signs` hold for each muscle, and each `value` is that of the worst muscle, so one wrong muscle among hundreds fails.

- **`operating_range`** is each muscle's 99th percentile over 200 poses of the normalised length difference, against 0.05. That is 5 % of the muscle's own range. At the steepest point of its force-length curve this moves active force by up to 6 % of peak in the median muscle of `myofullbody`, and 18 % in the worst. The percentile leaves out a pose or two per muscle, close to a wrap's change of path. The detail names the muscles above the limit.
- **`moment_arm_signs`** is each muscle's fraction of moment arms with the reference's sign, in the assembled default pose, where the reference's arm exceeds 5 mm. No muscle crosses 100 joints, so the limit of 0.99 allows no flip.
- **`stability`** and **`constraints`** bound the largest joint speed and equality residual of a rollout.

Each bound fails on a deliberate error: a range shifted by 6 % of its width on one of 416 muscles (it passes at 4 %), `glmax3` flipping on its own among 944 muscle-joint pairs, an oversized timestep, loose constraints.

A similarity passes them all. Other factors usually fail `operating_range` before the refit, since a muscle's excursion does not change as its length does, and a changed wrap path fails after the refit too. A failed check does not by itself make a model unusable, but it says which muscles to look at. On the default models, with three draws each of per-axis factors in [0.9, 1.1], either per OpenSim segment or per body:

| Model, factors | `operating_range`, before / after the refit | Muscles above 0.05, before / after | `moment_arm_signs`, flipped |
|---|---:|---:|---|
| `myolegs`, 1.1 everywhere | 0.000 / 0.000 | 0 / 0 | none |
| `myolegs`, PR #564's per-axis set | 1.185 / 1.193 | 9 / 2 | `glmax3` |
| `myolegs`, femur 1.12, tibia 0.95 | 0.101 / 0.026 | 18 / 0 | none |
| `myolegs`, per segment | 0.11 to 1.20 / 0.08 to 1.20 | 9 to 18 / 1 to 9 | `glmax3` in one draw |
| `myolegs`, per body | 0.15 to 1.18 / 0.08 to 1.20 | 26 to 30 / 7 to 11 | `glmax3` in one draw |
| `myofullbody`, 1.1 everywhere | 0.000 / 0.000 | 0 / 0 | none |
| `myofullbody`, PR #564's per-axis set | 1.174 / 1.185 | 16 / 8 | `glmax3` |
| `myofullbody`, femur 1.12, tibia 0.95 | 0.102 / 0.026 | 19 / 0 | none |
| `myofullbody`, per segment | 1.14 to 2.80 / 1.22 to 2.84 | 102 to 200 / 27 to 36 | `glmax3`, `IL_R7`, `IL_R9`, `TRIlong`, `SUP` |
| `myofullbody`, per body | 1.03 to 1.70 / 1.04 to 1.73 | 80 to 107 / 21 to 32 | `IL_R7`, `TRIlong_l` |
| `myoarm_r`, per segment | 0.55 to 2.03 / 0.57 to 2.03 | 13 to 23 / 2 to 3 | `TRIlong`, `SUP` |
| `myoarm_r`, per body | 0.97 to 1.71 / 1.04 to 1.75 | 10 to 21 / 3 to 6 | none |
| `myohand_r`, per segment | 0.73 to 0.74 / 0.69 to 0.73 | 5 to 17 / 1 to 2 | none |
| `myohand_r`, per body | 0.20 to 1.71 / 0.03 to 1.74 | 3 to 7 / 0 to 2 | none |

The muscles that stay high after the refit are mostly those whose path changes (`glmax3`, `INFSP`, `PECM1`, `RI2`, `EIP`, `SUP`, `DELT2`, `IL_L1`), and under independent factors some foot muscles such as `tibpost` and `perbrev`.

## Comparison with OpenSim

`scripts/validate_opensim_scaling.py` scales Rajagopal2016 (Rajagopal et al. 2016; [opensim-models](https://github.com/opensim-org/opensim-models) commit `d9b05d4`, SHA-256 `3f5c5f23...`) with OpenSim 4.6's `ModelScaler`, and stores the results in `tests/data/opensim_scaling/reference.json` (MIT notice in `LICENSE-Rajagopal.txt` next to it). `tests/test_opensim_comparison.py` scales `myolegs` through the ScaleSet route with the same factors (`fixture.json`, synthetic) and compares without OpenSim. Differences are of changes from each program's nominal model; the nominal joint centres of the two models agree within a micrometre.

| Quantity (all 10 joint centres, 80 muscles, 11 bodies) | 1.1 everywhere | Per-axis factors (PR #564's set) | Femur 1.12, tibia 0.95 |
|---|---:|---:|---:|
| Joint centre in the pelvis frame, largest difference | 3e-8 m | 0.32 mm | 1.38 mm |
| Thigh length, this code / OpenSim (nominal 404.43 mm) | 444.87 / 444.87 mm | 440.82 / 440.90 mm | 452.96 / 453.56 mm |
| Shank length, this code / OpenSim (nominal 400.12 mm) | 440.14 / 440.14 mm | 428.11 / 428.11 mm | 380.12 / 380.12 mm |
| Optimal fibre and slack length ratio, median difference | 4e-16 | 8.7e-5 | 1.5e-5 |
| Optimal fibre and slack length ratio, largest difference | 8e-7 | 0.0051 (`glmax3_r`) | 0.0043 (`vasmed`) |
| Rectus femoris ratio, this code / OpenSim | 1.1000 / 1.1000 | 1.0790 / 1.0793 | 1.0949 / 1.0972 |
| Soleus ratio, this code / OpenSim | 1.1000 / 1.1000 | 1.0660 / 1.0660 | 0.9527 / 0.9527 |
| Body mass ratio, largest difference | 2e-16 | 2e-16 | 0 |
| Inertia ratio | same (1.6105) | differs, see below | same (1.7623, 0.7738) |

Differences, and why they stay:

- **Child joint offset.** OpenSim scales a joint's offset in the child body with the child's factors; here a child body's position, which includes that offset, scales with the parent's. They agree when the offset is zero or both factors are equal. This is the whole of the joint-centre differences above.
- **Inertia under per-axis factors.** OpenSim models a body as a cylinder along its smallest principal moment; here the mass distribution stretches. A 10 kg body with moments (0.1, 0.2, 0.25) kg m² and factors (1.2, 0.8, 1.1) gives (0.0929, 0.2746, 0.2746) in OpenSim and (0.0826, 0.2981, 0.3168) here; both have mass 10.56 kg. The test in `test_scaling.py` checks the rule against a directly built stretched box.
- **Muscle model.** These are the scaling ratios of two different muscle models, and they do not show that the models are equivalent or accurate for a measured person.

**Over a pose grid.** `tests/test_opensim_pose_comparison.py`, adapted from MyoSuite PR #564, follows six right-leg muscles over twelve hip, knee and ankle poses (`pose_fixture.json`):
- iliacus
- rectus femoris
- vastus medialis
- soleus
- medial gastrocnemius
- tibialis anterior

It also follows joint centres and sole points. Every muscle is at full activation, in each program's isometric state.

`pose_reference.json` stores OpenSim's half, unchanged from PR #564, and a myo_sim half that the test uses as a baseline within 1e-7 relative.

The table gives the largest scaling discrepancy over the grid, with its bound from `pose_tolerances.json` in brackets. The discrepancy is `(m_s - o_s) - s^k (m_n - o_n)` for one factor and `(m_s - m_n) - (o_s - o_n)` otherwise. Mass scaled with volume and mass kept at the nominal total give the same values. OpenSim's case with per-axis factors and kept mass has no counterpart here, because `total_mass` keeps no per-body mass distribution.

| Quantity (12 poses, 6 muscles) | 1.1 everywhere | Per-axis factors (PR #564's set) |
|---|---:|---:|
| Joint centres, sole points and floor offset | 6e-16 m (1 nm) | 0.357 mm (0.40 mm) |
| Moment arms | 2.4e-6 m (3.7e-6 m) | 0.74 mm (0.83 mm) |
| Normalised fibre length | 5.2e-6 (7.8e-6) | 0.0193 (0.022) |
| Normalised tendon force | 9.2e-6 (1.4e-5) | 0.0164 (0.0184) |
| Torque of the six muscles | 0.0074 N m (0.012 N m) | 3.19 N m (3.6 N m) |

The bounds follow PR #564's rule:
- Under one factor, about 1.5 times the largest value, with a 1 nm floor for positions.
- Under per-axis factors, 8 to 16 % above the largest value.

The values are the same on MuJoCo 3.4, 3.9 and 3.17.

Each bound rejects by at least 25 % the 5 % factor errors listed for it under `covers`. No cross-model bound sees the torso's factors, nor the z factors of the leg segments under per-axis factors; the self-regression catches those errors.

`scripts/validate_opensim_scaling.py --refresh-myo-sim` remeasures the myo_sim half after a deliberate change.

Effect of the refit on `myolegs`, as the 99th percentile over all muscles and 200 poses of the normalised length difference from the unscaled model, and the same for its worst muscle (R² of the fits in brackets):

| Factors | All muscles, `scale_spec` / `fit_length_ranges` | Worst muscle, `scale_spec` / `fit_length_ranges` | R², median and lowest |
|---|---:|---:|---:|
| Per-axis (PR #564's set) | 0.067 / 0.026 | 1.185 / 1.193 (`glmax3`, others 0.076 / 0.035) | 0.9996, 0.989 |
| Femur 1.12, tibia 0.95 | 0.083 / 0.015 | 0.101 / 0.026 | 1.000, 0.996 |
| 1.1 everywhere | 0.000 / 0.000 | 0.000 / 0.000 | 1.000, 1.000 |

## Limits

- Anisotropic factors on a rotated primitive can require shear, and spheres and capsules cannot follow them exactly. `scale_report` lists these geoms. Radii use OpenSim's arithmetic mean of the factors across the axis. Under such factors, a muscle's wrapping may change a little and a wrapped path may change branch; `check_scaled_model` shows the effect.
- A length ratio from a wrap's changed path is corrected, but the path itself still jumps in those poses. Example: with a pelvis 1 % wider along x, `glmax3` goes the long way around `Gmax3_at_pelvis_wrap` in the default pose, and on PR #564's per-axis set it fails both `operating_range` (1.19, after the refit too) and `moment_arm_signs` (on one of its three joints). Shoulder muscles with sphere wraps, such as `INFSP`, `PECM1` and `DELT2`, are sensitive in the same way under per-axis humerus factors. A similarity has no such jumps.
- A tendon whose path differs from the unscaled one in every sampled pose keeps its default-pose ratio, and the report names it. Most of these change smoothly, as a wrap geom engages or disengages (`LTpT` in some draws of factors), but on `myofullbody` a `DELT2` then keeps a ratio about 0.17 too high in 6 of 20 draws of independent factors per body in [0.9, 1.1].
- Hinge joint stiffness, damping and armature, angle springs, contact margins and control ranges are not scaled. The report lists angle springs and the actuators whose units would need it.
- Connect and weld equalities anchored on scaled bodies, welds with an explicit `relpose` whose bodies the scaling moves relative to each other (an unscaled child of a scaled body, or a free-floating subject that root alignment moves), meshes shared by bodies needing different scales, flexes, height fields, SDF and plugin meshes on scaled bodies, and specs with `fusestatic` are refused. Site-anchored equalities follow their sites.
- An attached spec compiles with its own angle unit, euler sequence and meshdir. From MuJoCo 3.6 scaling follows them, and refuses a scaled body whose attached spec takes its inertia from geoms; older versions refuse a spec whose orientations do not resolve as compiled.
- A scaled model is a different model. A policy trained on the original is not validated for it, and `model_digest` identifies a model, not its accuracy.
- Fitting a person's factors, inverse kinematics, retargeting, environments, skins and checkpoints are not part of this package. MyoSuite can call `scale_spec` from its model builders; `make_env(scales=...)` and IK stay there.

## Tests

`tests/test_scaling.py` runs on synthetic specs (nested rotated frames, every orientation type, a wrapping tendon, a coupled slide, a tied slide node, ligaments, an automatic peak force), `myolegs` and `myofullbody`. One factor must be an exact similarity: every site, spatial tendon and muscle length grows by `s`, muscle forces stay, mass grows by `s^3` and inertia by `s^5`. `tests/test_opensim_comparison.py` holds the first table of the comparison to its bounds, and each bound must fail on a known-bad factor.

`tests/test_opensim_pose_comparison.py` does three things for the pose grid:
- It holds the grid to its bounds.
- It checks each bound against the mutations it covers.
- It holds the myo_sim half to its stored baseline.

Its OpenSim test runs only with `OPENSIM_PYTHON` set.

## References

- Delp SL, Anderson FC, Arnold AS, Loan P, Habib A, John CT, Guendelman E, Thelen DG (2007). OpenSim: open-source software to create and analyze dynamic simulations of movement. IEEE Trans Biomed Eng 54(11):1940-1950. doi:10.1109/TBME.2007.901024
- Modenese L, Ceseracciu E, Reggiani M, Lloyd DG (2016). Estimation of musculotendon parameters for scaled and subject specific musculoskeletal models using an optimization technique. J Biomech 49(2):141-148. doi:10.1016/j.jbiomech.2015.11.006
- Rajagopal A, Dembia CL, DeMers MS, Delp DD, Hicks JL, Delp SL (2016). Full-body musculoskeletal model for muscle-driven simulation of human gait. IEEE Trans Biomed Eng 63(10):2068-2079. doi:10.1109/TBME.2016.2586891
