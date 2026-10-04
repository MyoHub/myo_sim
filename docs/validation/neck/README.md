# Neck validation summary

Recorded comparison for the opt-in `myohead` and `myofullbody_neck` models,
2026-10-03. Default `myofullbody` is unchanged. This is source verification and
limited experimental agreement, not independent validation of the whole body.
Datasets, export/calibration scripts and generated trajectory archives are not
retained in this repository or distributed in the Python package.

## Source and model adjustments

The scaled HYOID model is from Mortensen, Vasavada and Merryweather
([2018 paper](https://doi.org/10.1371/journal.pone.0199912)), using the
[MyoConverter source at cadf380](https://github.com/MyoHub/myoconverter/tree/cadf38059367a51239e6dc28c9fbe8b8fbd5149f/models/osim/Neck6D).
Input: `HYOID_1.2_ScaledStrenght_UpdatedInertia_adjusted.osim`, SHA-256
`908c37184bb31208506a5af6492a0d9885fe25478ea6a0d51ba6efb7ac2be8d6`.
The source MIT notice ships beside the model in
[`LICENSE.Neck6D`](../../../myo_sim/models/head/LICENSE.Neck6D).

The assets restore source joint-center offsets and rotational stiffness/damping;
joint springs are exact for sagittal motion and approximate the source XYZ Euler
bushings to first order for combined motion. The 36 right muscle paths are
mirrored into 72 actuators. Geniohyoid's negative-slack conversion fit is replaced
with source physical lengths and force. The chain has six independent coordinates
(`pitch/roll/yaw` at two cervical levels); MuJoCo realizes them as 24 hinges with
18 equality couplings on the auxiliaries. Other converted muscle parameters remain
unchanged.

OpenSim 4.6 reference measurements at 729 poses gave maximum canonical-right and
mirrored-left discrepancies of **0.0059 mm in length** and **0.0426 mm in signed
moment arm**. The original asymmetric source left differs by up to 2.6613 mm;
generated bilateral symmetry intentionally does not preserve those asymmetries.

## Neutral strength

Experimental male means and SDs are from Vasavada, Li and Delp
([2001](https://doi.org/10.1097/00007632-200109010-00018)), also tabulated in
Mortensen 2018, Table 3. A linear program maximized helmet load while balancing
all six cervical coordinates, including gravity and passive muscle forces,
with activation in [0,1] and no reserve actuators. The horizontal load is applied
20 cm above skull–C1 and reported using the published 32 cm C7–T1 lever arm;
axial rotation uses a pure moment. Other body coordinates are prescribed.

| Direction | Model (Nm) | Experiment mean ± SD (Nm) |
|---|---:|---:|
| Flexion | 29.40 | 30 ± 5 |
| Extension | 44.31 | 52 ± 11 |
| Lateral bending | 37.91 | 36 ± 8 |
| Axial rotation | 12.83 | 15 ± 4 |

All four lie within one SD. Standalone and integrated values agree within
10⁻⁸ Nm. These strength targets were already used to calibrate the HYOID source;
they are supporting agreement, not independent evidence.

## Dynamic literature comparison

Reference: Wochner, Nölle, Martynenko and Schmitt,
[“Falling heads” (2022)](https://doi.org/10.1186/s12938-022-00994-9).
Measured trajectories: [DARUS-1132 v1.1](https://doi.org/10.18419/DARUS-1132),
17 participants × three trials per direction at nominal 462 Hz.
Marker 2 supplies the head COM proxy; marker 1–2 orientation supplies unsigned
rotation. Three repeats are averaged before group mean/sample SD. Values are
zeroed at release frame 17, rather than the authors' first-frame baseline;
no additional filtering or differentiation is applied.

The comparison fixes the torso, disables contact, starts in neutral posture with
zero velocity/activation and releases support after 4 ms. The benchmark-only
controller estimates fiber length from source tendon compliance and pennation,
uses 25 ms delayed graded feedback (paper Eq. 13), and stimulates 48 anatomically
matched actuators. This observer does not add compliant tendons to the production
force law. A single gain **3.5** was selected as the lowest passing value in a
2–20 sweep with step 0.25 on odd participant IDs (n=9), then evaluated on even
IDs (n=8). **Evaluation is exploratory, not blinded:** earlier development
inspected the full cohort and informed controller design. No production muscle
forces were fitted to these trajectories, and no controller ships in the package.

Acceptance here means both displacement and rotation RMSE ≤ RMS participant SD
over **20–250 ms**. This is a descriptive convention, not a published tolerance
or confidence interval, and does not imply pointwise containment in the corridor.

| Comparison | Drop RMSE (cm) | Rotation RMSE (°) | Drop / SD RMS | Rotation / SD RMS |
|---|---:|---:|---:|---:|
| Supine, all 17 | 1.136 | 2.29 | 0.725 | 0.373 |
| Prone, all 17 | 0.620 | 2.64 | 0.421 | 0.589 |
| Supine, evaluation 8 | — | — | 0.609 | 0.327 |
| Prone, evaluation 8 | — | — | 0.609 | 0.950 |

All criteria passed. The initial binary transfer failed prone release
(1.857 cm / 7.90°); correcting passive mechanics, fiber estimation and muscle
selection and calibrating graded feedback reduced those errors. Evaluation prone
rotation remains close to the bound. At 0.125 ms timestep, timestep halving
changed motion by at most 0.014 mm / 0.0061°, coupling residual was below
0.000060 rad, and standalone/integrated motion agreed within 10⁻⁸ m/rad.

![Recorded experimental comparison](neck_dynamics.png)

## Limits and attribution

Later supine recovery beyond 250 ms is imperfect. Contact, trapdoor settling,
torso motion, individual anatomy, shoulder–neck interaction, whole-body balance,
EMG, acceleration, injury response and individual muscle force curves were not
validated. Production tendons are rigid; converted cervical inertias are balanced
isotropic values, not verified anatomical measurements. Cervical mesh segmentation
is heuristic. Opt-in body mass is 89.60644 kg versus the default 84.26499 kg.

The figure derives from Wochner, Nölle, Martynenko and Schmitt's 2022 trajectory
data, distributed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Controller muscle correspondence used their
[DARUS-1145 v1.1 neck parameters](https://doi.org/10.18419/DARUS-1145), also CC BY
4.0. Processing follows their [DARUS-2526 scripts](https://doi.org/10.18419/DARUS-2526)
with the baseline difference stated above. No author endorsement is implied.

The retained runtime tests (`tests/test_head_build.py`) check passive restoring
and damping forces, bilateral muscle parameters and preservation of existing
full-body landmarks and actuators; they do not re-run the literature benchmark.
