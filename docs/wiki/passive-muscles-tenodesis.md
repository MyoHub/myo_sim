# Passive muscle tension and tenodesis

The tendon slack lengths and passive-force scales (`fpmax`) of the extrinsic finger, thumb
and wrist muscles are fitted to measured passive moments. The fingers flex when the wrist
extends and open when it flexes (tenodesis), no muscle gives much more passive moment than
the measured total, and the flexors keep at least half their force in a full fist (the
extensors in full extension). The fitted values are the `lengthrange` and `fpmax` of these
muscles in `myoarm_r_muscle.xml`, so every composed model with a hand (`myofullbody`,
`myotorso_arms`, `myoarms`, `myohands`, `myoarm_r`, ...) has them on both arms (the left
arm is mirrored). The previous values (`reference_value_m`, `reference_value` in the
table) are those of the previous release.

## Method

Binder-Markey & Murray (2017, J Biomech 61:250) fitted the tendon slack lengths of
FDSI, FDPI, EDCI and EIP so that the muscles' passive force reproduces the extrinsic
component of the index MCP passive moment of Knutson et al. (2000, J Biomech 33:1675;
Eqs. 3-4, Table 1 medians) over wrist -60..60 deg, with a penalty on relative change
(their Eqs. 2-3). `scripts/fit_muscle_passive.py` does the same for MyoHand with
MuJoCo's passive force-length curve and MyoHand's muscle-tendon lengths and moment arms.
MyoHand tendons are rigid: the implied slack length is
`lengthrange[0] - range[0] * L0`, so a new slack length shifts `lengthrange` at constant
optimal length. MyoHand's MCP has no hyperextension, so the fit covers MCP 0..90 deg
(the sources: -45..90 deg).

MuJoCo's passive curve is about five times stiffer at 1.3 L0 than the one Binder-Markey
used, and rigid tendons put the whole muscle-tendon excursion into the fibres. Lengthening
the slack lengths alone (the first version of this fit) removed the excess passive moment,
but it left the finger flexors at or below `lmin` in a fist, with almost no active force.
Each muscle therefore also gets its own `fpmax`, which scales the passive force only (the
active force is unchanged), with a mild penalty on `|ln(fpmax / 1.3)|` (1 N cm^2 per e-fold)
and bounds 0.002..2.6.

Hard constraint (`active_constraint` in the table): MuJoCo's active force-length factor is
at least 0.5 (half the isometric force) for every finger and thumb flexor in a full fist
(every finger and thumb joint at its MyoHand flexion limit) with the wrist neutral and
flexed 30 deg, and for every extensor in full extension (the same joints at their
extension limits) with the wrist extended 30 deg. On MuJoCo's curve the factor is 0.5 at
`(lmin + 1) / 2` and `(1 + lmax) / 2`, so the constraint is a window on each slack length.
A slack length is shortened only as far as its window forces it: shortening otherwise
only fills deficits the joint springs supply, and with `fpmax` free that direction is flat.

No extrinsic-only data exist beyond the index, so the other extrinsic muscles (FDS3-5,
FDP3-5, EDC3-5, EDM, FPL, EPL, EPB, APL, ECRB, ECU) are fitted, with the same penalties, to
the **total** passive moment of each joint they cross (`joint_targets` in the table):

- wrist flexion and deviation: Formica et al. (2012), in their posture (hand closed around
  the handle, elbow 60 deg; the grip angles are assumed: MCP 45, PIP 60, DIP 30 deg,
  thumb MP / IP 20 deg flexion);
- MCP of digits 3-5: Knutson's intrinsic plus extrinsic moment of the index, over the same
  MCP / wrist grid (transferred target);
- PIP / DIP of digits 3-5: Kamper et al. (2002) index curves (transferred target, as
  McFarland et al. 2023 did with scaled curves);
- thumb MP / IP: the index MCP / PIP curves of Kamper (Barry et al. 2018; transferred target).

The loss is one-sided: a muscle moment beyond the total, or against its sign, costs 100
times a deficit of the same size. A deficit is what the joint structures supply (joint
springs fitted to the residual); an excess could only be removed by an element that pushes
the joint away from its passive equilibrium. Slack changes are bounded by 10 %.

## Result (on the anatomical muscle parameters of `fix/anatomical-muscle-parameters`)

| Muscle | Slack | fpmax | Muscle | Slack | fpmax |
| --- | --- | --- | --- | --- | --- |
| FDS2 | -0.69 % | 0.0305 | EDC2 | -2.18 % | 0.518 |
| FDP2 | -5.37 % | 0.002 | EIP | -1.52 % | 0.989 |
| FDS3 | -7.42 % | 0.002 | EDC3 | -2.71 % | 0.0525 (release) |
| FDS4 | -6.84 % | 0.0393 | EDC4 | -1.37 % | 0.0525 (release) |
| FDS5 | -0.29 % | 0.002 | EDC5 | +1.57 % | 0.0525 (release) |
| FDP3 | -7.62 % | 0.002 | EDM | -0.39 % | 0.0525 (release) |
| FDP4 | 0 | 0.002 | EPL | -0.17 % | 0.002 |
| FDP5 | -5.51 % | 0.0679 | EPB | -1.76 % | 0.405 |
| FPL | 0 | 0.0626 | APL | -0.32 % | 1.3 |
| ECRB | +2.37 % | 1.3 | ECU | +3.35 % | 1.766 |

Every negative slack change is forced by the active constraint (the anatomical slack
lengths already violate it for FDS3, FDS4, FDP2, FDP3, FDP5 and EDC2-4). Six muscles sit at
the `fpmax` floor (0.002), and EDC3-5 and EDM did before the release step (below): with the
shortened slack lengths and rigid tendons they reach 1.6-2 L0 in extension, beyond `lmax`,
where MuJoCo's passive curve keeps growing linearly.

| Joint (all muscles, act = 0) | Target peak (N m) | Max excess: anatomical / slack-only fit / fit before the release step (N m) |
| --- | --- | --- |
| Wrist flexion (grip) | 0.30 | 0.76 / 0.08 / 0.08 |
| Wrist deviation (grip) | 0.51 | 2.58 / 0.02 / 0.03 |
| MCP 3 / 4 / 5 | 0.34 | 3.50 / 2.01 / 1.20 -> 0.00 / 0.01 / 0.00 -> 0.02 / 0.03 / 0.05 |
| PIP 3 / 4 / 5 | 0.05 | 0.06 / 1.25 / 0.40 -> 0 / 0 / 0.13 -> 0.01 / 0.01 / 0.11 |
| DIP 3 / 4 / 5 | 0.08 | 0.01 / 0.68 / 0.13 -> 0 -> 0.00 / 0.00 / 0.02 |
| Thumb MP / IP | 0.06 / 0.05 | 0.98 / 1.18 -> 0 / 0.002 -> 0.02 / 0.03 |

The index fit to Knutson's extrinsic moment has an RMSE of 3.4 N cm (98.4 with the
anatomical values, 3.0 for the slack-only fit). Remaining excesses: EDC2 (fpmax 0.52, its
slack forced 2.2 % shorter) and EIP give 0.08 N m of wrist extension at neutral in the
assumed grip; FDP5 gives 0.05 N m at MCP5 with the wrist extended; the digit-5 intrinsics
LU_RB5 and UI_UB5 give 0.11 N m of PIP extension at full flexion (not fitted).

## Release step

With EDC3-5 and EDM at the `fpmax` floor nothing but joint springs opened digits 3-5 when
the wrist flexed: in the release trial of McFarland et al. 2023 (tenodesis trial B: wrist
from 60 deg extension to 60 deg flexion at 20 deg/s, muscles at act = 0, run by
`scripts/validate_hand_function.py`) their MCPs moved 2, -12 and 1 deg. A last step keeps every
slack length and refits only these four `fpmax`. Their lower bound is the smallest common value
at which each MCP of digits 3-5 extends at least 30 deg in that trial, the lower edge of the
protocol's MCP range band mirrored; it is found by bisection, and above it the same one-sided
joint loss sets the values. All four come out at the bound, 0.0525, and the MCPs extend 70, 31
and 30 deg.

The price is extension moment beyond the measured totals where these muscles are stretched:
the largest wrist-flexion excess in Formica's grip posture rises from 0.080 to 0.159 N m
(target peak 0.303 N m), the wrist-deviation excess falls from 0.030 to 0.014 N m, and the
finger and thumb rows do not change. Each row records its excess without the step
(`fit.max_excess_Nm_without_release`); `release_constraint` records the bound and the trial
results. With the joint springs of MyoHub/myo_sim#139 fitted on top, the integrated hand
releases digits 2-5 by 78, 78, 74 and 27 deg.

The joint fit records the muscles it counted besides the fitted ones
(`joint_fit.other_muscles`), so muscles added later, such as the thenar muscles, leave the
recorded errors unchanged; the joint springs are fitted to the residual of all muscles.
`--release-only` reruns only this step.

Active force-length factor (fist: every joint at its flexion limit):

| Muscle | Fist, wrist 0 | Fist, wrist +30 | Power grip, wrist -30 | Flat hand, wrist 0 |
| --- | --- | --- | --- | --- |
| FDS2 / FDP2 | 0.86 / 0.81 | 0.50 / 0.50 | 0.94 / 0.57 | 0.76 / 0.00 |
| FDS3 / FDP3 | 0.85 / 0.76 | 0.50 / 0.50 | 0.55 / 0.57 | 0.00 / 0.00 |
| FDS4 / FDP4 | 0.98 / 0.96 | 0.50 / 0.74 | 0.90 / 0.75 | 0.72 / 0.29 |
| FDS5 / FDP5 | 0.90 / 0.99 | 0.50 / 0.50 | 0.98 / 0.83 | 0.47 / 0.69 |
| FPL | 0.90 | 0.64 | 0.69 | 0.69 |

The extensors are at 0.50 or more in full extension with the wrist extended 30 deg. With
#123's force-length widths (`(lmax - lmin) / 2`, 0.42-0.55 L0 for these flexors) the fibre
excursion from the wrist-flexed fist to the flat hand (0.55-0.67 L0) does not fit in one
window for FDS3, FDS5, FDP2, FDP3 and FDP4, so these flexors lose force with the hand flat
(FDS3, FDP2 and FDP3: none). In the same way EDC3, EPL and EPB have less than half their
force in a fist. Compliant tendons would change these operating ranges, so the windows and
the slack-length fit are left as they are until the tendons are compliant.

Model drift: dev's MyoHub/myo_sim#135 (after #123 branched) moved the EDC torus wraps. On
dev with #123 and this fit, EDC3's factor in full extension with the wrist extended 30 deg is
0.498 instead of 0.5, and the wrist row's recorded error differs by 2e-4 N m; with the two wrap
positions of #123's base both match. The fit is rerun once #123 is on current dev.

Relaxed hand on `myofullbody` (no gravity, no excitation, wrist held): the fingers rest
partly flexed at wrist neutral (MCP 34-74, PIP 18-58 deg), flex further with the wrist
extended 60 deg (MCP 62-67 deg; the little-finger MCP and PIP reach their flexion limits,
the PIP within 6 s) and extend to their 0 deg limits with the wrist flexed 60 deg. The
index PIP is 41.7 deg more flexed with the wrist extended 60 deg than flexed 60 deg; the
test asks for 10 deg. The thumb rests inside its range at all three wrist angles.

## Data

`build/data/muscle_passive.json` is the provenance and fitting input: absolute slack
lengths (m) and `fpmax` with the values they change and the percent change, the slack
window allowed by the active constraint, tagged `scales_with: length` (reference segment:
forearm) for personalisation pipelines; `joint_targets` holds the source curves, postures
and per-row errors of the joint fit, `active_constraint` the postures and muscles of the
constraint, and each muscle row the rows it is constrained by and its status
(`transferred target` when a curve measured on another digit constrains it).
`scripts/fit_muscle_passive.py` refits from the reference values, rewrites the table and
writes the `lengthrange` and `fpmax` attributes into `myoarm_r_muscle.xml`; `--release-only`
reruns only the release step; `--xml-only`
writes the tabled values without refitting. `tests/test_muscle_passive.py` checks that the
XML holds the tabled values on both arms, the recorded errors, the active constraint with
MuJoCo's own force-length function, the release step, tenodesis and the relaxed hand.
