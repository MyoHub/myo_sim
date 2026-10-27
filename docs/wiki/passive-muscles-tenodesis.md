# Passive muscle tension and tenodesis

The tendon slack lengths of the extrinsic finger, thumb and wrist muscles are fitted to
measured passive moments, so the fingers flex when the wrist extends and open when it
flexes (tenodesis), and no muscle gives more passive moment than the measured total. The
fitted values are the `lengthrange` of these muscles in `myoarm_r_muscle.xml`, so every
composed model with a hand (`myofullbody`, `myotorso_arms`, `myoarms`, `myohands`,
`myoarm_r`, ...) has them on both arms (the left arm is mirrored). The previous slack
lengths (`reference_value_m` in the table) are those of the previous release.

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

No extrinsic-only data exist beyond the index, so the other extrinsic muscles (FDS3-5,
FDP3-5, EDC3-5, EDM, FPL, EPL, EPB, APL, ECRB, ECU) are fitted, with the same penalty, to
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
the joint away from its passive equilibrium. The changes are bounded by 10 % (Binder-Markey's
range). The index fit is unchanged and fixed while the others are fitted.

## Result (on the anatomical muscle parameters of `fix/anatomical-muscle-parameters`)

| Muscle | Slack before (m) | Slack after (m) | Change | Binder-Markey 2017 (OpenSim) |
| --- | --- | --- | --- | --- |
| FDS2 | 0.2750 | 0.3014 | +9.62 % | 0.275 -> 0.2772 |
| FDP2 | 0.3015 | 0.3169 | +5.09 % | 0.3015 -> 0.3044 |
| EDC2 | 0.3650 | 0.3637 | -0.36 % | 0.365 -> 0.3486 |
| EIP | 0.2100 | 0.2054 | -2.19 % | 0.21 -> 0.1911 |

With the starting values (MoBL-ARMS / Saul et al. 2015, also Binder-Markey's initial
values) MyoHand's rigid tendons give up to 327 N cm of extrinsic passive moment at the
index MCP (Knutson: at most 30 N cm; RMSE 98.4 N cm); after the fit the RMSE is 3.0 N cm
(max 9.5 N cm, at MCP 90 deg with the wrist flexed). The changes go the same way as
Binder-Markey's (flexors longer, extensors shorter); the flexor changes are larger because
MyoHand's tendons are rigid.

| Muscle | Change | Muscle | Change | Muscle | Change | Muscle | Change |
| --- | --- | --- | --- | --- | --- | --- | --- |
| FDS3 | +3.52 % | FDP3 | +3.15 % | EDC3 | +1.20 % | FPL | +6.56 % |
| FDS4 | +5.56 % | FDP4 | +10.00 % (bound) | EDC4 | +2.65 % | EPL | +6.79 % |
| FDS5 | +6.61 % | FDP5 | +7.61 % | EDC5 | +4.25 % | EPB | +1.31 % |
| ECRB | +2.37 % | ECU | +3.27 % | EDM | +1.23 % | APL | 0 |

| Joint (all muscles, act = 0) | Target peak (N m) | Max excess before -> after (N m) |
| --- | --- | --- |
| Wrist flexion (grip) | 0.30 | 0.90 -> 0.08 |
| Wrist deviation (grip) | 0.51 | 2.55 -> 0.02 |
| MCP 3 / 4 / 5 | 0.34 | 3.50 / 2.01 / 1.20 -> <= 0.01 |
| PIP 3 / 4 / 5 | 0.05 | 0.06 / 1.25 / 0.40 -> 0 / 0 / 0.13 |
| DIP 3 / 4 / 5 | 0.08 | 0.01 / 0.68 / 0.13 -> 0 |
| Thumb MP / IP | 0.06 / 0.05 | 0.98 / 1.18 -> 0 / 0.002 |

Before the fit, FDP4, FPL and FDP5 (normalised length 1.37, 1.22 and 1.31 at neutral) and
FDS3 / FDP3 (with the wrist extended) dominated; with the fingers straight the muscles gave
3.2 N m of wrist flexion at neutral (Formica: 0.3 N m peak). The two remaining excesses are
not from the fitted muscles: at the wrist, EIP (index fit) stretched by the assumed grip
gives 0.08 N m of extension at neutral (EIP +4 % would remove it, at a Knutson RMSE of
4.7 instead of 3.0 N cm); at the little-finger PIP, LU_RB5 and UI_UB5 give 0.13 N m of
extension at full flexion. FDP4 sits at the 10 % bound (10.1 % without it, with a
negligible gain).

Tenodesis on `myofullbody` (no gravity, no excitation, wrist held): with the wrist
extended 60 deg the index MCP / PIP / DIP settle at about 17 / 16 / 10 deg flexion and the
MCPs of digits 3-5 at 13 / 26 / 34 deg; at neutral and with the wrist flexed the fingers
extend to their 0 deg limits, and the thumb rests inside its range. With the reference
slack lengths digits 4 and 5 run into their flexion limits and the thumb IP and CMC into
theirs. Passive joint springs (if present) give intermediate rest postures.

Active force: the slack changes move the operating range on the force-length curve to
shorter normalised fibre lengths (by `shift / L0`: 0.32 and 0.20 for FDS2 and FDP2, 0.14 /
0.25 / 0.44 for FDS3-5, 0.11 / 0.37 / 0.29 for FDP3-5, 0.23 for FPL, 0.29 for EPL). In a
fist (MCP / PIP 90, DIP 60 deg, wrist neutral) the finger flexors then lie at or below
`lmin`, where MuJoCo's force-length curve gives little or no active force. FDS4 and FDP5
have `lmin` 0.95 in the anatomical parameters, so FDS4 gives no active force at neutral after
the fit.

## Data

`build/data/muscle_passive.json` is the provenance and fitting input: absolute slack
lengths (m) with the value they change (`reference_value_m`) and the percent change,
tagged `scales_with: length` (reference segment: forearm) for personalisation pipelines;
`joint_targets` holds the source curves, postures and per-row errors of the joint fit, and
each muscle row the rows it is constrained by and its status (`transferred target` when a
curve measured on another digit constrains it). `scripts/fit_muscle_passive.py` refits from
the reference values, rewrites the table and writes the `lengthrange` attributes into
`myoarm_r_muscle.xml`; `--xml-only` writes the tabled slack lengths without refitting.
`tests/test_muscle_passive.py` checks that the XML holds the tabled values, the recorded
errors, tenodesis and that the relaxed hand rests off its limits.
