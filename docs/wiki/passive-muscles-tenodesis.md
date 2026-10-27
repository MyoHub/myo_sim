# Passive muscle tension and tenodesis

The tendon slack lengths of the index finger's extrinsic muscles (FDS2, FDP2, EDC2, EI)
are fitted to measured passive moments, so the finger flexes when the wrist extends and
opens when it flexes (tenodesis). The fitted values are the `lengthrange` of these four
muscles in `myoarm_r_muscle.xml`, so every composed model with an index finger
(`myofullbody`, `myotorso_arms`, `myoarms`, `myohands`, `myoarm_r`, ...) has them on both
arms (the left arm is mirrored). The previous slack lengths (`reference_value_m` in the
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

## Result (on the anatomical muscle parameters of `fix/anatomical-muscle-parameters`)

| Muscle | Slack before (m) | Slack after (m) | Change | Binder-Markey 2017 (OpenSim) |
| --- | --- | --- | --- | --- |
| FDS2 | 0.2750 | 0.3014 | +9.62 % | 0.275 -> 0.2772 |
| FDP2 | 0.3015 | 0.3169 | +5.09 % | 0.3015 -> 0.3044 |
| EDC2 | 0.3650 | 0.3637 | -0.36 % | 0.365 -> 0.3486 |
| EIP | 0.2100 | 0.2054 | -2.19 % | 0.21 -> 0.1911 |

The starting values are the MoBL-ARMS / Saul et al. (2015) slack lengths, which are also
Binder-Markey's initial values. With them, MyoHand's rigid tendons give up to 327 N cm of
extrinsic passive moment at the index MCP (Knutson: at most 30 N cm; RMSE 98.4 N cm);
after the fit the RMSE is 3.0 N cm (max 9.5 N cm, at MCP 90 deg with the wrist flexed).
The changes go the same way as Binder-Markey's (flexors longer, extensors shorter); the
flexor changes are larger because MyoHand's tendons are rigid.

Tenodesis on `myofullbody` (no gravity, no excitation, wrist held): with the wrist
extended 60 deg the index MCP / PIP / DIP settle at about 17 / 16 / 10 deg flexion; at
neutral and with the wrist flexed they extend to their limits. With the reference slack
lengths, the same test runs the MCP into its 90 deg flexion limit with the wrist
extended. Passive joint springs (if present) give intermediate rest postures.

Active force: the slack changes move the operating range on the force-length curve (FDS2
and FDP2 operate at shorter normalised fibre lengths, by 0.32 and 0.20 L0 at a fixed
posture).

## Data

`build/data/muscle_passive.json` is the provenance and fitting input: absolute slack
lengths (m) with the value they change (`reference_value_m`) and the percent change,
tagged `scales_with: length` (reference segment: forearm) for personalisation pipelines.
`scripts/fit_muscle_passive.py` refits from the reference values, rewrites the table and
writes the four `lengthrange` attributes into `myoarm_r_muscle.xml`; `--xml-only` writes
the tabled slack lengths without refitting. `tests/test_muscle_passive.py` checks that the
XML holds the tabled values. Index finger only: no extrinsic passive data exist for the
other digits.
