# Fourth and fifth carpometacarpal joints

The ring and little metacarpals (`fourthmc_r`, `fifthmc_r`) were fixed to the capitate, so the palm could
not cup. Since step S3 of the hand rebuild each has a flexion hinge, and the two are coupled. Numbers
against the literature: `hand-literature-comparison.md` (S3).

| What | Where | Written by |
| --- | --- | --- |
| `cmc4_flexion_r`, `cmc5_flexion_r` (pos, axis, range, damping) and the moved sites | `myoarm_r_chain.xml` | `scripts/fit_cmc_joints.py` from `build/data/cmc_joints.json` |
| Coupling `cmc45_coupling_r` | generated block of `myoarm_r_assets.xml` | same script |
| End-range springs (`passive_digit{4,5}_cmc_flexion_*`) | generated block of `myoarm_r_tendon.xml` | `scripts/fit_passive_moments.py` from `passive_joint_moments.json` |

The left hand is mirrored; the hand models keep the joints and the equality through the pruning.

## Joints

- **Centre:** a least-squares sphere through the proximal 15 % of each metacarpal mesh (Buffi et al. 2013's
  method; El-Shennawy et al. 2001 place the flexion-extension axis in the metacarpal base). Radii 5.2 and
  5.5 mm.
- **Axis:** CMC5 is the mean of Shimawaki et al. 2019's three grasp axes (Table 2, n = 9, in vivo CT), an
  oblique axis, so flexion carries some supination. CMC4 uses the CMC5 pattern: Shimawaki's CMC4 mean
  with its dorsal-palmar tilt replaced by CMC5's, because that tilt is not robust in the paper (an
  inconsistent X sign between methods and figure caption, rotations of only 4.4 ± 1.4 deg, per-subject
  angles that differ from the mean vector). Positive angles flex (the metacarpal head moves palmarly).
- **Range:** CMC4 -5 to 16 deg, CMC5 -10.3 to 33 deg (2.06 times CMC4's, so the limits never fight the
  coupling). The arcs (21 and 43 deg) match El-Shennawy's passive arcs (20 ± 10 and 44 ± 8 deg, n = 5). The
  split into flexion and extension is assumed: El-Shennawy reports arcs only, and Aljasim et al. 2025 find
  more extension than flexion from the MC3 axis under a 100 N load. The active maxima of Büren et al. 2020
  and Al Maktary 2019 (25.1 ± 3.5 and 12.2 ± 3.0 deg) lie inside.
- **Damping:** as the finger MCP joints (Kamper et al. 2002), an assumption.

## Coupling and its caveats

`cmc5_flexion = 2.06 x cmc4_flexion` (joint equality). The ratio is the mean active maximum of CMC5
(25.1 deg, Büren et al. 2020, ultrasound, 44 hands) over that of CMC4 (12.2 deg, Al Maktary 2019, the same
hands), both relative to MC3, in maximal little-finger opposition. Shimawaki et al. 2019 give 2.05 in grasp
(n = 9); Buffi et al. 2013's single subject 2.72.

- **Inter-subject variability.** The ratio is a ratio of means. Individual ratios are not reported; the
  SDs alone are 14 % (CMC5) and 25 % (CMC4) of the means, so the ratio of a given hand can differ widely.
  It is a personalisation parameter (`cmc_joints.json`, `coupling.personalisable`).
- **Task.** It was measured in active opposition. Under passive loading (an object pressing on the palm)
  the ratio may differ, and the equality imposes the active ratio there too. El-Shennawy's passive arcs
  bound a passive end-range ratio at about 2.2-2.75.
- **Independence.** The equality ties CMC4 to every CMC5 motion. With CMC4 pinned, CMC5 still moves 28 ± 3
  deg in El-Shennawy's cadavers, which the equality reduces to 0. Two free joints with an intermetacarpal
  spring or ligament replace it once stiffness data exist.

## Paths across the new joints

Moving MC4 and MC5 showed five sites on the wrong bone. Each now sits on the bone it belongs to, at the
same world position at q = 0, so every path is unchanged with CMC4/5 at 0 (checked to 1e-16 m):

| Site | From | To | Why |
| --- | --- | --- | --- |
| `FDS5-P4_r`, `FDP5-P4_r` | MC4 | MC5 | The digit-5 flexors would flex CMC4 (FDP5 16.9 mm) and FDP5 extend CMC5 (10.7 mm); on MC5 they flex CMC5 (9.3 and 6.4 mm). On the hamate the point lies distal of the CMC5 centre, and FDP5 would still extend CMC5. |
| `RI4-P1_r`, `UI_UB4-P1_r` (PI2, DI4 since S5) | MC3 | MC4 | These ring-finger interossei arise from MC4 (and MC5) and must not cross CMC4. |
| `RI5-P1_r` (PI3 since S5) | MC3 | MC5 | This little-finger interosseous arises from MC5; from MC3 it extended CMC5 (7.1 mm). |

`tests/test_cmc_joints.py` checks the sign of every crossing (FDS4, FDP4, LUM3 flex and EDC4 extends
CMC4; FDS5, FDP5, FCU, ADM, FDM, ODM flex and EDC5, EDM, ECU extend CMC5) and that the moved interossei cross
neither joint, nor does LUM4, whose origin moved next to FDP5 on MC5 in S5. Since S5 the three interosseous origins
are start points of the intrinsic table (`hand_intrinsic_muscles.json`), not site moves of `cmc_joints.json`.

## Passive moments

No moment-angle curve of these joints is published. The end-range load of El-Shennawy et al. 2001 (22 N on
a pin through the metacarpal, about 0.96 N m on MyoHand's 43.8 mm CMC5-to-MCP5 lever) defines an `assumed`
row per joint: a linear stiffness reaching that moment at the flexion and extension limits, fitted to the
residual over the muscles' passive moment like every other row. Without it the muscles' passive tension
(mostly FDP5) cupped the relaxed hand to 10 / 21 deg; with it the hand rests within 0.2 deg of neutral.
Aljasim et al. 2025 (100 N at the MC5 head, about 4.4 N m) is a stiffer second point for a later fit.

**Not checked against data:** the hinge location. El-Shennawy's normalised distance from the hamate to the
CMC5 axis (0.60 with CMC4 free) is larger than the model's (0.35 with its axis), so the hinge passes 2-6 mm
nearer the hamate; with n = 5 and no SD this is reported, not fitted.
