# Hand function validation (tenodesis, grip, pinch)

`scripts/validate_hand_function.py` reruns three functional tests of McFarland et al. 2023
(IEEE TBME 70(5):1424-1435, doi:10.1109/TBME.2022.3217722) on the right hand of `myoarm_r`:
passive tenodesis, maximum power grip and lateral pinch. Postures, targets, tolerances and their
justification are in `build/data/hand_function.json`, read with
`myo_sim.build.hand_function.load_hand_function_table`. The paper's numbers are results of its ARMS
model, which is non-commercial and is cited only: none of its files, parameters or coordinates are
used. The scenes are composed through MjSpec (locks, dynamometer, contact pairs); the protocols change
no model XML.

```bash
uv run python scripts/validate_hand_function.py --out results.json   # all protocols, about 4 min
MYO_SIM_SLOW=1 uv run pytest tests/test_hand_grip.py                 # with the grip optimisation
```

The script uses whichever `myo_sim` is importable, so another model version runs with
`PYTHONPATH=<worktree>`; the table is read next to the script.

## Tests

- `tests/test_hand_grip.py` re-evaluates the cached grip optimum of the model it runs on. The table
  keeps one optimum per model, matched by a fingerprint read from the model (passive spring tendons
  present, index MCP damping, EDC3 `fpmax`), because the activation levels that balance the wrist of
  one model leave a lock moment on another. Without a matching entry the tests skip.
- `tests/test_hand_pinch.py` needs the thenar muscles (AdP, FPB, APB) and skips without them.
- `tests/test_hand_tenodesis.py` needs the fitted extrinsic muscles of `build/data/muscle_passive.json`
  and joint springs fitted on top of them. Both are read from the data: the tabled `fpmax` in the model,
  and the muscle baseline each spring row recorded at its fit equal to the current model's. Otherwise
  the tests skip.

## Method

**Common.** Shoulder neutral, elbow 90 deg, forearm neutral. Locked joints are joint equalities to
a constant (`simulation.lock`, timeconst 2 x timestep); the prescribed wrist is the same equality with
its constant moved every step. Collisions are off except the explicit dynamometer pairs. Angles in the
table are anatomical; the script maps them with a sign per joint (MyoHand's `cmc_flexion`,
`mp_flexion` and `ip_flexion` are positive in extension, read from the muscle actions).

**Tenodesis.** All muscles at act = 0. Free: MCP, PIP and DIP flexion of digits 2-5; MCP abduction and
the thumb locked at 0. Trial A holds the wrist at 60 deg flexion for 1 s and extends it to 60 deg at
20 deg/s; trial B (release) continues from A: 1 s at 60 deg extension, then to 60 deg flexion. Reported
per digit: range of motion over the sweep, monotonicity, net motion and the largest instantaneous
spread between digits.

**Grip.** A massless elliptical prism (48 mm palmar span, Jamar setting II; 32 mm along the digit,
assumed) in the third-metacarpal body, its axis through the MCP 2 and MCP 5 joint centres, touching the
palm, its distal face one proximal-phalanx skin radius proximal of MCP 3. Contact pairs to every
phalanx collision geom, friction 0.29 as in the paper. The grip is the contact force of the proximal and
middle phalanges perpendicular to the axis. The thumb is locked with its intrinsics at 0; the finger
intrinsics are at 1. Each evaluation starts from the hand closed on the dynamometer and simulates
0.15 s (force averaged over the last 0.05 s). Free levels as in the paper: wrist muscles, extrinsic
thumb, EIP, EDM, and one level each for FDS, FDP and EDC. The wrist is held; the paper's penalty on
wrist motion is a penalty on the moment the wrist lock supplies. The ten wrist and extrinsic thumb
levels come from a linear program, the other five from bounded Powell with five starts. "Held" runs
drop the penalty.

**Settled grip.** Next to every optimum the script reports the grip at the same levels after 2 s
from the same closed hand (`grip.settled` in the table; force and digit shares averaged over the
last 0.05 s), with the change of the grip between 1 and 2 s as a check on how settled it is. It is
reported only: the optimisation, the tolerances and `tests/test_hand_grip.py` use the paper's 0.15 s.

**Lateral pinch.** Wrist 0/0, fingers locked. Thumb: CMC 15 deg extension and 20 deg adduction, MP
20 deg and IP 40 deg flexion. Static on the four thumb DOFs: the tip presses on a sensor, and the tip
force along the palmar normal of the distal phalanx is maximised with the off-axis part at most 17 %,
over FPL, EPL, EPB, APL, OP, AdP, FPB and APB. MuJoCo's own forces with the reaction applied at the tip
confirm the equilibrium.

## Results

Trial A unless marked B. "Integration" is MyoHub/myo_sim#123 (anatomical muscle parameters) plus
Vittorio-Caggiano/myo_sim#2 (extrinsic slack lengths and `fpmax`, with the release step) plus
MyoHub/myo_sim#139 (springs, thenar muscles, RI2 path, finger damping), with the springs refitted on #2's
muscles. #2 alone carries #123.

| Quantity | Paper (ARMS) | Integration | #2 alone | #139 alone | dev |
| --- | --- | --- | --- | --- | --- |
| MCP ROM 2 / 3 / 4 / 5 (deg) | mean 65.2 | 77 / 78 / 73 / 51 | 0 / 62 / 32 / 84 | 37 / 33 / 29 / 31 | 0 / 11 / 0 / 0 |
| PIP ROM mean (deg) | 16.1 | 28.2 | 50.3 | 0.2 | 4.3 |
| DIP ROM max (deg) | constant | 0.2 | 44.9 | 0.0 | 3.5 |
| Index gap MCP / PIP (deg) | 49.2 / 14.6 | 67 / 77 | 90 / 35 | 11 / 0 | 11 / 6 |
| Release (B): MCP extension 2 / 3 / 4 / 5 (deg) | digits extend | 78 / 78 / 74 / 27 | 0 / 70 / 31 / 30 | -3 / -3 / -3 / -3 | 0 / 2 / 1 / 0 |
| Tenodesis checks failed | | none | MCP 2, PIP, DIP, spread, release | MCP 4, PIP, release | MCP 2-5, PIP, release |
| Grip, self-selected (N) | 436.4 (exp. 402.1 +- 131.4) | 294 | 374 | 295 | 297 |
| Shares 2 / 3 / 4 / 5 (%) | 23.3 / 42.2 / 23.0 / 11.5 | 30 / 37 / 25 / 8 | 30 / 38 / 20 / 12 | 52 / 36 / 6 / 6 | 48 / 34 / 12 / 6 |
| Grip, neutral / flexed wrist | -41 % / -46 % | -27 % / -65 % | -44 % / -74 % | -26 % / -66 % | -22 % / -65 % |
| Grip, wrist held (N) | | 356 | 395 | 315 | 309 |
| Settled grip, self-selected (N); change 1-2 s | | 259; -6 % | 301; +3 % | 300; +2 % | 261; +0 % |
| Settled shares 2 / 3 / 4 / 5 (%) | | 37 / 42 / 12 / 9 | 47 / 35 / 4 / 15 | 52 / 35 / 6 / 7 | 54 / 38 / 0 / 8 |
| Settled grip, neutral / flexed wrist | | -18 % / -61 % | -45 % / -67 % | -36 % / -67 % | -33 % / -64 % |
| Settled grip, wrist held (N) | | 337 | 301 | 322 | 323 |
| Lateral pinch (N) | 66.3 (in vivo 51.9 +- 20.4) | 38.7 | none | 39.5 | none |

"none" for the pinch: no thumb force meets the 17 % off-axis limit without AdP, FPB and APB (best palmar
force 10-11 N at off-axis 1.5). No model reaches the normative pinch of 96-116 N; it is not tested.

Each change of this round on the integration, in order (springs refitted after every change of the
muscles):

| Quantity | Before | + RI2 path | + finger damping | + extensor release |
| --- | --- | --- | --- | --- |
| MCP ROM 2 / 3 / 4 / 5 (deg) | 7 / 71 / 48 / 72 | 69 / 71 / 48 / 72 | 77 / 63 / 61 / 49 | 77 / 78 / 73 / 51 |
| PIP ROM mean / DIP max (deg) | 30.1 / 0.2 | 26.8 / 0.2 | 26.7 / 0.2 | 28.2 / 0.2 |
| Index gap MCP / PIP (deg) | 77 / 41 | 51 / 55 | 71 / 77 | 67 / 77 |
| Release (B): MCP extension 2 / 3 / 4 / 5 (deg) | 7 / 17 / 4 / 17 | 76 / 17 / 4 / 17 | 78 / 26 / 22 / 18 | 78 / 78 / 74 / 27 |
| Tenodesis checks failed | MCP 2 | none | none | none |
| Grip self-selected (N); shares (%) | 375; 32 / 38 / 22 / 9 | 360; 31 / 38 / 22 / 9 | 298; 42 / 25 / 16 / 17 | 294; 30 / 37 / 25 / 8 |
| Grip neutral / flexed; held (N) | -40 % / -72 %; 384 | -38 % / -71 %; 393 | -28 % / -66 %; 353 | -27 % / -65 %; 356 |
| Lateral pinch (N) | 38.7 | 38.7 | 38.7 | 38.7 |

Tests that pass on the integration: all three files, including the index MCP range of motion and both
negative controls of the tenodesis tolerances (springs removed; the extrinsic slack lengths and `fpmax`
of #123). The grip passes on #2 alone and on the integration with their own cached optima; the pinch
passes wherever the thenar muscles are present.

## Known issues

- **The 0.15 s grip is not settled.** At fixed activation levels the grip keeps changing well after
  0.15 s, in every model, so the script reports the settled grip next to it (rows "Settled" above, each
  at the model's own 0.15 s optimum). Self-selected, the grip settles at 259-301 N with digit 4 at 0-12 %
  of the grip, mostly below the share band; the integration drops from 294 to 259 N and #2 alone from
  374 to 301 N. Some optima are still moving at 2 s (the neutral-wrist optima of #2 alone and dev by
  13-19 % between 1 and 2 s), and the levels balanced for the 0.15 s state leave up to 0.9 N m on the
  wrist lock once settled. The finger damping changes the transient, not the settled force: at #2's
  levels the integration with and without it converges to the same 297 N. The test and the tolerances
  keep the paper's 0.15 s evaluation; optimising the settled grip (about 2 s per evaluation, over ten
  times the cost) would move every grip result.
- **Little-finger release.** On the integration the digit-5 MCP extends 27 deg in trial B (30 deg on #2
  alone, where the release bound was found): the digit-5 joint springs of the integration hold it back.
- **Active force-length windows.** With #123's per-muscle windows, FDS4 and FDP5 (`lmin` 0.95) lose all
  active force below 0.95 L0: FDS4 has none in the flexed-wrist grip, part of why that grip drops about
  65 % against the paper's 46 %. Widening these windows waits for compliant tendons, which change the
  operating ranges themselves.
- **Grip geometry.** The grip depends on the dynamometer placement much more than on the contact
  parameters: with FDS and FDP at full activation the self-selected grip was 391, 1101, 335 and 45 N with
  the dynamometer's distal face 16 mm distal of, at, 9 mm proximal of (the table) and 19 mm proximal of
  MCP 3. The placement rule keeps the contacts along the phalanges.
- **Pinch posture.** The CMC angles are measured from each model's own zero. Flipping the flexion sign
  leaves no feasible force; 20 deg abduction instead of adduction gives 68.8 N; the zero posture gives
  60.1 N. FPB and APB are at full activation at the optimum.
