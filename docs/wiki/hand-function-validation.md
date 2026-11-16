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
its constant moved every step. Collisions are off except the explicit dynamometer pairs: the scenes switch
geom collisions off and remove the model's own contact pairs (forearm-torso pairs and, since the hand
rebuild's S1, the hand self-contacts; `build_scene(..., keep_model_pairs=False)`). Angles in the
table are anatomical; the script maps them with a sign per joint (MyoHand's `cmc_flexion_r`,
`mp_flexion_r` and `ip_flexion_r` are positive in extension, read from the muscle actions).

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
last 0.05 s), with the change of the grip between 1 and 2 s as a check on how settled it is. The optimisation and the force
tolerances use the paper's 0.15 s.

**Finger shares.** `test_finger_shares_at_the_cached_optimum` compares the settled state with Kong et al. 2011, counting all three
phalanges. Kong's load cells take the whole finger and the subjects held for 4 s; Amis 1987 also sums all three phalanges and finds
the distal one carrying the most. The grip force stays McFarland's: proximal and middle phalanges, 0.15 s.

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

## Hand rebuild S1 (pads and self-contacts)

With the palmar fingertip pads and the palm pads of S1 (`hand-contacts-and-pads.md`), the dynamometer is
placed against the palm pad, 11 mm further palmar than against the metacarpal capsules. Fresh optimum
(before = S0 in the same scenes, which now remove the model's contact pairs):

| Quantity | S0 | S1 |
| --- | --- | --- |
| Grip, self-selected (N) | 288.5 | 232.5 (below the band, 270.7-533.5) |
| Shares 2 / 3 / 4 / 5 (%) | 45 / 28 / 12 / 15 | 21 / 41 / 27 / 12 |
| Grip, neutral / flexed wrist | -26 % / -64 % | -30 % / -68 % |
| Settled grip, self-selected (N) | 296 | 133 |
| Tenodesis, lateral pinch | unchanged | unchanged (no contacts in their scenes) |

The cached optimum of S1 (fingerprint `palm_pad_m`) fails `test_grip_at_the_cached_optimum`: the grip is
below the experimental mean − 1 SD. With the hand's self-contacts kept in the tenodesis scene, the
direction check fails (largest reversal 2.6 deg): with MCP abduction locked, digits 2-4 converge as they
flex and push each other.

## Hand rebuild S1b (PIP flexion range)

The PIP flexion limit of digits 2-5 rises from 90 to 115 deg (Leijnse et al. 2010) and the palm pads are
refitted to the fist at the new limits: palm pad 8.0 mm (was 19.1 mm), hypothenar pad 16.8 mm. The
dynamometer, placed against the palm pad, sits 11 mm closer to the metacarpals again. Fresh optimum:

| Quantity | S1 | S1b |
| --- | --- | --- |
| Grip, self-selected (N) | 232.5 (below the band) | 310.3 (inside the band, 270.7-533.5) |
| Shares 2 / 3 / 4 / 5 (%) | 21 / 41 / 27 / 12 | 30 / 38 / 23 / 10 |
| Grip, neutral / flexed wrist | -30 % / -68 % | -34 % / -68 % |
| Settled grip, self-selected (N) | 133 | 283 |
| Tenodesis | all checks pass | all checks pass (digit 5 PIP range 80 deg in trial A) |
| Lateral pinch | 38.7 N | 38.7 N (unchanged) |

`test_grip_at_the_cached_optimum` passes again with the S1b entry (fingerprint `palm_pad_m` 0.007986).

## Hand rebuild S2 (thumb and first-dorsal-interosseous heads)

The pinch muscle set is OP, AdPt, AdPo, FPBs, FPBd, APB plus the extrinsics; DI1s and DI1d join the
finger intrinsics. Fresh optimum (before = S1b):

| Quantity | S1b | S2 |
| --- | --- | --- |
| Lateral pinch (N) | 38.7 | 52.9 (in vivo 51.9 ± 20.4; levels FPL 0.79, APL 1.0, AdPt 0.42, AdPo 1.0, FPBs 1.0, FPBd 1.0) |
| Grip, self-selected (N) | 310.3 | 334.8 (the intrinsics are inactive; their passive forces changed) |
| Shares 2 / 3 / 4 / 5 (%) | 30 / 38 / 23 / 10 | 34 / 35 / 21 / 10 |
| Settled grip (N) | 283 | 306 |
| Tenodesis | all checks pass | all checks pass |

With the AdP heads fitted to Smutz's flexion arms the lateral pinch has no solution within its off-axis
limit (best palmar force 20.5 N at 1.9 times off-axis), which is why they keep their registered paths
(`passive-moments-and-thenar.md`). The grip cache fingerprint now also holds the actuator count and OP's
passive scale, which tell S1b and S2 apart.

## Grip and fist diagnosis

Why the middle finger carried a quarter of its measured share and the little finger too much (35.9 / 10.3 / 13.9 / 39.8 % at 0.15 s
at the start of this round), hypothesis by hypothesis on the model before the changes below. Each row is a fresh optimisation (ten
starts, except the baseline's 32); "settled" holds the same levels for 2 s; "whole finger" counts all three phalanges.

**The optimiser first.** The optimum is not one flat top but three modes. Over 32 starts:

| Mode | FDS / FDP levels | Grip (N) | Shares 2 / 3 / 4 / 5 at 0.15 s (%) |
| --- | --- | --- | --- |
| FDS and FDP high (the best admissible) | 0.93-1.0 / 0.68-0.85 | 306-319 | 30-33 / 16-18 / 20-23 / 28-32 |
| FDP high, FDS low | 0.07-0.75 / 0.73-0.96 | 260-314 | 29-41 / 4-14 / 6-17 / 39-49 |
| FDS high, FDP low | 0.84-1.0 / 0.18-0.37 | 256-288 | 31-34 / 22-29 / 14-18 / 23-27 |

Within a mode the spread is about 2 points per digit; between modes the middle finger moves by up to 25 points at grip forces within
15 %. The earlier 10.3 % was the second mode, reached by chance, so a change is real only when it exceeds 2 points within one mode.

**Hypotheses.** Best admissible optimum of the first mode unless marked:

| Change | Grip (N) | Shares at 0.15 s (%) | Settled grip (N) | Settled shares (%) | Settled, whole finger (%) |
| --- | --- | --- | --- | --- | --- |
| Baseline | 317.2 | 31.5 / 15.6 / 21.6 / 31.4 | 254.4 | 38.1 / 16.3 / 15.2 / 30.3 | 32.0 / 26.8 / 13.7 / 27.6 |
| Finger-finger contact pairs kept in the scene | 363.5 | 27.3 / 29.8 / 19.9 / 23.0 | 270.7 | 37.0 / 18.3 / 13.7 / 30.9 | 33.0 / 25.9 / 13.3 / 27.8 |
| MCP abduction spring tightening to 0 deg by 45 deg flexion, 2 N m/rad | 311.0 | 30.3 / 17.0 / 19.1 / 33.6 | 253.3 | 36.9 / 17.2 / 15.0 / 30.9 | 33.2 / 26.1 / 13.3 / 27.4 |
| ″ 8 N m/rad | 333.2 | 26.2 / 23.4 / 20.7 / 29.7 | 258.2 | 34.9 / 22.2 / 15.7 / 27.2 | 31.1 / 28.5 / 13.9 / 26.4 |
| MCP abduction at its limits (range ±0.06 deg, soft) | 370.7 | 23.8 / 30.5 / 18.8 / 26.8 | 261.4 | 32.0 / 23.6 / 15.3 / 29.1 | 28.6 / 29.1 / 14.1 / 28.2 |
| Collateral strands, 80 N/mm (adopted below) | 369.9 | 24.8 / 30.0 / 18.7 / 26.5 | 252.6 | 35.4 / 19.3 / 16.0 / 29.3 | 29.2 / 28.4 / 14.3 / 28.2 |
| CMC4/5 locked | 267.6 | 35.6 / 20.2 / 22.6 / 21.5 | 230.2 | 41.7 / 19.0 / 14.7 / 24.6 | 32.6 / 29.0 / 14.3 / 24.1 |
| All finger intrinsics at 0.5 (third mode) | 211.8 | 35.0 / 29.7 / 22.2 / 13.2 | 186.4 | 39.8 / 27.0 / 23.8 / 9.5 | 32.3 / 38.6 / 19.0 / 10.1 |
| All finger intrinsics at 0 (second mode) | 168.2 | 46.0 / 12.1 / 9.7 / 32.2 | 102.0 | 75.3 / 13.7 / 0.0 / 10.9 | 41.9 / 30.1 / 11.7 / 16.3 |
| Lumbricals at 0 | 311.6 | 30.5 / 15.6 / 22.0 / 31.9 | 246.1 | 39.0 / 15.2 / 16.0 / 29.8 | 31.9 / 26.9 / 14.0 / 27.1 |
| Both DI1 heads at 0 | 284.6 | 24.2 / 18.4 / 23.6 / 33.7 | 220.8 | 29.9 / 19.4 / 18.7 / 32.0 | 25.9 / 29.5 / 15.2 / 29.4 |
| ADM / FDM / ODM at activation 0, one at a time | 297 / 315 / 287 | 32-33 / 15-17 / 21-23 / 29-33 | 223-237 | within 3 points of baseline | within 3 points |
| ADM, FDM and ODM removed, no passive force | 217.8 | 42.4 / 20.8 / 19.8 / 16.9 | 155.4 | 59.5 / 23.7 / 8.3 / 8.5 | 40.3 / 31.3 / 13.7 / 14.8 |
| FDP5 force x 0.5 | 292.0 | 34.4 / 17.7 / 23.2 / 24.7 | 237.6 | 42.3 / 17.2 / 19.7 / 20.8 | 38.4 / 30.5 / 16.1 / 15.0 |
| FDP5 force x 1.25 | 329.8 | 28.2 / 11.7 / 16.6 / 43.4 | 227.1 | 41.3 / 15.4 / 14.8 / 28.4 | 33.3 / 26.1 / 13.3 / 27.3 |
| FDS5 force x 0.5 / x 1.25 | 327 / 312 | 31 / 14-15 / 20 / 35 | 225 / 254 | within 3 points | within 3 points |
| Baseline, whole fingers optimised, 1 s, wrist neutral | 242.8 whole finger | | | | 32.8 / 16.2 / 14.1 / 36.9 |
| Kong's double-frustum handle (48 / 55 / 55 / 48 mm), wrist neutral, 1 s | 201.8 whole finger | | | | 37.4 / 19.8 / 12.7 / 30.2 |

**What matters.**

- **Which phalanges count, at the settled state: 10 points on the middle finger.** The protocol counts the proximal and middle phalanges
  (McFarland), but Kong's load cells take the whole finger, and Amis 1987 sums all three phalanges and finds the distal phalanx carrying
  the most, with middle and proximal similar. In the model the middle finger rests on its tip: 0 / 29 / 83 N on its proximal, middle and
  distal phalanges at the settled state. The fix is in the protocol (below).
- **The transient.** At 0.15 s neighbour contacts and MCP abduction restraint each move the middle finger by 8-15 points, far above the
  spread, because they change how the fingers arrive. Settled, the same changes give 2-7 points. Kong's subjects held for 4 s.
- **The FDS / FDP level.** The middle finger's strength is its FDS. The wrist balance decides how much FDS the optimum affords: FDS has the
  larger wrist moment arm, so the second mode wins whenever extensors run short (wrist neutral: FDS 0.07-0.11, FDP 0.6, grip 40 % lower).
  With FDS high and FDP low the whole-finger shares reach Kong's middle finger (38.6 %) at 186 N. No single level satisfies grip and shares.
- **The little finger is its FDP.** FDP5 has the largest FDP force in the model (Lieber et al. 1992's PCSA 2.20 cm2, equal to FDP3's 2.23, against
  Ketchum's in-vivo 5.27 kg, the smallest FDP). Half of it brings the little finger to 15.0 % settled, whole finger, Kong's value, which one
  muscle's force must not be tuned to reach. The hypothenar muscles at full activation add about 5 points; no single one matters.
- **What does not matter:** lumbricals, FDS5, a single hypothenar muscle.
- **Unexplained:** the ring finger stays at 13-16 % (settled, whole finger) in every row but one, against 22-25 %. EDC at 0.8-0.95 in the optimum (the
  optimiser uses it as a wrist extensor, while Chao et al. 1976 have the extensors passive in grasp) takes most from the ring finger, the largest
  EDC against its flexors; at EDC 0 the grip rises to 388 N but the wrist lock supplies 5.3 N m. The root of the low share may be the wrist balance
  itself.

**Fist.** Which contact pairs hold the middle finger: removing the pairs of digits 2-3 lets it reach the palm (-1.6 mm), as does removing 3-4 (-1.1 mm); 2-4 gives
9.2 mm, 3-5 / 2-5 / 4-5 none. It is wedged between two neighbours: at their flexion limits the proximal capsules overlap 8.7 and 6.0 mm, and the ring finger
drifts radially by up to 20 deg, past the MCP's soft limit, under its FDS (abduction arm -4.3 mm at 90 deg MCP flexion against +0.9 mm at 0 deg). A tight
abduction spring alone makes it worse (gap 17-22 mm): with nowhere to go the overlap remains. Fixes: the active fist pairs and the adjacent-digit margin.

**History against `dev`.** `dev`'s shares in this protocol are 52.3 / 36.2 / 4.3 / 7.2 % at 0.15 s: the middle finger is inside Kong's band, the others are not. The
middle finger's recorded share falls 40.7 (S1) -> 38.0 (S1b) -> 35.0 (S2) -> 34.0 (S3) -> 30.9 (S4, hypothenar muscles) -> 29.7 (S5) -> 25-30 (S6-S2c) -> 27.3 (wrap sides) ->
17.2 (`vmax` 10) -> 10.3 (ECU), but the last two are mode changes as well as model changes: at the old levels `vmax` 10 gave 18 % and the 32-start optimum of the ECU step is
15.6 %. Hypothenar force pulled share to digit 5 (S4), and each step of muscle speed moved the 0.15 s reading toward the settled distribution.

**Protocol, as a result.** The share test now takes whole fingers in the settled state (above). The tolerance (1 SD of Kong) and the target are unchanged; the test still fails.

## Strength: one specific tension

**Reference subject.** An adult man, right or dominant hand, mean over all ages: the strength targets are men's norms and the head is sized with de Leva's male share, and these are the largest cohorts with SDs. `myofullbody` totals 76.51 kg, which follows from the segment masses of its sources and was not chosen (the Rajagopal OpenSim model the legs come from totals 75.34 kg). Women's norms and the mixed cohorts stay as context. The hand-function bands now follow this subject: grip 338.0-589.8 N (Mathiowetz 1985, was 270.7-533.5 N, O'Driscoll 1992), lateral pinch 88.5-129.5 N (was 31.5-72.3 N, Valero-Cuevas 2003); each is the source's mean ± 1 SD. The grip's negative control (flexors at half force below the band) still holds.

**Fit.** Peak force = PCSA x one value for every wrist and hand muscle (Holzbaur 2005's Table 1 for the extrinsics, Jacobson 1992 for the intrinsics, `build/data/specific_tension.json`). Weighted least squares over the four functional targets, at a grid of values with each muscle's passive force kept:

| Target (men) | Mean ± SD | Value it asks for (N/cm2) | Model at 166 N/cm2 |
| --- | --- | --- | --- |
| Index MCP abduction, RIHM (McGee 2020) (N m) | 2.71 ± 0.84 | 220 | 2.02 |
| Little-finger abduction, RIHM (McGee 2020) (N m) | 1.56 ± 0.47 | 134 | 1.94 |
| Grip (Mathiowetz 1985) (N) | 463.9 ± 125.9 | 160 | 483 |
| Key pinch (Mathiowetz 1985) (N) | 109.0 ± 20.5 | 180 | 100 |

Result: **166 N/cm2**. The held-out checks: fitted on the two moments alone, 156 N/cm2 predicts grip 455 N and pinch 95 N; fitted on grip and pinch alone, 172 N/cm2 predicts 2.09 and 2.01 N m; leaving out one target at a time, it is predicted within 1.1 SD (index -0.9, little +1.1, grip +0.2, pinch -0.7). The functional targets agree with each other.

Not fitted, at 166 N/cm2:

| Check | Model | Measured | Value it asks for (N/cm2) |
| --- | --- | --- | --- |
| Wrist extension, ECRL, ECRB, ECU (N m) | 8.5 | 6.83 (Ketchum 1978, fingers flaccid), 6.5 (Delp 1996) | 134 |
| Wrist flexion, FCR, FCU, PL (N m) | 10.0 | 11 (Delp 1996, finger activity not stated) | 183 |
| Wrist flexion, with FDS, FDP, FPL at full activation (N m) | 45.8 | 11, as above | 40 |
| Wrist extension, with EDC, EDM, EIP, EPL (N m) | 19.5 | 6.5, as above | 55 |
| Index MCP flexion, DI1, PI1, LUM1 (N m) | 1.33 | 2.24 (McGee 72.0 N x 42.6 mm x Ketchum's 73 % for the intrinsics) | 220 |
| First dorsal interosseous force (N) | 249 | 53.4 ± 24.7 (Ketchum), 93.3 (Infantolino 2014) | 36-62 |
| First palmar interosseous force (N) | 124.5 | 27.2 ± 11.5 (Ketchum) | 36 |
| First lumbrical force (N) | 18.3 | 19.9 ± 8.8 (Ketchum) | 181 |
| FDP2 / FDS2 force (N) | 252 / 226 | 60.6 / 67.8 (Ketchum) | 40 / 50 |

**Against the published range.** 166 N/cm2 lies outside it: 25 (Loren 1996), 35 (Brand 1981), 45 (Holzbaur 2005, forearm), 55 ± 11 (O'Brien 2010, quadriceps in vivo), 34-100 (Christophy 2012's survey); only Holzbaur's 140 N/cm2 for the arm comes near. Holzbaur already attributes his high values to disuse atrophy in elderly specimens. Two sources here point the same way: Linscheid 1991's PCSA of the same intrinsics are about 2.5 times Jacobson 1992's, so 166 N/cm2 on Jacobson's is about 66 N/cm2 on Linscheid's; Infantolino 2014 find an FDI PCSA in vivo twice the cadaveric. So the value is best read as "physiological tension on a PCSA about three times too small", not as a tissue property. Individual peak forces are then large against the in-vivo single-muscle estimates. Those estimates come from single-joint efforts split by mass fractions and can't be right for grip either way: Ketchum's FDS and FDP sum to 461 N over the four fingers, and on the model's mechanics that gives about 120 N of grip (Chao 1976 and Amis 1987 also find tendon forces well above the external force in grasp).

**Forces.** FCR 408 -> 273 N, FCU 480 -> 475 N, ECRL 337 -> 362 N, ECRB 253 -> 371 N, ECU 193 -> 344 N, FDS3 259 -> 336 N, FDP3 212 -> 302 N, EDC4 109 -> 125 N, FPL 201 -> 285 N, OP 180 -> 169 N, DI1 129 -> 249 N, ADM 77 -> 148 N. BRD, PT, PQ and SUP keep their forces; no target here tests them.

**Numerics.** Force-velocity damping grows with peak force. The thumb needs more armature to stay stable at the 2 ms step: CMC 2e-4 -> 5e-4 kg m2 on both axes, MP 1e-4 -> 3e-4 kg m2 (`passive_joint_moments.json`), the smallest values with no period-2 oscillation over random thumb poses and muscle subsets at full activation. The thumb's CMC inertia is now 6-8 times anthropometric.

**The optimiser.** After the search the wrist-muscle levels are balanced again at their own levels (`balance_iterations`), because ECU and FCU also cross CMC5: the wrist lock at the optimum falls from 0.83 to 0.01 N m.

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
  65 % against the paper's 46 %. Widening these windows would change the
  operating ranges themselves.
- **Grip geometry.** The grip depends on the dynamometer placement much more than on the contact
  parameters: with FDS and FDP at full activation the self-selected grip was 391, 1101, 335 and 45 N with
  the dynamometer's distal face 16 mm distal of, at, 9 mm proximal of (the table) and 19 mm proximal of
  MCP 3. The placement rule keeps the contacts along the phalanges.
- **Pinch posture.** The CMC angles are measured from each model's own zero. Flipping the flexion sign
  leaves no feasible force; 20 deg abduction instead of adduction gives 68.8 N; the zero posture gives
  60.1 N. FPB and APB are at full activation at the optimum.

## Hand rebuild S3 (CMC4/5 joints)

CMC4/5 are free in every scene (coupled). Fresh optimum (before = S2):

| Quantity | S2 | S3 |
| --- | --- | --- |
| Grip, self-selected (N) | 334.8 | 314.9 |
| Shares 2 / 3 / 4 / 5 (%) | 34 / 35 / 21 / 10 | 34 / 34 / 20 / 12 |
| Settled grip (N) | 306 | 278 |
| Grip, neutral / flexed wrist | -32 % / -65 % | -32 % / -66 % |
| Tenodesis, lateral pinch | pass, 52.9 N | pass, 52.9 N |

## Hand rebuild S4 (hypothenar muscles)

ADM (was UI_UB5), FDM and ODM join the finger intrinsics; none is in the grip's muscle set. Fresh optimum
(before = S3):

| Quantity | S3 | S4 |
| --- | --- | --- |
| Grip, self-selected (N) | 314.9 | 377.2 |
| Shares 2 / 3 / 4 / 5 (%) | 34 / 34 / 20 / 12 | 30 / 31 / 19 / 21 |
| Settled grip (N) | 278 | 340 |
| Grip, neutral / flexed wrist | -32 % / -66 % | -37 % / -62 % |
| Tenodesis | pass | pass; little-finger PIP range 80 -> 40 deg (ADM has no lateral-band insertion) |
| Lateral pinch (N) | 52.9 | 52.9 |

## Hand rebuild S5 (interossei and lumbricals)

PI1-3, DI2-4 and LUM1-4 replace the RI / LU_RB / UI_UB bundles in the `finger_intrinsic` group, which the
grip holds at full activation. Their forces follow Jacobson 1992 (lumbricals 5-10 N instead of 48-98 N).
Fresh optimum (before = EM-a):

| Quantity | EM-a | S5 |
| --- | --- | --- |
| Grip, self-selected (N) | 389.5 | 335.4 |
| Shares 2 / 3 / 4 / 5 (%) | 28 / 33 / 17 / 22 | 29 / 30 / 18 / 23 |
| Settled grip (N) | 363 | 267 |
| Grip, neutral / flexed wrist | see EM-a | -29 % / -62 % |
| Tenodesis | pass | pass; mean PIP range 10.9 -> 16.1 deg (ARMS 16.1) |
| Lateral pinch (N) | 53.1 | 51.5 |

The middle finger's share (29.7 %) is 0.5 points outside the 12-point band around ARMS's 42.2 %, so
`test_grip_at_the_cached_optimum` fails since S5 (tolerance unchanged; listed under the open checks in
`hand-literature-comparison.md`). Since the measured-share revision the target is measured: Kong et al.
2011's finger contributions at 100 % MVC (below). Setting the dorsal-interosseous hood fractions to 0.5 each instead of
Salsbury's shares moves the shares by less than one point.

## Digit shares: measured target

The grip test's digit-share target was the ARMS model's own shares (23.3 / 42.2 / 23.0 / 11.5 %, within 12
points). It is now a measurement with its spread, mean ± 1 SD as for the grip force:

| Source | Protocol | Shares 2 / 3 / 4 / 5 (%) |
| --- | --- | --- |
| Kong et al. 2011 (test target) | maximal power grip of a multi-finger handle (spans 48 / 55 / 55 / 48 mm), seated, elbow 90 deg, forearm and wrist neutral; 72 men; Table 3, 100 % MVC | 21.1 ± 4.7 / 38.5 ± 4.2 / 25.5 ± 4.7 / 15.0 ± 4.7 |
| Amis 1987 | maximal grasp of cylinders 31-116 mm, one finger instrumented per trial, 17 subjects; abstract and Fig. 6 (no SDs; shares independent of diameter) | 30 / 30 / 22 / 18 |
| Ohtsuki 1981 | four fingers pulling on rings over the middle phalanges, hand and proximal phalanges in a cast (extrinsic flexors only); right hand, Experiment I | 24.7 ± 3.9 / 32.8 ± 3.4 / 27.0 ± 2.2 / 15.5 ± 2.0 |
| Ejeskar & Ortengren 1981 | force to pull each fingertip off a gripped 20-35 mm cylinder, one finger at a time; 20 men, right hand (Table 2), as shares of the four forces | 26 / 29 / 23 / 22 |

Kong et al.'s protocol matches the model's best: simultaneous four-finger forces in a maximal grip of a
handle whose spans bracket the dynamometer's 48 mm, elbow at 90 deg, with per-finger SDs. Amis's
cylinder study is the closest in posture but reports no spread; the index and middle fingers share
equally there, and the model (30 / 27 / 19 / 24 % after the wrap-side revision) lies close to it but
outside Kong's band for every finger: the index and little fingers carry too much, the middle and ring
fingers too little. The test fails on this until the model's grip changes; its tolerance is not widened.
Kinoshita et al. 1996 (holding cylinders with a circular five-finger grip) and Radwin et al. 1992
(submaximal five-finger pinch) measure other tasks and are not used.

## Force-velocity step: grip at 0.15 s and settled

`vmax` 10 and the thumb CMC armature (`hand-literature-comparison.md`) leave the tenodesis (activation 0) and the
lateral pinch (static) unchanged to the printed digit. The grip protocol reads the force at 0.15 s, a transient, and the
faster muscles reach the settled state sooner:

| Quantity | Before | Now |
| --- | --- | --- |
| Grip, self-selected wrist, balanced (N) | 338.9 | 333.7 |
| Shares 2 / 3 / 4 / 5 at 0.15 s (%) | 30 / 27 / 19 / 24 | 31 / 17 / 21 / 31 |
| Settled grip (N) | 263.8 | 264.1 |
| Settled shares (%) | 39 / 17 / 17 / 27 | 40 / 16 / 15 / 29 |
| Grip with the wrist held (N) | 455.9 | 454.8 |
| Neutral / flexed wrist against self-selected (%) | -28 / -61 | -32 / -58 |
| Lateral pinch (N) | 67.5 | 67.5 |

At the old optimum's levels the new model gives 323.8 N with shares 31 / 18 / 21 / 30 %, so the change in shares is the
muscles, not the optimiser. The middle-finger share at 0.15 s moves further from Kong et al. 2011's 38.5 %; the settled
share has been that low since before this step. `test_grip_at_the_cached_optimum` keeps failing on the shares.

The optimiser's random starts went from 4 to 8. The objective is flat near its top and multi-modal (ten starts on this
model: 254-334 N); the two fixed and first four random starts reached at best 327.2 N with a 1.32 mm transient
penetration, above the test's 1 mm, and the eighth start 333.7 N with 0.04 mm.
