# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.

## 2026-10-28 — RI2 path, Kamper finger damping, pinch and tenodesis validation after McFarland et al. 2023
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml`, `myoarm_r_chain.xml`, `myo_sim/build/data/passive_joint_moments.json`, `scripts/fit_passive_moments.py`, `tests/test_passive_thenar.py`, `scripts/validate_hand_function.py`, `myo_sim/build/data/hand_function.json`, `myo_sim/build/hand_function.py`, `tests/test_hand_pinch.py`, `tests/test_hand_tenodesis.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/hand-function-validation.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`, `myo_sim/models/arm/README.md`, `myo_sim/models/arm/README_hand.md`
Why: RI2's path jumped by 7 mm because its insertion entered a wrap cylinder; finger damping was three to six times Kamper's measurement; the McFarland et al. 2023 validation found both. The pinch and tenodesis protocols and tests come with it (the script, table and loader are identical in Vittorio-Caggiano/myo_sim#2).

## 2026-10-27 — Thenar muscles and passive joint moments in the arm's own XML; springs fitted to the residual
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml`, `myoarm_r_muscle.xml` (generated blocks), removed `myoarm_r_thenar_*.xml` and `myoarm_r_passive*_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/__init__.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_muscles.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: One reference model per part, no flags: the thenar muscles and all measured and assumed passive rows are in the right-arm files (script-written blocks), and every spring is fitted to the source curve minus the model's own passive muscle moment, so nothing is counted twice; the `thenar_muscles` / `passive_moments` flags, the separate fragments and `subject_scale` (later PRs) are removed.
## 2026-10-28 — Release step for EDC3-5 and EDM; grip validation after McFarland et al. 2023
Changed: `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `tests/test_muscle_passive.py`, `scripts/validate_hand_function.py`, `myo_sim/build/data/hand_function.json`, `myo_sim/build/hand_function.py`, `tests/test_hand_grip.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/hand-function-validation.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`, `myo_sim/models/arm/README.md`
Why: EDC3-5 and EDM sat at the fpmax floor, so the digits barely opened when the wrist flexed; only their fpmax is refit, above the smallest value that opens digits 3-5 in the McFarland et al. 2023 release trial, with the slack lengths unchanged. The grip protocol and test of that validation come with it (the script, table and loader are identical in MyoHub/myo_sim#139).

## 2026-10-27 — Fit fpmax with the slack lengths, under an active force-length constraint
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/testing-guide.md`
Why: Lengthening the slack lengths alone left the finger flexors with almost no active force in a fist, so each extrinsic muscle now also gets its own passive-force scale (`fpmax`) and the fit keeps the flexors' active factor at least 0.5 in a full fist and the extensors' in full extension.

## 2026-10-27 — Fit the slack lengths of the other extrinsic finger, thumb and wrist muscles
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`
Why: On the anatomical parameters FDP4, FPL, FDP5, FDS3/FDP3 and ECU gave passive moments up to 25 times the measured totals (wrist, MCP/PIP/DIP 3-5, thumb MP/IP), so their slack lengths are now fitted per Binder-Markey to those totals with a one-sided loss (changes 0-10 %).

## 2026-10-27 — Fitted index extrinsic slack lengths written into the arm muscle XML
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `myo_sim/build/compose.py`, `myo_sim/__init__.py`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`
Why: Review: muscle parameters belong in the part's XML, not in a build step. The fitted `lengthrange` of FDS2/FDP2/EDC2/EIP is now in `myoarm_r_muscle.xml` (written by the fit script; the JSON stays as provenance), so tenodesis is in every model; the unreleased `passive_muscles` flag, `build/muscle_passive.py` and its `subject_scale` slack factor are removed.

## 2026-10-27 — Add opt-in passive muscle tension of the index extrinsic muscles (tenodesis)
Changed: `myo_sim/build/muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `scripts/fit_muscle_passive.py`, `myo_sim/build/compose.py`, `myo_sim/__init__.py`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`
Why: With the anatomical slack lengths and rigid tendons, FDS2/FDP2/EDC2/EI give up to 327 N cm of passive index MCP moment against at most 30 N cm measured (Knutson et al. 2000); `passive_muscles=True` sets slack lengths fitted per Binder-Markey & Murray 2017 (RMSE 98.4 -> 3.0 N cm) and gives tenodesis on both hands. Off by default, so models and checkpoints are unchanged. The other digits and the thumb show the same excess but have no extrinsic passive data.

## 2026-08-02 — Widen force-length curve bounds for muscles whose refit operating range outgrew them
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `myo_sim/models/torso/assets/myotorso_muscle.xml`
Why: Restoring anatomical fiber length for 26 muscles (see the anatomical-parameter entries below) shifted their normalized operating range partly outside the shared default force-length curve bounds (`lmin`/`lmax`); widened bounds per muscle to contain the new range, respecting MuJoCo's `lmin < 1 < lmax` requirement.

## 2026-08-02 — Add inter-finger self-collision pairs to the hand
Changed: `myo_sim/models/contacts/myohand_contacts.xml`
Why: No finger-to-finger contact pairs existed at all (only forearm-torso and left/right-forearm pairs), so fingers could pass through each other; added adjacent-digit pairs at each phalanx level plus thumb-opposition pairs, verified on both single-hand and bilateral builds.

## 2026-08-02 — Widen wrist deviation and shoulder rotation ranges
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml`
Why: Real motion-capture data (throwing, catching, tennis) required wrist deviation and shoulder rotation angles beyond the shipped joint limits; widened both to cover the observed range plus a literature-consistent margin.

## 2026-08-02 — Correct lumbar axial rotation and wrist flexion joint ranges
Changed: `myo_sim/models/torso/assets/myotorso_chain.xml`, `myo_sim/models/arm/assets/myoarm_r_chain.xml`
Why: `L4_L5_AR` was ~18x wider than published per-segment lumbar ROM and wrist `flexion_r` was roughly half the typical clinical range; corrected both to match standard normative values.

## 2026-08-02 — Restore anatomical muscle parameters for hand extrinsics
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`
Why: All 24 flagged finger/wrist flexor-extensor muscles had never received individual calibration and were compiling against MuJoCo's raw default fiber-length range; fit each to its OpenSim/MoBL-ARMS anatomical optimal fiber length, tendon slack length, and peak force.

## 2026-08-02 — Restore anatomical muscle parameters for shoulder muscles
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`
Why: 7 of 8 shoulder muscles flagged with inflated optimal fiber length were refit to their anatomical values; the 8th (`CORB`) was left unchanged after its excursion pattern matched the profile of muscles where refitting measurably hurts force accuracy.

## 2026-08-02 — Restore anatomical muscle parameters for the leg
Changed: `myo_sim/models/leg/assets/myolegs_muscle.xml`
Why: 6 of 10 flagged leg muscles were refit to anatomical optimal fiber length/tendon slack/peak force after remeasuring their physiological length range; the other 4 were deliberately left unchanged because refitting was cross-validated to increase force error against the OpenSim reference, not reduce it.

## 2026-08-02 — Restore anatomical muscle parameters for the torso
Changed: `myo_sim/models/torso/assets/myotorso_muscle.xml`
Why: 25 torso muscles had optimal fiber length inflated 5-30x versus the anatomical reference (a conversion artifact, not a real path-length problem); refit each to match, remeasuring the physiological joint range for the two muscles whose fit initially came out physically invalid.
## 2026-08-12 — Remove 12.87 kg of phantom mass from the `chest_r` wrap scaffold
Changed: `myo_sim/models/torso/assets/myotorso_assets.xml`, `myo_sim/models/torso/assets/myotorso_chain.xml`, `tests/test_chest_ownership.py`, `docs/wiki/testing-guide.md`
Why: `chest_r` contains only muscle-wrap ellipsoids and had no `<inertial>`, and the `myotorso_wrap` default class set no mass or density, so MuJoCo meshed its six r=0.08 m spheres at the default 1000 kg/m^3 — 6 x (4/3)pi(0.08)^3 x 1000 = 12.868 kg of parasitic mass welded into the torso. `build_model("myofullbody")` totalled 97.132 kg against the MuscleMimic reference's 84.270 kg, with `chest_r` the only body that differed; MuscleMimic sets its equivalent `thorax` body to 0.001 kg, and Winter's anthropometric segment fractions applied to the thigh, shank, foot and forearm masses all imply an 81-84 kg subject. Added `mass="0"` to the `myotorso_wrap` default class so no wrap geom can ever contribute mass again, plus an explicit `mass="0.001"` inertial on `chest_r` (required — with every geom massless and no inertial, MuJoCo rejects the body with "mass and inertia must be larger than mjMINVAL"). The only other `myotorso_wrap` geom, `pelvis_wrap` in `lumbar5`, sits in a body with its own explicit 1.824 kg inertial and is unaffected. Total is now 84.265 kg and `chest_r` is the sole body whose mass changed. The explicit inertial survives `spec.to_xml()` round-tripping, so the `explicitinertial` workaround in `myo_sim/build/elastic.py` is not needed here.

## 2026-10-27 — Thenar muscles and passive joint moments as arm XML fragments, on by default
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml`, `myoarm_r_thenar_tendon.xml`, `myoarm_r_thenar_muscle.xml`, `myoarm_r_passive_tendon.xml`, `myoarm_r_passive_assumed_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/build/hand.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_fragments.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/model-authoring.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: Review of the first version: the muscles and springs were added from Python instead of the part's XML files. Sites now live in the chain, tendons and actuators in fragments written by scripts from the JSON tables, merged and mirrored by compose.py; `thenar_muscles` / `passive_moments` default to on (+3 actuators per hand) and opt out to the previous model; MuJoCo floor raised to 3.7 for the polynomial tendon stiffness.
