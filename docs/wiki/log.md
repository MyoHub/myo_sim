# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

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

## 2026-07-11 — Reconcile fragment and default-class naming
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml` (from `_muscles`), `myoarm_r_tendon.xml` (from `_tendons`), `myo_sim/models/torso/assets/myotorso_{assets,chain,muscle,abdomen_muscle}.xml`, `myo_sim/models/leg/assets/myolegs_assets.xml`, `myo_sim/build/compose.py`, `docs/wiki/engineering-standards.md`, `docs/wiki/model-authoring.md`, `docs/wiki/repository-map.md`
Why: Closes the naming inconsistencies behind #64 and design principle #4 of #75. The role suffix was singular everywhere (`_muscle`, `_tendon`) except the arm, which used plural `_muscles`/`_tendons`; renamed the two arm files to the singular form. The 2026-06-21 class-scoping pass fixed arm and leg but missed the torso, which still declared `myoBack_muscle`, `myoBack_wrap`, `motor`, and `sidesite` — none scoped to the part; renamed them to `myotorso_muscle`, `myotorso_back_wrap`, `myotorso_motor`, `myotorso_sidesite`, updating every `class="..."` reference while leaving the unrelated `sidesite="..."` MJCF tendon attribute untouched. Also renamed the stray singular `myoleg_matskin` material to `myolegs_matskin`. Documented the `_r` marker semantics (present only for a right-only fragment mirrored at compose time; omitted for bilateral parts like `myolegs`) and the singular-role rule in engineering-standards, and fixed the wiki examples that cited the old `myoleg_muscle`/`class="wrap"`/`myotorso_r_chain.xml` forms. Verified: `uv run ruff check .`/`format --check .` clean, full suite 105 passed, every registered model compiles via `python -m myo_sim.build.compose`. Pure renames — no numeric or structural model change.

## 2026-06-29 — Restore full-body observation sensors
Changed: `myo_sim/models/sensors/myofullbody_sensors.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `docs/wiki/build-and-composition.md`
Why: Added a separate sensor XML and MjSpec injection helper for the MuscleMimic full-body frame velocity sensors, restoring the 110-sensor / 322-value observation layout while keeping leg touch sensors in the leg assets.

## 2026-06-26 — Document uv compose commands
Changed: `docs/wiki/agent-workflow.md`, `docs/wiki/build-and-composition.md`
Why: Updated compose CLI examples to use `uv run python -m myo_sim.build.compose`, keeping agent and contributor instructions aligned with the repository's `uv run` standard.

## 2026-06-21 — Scope remaining generic default class names
Changed: `myo_sim/models/arm/assets/myoarm_r_*.xml`, `myo_sim/models/leg/assets/myolegs_*.xml`, `myo_sim/models/legacy/osl/assets/myolegs_osl_*.xml`, `docs/wiki/engineering-standards.md`
Why: `myoarm_r_assets.xml` left `arm`, `muscle`, `elbow`, `wrist`, `fingers`, `upperarm_muscle`, and `forearm_muscle` unscoped, and `myolegs_assets.xml` mixed three naming conventions (`myolegs_`, `myoleg_`, `myo_leg_`) plus a bare `coll` class — both violating the naming rule this same wiki page states. Confirmed real, not cosmetic: PR #113's bilateral `myoarms` build hit exactly these unscoped names as duplicate-class collisions requiring a sanitization workaround. Renamed every nested class to a consistent `<part>_<role>` form and updated the wiki's example (the old text cited the now-fixed `myoleg_wrap`/`myo_leg_marker` as compliant). Re-verified: `uv run ruff check .`/`format --check .` clean, full test suite (89 tests) passes, `debug_muscle_leg.py` 101/114 (same 13 pre-existing knee-actuator discrepancies as before), `debug_muscle_torso.py` 208/208, `debug_muscle_bimanual.py` 215/215 — no regressions.

## 2026-06-20 — Document compose XML generation
Changed: `docs/wiki/build-and-composition.md`
Why: Added the `compose.py --generate` workflow and target XML outputs for primary composed model assemblies.

## 2026-06-09 — Initial wiki creation
Changed: docs/wiki/ (all files created)
Why: Established docs/wiki/ as the persistent knowledge layer for agents and contributors, following the myosuite pattern.
