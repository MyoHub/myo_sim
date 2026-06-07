# Issue Tracker

Tracks open GitHub issues, their status, dependencies, and assigned phase. Update this file as work progresses.

Source: https://github.com/MyoHub/myo_sim/issues

---

## Status legend

| Symbol | Meaning |
|---|---|
| `[ ]` | Not started |
| `[~]` | In progress |
| `[x]` | Done / closed |
| `[!]` | Blocked |

---

## Phase 1 — Foundation (unblocks most other work)

### [~] #75 — Restructure into Kinematics + Parts + Build
**Type:** Architecture  **Effort:** M  **Branch:** `mm_refactor_mjspec`

The file-naming convention (`*_chain.xml`, `*_muscle.xml`, `*_tendon.xml`, `*_assets.xml`) already implements the kinematics/parts/build separation for all body parts. Leg, torso, and arm are each self-contained with no active cross-part via-point dependencies. The one remaining structural gap is `chest_r` ownership: the thorax body is currently defined inside `myoarm_r_chain.xml`, which is anatomically incorrect and causes the bilateral model to create two chest bodies (one per mirrored arm).

An `interfaces/` layer is not needed and cannot be cleanly implemented: MJCF sites must be children of body elements, and `<include>` cannot inject into an already-parsed body from a separate file. Sites follow their body — they belong in whichever `*_chain.xml` owns that body.

See `wiki/issue-75-gap-analysis.md` for full analysis. The open architectural question for the branch owner is whether standalone `myoarm_r.xml` loading (without a torso) must remain supported after `chest_r` moves.

**Acceptance criteria:**
- [x] `chest_r` ownership resolved — moved to torso chain; `test_chest_ownership.py` asserts it (`b8fb32c`)
- [x] `prototype_mjspec_attach.py` renamed to `myo_sim/build/compose.py`
- [x] `ModelRegistration` boolean flags replaced with `BuildStrategy` enum and dispatch table
- [x] All model content under `myo_sim/models/`; build code under `myo_sim/build/`
- [x] `myo_sim.MODELS_DIR` resolves correctly after `pip install -e .` — verified by PR #81
- [ ] `test_sims.py` `model_paths` covers all currently loadable static models (reduced set, no elbow/finger/hand/body/ paths that no longer exist)
- [ ] CI green on `mm_refactor_mjspec` ← **in progress**, PRs #78 #80 #81 pending merge

**PRs:** [#78](https://github.com/MyoHub/myo_sim/pull/78) [#80](https://github.com/MyoHub/myo_sim/pull/80) [#81](https://github.com/MyoHub/myo_sim/pull/81)
**Depends on:** nothing — this is the blocker for #48, #62, #69

---

### [~] #62 — Remove duplicated torso model (replace static XML with MjSpec build)
**Type:** Cleanup  **Effort:** M

`torso_arm_chain.xml` duplicates what the MjSpec path already assembles. The static `body/` directory models (myofullbody, myoupperbody, myobody) are absent from the main tree and only live in `sandbox/`. The MjSpec-built versions should be written to canonical paths so `test_sims.py` and `FragmentRegistry` resolve them.

**Acceptance criteria:**
- [x] `FragmentRegistry` and `REGISTRY` now derive from the same `_FRAGMENT_CATALOG` — PR #78
- [x] Duplicate `mjspec/` removed; canonical path is `myo_sim/build/compose.py`
- [ ] Static `body/myofullbody.xml` artifact committed or smoke test updated to use `build_model()`
- [ ] `test_sims.py` `model_paths` pruned to only paths that actually exist

**Depends on:** #75 ✓

---

## Phase 2 — Model correctness bugs

### [ ] #48 — Terminate torso with sacrum, not pelvis
**Type:** Model correctness  **Effort:** S

`myotorso_chain.xml` already terminates at `sacrum` (correct). `myolegs_chain.xml` already starts at `pelvis` (correct). The gap is that there is no named attachment site on `sacrum` for the legs to attach to in the MjSpec build — the legs are currently attached via an ad-hoc frame added in Python rather than a declared site. This is prerequisite for the multi-segment foot (#69).

**Acceptance criteria:**
- [ ] `myotorso_chain.xml` sacrum body has an explicit named `legs_root_attach` site
- [ ] MjSpec build attaches legs via that site, not via a dynamically added frame
- [ ] `myofullbody` loads and pelvis does not float

**Depends on:** #75 (MjSpec attach path)

---

### [ ] #49 — No collision geoms in myoback
**Type:** Bug  **Effort:** M

The back segment currently relies on visual meshes for collision, unlike other torso segments which have explicit capsule/ellipsoid collision geoms. Contact behavior is undefined.

**Acceptance criteria:**
- [ ] Capsule/ellipsoid collision geoms added to myoback bodies following the pattern in other torso segments
- [ ] Geom sizes cross-checked with `myotorso_chain.xml` so they are consistent when composed
- [ ] No new self-intersections in `myofullbody`

**Depends on:** nothing (can proceed independently)

---

### [ ] #70 — Myotorso IO muscles asymmetric
**Type:** Bug  **Effort:** M

Internal oblique muscles `IO1_r`, `IO2_r`, `IO3_r` are visually shorter than their left counterparts. Almost certainly a via-point coordinate error on the right side introduced by a manual edit.

**Acceptance criteria:**
- [ ] `compute_moment_arm_curve` for `IO1_r` vs `IO1_l` (and IO2, IO3) are bilaterally symmetric within tolerance
- [ ] Regression test added to `tests/debug_muscle_torso.py` or promoted to a pytest parametrized case
- [ ] Fix confirmed via MuJoCo viewer visual inspection

**Depends on:** nothing

---

### [ ] #52 — Flexor-extensor role flipping
**Type:** Bug (biomechanics)  **Effort:** M

A group of muscles exhibits moment arm sign reversal at some joint angles, indicating a wrapping surface via-point jumping sides. Needs literature cross-check before closing.

**Acceptance criteria:**
- [ ] Affected muscles and joints identified by sweeping `compute_moment_arm_curve` across full range
- [ ] Wrapping surface geometry adjusted so moment arm sign is monotone or matches anatomical expectation
- [ ] Literature reference cited in the fix commit message or README
- [ ] Regression test added

**Depends on:** nothing

---

## Phase 3 — Naming and consistency

### [ ] #64 — `myoleg` naming inconsistency
**Type:** Naming cleanup  **Effort:** S

The prefix `myoleg` (singular) appears in some places, `myolegs` (plural) in others. Pick `myolegs` as canonical (already dominant in the codebase) and do a single mechanical rename across all XML, Python, and test files.

**Acceptance criteria:**
- [ ] `grep -r "myoleg[^s]"` in XML and Python files returns no hits (excluding mesh filenames that are fixed)
- [ ] `test_sims.py` and `FragmentRegistry` updated accordingly

**Depends on:** nothing

---

### [ ] Registry consolidation (no issue yet — file one)
**Type:** Internal cleanup  **Effort:** S

Three parallel model inventories (`_FRAGMENT_CATALOG`, `REGISTRY` in `myo_sim/__init__.py`, and `model_paths` in `test_sims.py`) must be kept in sync manually. Collapse to one source of truth.

**Acceptance criteria:**
- [ ] Single catalog drives `FragmentRegistry`, `REGISTRY`, and the smoke test model list
- [ ] Adding a model requires editing exactly one place

**Depends on:** #62

---

## Phase 4 — Infrastructure

### [~] #66 — Add CI and enhanced tests
**Type:** Infrastructure  **Effort:** M

**Acceptance criteria:**
- [x] GitHub Actions workflow (`.github/workflows/ci.yml`) with lint + test jobs — merged via PR #76, #79, #80
- [x] Lint job: `ruff check` + `ruff format --check`, runs in ~10s
- [x] Test job: `pytest tests/ -x -n auto` excluding heavy equivalence tests, runs in ~45s
- [x] CI triggers on push/PR to `main` and `mm_refactor_mjspec`
- [x] `_SHARED_MUSCLES` module-level build moved to lazy `pytest_generate_tests` hook — PR #78
- [ ] `test_equivalence.py` wired as a separate nightly/manual workflow (currently just excluded)

**PRs:** [#76](https://github.com/MyoHub/myo_sim/pull/76) (merged) [#78](https://github.com/MyoHub/myo_sim/pull/78) [#79](https://github.com/MyoHub/myo_sim/pull/79) (merged) [#80](https://github.com/MyoHub/myo_sim/pull/80) (merged)
**Depends on:** #62 ✓

---

## Phase 5 — Model quality

### [~] #73 — WIP musclemimic model updates (left arm, muscle wrapping, L-R symmetry)
**Type:** Model quality  **Effort:** L  **External contributor:** Bianca Ziliotto, Chengkun Li, Huiyi Wang

Open PR with stale TODOs. The MjSpec mirroring path makes the `Sync ArmL with ArmR` task substantially easier — the left arm should be derived via mirror rules rather than maintained as a separate hand-edited XML.

**Open TODOs from the PR:**
- [ ] Sync ArmL with ArmR (use MjSpec mirror rather than hand-edit)
- [ ] Add proper changelog to individual models
- [ ] Add references to model studies
- [ ] Add model test scripts
- [ ] Comply with all test scripts

**Depends on:** #75 (mirror path stable)

---

### [ ] #51 — Improve passive dynamics of torso
**Type:** Model quality / research  **Effort:** L

Most torso variants have poor passive dynamics. Requires literature survey and parameter sweep for physiological stiffness/damping values.

**Acceptance criteria:**
- [ ] Literature references identified for spinal soft tissue stiffness/damping
- [ ] Parameter sweep conducted; resting posture compared to neutral standing
- [ ] Separate fix per spinal region (lumbar, thoracic, cervical) if needed

**Depends on:** nothing (but easier after #70 and #52 are resolved)

---

## Phase 6 — New features (post-stabilization)

### [ ] #69 — Multi-segment foot
**Type:** New feature  **Effort:** XL

Add a multi-segment foot to `myolegs` based on the SimTK foot-ankle model. Only makes sense after the sacrum termination (#48) and MjSpec composition (#75) are stable so the foot attaches cleanly as a separate part.

**Depends on:** #75, #48

---

### [ ] #15 — Amputation / OSL model
**Type:** New feature  **Effort:** L

Per repo policy, the prosthesis and OSL controller belong in `myoassist`, not `myo_sim`. Work here is limited to exposing a clean amputation stub site on `myolegs` (`amputation_r` variant with a stump attachment site). The device side goes in `myoassist`.

**Depends on:** #75, #48

---

## Recommended sequencing

```
Phase 1: #75 chest_r decision → rename prototype → fix body/ paths
         #62 deduplicate static XMLs
Phase 2: #48, #49, #70, #52 in parallel (independent model bugs)
Phase 3: #64, registry consolidation
Phase 4: #66 CI (after model paths are stable)
Phase 5: #73, #51 (quality, ongoing)
Phase 6: #69, #15 (new features, post-stable)
```

Hard dependency chain: **#75 → #62 → #66** and **#75 + #48 → #69 + #15**.

> Note: the `interfaces/` layer described in issue #75 has been removed from scope. MJCF sites are tree-structured and cannot be injected into existing bodies from a separate file. The file-naming convention (`*_chain.xml` owns all sites on its bodies; `*_muscle.xml`/`*_tendon.xml` own actuation) is sufficient for all current body parts. The only structural gap is `chest_r` ownership, which is addressed directly in the #75 acceptance criteria above.
