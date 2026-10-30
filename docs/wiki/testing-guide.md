# Testing Guide

Map of the test suite for agents working in `myo_sim`.

## Test Categories

| File | What it tests | When to run |
|---|---|---|
| `test_sims.py` | Static XML models load without error via `mujoco.MjModel.from_xml_path` | Always before PR |
| `test_build_registry.py` | `BuildStrategy` enum fields, `MODEL_REGISTRY` completeness | After `compose.py` changes |
| `test_contact_paths.py` | Contacts are centralized in `contacts/`, not embedded in `assets/` | After XML or `compose.py` changes |
| `test_mirror_symmetry.py` | Body positions and joint axes are bilateral reflections across the sagittal plane | After arm chain or mirror rule changes |
| `test_chest_ownership.py` | `chest_r` in torso chain, not arm chain; no `chest_l` in bilateral models; `chest_r` stays massless and `myofullbody` totals ~84.3 kg | After chain XML changes |
| `test_mass_convention.py` | `myofullbody` total mass and inertia sanity, only attachment frames massless and the head holds about 6.9 % of body mass | After mass, inertia or head/torso changes |
| `test_fragment_registry.py` | FragmentRegistry legacy entries and all composed `myo_sim.load()` names build and forward | After `__init__.py` changes |
| `test_passive_torso_build.py` | Passive torso scaffold builds without error | After torso XML or compose changes |
| `test_legs_abdomen_build.py` | `myolegs_abdomen` model builds without error | After legs or abdomen changes |
| `test_leg_muscle_symmetry.py` | Leg muscle moment arms are symmetric between left and right | After `myolegs` XML changes |
| `test_torso_muscle_symmetry.py` | Torso muscle moment arms are symmetric between left and right | After `myotorso` XML changes |
| `test_equivalence.py` | `myofullbody` joint and actuator counts match musclemimic reference spec | Manual only — needs musclemimic reference values |
| `test_bimanual_muscle_symmetry.py` | Bimanual arm muscle moment arms are symmetric | Manual only |
| `test_passive_thenar.py` | Passive joint moments and hand intrinsic muscles: generated XML blocks (main and labelled) equal what the scripts write from the JSON, chain sites match the table, the intrinsic path and passive-scale fit reproduces the table and keeps its fibre-length window, intrinsic moment-arm signs (Smutz 1998, An 1983), PIP ranges as tabled, every applied row and the intrinsic muscles on myofullbody / torso_arms / hands / arm_r, both arms from the right-arm files, finger damping as tabled on both hands, springs plus muscles against Formica's wrist stiffness, CPU vs mujoco-warp (with joint velocities), left/right symmetry, relaxed settling, rows refit from their sources (springs never worsen the total) | After arm tendon/muscle/chain, `build/hand.py`, `build/hand_data.py`, `scripts/fit_passive_moments.py` or `scripts/generate_hand_intrinsic_muscles.py` changes |
| `test_interossei_lumbricals.py` | Interossei and lumbricals (S5): one branched row per renamed bundle, new names on both hands of every model with a hand, pulley divisors compose the hood and band fractions, MuJoCo moment arms equal the fitted ones | After intrinsic table, extensor fractions or finger chain changes |
| `test_cmc_joints.py` | CMC4/5 joints: chain and assets block equal what `scripts/fit_cmc_joints.py` writes, the fit reproduces the table, joints and coupling on both hands of every model, crossing-muscle signs, coupling held under activation, relaxed hand at neutral | After chain, `cmc_joints.json` or CMC spring changes |
| `test_lumbrical_nodes.py` | Lumbrical junction nodes (S6): four node slides and tendon-equality ties per hand in every model with a hand, left nodes mirror the right in a fist, the tie holds (residual < 1 mm) in a fist and with LUM1 active, LUM1 alone flexes the MCP with the IPs extended without force chatter, the lumbrical lengthens by about 31 mm into the fist (Ranney & Wells 1988), `fold_node_forces` equals the tie reaction | After intrinsic table, FDP path or finger chain changes |
| `test_extrinsic_muscles.py` | Extrinsic revisions (S7): generated blocks equal what `scripts/generate_extrinsic_muscles.py` writes, the junctura nodes, ties and links in every model with a hand, links slack at q0 and in the resting posture and mirrored left to right, the recorded traction transfer is reproduced, FCU arms within Horii 1993's bands at every forearm rotation, FDS5 force | After EDC, FCU or FDS5 path changes, or `extrinsic_muscles.json` |
| `test_extensor_mechanism.py` | Finger extensor mechanism: chain and tendon blocks equal what `scripts/fit_extensor_mechanism.py` writes, pulley divisors are the tabled fractions, the index arms reproduce the recorded c and lumbrical fits, every branched tendon (extensors and band-inserting intrinsics) extends PIP and DIP over their ranges, the left hand mirrors it, the ORL is slack in unresisted flexion and taut in Ueba's posture | After finger chain, extensor tendon or `extensor_mechanism.json` changes |
| `test_hand_pinch.py` | Lateral pinch after McFarland et al. 2023 (`scripts/validate_hand_function.py`, `build/data/hand_function.json`): force in the in vivo band under the off-axis limit, MuJoCo static check, none without the thenar muscles, posture mapping; skipped without the thenar muscles | After thumb muscle, tendon or chain changes |
| `test_hand_tenodesis.py` | Passive tenodesis after McFarland et al. 2023: trial A range of motion, direction, PIP / DIP, spread, trial B release, two negative controls; runs only with the fitted extrinsic muscles (`muscle_passive.json`) and springs fitted on them, read from the data | After hand muscle, tendon, spring, damping or chain changes |
| `test_muscle_passive.py` | Extrinsic hand-muscle slack lengths and fpmax: XML holds the tabled values on both arms of every model, recorded fit errors (Knutson 2000 index fit, one-sided joint fit), active force-length factor >= 0.5 of the flexors in a full fist and the extensors in full extension, the release step (EDC3-5 and EDM at the smallest fpmax that opens digits 3-5 in the McFarland trial B), tenodesis and relaxed-hand rest off the joint limits on myofullbody, both hands settling alike (compared at rest, 10 s, and solver tolerance 1e-12) | After hand muscle, `build/data/muscle_passive.json` or `scripts/fit_muscle_passive.py` changes |
| `test_hand_pruning.py` | Hand models are the arm pruned: every arm actuator, tendon, equality, sensor and contact pair that should survive is in `myohand_r`, `myohands` and the bare hand fragments with identical parameters; tendon lengths agree at random hand postures; no dangling references; `rename_elements` keeps every reference and the numerics | After `build/hand.py`, `compose.py` or arm XML changes |
| `test_hand_contacts.py` | Hand self-contacts and pads: the contacts file and the pad geoms equal what their scripts write from the tables; every model with a hand carries every pair on each hand, the left mirrored with identical parameters; no self-contact active at rest (negative control: the two overlapping pairs without their margin); active fist penetration below 1 mm; fingertip soft tissue on target; fitted palm pads just touch the fist | After hand chain, contact or `compose.py` changes (rerun `scripts/fit_hand_pads.py` / `scripts/generate_hand_contacts.py`) |
| `test_naming.py` | Every right-arm name carries `_r` (muscle tendons `<actuator>_tendon`); the mirrored left arm has exactly the mirrored names; hand and arm models share joint, actuator, tendon, geom and camera names | After arm XML, `MirrorRules` or `build/hand.py` changes |
| `test_hand_literature.py` | `docs/wiki/hand-literature-comparison.md` matches `build/data/hand_literature.json`; every literature value cites a known source; the latest recorded step's fingertip, palm-pad and fist values match the current model; negative control for the fingertip measurement | After hand chain, contact or muscle changes (rerun `scripts/hand_literature_comparison.py`) |
| `test_hand_grip.py` | Maximum power grip after McFarland et al. 2023 (`scripts/validate_hand_function.py`, `build/data/hand_function.json`): the cached optimum of the model it runs on (band, digit shares, wrist balance, penetration) and against weak flexors; skipped without a cached optimum for the model. `MYO_SIM_SLOW=1` adds the optimisation and posture sensitivity | After hand muscle, tendon, spring or chain changes |

## Fast Gate (Run Before Every PR)

Run this before opening or updating a PR. It skips the two manual-only tests:

```bash
uv run pytest tests/ -x -n auto \
  --ignore=tests/test_equivalence.py \
  --ignore=tests/test_bimanual_muscle_symmetry.py
```

The `-x` flag stops on first failure. The `-n auto` flag runs in parallel across available CPUs.

To run a single test file:

```bash
uv run pytest tests/test_build_registry.py -v
```

## How to Add a Regression Test for a New Model

**New static XML entry point** (a standalone model loaded via `mujoco.MjModel.from_xml_path`): add its path (relative to `MODELS_DIR`) to the `model_paths` list in `tests/test_sims.py`.

**New composed model** (built via `build_model()`): add an entry to the `expected` dict in `test_every_registered_model_has_build_strategy` in `tests/test_build_registry.py`. If the model is bilateral (has left and right arms), also add its name to the `@pytest.mark.parametrize` decorator in `tests/test_mirror_symmetry.py`.

**New muscle symmetry check**: use `discover_bilateral_suffix_pairs()` from `tests/muscle_symmetry_checks.py` to find left/right muscle pairs automatically. Do not hardcode pairs in the test — that will cause the test to silently miss new muscles.

## Analysis Scripts (Not Pytest)

These scripts produce diagnostic plots and are not collected by pytest. Run them manually when investigating muscle mechanics.

```bash
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
cd tests && uv run python debug_muscle_bimanual.py
```

Output goes to `tests/output/muscle_analysis/`. The scripts use helpers from `muscle_analysis_utils.py`:

- `compute_moment_arm_curve` / `compute_force_length_curve` — sweep joint angles and record muscle properties.
- `parse_model_joint_equalities` / `apply_eq_constraints` — resolve MuJoCo joint equality constraints before computing forward kinematics.
- `plot_pair` — compare left/right muscle pairs visually.
