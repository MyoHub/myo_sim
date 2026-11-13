# Testing Guide

Map of the test suite for agents working in `myo_sim`.

## Test Categories

| File | What it tests | When to run |
|---|---|---|
| `test_sims.py` | Static XML models load without error via `mujoco.MjModel.from_xml_path` | Always before PR |
| `test_head_build.py` | Neck passive mechanics, muscle parameters, preservation of full-body landmarks/actuators, cervical inertias and no ringing at the 2 ms step | After neck assets, mass or composition changes |
| `test_build_registry.py` | `BuildStrategy` enum fields, `MODEL_REGISTRY` completeness | After `compose.py` changes |
| `test_contact_paths.py` | Contacts are centralized in `contacts/`, not embedded in `assets/` | After XML or `compose.py` changes |
| `test_mirror_symmetry.py` | Body positions and joint axes are bilateral reflections across the sagittal plane | After arm chain or mirror rule changes |
| `test_chest_ownership.py` | `chest_r` in torso chain, not arm chain; no `chest_l` in bilateral models; `chest_r` stays massless and `myofullbody` totals ~76.5 kg | After chain XML changes |
| `test_knee_coupling_ranges.py` | Every coordinate the knee drives stays inside its declared range along its coupling polynomial | After knee chain XML or coupling changes |
| `test_knee_joint_centre.py` | The femur's origin seen from the tibia against stored OpenSim values on both knees, with the knee's couplings active | After knee chain XML changes |
| `test_pelvis_mass_once.py` | In the four models with legs the sacrum is massless beside the legs' pelvis; the sacrum moves only in those models | After torso or leg chain mass changes |
| `test_mass_convention.py` | `myofullbody` total mass (76.5 kg) and inertia sanity, only attachment frames massless, skull and cervical chain (`myohead_base` subtree) hold 6.94 % of body mass; part models keep the rigid head at 5.329 kg | After mass, inertia or head/torso changes |
| `test_fragment_registry.py` | FragmentRegistry legacy entries and all composed `myo_sim.load()` names build and forward | After `__init__.py` changes |
| `test_passive_torso_build.py` | Passive torso scaffold builds without error | After torso XML or compose changes |
| `test_legs_abdomen_build.py` | `myolegs_abdomen` model builds without error | After legs or abdomen changes |
| `test_leg_muscle_symmetry.py` | Leg muscle moment arms are symmetric between left and right | After `myolegs` XML changes |
| `test_arm_rigid_conversion.py` | Adopted arm maps match the published MoBL-ARMS rigid-tendon fibre length | After `myoarm_r_muscle.xml` or `scripts/fit_arm_rigid_maps.py` changes |
| `test_leg_passive.py` | Leg passive moments reproduce the recorded fit to Silder et al. (2007); active-strength constraint; L/R identical (the XML and the recorded errors are what the fit gives) | After `myolegs_muscle.xml` or `build/leg_passive.py` changes |
| `test_leg_rigid_conversion.py` | Adopted leg maps match the published Rajagopal rigid-tendon fibre lengths over the full joint range; muscles outside the passive fit hold what the script writes | After `myolegs_muscle.xml` or `scripts/fit_leg_rigid_maps.py` changes |
| `test_torso_muscle_symmetry.py` | Torso muscle moment arms are symmetric between left and right | After `myotorso` XML changes |
| `test_model_stability.py` | Every composed model steps without going non-finite; `myofullbody` survives random-activation rollouts (3 seeds) from qpos0 and from a stored state where BIClong grazed its former elbow wrap | After XML, tendon or wrap changes |
| `test_tendon_path_continuity.py` | No arm tendon length jumps by more than 1 mm per 1e-3 rad over the elbow range plus 0.13 rad past each limit, for 9 forearm rotations | After arm tendon, wrap or sidesite changes |
| `test_side_sites.py` | The copy of both wrap rules in `scripts/migrate_side_sites.py` reproduces the running MuJoCo; moved, removed and via entries of `build/data/side_sites.json` are in the model; no wrap jumps on the MuJoCo 4 rule by more than `jumps_mm` allows, along sweeps and coupled lines in range and past the limits | After a change to a wrap geom, side site or via point next to a wrap |
| `test_deep_hip_muscles.py` | Pectineus, quadratus femoris and gemelli: generated blocks equal what `scripts/generate_deep_hip_muscles.py` writes, they come after the other leg muscles, neutral moment arms within 11 mm of Gait2392, left/right lengths, the rigid-tendon map, a finite rollout | After leg chain, tendon or muscle changes, or `deep_hip_muscles.json` |
| `test_warp_parity.py` | Every tendon length in `myohands` and `myoarms` matches between CPU MuJoCo and mujoco-warp over random postures (to 1e-5 m) | After tendon or wrap changes; skipped without mujoco-warp or a CUDA device, and an expected failure on mujoco-warp < 3.16 for tendons with a sidesite inside their wrap geom |
| `test_warp_version_warning.py` | `load_spec()` warns once when mujoco-warp < 3.16 is installed and the model has arm or hand tendons; silent otherwise |
| `test_postures.py` | The posture keyframes: JSON matches its script, couplings hold, only the feet touch at t = 0, joint speeds over 50 ms, muscle lengths, attach and spec editing, with `qpos0` as negative control | After joint, coupling, contact or `compose.py` changes |
| `test_equivalence.py` | `myofullbody` joint and actuator counts match musclemimic reference spec | Manual only — needs musclemimic reference values |
| `test_bimanual_muscle_symmetry.py` | Bimanual arm muscle moment arms are symmetric | Manual only |
| `test_passive_thenar.py` | Passive joint moments and hand intrinsic muscles: generated XML blocks (main and labelled) equal what the scripts write from the JSON, chain sites match the table, the intrinsic path and passive-scale fit reproduces the table and keeps its fibre-length window, intrinsic moment-arm signs (Smutz 1998, An 1983), PIP ranges as tabled, every applied row and the intrinsic muscles on myofullbody / torso_arms / hands / arm_r, both arms from the right-arm files, finger damping as tabled on both hands, springs plus muscles against Formica's wrist stiffness, CPU vs mujoco-warp (with joint velocities), left/right symmetry, relaxed settling, rows refit from their sources (springs never worsen the total) | After arm tendon/muscle/chain, `build/hand.py`, `build/hand_data.py`, `scripts/fit_passive_moments.py` or `scripts/generate_hand_intrinsic_muscles.py` changes |
| `test_interossei_lumbricals.py` | Interossei and lumbricals (S5): one branched row per renamed bundle, new names on both hands of every model with a hand, pulley divisors compose the hood and band fractions, MuJoCo moment arms equal the fitted ones | After intrinsic table, extensor fractions or finger chain changes |
| `test_cmc_joints.py` | CMC4/5 joints: chain and assets block equal what `scripts/fit_cmc_joints.py` writes, the fit reproduces the table, joints and coupling on both hands of every model, crossing-muscle signs, coupling held under activation, relaxed hand at neutral | After chain, `cmc_joints.json` or CMC spring changes |
| `test_lumbrical_nodes.py` | Lumbrical junction nodes (S6): four node slides and tendon-equality ties per hand in every model with a hand, left nodes mirror the right in a fist, the tie holds (residual < 1 mm) in a fist and with LUM1 active, LUM1 alone flexes the MCP with the IPs extended without force chatter, the lumbrical lengthens by about 31 mm into the fist (Ranney & Wells 1988), `fold_node_forces` equals the tie reaction | After intrinsic table, FDP path or finger chain changes |
| `test_extrinsic_muscles.py` | Extrinsic revisions (S7): generated blocks equal what `scripts/generate_extrinsic_muscles.py` writes, the junctura nodes, ties and links in every model with a hand, links slack at q0 and in the resting posture and mirrored left to right, the recorded traction transfer is reproduced, FCU arms within Horii 1993's bands at every forearm rotation, FDS5 force, the ECU groove site on the ulna of every arm without a wrap geom, its recorded arms and their change with forearm rotation (Horii 1993, Loren 1996), left/right ECU lengths, the forearm rotation axis within 2 mm of the radial and ulnar head centres | After EDC, FCU, ECU or FDS5 path changes, forearm chain changes, or `extrinsic_muscles.json` |
| `test_extensor_mechanism.py` | Finger extensor mechanism: chain and tendon blocks equal what `scripts/fit_extensor_mechanism.py` writes, pulley divisors are the tabled fractions, the index arms reproduce the recorded c and lumbrical fits, every branched tendon (extensors and band-inserting intrinsics) extends PIP and DIP over their ranges, the left hand mirrors it, the ORL is slack in unresisted flexion and taut in Ueba's posture | After finger chain, extensor tendon or `extensor_mechanism.json` changes |
| `test_hand_pinch.py` | Lateral pinch after McFarland et al. 2023 (`scripts/validate_hand_function.py`, `build/data/hand_function.json`): force in the in vivo band under the off-axis limit, MuJoCo static check, none without the thenar muscles, posture mapping; skipped without the thenar muscles | After thumb muscle, tendon or chain changes |
| `test_hand_tenodesis.py` | Passive tenodesis after McFarland et al. 2023: trial A range of motion, direction, PIP / DIP, spread, trial B release, two negative controls; runs only with the fitted extrinsic muscles (`muscle_passive.json`) and springs fitted on them, read from the data | After hand muscle, tendon, spring, damping or chain changes |
| `test_muscle_passive.py` | Extrinsic hand-muscle slack lengths and fpmax: XML holds the tabled values on both arms of every model, recorded fit errors (Knutson 2000 index fit, one-sided joint fit), the `physiological` muscles (APL) carry the L0 and reference slack derived from their cited optimal fibre length and stay within 0.75-1.25 of it over 5-95 % of postures, active force-length factor >= 0.5 of the flexors in a full fist and the extensors in full extension, the release step (EDC3-5 and EDM at the smallest fpmax that opens digits 3-5 in the McFarland trial B), tenodesis and relaxed-hand rest off the joint limits on myofullbody, both hands settling alike (compared at rest, 10 s, and solver tolerance 1e-12) | After hand muscle, `build/data/muscle_passive.json` or `scripts/fit_muscle_passive.py` changes |
| `test_hand_pruning.py` | Hand models are the arm pruned: every arm actuator, tendon, equality, sensor and contact pair that should survive is in `myohand_r`, `myohands` and the bare hand fragments with identical parameters; tendon lengths agree at random hand postures; no dangling references; `rename_elements` keeps every reference and the numerics | After `build/hand.py`, `compose.py` or arm XML changes |
| `test_hand_contacts.py` | Hand self-contacts and pads: the contacts file and the pad geoms equal what their scripts write from the tables; every model with a hand carries every pair on each hand, the left mirrored with identical parameters; no self-contact active at rest (negative control: the two overlapping pairs without their margin); active fist over 12 nudged starts: penetration below 1 mm and every fingertip on the palm in most runs; fingertip soft tissue on target; fitted palm pads just touch the fist | After hand chain, contact or `compose.py` changes (rerun `scripts/fit_hand_pads.py` / `scripts/generate_hand_contacts.py`) |
| `test_naming.py` | Every right-arm name carries `_r` (muscle tendons `<actuator>_tendon`); the mirrored left arm has exactly the mirrored names; hand and arm models share joint, actuator, tendon, geom and camera names | After arm XML, `MirrorRules` or `build/hand.py` changes |
| `test_hand_literature.py` | `docs/wiki/hand-literature-comparison.md` matches `build/data/hand_literature.json`; every literature value cites a known source; the latest recorded step's fingertip, palm-pad and fist values match the current model; negative control for the fingertip measurement | After hand chain, contact or muscle changes (rerun `scripts/hand_literature_comparison.py`) |
| `test_numerical_stability.py` | No period-two velocity oscillation under Euler at the model's own step: activation sweep with all muscles and with each muscle group on `myohand_r` and `myoarm_r`, random thumb muscle subsets at activation 1, force-velocity rate bound, `vmax` and CMC armature on both sides equal their tables; negative control with the old `vmax` | After muscle parameters, moment arms, armature, inertia or damping change |
| `test_specific_tension.py` | Every wrist and hand peak force is PCSA x the tabled value on both hands, elbow and rotation muscles left out, recorded fit is the minimum of its recorded curves | After muscle force or `build/data/specific_tension.json` changes |
| `test_scapulothoracic_data.py` | Seth et al. 2019 shoulder table: DOI, CC BY 4.0 licence file, identical clavicle/scapula/humerus meshes, and the sternoclavicular centre the table was registered on still matches the model; the girdle (qpos0 on the ellipsoid, springs, armature) matches the table | After `build/data/scapulothoracic.json`, shoulder chain or arm attachment changes |
| `test_scapular_muscles.py` | Scapular muscles and cuff sub-regions: XML blocks equal the table, force = PCSA x tension, fibres within 0.5-1.5 along Seth's motions, cervical origins on the vertebrae (muscular neck) or the rigid neck, left sites mirror the right | After `build/data/scapular_muscles.json` or shoulder path changes |
| `test_hand_grip.py` | Maximum power grip after McFarland et al. 2023 (`scripts/validate_hand_function.py`, `build/data/hand_function.json`): the cached optimum of the model it runs on (band, wrist balance, penetration; whole-finger shares settled, against Kong et al. 2011) and against weak flexors; skipped without a cached optimum for the model. `MYO_SIM_SLOW=1` adds the optimisation and posture sensitivity | After hand muscle, tendon, spring or chain changes |

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

**New composed model** (built via `build_spec()`): add an entry to the `expected` dict in `test_every_registered_model_has_build_strategy` in `tests/test_build_registry.py`. If the model is bilateral (has left and right arms), also add its name to the `@pytest.mark.parametrize` decorator in `tests/test_mirror_symmetry.py`.

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

The recorded neck literature comparison is summarized in
`docs/validation/neck/README.md`. Experimental datasets, calibration/export scripts
and generated trajectory archives are intentionally omitted from this repository.
Runtime regression tests use only the model assets and existing dependencies.
