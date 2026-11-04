# Repository Map

Navigation guide for agents and contributors.

## Top-level layout

```
myo_sim/              # installable Python package
  __init__.py         # MODELS_DIR, load(), get_path(), composed-model aliases
  fragments.py        # FragmentInfo and _FragmentRegistry definitions
  models/             # all MJCF content — packaged into wheels
    arm/assets/       # myoarm_r_chain.xml, myoarm_r_muscle.xml, myoarm_r_tendon.xml, myoarm_r_assets.xml
                      # (_tendon/_muscle hold script-generated blocks: hand intrinsic muscles, passive joint springs)
    leg/assets/       # myolegs_chain.xml, myolegs_muscle.xml, myolegs_tendon.xml, myolegs_assets.xml
    torso/assets/     # myotorso_chain.xml (+abdomen variant), myotorso_muscle.xml, _tendon.xml, _assets.xml
    head/assets/      # myohead_chain.xml, myohead_r_muscle.xml, myohead_r_tendon.xml, myohead_assets.xml, myohead_simple_assets.xml, myohead_rigid_chain.xml
    meshes/           # 127 shared STL files
    contacts/         # myoarm_contacts.xml, myolegs_contacts.xml, myofullbody_contacts.xml, myohand_self_contacts.xml (generated)
    scene/            # scene wrapper XMLs for rendering
    textures/         # shared textures
  build/
    compose.py        # MODEL_REGISTRY, BuildStrategy, build_spec(), SPEC_BUILDERS
    utils.py          # MirrorRules, mirror_element(), add_contact_pairs(), XML composition helpers
    hand.py           # prune_arm_spec_to_hand()
    hand_data.py      # intrinsic-muscle / passive-moment tables, passive tendon names, generated-block read/write, lumbrical node placement (place_tendon_nodes, fold_node_forces)
    rigid_conversion.py  # library for scripts/fit_{leg,arm}_rigid_maps.py (not used at load time)
    references.py        # cited table values (Rajagopal, MoBL-ARMS, Silder)
    leg_passive.py       # library for scripts/fit_leg_passive.py
    keyframes.py      # add_standing_keyframe(), reads data/standing_keyframes.json
    data/             # JSON provenance and fitting input of the intrinsic muscles and passive springs
scripts/
  fit_passive_moments.py       # refits passive springs, rewrites their block in myoarm_r_tendon.xml
  generate_hand_intrinsic_muscles.py # fits (--fit) and writes the intrinsic sites and blocks in the arm XML, the lumbrical nodes (--place-nodes) and their ties
  fit_cmc_joints.py            # fits (--fit) and writes the CMC4/5 joints, their coupling and the moved sites
  fit_extensor_mechanism.py    # fits (--fit) and writes the extensor-mechanism sites, branched tendons and ORL ligaments
  generate_extrinsic_muscles.py # fits (--fit, --fit-ecu) and writes the juncturae nodes and links, the FCU via points, the ECU groove site and FDS5's force
  fit_leg_rigid_maps.py / fit_arm_rigid_maps.py / fit_leg_passive.py
  generate_deep_hip_muscles.py # fits (--fit) and writes pectineus, quadratus femoris and gemelli into the leg XML
  generate_standing_keyframes.py # writes data/standing_keyframes.json; --report prints the wiki tables
tests/
  test_sims.py                     # smoke: loads static XML models
  test_build_registry.py           # BuildStrategy enum, MODEL_REGISTRY completeness
  test_contact_paths.py            # contacts centralized, not embedded
  test_mirror_symmetry.py          # body positions and axes are bilateral reflections
  test_chest_ownership.py          # chest_r in torso chain, not arm chain
  test_leg_muscle_symmetry.py      # leg moment arms symmetric L/R
  test_leg_rigid_conversion.py     # adopted leg maps vs published rigid-tendon fibre lengths
  test_arm_rigid_conversion.py     # adopted arm maps vs published MoBL-ARMS rigid-tendon fibre lengths
  test_leg_passive.py              # leg passive moments vs the recorded fit to Silder et al. 2007
  test_deep_hip_muscles.py         # deep hip muscles: generated blocks, order, Gait2392 moment arms, mirroring, rollout
  test_torso_muscle_symmetry.py    # torso moment arms symmetric L/R
  test_fragment_registry.py        # no stale static FragmentRegistry aliases
  test_passive_torso_build.py / test_legs_abdomen_build.py
  test_standing_keyframe.py        # the standing keyframe: pose quality, attach, spec editing
  test_equivalence.py              # manual only — needs musclemimic_models
  test_bimanual_muscle_symmetry.py # manual only
  muscle_analysis_utils.py / muscle_symmetry_checks.py
  debug_muscle_leg.py / debug_muscle_torso.py / debug_muscle_bimanual.py
docs/wiki/            # canonical wiki location
docs/validation/      # concise literature validation reports and figures; not packaged
```

## Where to make each kind of change

| Task | Location |
|---|---|
| Set rigid muscle parameters from the reference model | `scripts/fit_leg_rigid_maps.py` / `scripts/fit_arm_rigid_maps.py` (cited values in `build/references.py`) |
| Edit a muscle path or via-point | `myo_sim/models/<part>/assets/*_muscle.xml` and `*_tendon.xml` |
| Change hand intrinsic muscles or passive joint moments | `myo_sim/build/data/*.json`, then rerun `scripts/generate_hand_intrinsic_muscles.py` / `scripts/fit_passive_moments.py` (they rewrite only their generated blocks) |
| Edit skeleton geometry | `myo_sim/models/<part>/assets/*_chain.xml` |
| Add a composed model | `myo_sim/build/compose.py` — new `BuildStrategy`, builder, `SPEC_BUILDERS` and `MODEL_REGISTRY` entries |
| Add cross-part contact pairs | `myo_sim/models/contacts/` then `add_contact_pairs()` in `build/compose.py` |
| Add a mesh | `myo_sim/models/meshes/` |
| Change the standing pose | `scripts/generate_standing_keyframes.py`, then rerun it |
| Edit public composed models | `myo_sim/build/compose.py` — `MODEL_REGISTRY` |
