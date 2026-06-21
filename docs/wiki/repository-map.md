# Repository Map

Navigation guide for agents and contributors.

## Top-level layout

```
myo_sim/              # installable Python package
  __init__.py         # MODELS_DIR, load(), get_path(), composed-model aliases
  fragments.py        # FragmentInfo and _FragmentRegistry definitions
  models/             # all MJCF content — packaged into wheels
    arm/assets/       # myoarm_r_chain.xml, myoarm_r_muscles.xml, myoarm_r_tendons.xml, myoarm_r_assets.xml
    leg/assets/       # myolegs_chain.xml, myolegs_muscle.xml, myolegs_tendon.xml, myolegs_assets.xml
    torso/assets/     # myotorso_chain.xml (+abdomen variant), myotorso_muscle.xml, _tendon.xml, _assets.xml
    head/assets/      # myohead_rigid_chain.xml, myohead_simple_assets.xml
    meshes/           # 127 shared STL files
    contacts/         # myoarm_contacts.xml, myolegs_contacts.xml, myofullbody_contacts.xml
    scene/            # scene wrapper XMLs for rendering
    textures/         # shared textures
  build/
    compose.py        # MODEL_REGISTRY, BuildStrategy, build_model(), all builder functions
    utils.py          # MirrorRules, mirror_element(), add_contact_pairs(), XML composition helpers
    hand.py           # prune_arm_spec_to_hand()
tests/
  test_sims.py                     # smoke: loads static XML models
  test_build_registry.py           # BuildStrategy enum, MODEL_REGISTRY completeness
  test_contact_paths.py            # contacts centralized, not embedded
  test_mirror_symmetry.py          # body positions and axes are bilateral reflections
  test_chest_ownership.py          # chest_r in torso chain, not arm chain
  test_leg_muscle_symmetry.py      # leg moment arms symmetric L/R
  test_torso_muscle_symmetry.py    # torso moment arms symmetric L/R
  test_fragment_registry.py        # no stale static FragmentRegistry aliases
  test_passive_torso_build.py / test_legs_abdomen_build.py
  test_equivalence.py              # manual only — needs musclemimic_models
  test_bimanual_muscle_symmetry.py # manual only
  muscle_analysis_utils.py / muscle_symmetry_checks.py
  debug_muscle_leg.py / debug_muscle_torso.py / debug_muscle_bimanual.py
docs/wiki/            # canonical wiki location
```

## Where to make each kind of change

| Task | Location |
|---|---|
| Edit a muscle path or via-point | `myo_sim/models/<part>/assets/*_muscle.xml` and `*_tendon.xml` |
| Edit skeleton geometry | `myo_sim/models/<part>/assets/*_chain.xml` |
| Add a composed model | `myo_sim/build/compose.py` — new `BuildStrategy`, builder, `BUILDERS` and `MODEL_REGISTRY` entries |
| Add cross-part contact pairs | `myo_sim/models/contacts/` then `add_contact_pairs()` in `build/compose.py` |
| Add a mesh | `myo_sim/models/meshes/` |
| Edit public composed models | `myo_sim/build/compose.py` — `MODEL_REGISTRY` |
