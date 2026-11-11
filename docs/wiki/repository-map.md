# Repository Map

Navigation guide for agents and contributors.

## Top-level layout

```
myo_sim/              # installable Python package
  __init__.py         # MODELS_DIR, load(), get_path(), composed-model aliases
  fragments.py        # FragmentInfo and _FragmentRegistry definitions
  models/             # all MJCF content — packaged into wheels
    arm/assets/       # myoarm_r_chain.xml, myoarm_r_muscle.xml, myoarm_r_tendon.xml, myoarm_r_assets.xml
    leg/assets/       # myolegs_chain.xml, myolegs_muscle.xml, myolegs_tendon.xml, myolegs_assets.xml
    torso/assets/     # myotorso_chain.xml (+abdomen variant), myotorso_muscle.xml, _tendon.xml, _assets.xml
    head/assets/      # myohead_rigid_chain.xml, myohead_simple_assets.xml
    meshes/           # 127 shared STL files
    contacts/         # myoarm_contacts.xml, myolegs_contacts.xml, myofullbody_contacts.xml
    scene/            # scene wrapper XMLs for rendering
    textures/         # shared textures
  build/
    compose.py        # MODEL_REGISTRY, BuildStrategy, build_spec(), SPEC_BUILDERS
    utils.py          # MirrorRules, mirror_element(), add_contact_pairs(), XML composition helpers
    hand.py           # prune_arm_spec_to_hand()
    keyframes.py      # list_postures(), add_posture_keyframes(), reads data/postures.json
    data/             # tables written by scripts/, with provenance
  physics/            # runtime physics the models need beyond MuJoCo (NumPy core; torch/JAX lazily)
    joint_coupling.py # put coupled joints (mjEQ_JOINT) on their polynomials
    fatigue.py        # 3CC-r muscle fatigue: NumPy, batched torch (torch.compile on CUDA)
    fatigue_jax.py    # the same fatigue step in JAX
    muscle_groups.py  # functional muscle groups, keys of the fatigue parameters
  muskemo/            # optional authoring: anatomy interchange, selected edits, Blender bridge and CLI
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
  test_postures.py                 # the posture keyframes: pose quality, attach, spec editing
  test_physics_*.py                # myo_sim.physics: couplings, fatigue (NumPy/torch/JAX), endurance times
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
| Add a composed model | `myo_sim/build/compose.py` — new `BuildStrategy`, builder, `SPEC_BUILDERS` and `MODEL_REGISTRY` entries |
| Add cross-part contact pairs | `myo_sim/models/contacts/` then `add_contact_pairs()` in `build/compose.py` |
| Add a mesh | `myo_sim/models/meshes/` |
| Change a starting posture | the constants in `scripts/generate_postures.py`, then rerun it |
| Edit public composed models | `myo_sim/build/compose.py` — `MODEL_REGISTRY` |
| Exchange anatomy or selected source-model edits with Blender/MuSkeMo | `myo_sim/muskemo/`; see `docs/wiki/muskemo-cli.md` |

## Publishing

After merging the release PR, run **Publish to PyPI and GitHub** from Actions on `main`.
The workflow reads the stable `X.Y.Z` version from the built wheel, publishes to PyPI
using the existing `pypi` environment and Trusted Publisher, then tags the exact build
commit as `vX.Y.Z`. It creates a GitHub release with generated notes and attaches the
same wheel and source archive before publishing the release. It never bumps versions
or writes to `main`. A tag pointing to another commit fails before the PyPI upload.

If GitHub release creation fails after PyPI succeeds, use **Re-run failed jobs** on
that run. The release job reuses the original artifacts, accepts an existing tag only
at the original commit, and resumes an existing draft/release. Do not restart the whole
workflow for an already published version: PyPI rejects duplicate uploads by design.
Tag creation requires the repository's tag rules to permit the Actions token to create
`v*` tags; the workflow does not bypass rules.
