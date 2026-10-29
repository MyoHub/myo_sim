# Model Authoring Guide

How to add to, change and maintain the models in `myo_sim`. It applies to human contributors and to agents alike. Read the policy first: it decides *where* a change goes before this guide explains *how*.

## Model policy

1. **One canonical model.** Each body part has exactly one canonical model: the full, most realistic one we can support. Every composed model (`myoarms`, `myohands`, `myofullbody`, ...) is built from these parts.
   - Models that differ from the canonical one only by removing muscles or joints (for example `myohand_r` from the arm, or torso models without the neck) are derived procedurally from it, never kept as separate XML.
2. **Improvements overwrite the model.** A change that makes the model more realistic is written directly into the part's XML files and is active for every user. This covers new muscles, tendons, ligaments, passive joint moments, contacts, and refitted masses, inertias or muscle parameters.
3. **No feature flags.** Never add a build argument, `build_kwargs` entry or `load()` parameter that switches a model feature on or off, not even for backward compatibility. Previous behaviour lives in previous release tags.
4. **Variants are separate registered models.** A separate model is justified only for a truly simplified model, for example several muscles combined into one, or when a feature makes simulation substantially more expensive (step time or memory) or is genuinely optional rather than more realistic. The variant is registered under its own name in `MODEL_REGISTRY` and has its own README section. Existing examples: `myolegs26` (a reduced leg muscle set) and `myotorso_abdomen`.
   - Prefer generic post-processing functions over per-model variants for simplifications that apply to many models. For example, `myo_sim.disable_collisions(spec, bodies=..., geoms=...)` (in `myo_sim/postprocess.py`) disables collisions on any loaded `MjSpec`, either all of them or the geoms selected by body or geom name patterns, and replaces a "no-collision" variant of each model. Such functions run in the user's code after loading; they never change the reference model and are not build options. New ones go in `myo_sim/postprocess.py`.
   - Muscle or tendon models that MuJoCo and mujoco-warp cannot run natively, for example through an engine plugin or a custom step loop that applies forces around `mj_step`, ship as separately registered variants under this policy, never as an option of the reference model. Their parameters are generated from provenance JSON into the variant's `*_muscle.xml`, like any other fitted value (see *Provenance*), not computed when the model is loaded. Parameter overrides a task needs stay in the task, applied to the loaded spec, not in the model. The reference model adopts such a muscle or tendon model once MuJoCo and mujoco-warp support it natively.
5. **Python composes, XML defines.** `build/compose.py` and `build/utils.py` only attach parts, mirror the right side, and inject the contacts, sensors and keyframes declared in XML. They never create bodies, joints, sites, tendons or actuators.
   - Procedural removal (pruning, as in `build/hand.py`) is the sanctioned way to derive removal-only models (policy 1). It may re-create an element only where MjSpec cannot rename its references; each such case is documented in the code and covered by a test.
   - `build_kwargs` belong to a registration and describe how that registered model is assembled (attachment, root joint, placement). `load_spec()` takes no user `build_kwargs`. Advanced users who need a different assembly register their own recipe (a `ModelRegistration` with its own `build_kwargs`) in their code.
6. **Every non-trivial value has a source.** Parameters come from a cited measurement or a documented fit (see *Provenance*), never from tuning a downstream task.

## The four file roles

Every body part under `myo_sim/models/<part>/assets/` is defined by four files. Never collapse these roles into fewer files, and never add other roles (fragment naming: `docs/wiki/engineering-standards.md`).

| File | Contains | Never contains |
|---|---|---|
| `*_chain.xml` | Bodies, joints, collision and visual geoms, all sites (attachment, via-point, landmark, wrap sidesites), wrapping geoms | Tendons, actuators |
| `*_tendon.xml` | `<spatial>` muscle paths and `<fixed>` tendons (passive joint springs, joint couplings) | Sites, bodies |
| `*_muscle.xml` | `<muscle>` / `<general>` actuators with `gainprm`, `biasprm`, `lengthrange` | Sites, bodies, tendons |
| `*_assets.xml` | Meshes, materials, textures, defaults, and part-wide `<compiler>`, `<option>`, `<size>`, `<equality>`, `<sensor>` | Bodies, actuators |

Cross-part contact pairs live in `myo_sim/models/contacts/` and sensors used only by composed models in `myo_sim/models/sensors/`. Both are injected by `compose.py`.

## Where each kind of change goes

| Change | Edit | Also |
|---|---|---|
| New muscle | Sites in `*_chain.xml`, path in `*_tendon.xml`, actuator in `*_muscle.xml` | Re-check symmetry; downstream action spaces change |
| Muscle parameter refit (force, fibre or slack length, `lengthrange`) | The actuator in `*_muscle.xml` | Provenance row and fit script |
| Passive joint moment or ligament | A one-joint `<fixed>` tendon with `springlength` and polynomial `stiffness` in `*_tendon.xml` | Fit it to the residual (see below) |
| Joint range, axis, mass, inertia | `*_chain.xml` | Re-check `lengthrange`s of muscles crossing the joint |
| Contact pairs between parts | `myo_sim/models/contacts/` | — |
| Something expensive or optional | A new registered model (policy 4) | README section, smoke test |

### Passive joint moments: fit the residual

Measured passive joint moments include the passive force of the muscles crossing the joint. The model's muscles already produce part of that moment, so fit a joint spring only to the remainder:

`spring = measured curve − model muscle passive moment (activation 0, the source's posture)`

Add no spring where the remainder is zero or negative, and document that. Refit every affected spring whenever muscles crossing the joint change.

Passive joint properties (stiffness and damping) are added only where measured data exist and a validation shows they improve fidelity, for example tenodesis or the rest posture of the relaxed hand. Elsewhere, joints keep only what numerical stability needs. Joint impedance depends on many factors that are hard to measure, so each property is fitted once per body part, not per joint by hand. Use MuJoCo's polynomial stiffness and damping (MuJoCo 3.7 or later) for nonlinear curves instead of approximating them.

## Provenance

- **Source data:** cited source data and fit inputs live in `myo_sim/build/data/<topic>.json`, one row per element. Each row holds the citation and DOI, the source curve or table values, the status (`measured`, `transferred` with the stated assumption, or `not added` with the reason), and absolute SI units. JSON is provenance and fit input only: the model never reads it at load time.
- **Fit scripts:** a fit or generator script (`scripts/fit_<topic>.py`) writes the values into the XML, inside a delimited block:

  ```xml
  <!-- BEGIN generated by scripts/fit_passive_moments.py; do not edit by hand -->
  ...
  <!-- END generated -->
  ```

  A test checks that each generated block matches what its script produces. Hand edits go outside these blocks. Fit scripts run only during development and release preparation, and their output is committed: the package never edits MJCF files when it is used.
- **Third-party data:** add a `LICENSE-<SOURCE>.txt` next to the data. Data that does not allow redistribution is cited, never copied. Papers are cited, never committed.
- **Validation data:** keep it in the data file as well, and test against it.
- **Inherited values:** a parameter taken over from an earlier model (e.g. Holzbaur, MoBL-ARMS, Rajagopal) is recorded as `inherited`, naming that model and its version. Every parameter has an entry, not only new ones.
- **Method pages:** every identification or fit method has a wiki page. It covers the data, the target, the loss and bounds, the fit error and the known limitations.
- **Parameter reference:** a script generates a per-part reference from the data files and the XML. It has one row per parameter: element, value, status, source, method page and fit error. A test fails when a parameter in the XML has no provenance row. The reference is regenerated for every release, and the release notes link to it.

## Updating parameters within a release

A new experimental dataset, or one person's measurements, does not need a new release to be used. Functions that take a loaded `MjSpec` and return it with updated parameters (for example refitted passive springs, or a personalised scaling) apply such data at run time, without writing files. When new data should become the reference, the fit script regenerates the XML for the next release, and the old values stay available through older release tags.

## Adding a new muscle (worked example)

The leg (`myo_sim/models/leg/assets/`) is the running example.

**Step 1.** Add the via-point sites to the bodies they anatomically belong to, in `*_chain.xml`.

```xml
<site name="newmuscle-P1_r" pos="0.01 -0.05 0.02"/>
<site name="newmuscle-P2_r" pos="0.02 -0.15 0.01"/>
```

Wrapping sidesites follow the pattern `<WrapObjectName>_site_<musclename>_r`.

**Step 2.** Add the path to `*_tendon.xml`.

```xml
<spatial name="newmuscle_r_tendon" springlength="0.05" class="myolegs_muscle">
    <site site="newmuscle-P1_r"/>
    <geom geom="somewrap_r" sidesite="somewrap_site_newmuscle_r"/>
    <site site="newmuscle-P2_r"/>
</spatial>
```

**Step 3.** Add the actuator to `*_muscle.xml`. Its architecture comes from a cited source and its `lengthrange` from the path, not from tuning.

```xml
<general class="myolegs_muscle" name="newmuscle_r" tendon="newmuscle_r_tendon"
         gainprm="0.5 1.4 800 1 0.05 3.0 10 1.5 1.4 0"
         biasprm="0.5 1.4 800 1 0.05 3.0 10 1.5 1.4 0"
         lengthrange="0.10 0.25"/>
```

**Step 4.** Add a provenance row and run the validation checklist below.

## Adding a wrapping object

A wrapping geom (with the part's wrap class, e.g. `class="myolegs_wrap"`) and its sidesite go in `*_chain.xml`, on the body they belong to. The tendon references both:

```xml
<geom class="myolegs_wrap" name="NewMuscle_wrap_r" type="cylinder"
      pos="0.01 -0.10 0.0" quat="0.707 0.707 0 0" size="0.015 0.03"/>
<site name="NewMuscle_wrap_sidesite_r" pos="0.01 -0.10 0.015"/>
```

## Adding a new body part

1. Create `myo_sim/models/<part>/assets/` with the four role files, named `myo<part>[_r]_<role>.xml`. Use `_r` only for a right-only part that is mirrored at compose time.
2. If the part attaches to an existing one, add an attachment site `<part>_attach_r` to the parent's chain file. Then add a `BuildStrategy`, a builder in `compose.py`, and a `MODEL_REGISTRY` entry.
3. Put contact pairs with other parts in `myo_sim/models/contacts/myo<part>_contacts.xml`.
4. Add the model to the smoke test in `test_sims.py`.
5. Write `myo_sim/models/<part>/README.md` from `docs/wiki/model-readme-template.md`.

## Bilateral symmetry

The right side is the source of truth.

- Never hand-edit a left-side XML. `compose.py` derives the left side with `build_mirrored_child_xml()` and the `MirrorRules` in `build/utils.py`.
- Every name carries `_r` in the source files.
- Bodies with unusual orientations go in `MirrorRules.body_pos_x_mirror_names`.
- `tests/test_mirror_symmetry.py` and the `debug_muscle_*.py` scripts check the result.

## Validation checklist (every model change)

1. **Load and compose:**
   - Every registered model loads.
   - `uv run python -m myo_sim.build.compose --generate` regenerates the shipped XML snapshots.
2. **Fast gate:** see `docs/wiki/testing-guide.md`.
3. **Symmetry:** left/right symmetry tests for the parts you touched.
4. **Physics backends:** CPU and mujoco-warp give the same passive and actuator forces for elements MuJoCo evaluates natively.
5. **Cost:** `mj_step` time and model memory against `dev` for the affected registered models. Put the numbers in the PR. If the cost grows substantially, discuss a variant (policy 4) before merging.
6. **Validation against data:** the provenance data reproduces within the tolerance stated in the data file.
7. **Equivalence references:** if `tests/test_equivalence.py` compares against an external reference model that your change intentionally departs from, say so in the PR. Update the reference expectations instead of weakening the test. The comparison against the musclemimic reference is retired once that reference is synchronised into `myo_sim`.

## Releases and downstream impact

- **Breaking changes:** a model change that alters the number of actuators, joints or sites, or the dynamics, is a breaking change for downstream environments and trained policies.
- **Recording it:** add a `CHANGELOG.md` entry under *Unreleased* that states what changed and what downstream users must update. Append a line to `docs/wiki/log.md`.
- **Releasing it:** such changes ship together in the next release tag. Downstream packages (e.g. MyoSuite) pin a release tag, state which `myo_sim` versions they are compatible with, and move to a new one deliberately. Older behaviour stays available through older tags, never through flags. No release ships without an up-to-date parameter reference (see *Provenance*).
