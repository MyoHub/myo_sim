# Repository Map

This page describes how `myo_sim` is organized so agents can quickly find the right place to edit. The repository is a packaged MuJoCo XML model library: most source artifacts are MJCF XML fragments, shared meshes, and small Python utilities for validation and MjSpec-based composition.

## Top-Level Layout

- `myo_sim/` - Installable Python package with registry helpers, MjSpec code, and packaged model data.
- `myo_sim/models/` - Model directories and shared assets packaged into wheels.
- `myo_sim/models/<model>/assets/` - Model-local MJCF fragments for assets, kinematic chains, tendons, muscles, and related model pieces.
- `myo_sim/models/contacts/` - Shared contact-pair XML files used by MjSpec composition.
- `myo_sim/mjspec/` - Python prototypes for composing models with `mujoco.MjSpec`.
- `myo_sim/models/meshes/` - Shared `.stl` mesh files referenced by model XMLs.
- `myo_sim/models/textures/` - Shared texture assets.
- `myo_sim/models/scene/` - Scene wrappers, floors, cameras, lighting, and display assets.
- `tests/` - Regression tests and model analysis utilities.
- `test_sims.py` - Smoke tests that load models with MuJoCo.
- `wiki/` - Human- and agent-readable repository references.

Assistive devices do not belong in `myo_sim`. Keep this repository focused on biological musculoskeletal models; move device models, attachments, and device-specific assets to `myoassist`.

## Model Directory Structure

Each model directory should keep top-level entry points separate from reusable fragments.

```text
myo_sim/models/<model>/
├── <model>.xml              # top-level model entry point
└── assets/
    ├── *_assets.xml         # meshes, textures, materials, defaults, global assets
    ├── *_chain.xml          # bodies, joints, geoms, sites used for composition
    ├── *_tendon.xml         # tendon definitions
    ├── *_muscle.xml         # actuator/muscle definitions
    └── other local fragments
```

Top-level model XMLs are for complete loadable models. Asset fragments should be reusable and should avoid pulling in unrelated assembled-model behavior.

## XML Fragment Conventions

- Put mesh, material, texture, default, and compiler-level declarations in `*_assets.xml` when they are local to a model.
- Put bodies, joints, collision geoms, visual geoms, and structural sites in `*_chain.xml`.
- Put tendon definitions in `*_tendon.xml`.
- Put muscle actuator definitions in `*_muscle.xml`.
- Keep cross-part attachments explicit with named sites or frames.
- Avoid file duplication as much as possible. Prefer one canonical source file plus composition, mirroring, or parsing helpers over copied XML variants.
- Avoid duplicating skeleton definitions across model variants. Prefer one canonical chain fragment and compose from it.
- Use MjSpec mirroring for parts that are perfectly symmetric. Do not create and maintain manual left/right XML copies when a deterministic mirror rule can produce the counterpart.
- Keep generated or conversion-derived XML stable unless the change is intentional and verified against model loading or analysis tests.

## Contact Conventions

Contacts that are owned by a composed model should live in `contacts/`, not in a single part's `assets/` directory.

- `myo_sim/models/contacts/myoarm_contacts.xml` - Arm-to-torso and arm-to-arm contact pairs.
- `myo_sim/models/contacts/myolegs_contacts.xml` - Leg-to-leg contact pairs.
- `myo_sim/models/contacts/myofullbody_contacts.xml` - Cross-part full-body contact pairs, such as arm-to-leg pairs.

MjSpec composition should inject these contacts with `add_contact_pairs()` in `myo_sim/mjspec/utils.py`. Static XML entry points should only include contacts when that model directly owns the complete contact context. Cross-part contacts should not be hidden inside part-local XML fragments.

## MjSpec Structure

`myo_sim/mjspec/prototype_mjspec_attach.py` is the current MjSpec composition entry point. It registers named composed models and builds them from reusable torso, arm, leg, and hand specs.

Use `myo_sim/mjspec/utils.py` for shared XML parsing, mirroring, attachment, and contact-pair helpers. Use `myo_sim/mjspec/hand.py` for hand-specific pruning logic.

MjSpec should be the preferred place for deterministic composition logic: attaching parts, mirroring symmetric parts, injecting contact pairs, and deriving simple variants from canonical fragments. Static XML should remain the source for canonical part definitions, not a place to duplicate every assembled variant.

## MjSpec Model Naming

Registry keys should be clean user-facing model names. They should describe the intended model, not every implementation detail.

Preferred names:

- `myoarms` - Canonical bilateral arms model.
- `myofullbody` - Canonical full-body model.
- `myohand_r` - Canonical right-hand model.
- `myohands` - Canonical bilateral hands model.
- `myotorso_arms` - Prototype torso plus both arms.
- `myotorso_arm_r` - Prototype torso plus right arm.

Rules:

- Keep the `myo` prefix for model registry keys.
- Use short anatomical nouns: `torso`, `arm`, `arms`, `leg`, `legs`, `hand`, `hands`, `fullbody`.
- Use `_r` and `_l` only for unilateral models.
- Use plural nouns when both sides are present.
- Do not encode implementation details like `mirrored`, `pruned`, `from_arm`, or `base` in registry keys. Put those details in `description`.
- Keep the registry key and `ModelRegistration.name` identical.
- Prefer stable names for anything users or tests may call directly.

## Naming Conventions

- Model names use the `Myo` prefix in human-facing names and `myo` in file or registry names.
- Bilateral structures use `_r` and `_l` suffixes for right and left joints, muscles, bodies, geoms, and sites.
- Muscles follow OpenSim-style names where possible, such as `gaslat_r` or `psoas_l`.
- Attachment sites should include the body part and side when relevant, such as `arm_attach_r` or `arm_root_attach_l`.
- Sites used as markers or attachment points should remain visually unobtrusive and grouped consistently with the surrounding model.
- File names should be lowercase and descriptive. Prefer existing suffixes like `_assets.xml`, `_chain.xml`, `_tendon.xml`, and `_muscle.xml`.

## Adding Files

- Add new mesh files to `myo_sim/models/meshes/` unless a model has a strong reason to own a private mesh directory.
- Add model-local XML fragments under `myo_sim/models/<model>/assets/`.
- Add complete loadable model entry points under the relevant `myo_sim/models/<model>/` directory.
- Add cross-part contact definitions under `myo_sim/models/contacts/`.
- Add MjSpec composition helpers under `myo_sim/mjspec/`.
- Add regression tests under `tests/` when the change affects composition, naming, paths, parsing, or expected model behavior.
- Update the relevant model `README.md` when changing model behavior, conversion assumptions, or manual adjustments.
- Do not add assistive devices, exoskeletons, orthoses, prostheses, controllers, or device-specific assets to this repository. Put them in `myoassist` and keep only biological model interfaces here if needed.

## Parsing And Editing Conventions

- Prefer structured XML parsing for generated transformations, mirroring, contact injection, or broad XML edits.
- Keep manual XML edits localized and easy to review.
- Do not use ad hoc string manipulation for changes that depend on XML structure.
- Preserve existing numeric formatting unless the changed values are part of the intended edit.
- Keep comments short and useful; explain ownership or composition boundaries when they are not obvious.
- When moving files, update both static MJCF includes and any MjSpec path constants.

## Testing And Verification

- Use `uv run` for Python commands.
- For static model-loading changes, run focused MuJoCo load tests first, then broader tests as needed.
- For MjSpec composition changes, run the relevant registered model from `myo_sim/mjspec/prototype_mjspec_attach.py`.
- Add focused regression tests for path, registry, or parsing conventions so future moves fail loudly.

Examples:

```bash
uv run pytest tests/test_contact_paths.py
uv run python -m myo_sim.mjspec.prototype_mjspec_attach --model myoarms
uv run python -m myo_sim.mjspec.prototype_mjspec_attach --model myofullbody
```

## Navigation Heuristics

- If adding or changing a complete model entry point, start in the relevant model directory.
- If adding or changing reusable geometry, joints, bodies, or sites, start in `myo_sim/models/<model>/assets/*_chain.xml`.
- If adding or changing muscles, start in `myo_sim/models/<model>/assets/*_muscle.xml`.
- If adding or changing tendons, start in `myo_sim/models/<model>/assets/*_tendon.xml`.
- If adding or changing materials, meshes, or texture declarations, start in `myo_sim/models/<model>/assets/*_assets.xml`.
- If adding or changing cross-part contacts, start in `myo_sim/models/contacts/` and then check MjSpec injection.
- If adding or changing MjSpec composition, start in `myo_sim/mjspec/prototype_mjspec_attach.py` and `myo_sim/mjspec/utils.py`.
- If adding or changing assistive devices, work in `myoassist`, not `myo_sim`.
- If changing model quality or symmetry analysis, inspect `tests/muscle_analysis_utils.py` and the relevant `tests/debug_muscle_*.py` script.

## Core Invariant

Structural kinematics, local actuation, cross-part interfaces, and composed model builds should stay separate.

- Kinematics answer: what bodies, joints, geoms, and structural sites exist?
- Parts answer: what muscles, tendons, local wrapping objects, and local constraints belong to a body part?
- Interfaces answer: how do parts attach or contact each other?
- Build code answers: how are reusable pieces assembled into a loadable model?
- Assistive-device code answers: how external devices interact with biological models, and belongs in `myoassist`.

When a change crosses one of these boundaries, make the boundary explicit in the file location, name, or `description`.
