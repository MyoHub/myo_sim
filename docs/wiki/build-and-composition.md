# Build and Composition Pipeline

Reference for agents editing the MjSpec composition pipeline in `myo_sim/build/compose.py` and `myo_sim/build/utils.py`.

## Overview

`build_model(name: str)` is the single public entry point. It:

1. Looks up `name` in `MODEL_REGISTRY` to get a `ModelRegistration`.
2. Dispatches to the matching builder function via `BUILDERS[registration.build_strategy]`.
3. The builder loads component specs, attaches them using MuJoCo's `MjSpec.attach()` API, injects contact pairs, and calls `spec.compile()` to return an `MjModel`.

All file paths are resolved relative to `MODELS_DIR` (the packaged `myo_sim/models/` directory). The `compiler_meshdir` for child XMLs is always set to `MODELS_DIR` (the repo root of the models tree) so that relative mesh paths inside the XML resolve correctly.

## Generating Binary MJB Files

Run the compose CLI with `--generate` to write binary `.mjb` files for the primary shipped assemblies:

```bash
uv run python -m myo_sim.build.compose --generate
```

This generates:

- `myo_sim/models/arm/myoarms.mjb`
- `myo_sim/models/torso/myotorso.mjb`
- `myo_sim/models/leg/myolegs.mjb`
- `myo_sim/models/myofullbody.mjb`

Generation compiles each registered MjSpec-composed model and writes the binary model with `mujoco.mj_saveModel()`. Treat these `.mjb` files as loadable generated artifacts, not editable source files; make source changes in the component XML files and `compose.py`, then regenerate.

## MODEL_REGISTRY and BuildStrategy

`BuildStrategy` is a `str` enum. Each value maps to one builder function in `BUILDERS`.

| BuildStrategy | Builder function |
|---|---|
| `TORSO_ARMS` | `build_torso_arms_model` |
| `ARMS_BODY` | `build_arms_body_model` |
| `RIGHT_ARM_BODY` | `build_right_arm_body_model` |
| `RIGHT_HAND` | `build_right_hand_from_arm_model` |
| `BOTH_HANDS` | `build_both_hands_from_arm_model` |
| `FULLBODY` | `build_fullbody_model` |
| `LEGS_BODY` | `build_legs_body_model` |
| `TORSO_ABDOMEN` | `build_torso_abdomen_model` |
| `LEGS_ABDOMEN` | `build_legs_abdomen_model` |

### Registered models

| Name | BuildStrategy | Description |
|---|---|---|
| `myotorso_arms` | `TORSO_ARMS` | Torso with active muscles + right arm + mirrored left arm |
| `myotorso_arm_r` | `TORSO_ARMS` | Torso with active muscles + right arm only |
| `myoarms` | `ARMS_BODY` | Passive anatomical torso scaffold + mirrored arms |
| `myoarm_r` | `RIGHT_ARM_BODY` | Passive anatomical torso scaffold + right arm |
| `myohand_r` | `RIGHT_HAND` | Passive torso scaffold + right hand (pruned from right arm) |
| `myohands` | `BOTH_HANDS` | Passive torso scaffold + right hand + mirrored left hand |
| `myofullbody` | `FULLBODY` | Full body: torso + mirrored arms + legs; free-floating root |
| `myolegs` | `LEGS_BODY` | Passive anatomical torso scaffold + legs |
| `myotorso_abdomen` | `TORSO_ABDOMEN` | Simple abdomen scaffold |
| `myolegs_abdomen` | `LEGS_ABDOMEN` | Minimal abdomen scaffold + legs; free-floating root |

`ModelRegistration` fields that control composition:

- `left_arm_strategy`: `"mirror_right_to_left"` generates the left arm by mirroring; `"none"` omits it.
- `include_left_arm_contacts`: whether left-arm contact pairs from `myoarm_contacts.xml` are injected.
- `include_legs`: whether `myolegs_contacts.xml` pairs are injected.
- `include_fullbody_contacts`: whether `myofullbody_contacts.xml` pairs are injected.
- `add_root_freejoint`: adds a free joint to the root body (needed for locomotion models).
- `root_pos`: initial position of the root body in world space.
- `mirror_rules`: a `MirrorRules` instance controlling how names and geometry are transformed during mirroring.

## How to Add a New Composed Model

1. **Add a `BuildStrategy` enum value** in `compose.py`:
   ```python
   class BuildStrategy(str, Enum):
       ...
       MY_NEW_MODEL = "my_new_model"
   ```

2. **Write a builder function** following the existing pattern. It receives a `ModelRegistration` and must return an `MjModel`:
   ```python
   def build_my_new_model(registration: ModelRegistration):
       torso = load_torso_spec(registration)  # or load_passive_torso_spec
       torso.attach(load_right_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
       # ... additional attachments ...
       return torso.compile()
   ```

3. **Register it in `BUILDERS`**:
   ```python
   BUILDERS = {
       ...
       BuildStrategy.MY_NEW_MODEL: build_my_new_model,
   }
   ```

4. **Add a `ModelRegistration` to `MODEL_REGISTRY`**:
   ```python
   MODEL_REGISTRY["my_new_model"] = ModelRegistration(
       name="my_new_model",
       build_strategy=BuildStrategy.MY_NEW_MODEL,
       left_arm_strategy=LEFT_ARM_STRATEGY_MIRROR_RIGHT,
       description="...",
       include_left_arm_contacts=True,
   )
   ```

5. **Verify** it loads and compiles:
   ```bash
   uv run python -m myo_sim.build.compose --model my_new_model
   ```

6. Add a test entry in `tests/test_build_registry.py` per `docs/wiki/testing-guide.md`.

## Key Utilities in utils.py

### `MjSpec.attach(...)`

Builders attach child `MjSpec` objects directly with MuJoCo's `spec.attach(...)` API. Use the `site=` argument when attaching to a named site already declared in the parent XML (e.g., `arm_attach_r`, `arm_attach_l`). Use the `frame=` argument when attaching to a programmatically created frame (e.g., `root_body.add_frame(...)` for leg attachment).

### `build_child_xml_from_components(...)`

Assembles a standalone MJCF XML string from four separate component files (assets, tendons, muscles, chain). The resulting XML is passed to `mujoco.MjSpec.from_string()`. The function:
- Merges the asset XML children (skipping `<compiler>`, `<size>`, `<option>`).
- Appends tendon and muscle XML children.
- Wraps the chain XML inside a root `<body>` with an attachment site.

Use this for right-arm and legs, where the component files are already right-side files.

### `build_mirrored_child_xml(...)`

Same as `build_child_xml_from_components` but applies `mirror_element()` to every element before appending. Use this to produce the left-arm XML from the right-arm source files. All name references, positions, axes, and quaternions are transformed according to the provided `MirrorRules`.

### `MirrorRules`

A frozen dataclass that controls how `mirror_element()` transforms the right-side XML tree:

| Field | Type | What it controls |
|---|---|---|
| `mirrored_material` | `str` | Material name to substitute for `MatSkin` on the left side (default `"MatSkin_l"`) |
| `common_names` | `frozenset[str]` | Asset names that are shared between left and right and must not be renamed |
| `body_pos_x_mirror_names` | `frozenset[str]` | Body names whose `pos` x-component is negated instead of z-component |
| `lowercase_geom_prefixes` | `tuple[str, ...]` | Geom name prefixes that should be lowercased after mirroring |
| `replacements` | `tuple[tuple[str, str], ...]` | Substring replacements applied to name references |
| `prefix_replacements` | `tuple[tuple[str, str], ...]` | Prefix substitutions applied to name references |
| `mirror_file_attributes` | `bool` | Whether to apply mirroring to `file` attributes (mesh file names) |

Default `MirrorRules()` handles the standard `_r` → `_l` suffix replacement. Override specific fields only when the default logic produces incorrect results for a new body part.

### `add_contact_pairs(spec, contacts_xml, include_pair=None)`

Parses a contacts XML file (from `myo_sim/models/contacts/`) and injects each `<pair>` element into the `MjSpec` via `spec.add_pair()`. The optional `include_pair` callable receives the XML element and returns `False` to skip a pair (used to exclude left-arm pairs when `include_left_arm_contacts=False`).

### `find_body(spec, body_name)` and `find_site(spec, site_name)`

API-version-safe lookups. They try `spec.find_body()`, then `spec.find()`, then `spec.body()` in order to accommodate MuJoCo Python API differences across versions. Do not replace these with direct attribute access — the API surface has changed between MuJoCo releases and these wrappers ensure forward compatibility.

## Contact Injection Pattern

Contacts are stored in separate XML files under `myo_sim/models/contacts/`:

- `myoarm_contacts.xml` — contact pairs for arm geometries
- `myolegs_contacts.xml` — contact pairs for leg geometries
- `myofullbody_contacts.xml` — cross-body contact pairs for the full-body model

None of these files are referenced via `<include>` from any model XML. They are parsed and injected programmatically at build time by `load_torso_spec()`, which calls `add_contact_pairs()` based on flags in the `ModelRegistration`. This keeps contacts out of standalone XML models (which don't need them) while making them available in all composed models that do.

The `test_contact_paths.py` test enforces this: it verifies that contact XML paths appear only via `ROOT / "contacts" / ...` in `compose.py`, never embedded in `assets/` XMLs.

## Known Limitations and Gotchas

**Mirror axis convention.** `mirror_element` negates the z-component of `pos`/`scale`/`ipos` (treating z as the sagittal mirror axis) and negates x/y components of `axis` vectors. Quaternion mirroring negates the i and j components. If a new body uses `euler` orientation, mirroring is only applied for non-`body` elements (the `euler` branch skips bodies). Verify any new body using `euler` visually with `--view` before committing.

**`compiler_meshdir` must point to `MODELS_DIR`.** When building child XMLs from components, the `compiler_meshdir` and `texturedir` must be set to `ROOT` (which equals `MODELS_DIR`). If this is set incorrectly, MuJoCo will fail to resolve relative mesh paths after `MjSpec.from_string()`.

**`find_body` / `find_site` fallback logic.** Do not replace these with direct `spec.body(name)` or `spec.find("body", name)` calls. The fallback chain is needed for MuJoCo API version portability.

**`balanceinertia = True` on all specs.** Every loaded spec sets `compiler.balanceinertia = True` to prevent inertia-validation errors in OpenSim-converted bodies. Do not omit this when adding new component specs.
