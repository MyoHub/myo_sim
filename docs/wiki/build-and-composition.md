# Build and Composition Pipeline

Reference for agents editing the MjSpec composition pipeline in `myo_sim/build/compose.py` and `myo_sim/build/utils.py`.

## Overview

`build_spec(name: str)` is the core entry point; `build_model(name: str)` is a thin convenience wrapper (`build_spec(name).compile()`) for callers who just want a compiled `MjModel`. Both:

1. Look up `name` in `MODEL_REGISTRY` to get a `ModelRegistration`.
2. Dispatch to the matching builder function via `SPEC_BUILDERS[registration.build_strategy]`.
3. The builder loads component specs, attaches them using MuJoCo's `MjSpec.attach()` API, and injects contact pairs / sensors as needed. Every builder returns an editable `MjSpec` (not a compiled `MjModel`) — `build_spec()` stops there so callers can edit further before compiling; `build_model()` compiles it for you.

`myo_sim.load_spec(name)` / `myo_sim.load_model(name)` / `myo_sim.load(name)` (in `myo_sim/__init__.py`) are the package-level entry points most consumers should use instead of calling `compose.py` directly — they also resolve packaged legacy static XMLs and legacy aliases (`hand`, `myoarm`, ...), which `build_spec`/`build_model` do not.

All file paths are resolved relative to `MODELS_DIR` (the packaged `myo_sim/models/` directory). The `compiler_meshdir` for child XMLs is always set to `MODELS_DIR` (the repo root of the models tree) so that relative mesh paths inside the XML resolve correctly.

## Generating Compiled XML Files

Run the compose CLI with `--generate` to write compiled XML files for the primary shipped assemblies:

```bash
uv run python -m myo_sim.build.compose --generate
```

This generates:

- `myo_sim/models/arm/myoarms.xml`
- `myo_sim/models/torso/myotorso.xml`
- `myo_sim/models/leg/myolegs.xml`
- `myo_sim/models/leg/myolegs26.xml`
- `myo_sim/models/myofullbody.xml`

Generation writes sanitized `MjSpec.to_xml()` output after compiling each spec once for validation. Treat these XML files as generated snapshots for GUI viewing and compatibility; source edits belong in component XML files and `compose.py`.

## MODEL_REGISTRY and BuildStrategy

`BuildStrategy` is a `str` enum. Each value maps to one builder function in `SPEC_BUILDERS`. Every builder returns an `MjSpec`.

| BuildStrategy | Builder function |
|---|---|
| `TORSO_BODY` | `build_torso_body_spec` |
| `TORSO_ARMS` | `build_torso_arms_spec` |
| `ARMS_BODY` | `build_arms_body_spec` |
| `RIGHT_ARM_BODY` | `build_right_arm_body_spec` |
| `RIGHT_HAND` | `build_right_hand_from_arm_spec` |
| `BOTH_HANDS` | `build_both_hands_from_arm_spec` |
| `FULLBODY` | `build_fullbody_spec` |
| `LEGS_BODY` | `build_legs_body_spec` |
| `LEGS26_BODY` | `build_legs26_body_spec` |
| `TORSO_ABDOMEN` | `build_torso_abdomen_spec` |
| `LEGS_ABDOMEN` | `build_legs_abdomen_spec` |

### Registered models

| Name | BuildStrategy | Description |
|---|---|---|
| `myotorso` | `TORSO_BODY` | Torso scaffold with torso muscles |
| `myotorso_abdomen` | `TORSO_ABDOMEN` | Simple abdomen scaffold |
| `myotorso_arms` | `TORSO_ARMS` | Torso with active muscles + right arm + mirrored left arm |
| `myotorso_arm_r` | `TORSO_ARMS` | Torso with active muscles + right arm only |
| `myoarms` | `ARMS_BODY` | Passive anatomical torso scaffold + mirrored arms |
| `myoarm_r` | `RIGHT_ARM_BODY` | Passive anatomical torso scaffold + right arm |
| `myohand_r` | `RIGHT_HAND` | Passive torso scaffold + right hand (pruned from right arm) |
| `myohands` | `BOTH_HANDS` | Passive torso scaffold + right hand + mirrored left hand |
| `myolegs` | `LEGS_BODY` | Passive anatomical torso scaffold + legs |
| `myolegs26` | `LEGS26_BODY` | Passive anatomical torso scaffold + reduced 26-muscle legs (see [MyoLeg26](../../myo_sim/models/leg/README.md#myoleg26-reduced-26-muscle-legs-only)) |
| `myolegs_abdomen` | `LEGS_ABDOMEN` | Minimal abdomen scaffold + legs; free-floating root |
| `myofullbody` | `FULLBODY` | Full body: torso + mirrored arms + legs; free-floating root |

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

2. **Write a builder function** following the existing pattern. It receives a `ModelRegistration` and must return an editable `MjSpec` (not a compiled `MjModel` — compilation happens later, in `build_model()` or by the caller):
   ```python
   def build_my_new_spec(registration: ModelRegistration) -> mujoco.MjSpec:
       torso = load_torso_spec(registration)  # or load_passive_torso_spec
       torso.attach(load_right_arm_spec(), prefix="", suffix="", site=find_site(torso, RIGHT_ARM_ATTACH_SITE))
       # ... additional attachments ...
       return torso
   ```

3. **Register it in `SPEC_BUILDERS`**:
   ```python
   SPEC_BUILDERS = {
       ...
       BuildStrategy.MY_NEW_MODEL: build_my_new_spec,
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

### `add_sensors(spec, sensors_xml)`

Parses a sensors XML file (from `myo_sim/models/sensors/`) and injects supported frame sensors into the `MjSpec` via `spec.add_sensor()`. `myofullbody_sensors.xml` restores the MuscleMimic full-body observation layout by adding body linear and angular velocity sensors before the leg touch sensors are attached.

### `find_body(spec, body_name)` and `find_site(spec, site_name)`

Thin wrappers around `spec.body(name)` / `spec.site(name)` that raise a clear `ValueError` instead of silently propagating `None` when the name is missing (e.g. a typo'd attachment site). Prefer these over calling `spec.body()`/`spec.site()` directly when a missing name should fail loudly at build time.

## Contact Injection Pattern

Contacts are stored in separate XML files under `myo_sim/models/contacts/`:

- `myoarm_contacts.xml` — contact pairs for arm geometries
- `myolegs_contacts.xml` — contact pairs for leg geometries
- `myofullbody_contacts.xml` — cross-body contact pairs for the full-body model

None of these files are referenced via `<include>` from any model XML. They are parsed and injected programmatically at build time by `load_torso_spec()`, which calls `add_contact_pairs()` based on flags in the `ModelRegistration`. This keeps contacts out of standalone XML models (which don't need them) while making them available in all composed models that do.

The `test_contact_paths.py` test enforces this: it verifies that contact XML paths appear only via `ROOT / "contacts" / ...` in `compose.py`, never embedded in `assets/` XMLs.

## Sensor Injection Pattern

Full-body observation compatibility sensors live under `myo_sim/models/sensors/`. They are injected programmatically by `build_fullbody_spec()` after the torso and arms are attached, but before the legs are attached. This keeps the MuscleMimic frame velocity sensors first in `sensordata`, followed by the four leg touch sensors from `myolegs_assets.xml`.

## Known Limitations and Gotchas

**Mirror axis convention.** `mirror_element` negates the z-component of `pos`/`scale`/`ipos` (treating z as the sagittal mirror axis) and negates x/y components of `axis` vectors. Quaternion mirroring negates the i and j components. If a new body uses `euler` orientation, mirroring is only applied for non-`body` elements (the `euler` branch skips bodies). Verify any new body using `euler` visually with `--view` before committing.

**`compiler_meshdir` must point to `MODELS_DIR`.** When building child XMLs from components, the `compiler_meshdir` and `texturedir` must be set to `ROOT` (which equals `MODELS_DIR`). If this is set incorrectly, MuJoCo will fail to resolve relative mesh paths after `MjSpec.from_string()`.

**`find_body` / `find_site` error on missing names.** Prefer these over direct `spec.body(name)` / `spec.site(name)` calls so a typo'd attachment site fails with a clear `ValueError` at build time instead of a confusing `None`-attribute error deeper in `MjSpec.attach()`.

**`balanceinertia = True` on all specs.** Every loaded spec sets `compiler.balanceinertia = True` to prevent inertia-validation errors in OpenSim-converted bodies. Do not omit this when adding new component specs.
