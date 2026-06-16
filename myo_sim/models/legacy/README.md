# Legacy Models

Models in this directory are **included in the pip package for backwards compatibility only**. Do not build new workflows that depend on these paths or registry names — they will be removed once migration is complete.

## Why legacy?

`myo_sim` is scoped to biological musculoskeletal models. Models here either need proper integration into the active model tree, or belong in a separate package (`myoassist`) for assistive/exoskeletal devices.

## Contents and migration plan

| Directory | Registry names | Tracking issue | Migration target |
|-----------|---------------|---------------|-----------------|
| `elbow/`  | `elbow`, `myoelbow`, `myoelbow_2dof`, `myoelbow_exo` | [#97](https://github.com/MyoHub/myo_sim/issues/97) | Biological variants → `myo_sim/models/elbow/`; softexo variants → `myoassist` |
| `finger/` | `finger`, `myofinger` | [#99](https://github.com/MyoHub/myo_sim/issues/99) | `myo_sim/models/finger/` |
| `osl/`    | `osl`, `myolegs_osl` | [#100](https://github.com/MyoHub/myo_sim/issues/100) | `myoassist` |

## Usage

```python
import myo_sim
# Load by registry name (same as other models)
model, data = myo_sim.load("myoelbow")
model, data = myo_sim.load("myofinger")
model, data = myo_sim.load("osl")

# Or get the path directly
path = myo_sim.get_xml_path("elbow")
```

These registry names are provisional. Once each issue above is resolved, the name will either be reassigned to the new location or removed.

## Meshes to remove when legacy models migrate

The following STL files in `myo_sim/models/meshes/` exist solely to support legacy models and should be deleted once the corresponding model is migrated or removed:

| Mesh | Used by | Remove when |
|------|---------|-------------|
| `ground_jaw.stl` | `legacy/elbow/` (cosmetic body display) | #97 resolved |
| `ground_skull.stl` | `legacy/elbow/` (cosmetic body display) | #97 resolved |
| `ground_spine.stl` | `legacy/elbow/` (cosmetic body display) | #97 resolved |
| `human_lowpoly_norighthand.stl` | `legacy/elbow/` (cosmetic body display) | #97 resolved |
| `torso_lowpoly-v1.stl` | `torso/assets/myotorso_rigid_assets.xml` (OSL rigid torso) | #100 resolved |
