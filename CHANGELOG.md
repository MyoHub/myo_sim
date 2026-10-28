# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

### Changed (breaking)
- **No build options (model policy #140).** `myo_sim.load_spec()` and `myo_sim.build.compose.build_spec()` no longer accept `build_kwargs`; only `inertia_floor` remains. Registrations may set only the composition keys in `compose.COMPOSITION_KWARGS` (`left_arm_strategy`, `add_root_freejoint`, `root_pos`). Previously, user `build_kwargs` were also written into the global `MODEL_REGISTRY` and leaked into every later build in the process.
- **Finger collisions are always on.** The `disable_finger_collisions` build option is removed (no registered model set it). `compose.disable_finger_collision()` stays as a spec-editing helper for downstream tasks calibrated to the pre-#111 convention. The unused `CollisionMode` type is removed.
- **Contact sets are unconditional.** The `include_arm_contacts`, `include_left_arm_contacts`, `include_legs` and `include_fullbody_contacts` options are removed. Each builder injects its contact files after attaching its parts, keeping every pair whose two geoms exist in the model. Pairs are unchanged for every registered model except `myolegs_abdomen`.
- **`myolegs_abdomen` gains the 26 leg-leg contact pairs** of `myolegs_contacts.xml` (thigh, shank and foot pairs between the legs), which it was missing. Its dynamics change when the legs touch; joints, actuators and observations are unchanged.
