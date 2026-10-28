# Wiki Log

Append-only maintenance log, most recent first.

## 2026-10-28 — Generic disable_collisions() post-processing function
Changed: myo_sim/postprocess.py (new), myo_sim/__init__.py, myo_sim/build/compose.py, tests/test_postprocess.py (new), docs/wiki/model-authoring.md, CHANGELOG.md
Why: Model policy 4 prefers generic post-processing functions over per-model variants. `myo_sim.disable_collisions()` turns off all collisions or a pattern-selected subset on any loaded `MjSpec`; `disable_finger_collision()` keeps its signature and result and delegates to it.

## 2026-10-28 — Remove build options and make contact sets unconditional
Changed: myo_sim/build/compose.py, myo_sim/__init__.py, tests/test_no_opt_in.py (new), docs/wiki/build-and-composition.md, CHANGELOG.md
Why: Model policy (#140): no feature flags; `build_kwargs` describe composition only. User `build_kwargs`, `disable_finger_collisions` and the four contact-set options are removed; contact pairs follow the parts present. `myolegs_abdomen` gains the leg-leg contacts it lacked.

## 2026-10-28 — Model policy: non-native muscle and tendon models
Changed: docs/wiki/model-authoring.md
Why: muscle or tendon models that MuJoCo and mujoco-warp cannot run natively (plugins, custom step loops) ship as registered variants (policy 4) with parameters generated from provenance JSON into `*_muscle.xml`; task overrides stay in the task; the reference model adopts them once both engines support them natively.

## 2026-10-28 — Model policy: review round
Changed: docs/wiki/model-authoring.md
Why: load_spec takes no user build_kwargs (custom assemblies register their own recipe); generic post-processing functions instead of per-model simplification variants; fit scripts run only at development time; parameter updates within a release through MjSpec functions; passive joint properties only where data and a validation support them; downstream packages state compatible versions; the musclemimic equivalence comparison retires once synchronised; target structure without hand pruning.

## 2026-10-27 — Model policy and maintenance rules
Changed: docs/wiki/model-authoring.md, engineering-standards.md, index.md, repository-map.md, agent-workflow.md, CLAUDE.md, CHANGELOG.md
Why: state one reference model per part, no feature flags, variants as registered models, provenance and validation rules.
