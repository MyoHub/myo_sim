# Wiki Log

Append-only maintenance log, most recent first.

## 2026-10-29 — Merge dev; mass convention test covers the neck chain
Changed: tests/test_mass_convention.py, tests/test_head_build.py, myo_sim/build/mass.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: dev brought #138's mass test, which measured the head share on the `head` body and expected 84.3 kg. With the neck in `myofullbody`, the share is taken over the `myohead_base` subtree and the total is 84.0 kg; part models are checked for #138's rigid head. The neck mass check moves from `test_head_build.py` into this test, and `audit_mass` uses the neck chain when present.

## 2026-10-28 — Head and neck mass, cervical inertias and neck armature
Changed: myo_sim/models/head/assets/myohead_chain.xml, myo_sim/models/head/assets/myohead_assets.xml, myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, tests/test_head_build.py, tests/test_chest_ownership.py, myo_sim/models/head/README.md, docs/validation/neck/README.md, docs/wiki/testing-guide.md, CHANGELOG.md
Why: the neck raised body mass by 5.3 kg while the torso mass still contained the head. Head and neck now carry the de Leva share (6.94 %) and the torso drops to 14.897 kg as in #138; cervical inertias were 0.04 kg·m² placeholders, about 100 times too large, and come from a cylinder-slice approximation; with them the neck rang at 2 ms, so its joints use the torso's armature of 0.01. The commented-out rigid-head joints are removed.

## 2026-10-28 — Make the muscular neck part of myofullbody
Changed: myo_sim/build/compose.py, tests/test_head_build.py, tests/test_build_registry.py, tests/test_sims.py, tests/test_chest_ownership.py, README.md, myo_sim/models/head/README.md, docs/validation/neck/README.md, docs/wiki/build-and-composition.md, scripts/render_composed_models.py, docs/images/models/myofullbody.png, CHANGELOG.md (new)
Why: Model policy (#140): realism improvements overwrite the reference model and there are no feature flags. The neck costs +14-17 % `mj_step` time on `myofullbody`, below the threshold for a separate variant, so the `include_neck` flag and `myofullbody_neck` are removed. Neck actuators are appended last to keep existing action indices. The part models, `myotorso`, `myotorso_arms` and `myotorso_arm_r` included, keep the rigid head until the simple-versus-full model strategy is decided.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.

## 2026-10-04 — Align myohead muscle naming with other body parts
Changed: myo_sim/models/head/assets/, tests/test_head_build.py, myo_sim/models/head/README.md, docs/validation/neck/README.md, docs/wiki/repository-map.md
Why: Drop the `myohead_` actuator prefix, use `{muscle}_r_tendon` and `{muscle}-Pn_r` site names, scope materials as `myohead_matskin`, and clarify that 24 hinges are the coupled realization of 6 independent cervical DoFs.

## 2026-10-03 — Remove neck-preview scene backgrounds
Changed: scripts/render_composed_models.py, docs/images/models/myohead.png, docs/images/models/myofullbody_neck.png
Why: Render the two neck previews with transparent backgrounds and hide scene props/reflections without changing model assets.

## 2026-10-03 — Show the muscular neck in the model gallery
Changed: README.md, myo_sim/models/head/README.md, myo_sim/models/head/assets/myohead_assets.xml, docs/images/models/myohead.png, docs/images/models/myofullbody_neck.png
Why: Match the torso's tendon rendering width, document both full-body build variants and display the optional neck muscles without changing default full-body behavior.

## 2026-10-03 — Keep validation outside the installable package
Changed: docs/validation/neck/, myo_sim/models/head/, tests/test_head_build.py, scripts/, docs/wiki/testing-guide.md, docs/wiki/repository-map.md, docs/wiki/build-and-composition.md
Why: Retain a concise literature report and comparison figure in documentation, remove reproducibility archives/tooling, and keep only model assets, required licensing and runtime composition tests in the package changes.

## 2026-10-03 — Correct the prone neck-release benchmark
Changed: scripts/import_neck_source.py, scripts/validate_neck_dynamics.py, tests/test_neck_dynamics.py, myo_sim/models/head/assets/myohead_chain.xml, myo_sim/models/head/validation/, myo_sim/models/head/README.md, docs/images/validation/, docs/wiki/testing-guide.md
Why: Restore source passive mechanics, use anatomical fiber estimates and the published neck muscle correspondence, then calibrate graded feedback on nine participants and evaluate eight separately; keep measurements and acceptance bounds fixed.

## 2026-10-03 — Compare neck release dynamics with experimental trajectories
Changed: scripts/validate_neck_dynamics.py, tests/test_neck_dynamics.py, myo_sim/models/head/validation/, myo_sim/models/head/README.md, docs/images/validation/neck_dynamics.png, docs/wiki/testing-guide.md
Why: Add a checksum-pinned literature benchmark and unfit forward simulations; distinguish numerical correctness from failed prone biological agreement without altering production model parameters.

## 2026-10-03 — Validate the opt-in muscular neck in MyoFullBody
Changed: myo_sim/models/head/, myo_sim/build/head.py, myo_sim/build/compose.py, scripts/import_neck_source.py, scripts/validate_neck.py, tests/test_neck_validation.py, tests/test_build_registry.py, docs/wiki/build-and-composition.md
Why: Preserve the default full-body model while reproducing source kinematics and published neutral isometric neck strength in an opt-in assembly.
