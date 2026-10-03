## 2026-10-03 — Correct the prone neck-release benchmark
Changed: scripts/import_neck_source.py, scripts/validate_neck_dynamics.py, tests/test_neck_dynamics.py, myo_sim/models/head/assets/myohead_chain.xml, myo_sim/models/head/validation/, myo_sim/models/head/README.md, docs/images/validation/, docs/wiki/testing-guide.md
Why: Restore source passive mechanics, use anatomical fiber estimates and the published neck muscle correspondence, then calibrate graded feedback on nine participants and evaluate eight separately; keep measurements and acceptance bounds fixed.

## 2026-10-03 — Compare neck release dynamics with experimental trajectories
Changed: scripts/validate_neck_dynamics.py, tests/test_neck_dynamics.py, myo_sim/models/head/validation/, myo_sim/models/head/README.md, docs/images/validation/neck_dynamics.png, docs/wiki/testing-guide.md
Why: Add a checksum-pinned literature benchmark and unfit forward simulations; distinguish numerical correctness from failed prone biological agreement without altering production model parameters.

## 2026-10-03 — Validate the opt-in muscular neck in MyoFullBody
Changed: myo_sim/models/head/, myo_sim/build/head.py, myo_sim/build/compose.py, scripts/import_neck_source.py, scripts/validate_neck.py, tests/test_neck_validation.py, tests/test_build_registry.py, docs/wiki/build-and-composition.md
Why: Preserve the default full-body model while reproducing source kinematics and published neutral isometric neck strength in an opt-in assembly.
