# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-03 — Composition fixes from #150
Changed: myo_sim/build/compose.py, myo_sim/build/utils.py, tests/test_build_registry.py, tests/test_contact_paths.py, tests/test_legs_abdomen_build.py, tests/test_mirror_symmetry.py, tests/test_equivalence.py, scripts/render_composed_models.py, README.md, CHANGELOG.md, docs/wiki/build-and-composition.md, docs/wiki/engineering-standards.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md
Why: the build_kwargs leak, the pairs kept by disable_finger_collisions, the abdomen variants' contacts and heading, the mirrored cameras, and docs and a test that still used the removed build_model().

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
