# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-04 — Pelvis counted once
Changed: myo_sim/models/torso/assets/myotorso_chain.xml, tests/test_pelvis_mass_once.py, tests/test_mass_convention.py, tests/test_chest_ownership.py, docs/wiki/testing-guide.md, CHANGELOG.md
Why: the sacrum carried the lumbar model's pelvis mass while the leg chain brings its own pelvis, so the models with legs counted it twice.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
