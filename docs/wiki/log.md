# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-04 — Knee coupled ranges
Changed: myo_sim/models/leg/assets/myolegs_chain.xml, tests/test_knee_coupling_ranges.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: five coordinates the knee drives left their declared ranges along their own coupling polynomials.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
