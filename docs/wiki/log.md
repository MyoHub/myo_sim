# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-04 — LTpT_T12_l mirrored
Changed: myo_sim/models/torso/assets/myotorso_chain.xml, tests/test_torso_muscle_symmetry.py, myo_sim/models/torso/README.md, CHANGELOG.md
Why: the left T12 longissimus had the right muscle's path points, as in the source model; they now mirror the right ones.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
