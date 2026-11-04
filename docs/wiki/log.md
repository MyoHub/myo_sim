# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-04 — Knee joint centre
Changed: myo_sim/models/leg/assets/myolegs_chain.xml, tests/test_knee_joint_centre.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: the knee's rotations were anchored at the tibia's origin; they now sit at Rajagopal's walker_knee joint frame, as in the source model.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
