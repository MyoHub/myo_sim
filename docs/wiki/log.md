# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-03 — OpenSim pose comparison
Changed: scripts/validate_opensim_scaling.py, tests/test_opensim_pose_comparison.py, tests/data/opensim_scaling/pose_fixture.json, tests/data/opensim_scaling/pose_reference.json, tests/data/opensim_scaling/pose_tolerances.json, docs/wiki/model-scaling.md, docs/wiki/testing-guide.md, docs/wiki/repository-map.md, docs/wiki/log.md
Why: compare moment arms, fibre lengths, tendon forces and torques of six leg muscles with OpenSim over a pose grid, as MyoSuite PR #564 did, with its bounds re-derived for myolegs and scale_spec.

## 2026-11-03 — Model scaling
Changed: myo_sim/scaling.py, myo_sim/opensim_scales.py, myo_sim/scaling_checks.py, myo_sim/__init__.py, tests/test_scaling.py, tests/test_opensim_comparison.py, tests/data/opensim_scaling/, scripts/validate_opensim_scaling.py, docs/wiki/model-scaling.md, docs/wiki/index.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, README.md, CHANGELOG.md
Why: scale a model to known factors in myo_sim itself, compared with OpenSim's ModelScaler on Rajagopal2016. No model changes.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
