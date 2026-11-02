# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-02 — Clarify manual muscle length audit workflow
Changed: docs/wiki/testing-guide.md
Why: reframed tests/test_muscle_length_params.py and tests/recalc_from_fl_bounds.py as manual regression/audit infrastructure for muscle XML changes. The helper now documents that its output is a diagnostic report of remaining lengthrange / FL-bound gaps, and that heuristic recalc proposals are not a substitute for model-specific reference/conversion evidence.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
