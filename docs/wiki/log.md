# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-10-27 — Thenar muscles and passive joint moments in the arm's own XML; springs fitted to the residual
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml`, `myoarm_r_muscle.xml` (generated blocks), removed `myoarm_r_thenar_*.xml` and `myoarm_r_passive*_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/__init__.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_muscles.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: One reference model per part, no flags: the thenar muscles and all measured and assumed passive rows are in the right-arm files (script-written blocks), and every spring is fitted to the source curve minus the model's own passive muscle moment, so nothing is counted twice; the `thenar_muscles` / `passive_moments` flags, the separate fragments and `subject_scale` (later PRs) are removed.

## 2026-10-27 — Thenar muscles and passive joint moments as arm XML fragments, on by default
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml`, `myoarm_r_thenar_tendon.xml`, `myoarm_r_thenar_muscle.xml`, `myoarm_r_passive_tendon.xml`, `myoarm_r_passive_assumed_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/build/hand.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_fragments.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/model-authoring.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: Review of the first version: the muscles and springs were added from Python instead of the part's XML files. Sites now live in the chain, tendons and actuators in fragments written by scripts from the JSON tables, merged and mirrored by compose.py; `thenar_muscles` / `passive_moments` default to on (+3 actuators per hand) and opt out to the previous model; MuJoCo floor raised to 3.7 for the polynomial tendon stiffness.
