# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-10-28 — RI2 path, Kamper finger damping, pinch and tenodesis validation after McFarland et al. 2023
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml`, `myoarm_r_chain.xml`, `myo_sim/build/data/passive_joint_moments.json`, `scripts/fit_passive_moments.py`, `tests/test_passive_thenar.py`, `scripts/validate_hand_function.py`, `myo_sim/build/data/hand_function.json`, `myo_sim/build/hand_function.py`, `tests/test_hand_pinch.py`, `tests/test_hand_tenodesis.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/hand-function-validation.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`, `myo_sim/models/arm/README.md`, `myo_sim/models/arm/README_hand.md`
Why: RI2's path jumped by 7 mm because its insertion entered a wrap cylinder; finger damping was three to six times Kamper's measurement; the McFarland et al. 2023 validation found both. The pinch and tenodesis protocols and tests come with it (the script, table and loader are identical in Vittorio-Caggiano/myo_sim#2).

## 2026-10-27 — Thenar muscles and passive joint moments in the arm's own XML; springs fitted to the residual
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml`, `myoarm_r_muscle.xml` (generated blocks), removed `myoarm_r_thenar_*.xml` and `myoarm_r_passive*_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/__init__.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_muscles.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: One reference model per part, no flags: the thenar muscles and all measured and assumed passive rows are in the right-arm files (script-written blocks), and every spring is fitted to the source curve minus the model's own passive muscle moment, so nothing is counted twice; the `thenar_muscles` / `passive_moments` flags, the separate fragments and `subject_scale` (later PRs) are removed.

## 2026-10-27 — Thenar muscles and passive joint moments as arm XML fragments, on by default
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml`, `myoarm_r_thenar_tendon.xml`, `myoarm_r_thenar_muscle.xml`, `myoarm_r_passive_tendon.xml`, `myoarm_r_passive_assumed_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/build/hand.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_fragments.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/model-authoring.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: Review of the first version: the muscles and springs were added from Python instead of the part's XML files. Sites now live in the chain, tendons and actuators in fragments written by scripts from the JSON tables, merged and mirrored by compose.py; `thenar_muscles` / `passive_moments` default to on (+3 actuators per hand) and opt out to the previous model; MuJoCo floor raised to 3.7 for the polynomial tendon stiffness.
