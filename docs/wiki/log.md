# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-03 — Standing keyframe, 0.2.4
Changed: scripts/generate_standing_keyframes.py, myo_sim/build/keyframes.py, myo_sim/build/data/standing_keyframes.json, myo_sim/build/compose.py, myo_sim/__init__.py, tests/test_standing_keyframe.py, docs/wiki/standing-keyframe.md, docs/wiki/index.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, docs/wiki/build-and-composition.md, CHANGELOG.md, pyproject.toml, uv.lock
Why: qpos0 of the free-standing models violates the knee couplings and overlaps the arms with the thorax, so they had no clean start pose; the keyframe adds one without changing any model parameter.

## 2026-11-03 — Composition fixes from #150
Changed: myo_sim/build/compose.py, myo_sim/build/utils.py, tests/test_build_registry.py, tests/test_contact_paths.py, tests/test_legs_abdomen_build.py, tests/test_mirror_symmetry.py, tests/test_equivalence.py, scripts/render_composed_models.py, README.md, CHANGELOG.md, docs/wiki/build-and-composition.md, docs/wiki/engineering-standards.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md
Why: the build_kwargs leak, the pairs kept by disable_finger_collisions, the abdomen variants' contacts and heading, the mirrored cameras, and docs and a test that still used the removed build_model().

## 2026-10-28 — BIClong no longer wraps its elbow sphere
Changed: myo_sim/models/arm/assets/myoarm_r_tendon.xml, tests/test_model_stability.py, tests/test_tendon_path_continuity.py, tests/data/myofullbody_biclong_elbow_graze.json, docs/wiki/testing-guide.md, myo_sim/models/arm/README.md, CHANGELOG.md
Why: The wrap only acted past the elbow extension limit, where it gave NaN BIClong lengths (MuJoCo < 3.16) and 15-20 mm length jumps in random-activation rollouts of myofullbody.

## 2026-10-28 — CPU vs mujoco-warp tendon length parity test
Changed: tests/test_warp_parity.py, docs/wiki/testing-guide.md, myo_sim/models/arm/README.md
Why: EDC2 and the other tendons with a sidesite inside their wrap geom differed between CPU MuJoCo and mujoco-warp < 3.16 by up to about 3 mm (float32 inside-wrap fallback, google-deepmind/mujoco_warp#1713), so every hand and arm tendon is now checked on both backends.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
