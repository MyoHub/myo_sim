# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-06-16 — myohand_r promoted to static XML artefact
Changed: `myo_sim/models/hand/myohand_r.xml` added; `myo_sim/__init__.py` (`_FRAGMENT_CATALOG`, `_COMPOSED_MODELS`, `_FRAGMENT_SPEC_BUILDERS`); `myo_sim/build/compose.py` (`generate_hand_xml()`, `--generate` CLI); `.github/workflows/ci.yml` (`generated-xml` job); `docs/wiki/repository-map.md`; `docs/wiki/build-and-composition.md`.
Why: myosuite needs a stable file path to the hand model after pip install. The generated XML is numerically equivalent to the in-memory spec (verified: qpos/qvel/actuator_force atol=1e-6 over 100 steps with random activations).

---

## 2026-06-09 — Initial wiki creation
Changed: docs/wiki/ (all files created)
Why: Established docs/wiki/ as the persistent knowledge layer for agents and contributors, following the myosuite pattern.
