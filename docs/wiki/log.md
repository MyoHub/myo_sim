# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-06-21 — Scope remaining generic default class names
Changed: `myo_sim/models/arm/assets/myoarm_r_*.xml`, `myo_sim/models/leg/assets/myolegs_*.xml`, `myo_sim/models/legacy/osl/assets/myolegs_osl_*.xml`, `docs/wiki/engineering-standards.md`
Why: `myoarm_r_assets.xml` left `arm`, `muscle`, `elbow`, `wrist`, `fingers`, `upperarm_muscle`, and `forearm_muscle` unscoped, and `myolegs_assets.xml` mixed three naming conventions (`myolegs_`, `myoleg_`, `myo_leg_`) plus a bare `coll` class — both violating the naming rule this same wiki page states. Confirmed real, not cosmetic: PR #113's bilateral `myoarms` build hit exactly these unscoped names as duplicate-class collisions requiring a sanitization workaround. Renamed every nested class to a consistent `<part>_<role>` form and updated the wiki's example (the old text cited the now-fixed `myoleg_wrap`/`myo_leg_marker` as compliant). Re-verified: `uv run ruff check .`/`format --check .` clean, full test suite (89 tests) passes, `debug_muscle_leg.py` 101/114 (same 13 pre-existing knee-actuator discrepancies as before), `debug_muscle_torso.py` 208/208, `debug_muscle_bimanual.py` 215/215 — no regressions.

## 2026-06-20 — Document compose XML generation
Changed: `docs/wiki/build-and-composition.md`
Why: Added the `compose.py --generate` workflow and target XML outputs for primary composed model assemblies.

## 2026-06-09 — Initial wiki creation
Changed: docs/wiki/ (all files created)
Why: Established docs/wiki/ as the persistent knowledge layer for agents and contributors, following the myosuite pattern.
