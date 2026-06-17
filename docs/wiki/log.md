# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-06-17 — Add torque-driven actuation
Changed: docs/wiki/build-and-composition.md (new "Torque-driven actuation" section), docs/wiki/repository-map.md (file/table updates)
Why: Added per-part torque-motor actuator files (`myo<part>_torque.xml`) and a `build_model(name, actuation=...)` parameter supporting muscle/torque/mixed builds (e.g. torque torso+arms with muscle legs), modeled after MyoHub/myosuite's `torque` branch myoskeleton.

## 2026-06-09 — Initial wiki creation
Changed: docs/wiki/ (all files created)
Why: Established docs/wiki/ as the persistent knowledge layer for agents and contributors, following the myosuite pattern.
