# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-10-28 — Remove build options and make contact sets unconditional
Changed: `myo_sim/build/compose.py`, `myo_sim/__init__.py`, `tests/test_no_opt_in.py` (new), `docs/wiki/build-and-composition.md`, `CHANGELOG.md` (new), `docs/wiki/log.md` (new)
Why: Model policy (#140): no feature flags; `build_kwargs` describe composition only. User `build_kwargs`, `disable_finger_collisions` and the four contact-set options are removed; contact pairs follow the parts present. `myolegs_abdomen` gains the leg-leg contacts it lacked.
