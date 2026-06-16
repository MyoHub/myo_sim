# CLAUDE.md

`myo_sim` is a packaged library of MuJoCo musculoskeletal XML model definitions used by [MyoSuite](https://github.com/facebookresearch/myoSuite).

Read `docs/wiki/index.md` before making any substantial change.

## Quickstart

```bash
# Install (also installs pre-commit hooks)
uv sync
uv run pre-commit install

# Run tests
uv run pytest tests/ -x -n auto --ignore=tests/test_equivalence.py

# Compose a bilateral model
uv run python -m myo_sim.build.compose --model myoarms

# Regenerate myohand_r.xml after editing arm XML (do not edit it directly)
uv run python -m myo_sim.build.compose --generate

# Run a muscle-symmetry analysis script (standalone, not pytest)
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
cd tests && uv run python debug_muscle_bimanual.py
```

## More detail

Everything else — model architecture, file-naming conventions, XML standards, Python style, PR rules, naming conventions, build/composition pipeline, and muscle analysis utilities — is documented in `docs/wiki/`. Start with `docs/wiki/index.md`.
