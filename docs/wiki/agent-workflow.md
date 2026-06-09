# Agent Workflow

This is the required workflow for any agent taking on a task in this repository. Follow it in order.

## Before starting

1. Read `docs/wiki/index.md` and the wiki pages relevant to your task.
2. Read `CLAUDE.md`.
3. For every file you plan to edit, read it first.
4. Search before writing — grep for existing implementations before adding new helpers, strategies, or contact files.

## Core work loop

1. Read relevant wiki pages and source files.
2. Implement following `docs/wiki/engineering-standards.md`.
3. Run `uv run ruff check . && uv run ruff format --check .` — fix all violations before committing.
4. Run the test gate: `uv run pytest tests/ -x -n auto --ignore=tests/test_equivalence.py`.
5. Update any wiki page made stale by the change (code > wiki).
6. Append an entry to `docs/wiki/log.md` (see format below).
7. Push to your fork and open a PR to `mm_refactor_mjspec`.

## For model XML changes

After any XML edit, verify the model still loads:

```bash
uv run python -c "import mujoco, myo_sim; mujoco.MjModel.from_xml_path(str(myo_sim.MODELS_DIR / '<part>/<model>.xml'))"
```

For bilateral models composed via MjSpec (arms, fullbody), also verify composition:

```bash
uv run python -m myo_sim.build.compose --model myoarms
```

For muscle or tendon changes, run the relevant symmetry test:

```bash
uv run pytest tests/test_leg_muscle_symmetry.py
uv run pytest tests/test_torso_muscle_symmetry.py
```

## For build/compose.py changes

When adding a new composed model:
1. Add a `BuildStrategy` enum value in `myo_sim/build/compose.py`.
2. Write a builder function.
3. Register it in the `BUILDERS` dict and `MODEL_REGISTRY`.
4. Do not add boolean flags to `ModelRegistration` — use a new strategy instead.
5. Test that the new model compiles: `uv run python -m myo_sim.build.compose --model <name>`.

## Log entry format

Append entries at the top of `docs/wiki/log.md` (most recent first):

```
## YYYY-MM-DD — <short description>
Changed: <file(s)>
Why: <one sentence>
```
