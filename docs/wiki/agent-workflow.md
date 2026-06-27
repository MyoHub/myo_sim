# Agent Workflow

Required workflow for any task in this repository. Follow in order.

## Before starting

1. Read `docs/wiki/index.md` and the wiki pages relevant to your task.
2. Read `CLAUDE.md`.
3. Read every file you plan to edit before touching it.
4. Grep for existing implementations before adding new helpers, strategies, or contact files.

## Core work loop

1. Implement following `docs/wiki/engineering-standards.md`.
2. Run `uv run ruff check . && uv run ruff format --check .` — fix all violations.
3. Run the fast gate from `docs/wiki/testing-guide.md`.
4. Update any wiki page made stale by the change.
5. Append an entry to `docs/wiki/log.md`.
6. Push to your fork and open a PR to `dev`.

## For XML model changes

After any XML edit, verify the model loads:

```bash
uv run python -c "import mujoco, myo_sim; mujoco.MjModel.from_xml_path(str(myo_sim.MODELS_DIR / '<part>/<model>.xml'))"
```

For bilateral or composed models, also verify composition:

```bash
uv run python -m myo_sim.build.compose --model myoarms
```

For muscle or tendon changes, run the relevant symmetry test:

```bash
uv run pytest tests/test_leg_muscle_symmetry.py
uv run pytest tests/test_torso_muscle_symmetry.py
```

## For build/compose.py changes

See `docs/wiki/build-and-composition.md` for the full step-by-step guide.

## Log entry format

Append at the top of `docs/wiki/log.md` (most recent first):

```
## YYYY-MM-DD — <short description>
Changed: <file(s)>
Why: <one sentence>
```
