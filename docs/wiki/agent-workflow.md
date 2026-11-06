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

`log.md` is a changelog, not a lab notebook. Hard limits:

- **`Why:` is one sentence.** Not one paragraph, not "one sentence plus supporting detail" — one sentence. If the change needs more explanation than that, it belongs in a PR description, a model README's "Known limitations"/"Changelog" section, or a code comment near the change — not in this file.
- **No investigation narrative.** Don't log what you tried and ruled out, intermediate numbers, or how you debugged something. Log the conclusion and the file it landed in.
- **No paths outside the shipped package.** Never reference `sandbox/`, scratch scripts, or anything not committed to the repo — a reader without that untracked/local content can't follow the reference, and it won't exist in their clone. If a change was validated with an exploratory script, say what was validated, not where the script lives.
- **One entry per logical change**, not one entry per work session. If a session touches five files for one reason, that's one entry.
