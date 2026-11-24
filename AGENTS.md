# AGENTS.md

`myo_sim` provides musculoskeletal model definitions, anatomical composition and reusable model physics for [MyoSuite](https://github.com/MyoHub/myosuite) and other consumers.

Read `docs/wiki/index.md` before making any substantial change.

## Repository ownership

These rules apply before implementation and during agent review. The matching [MyoSuite review rules](https://github.com/MyoHub/myosuite/blob/dev/REVIEW.md) define the consumer side (introduced by [MyoSuite #902](https://github.com/MyoHub/myosuite/pull/902)).

- Own anatomical MJCF/assets and assembly, geometry/segment scaling, mass/inertia, muscle/tendon parameters, model-only calibration and mechanics (including fatigue and joint couplings), and dataset-free structural model checks.
- Keep tasks, observations, rewards, resets, CPU/mjlab environment wiring, measurement/source adapters, subject-fitting orchestration, motion/EMG analysis and experiment evaluation in MyoSuite. Method-specific trainers/networks stay in its top-level `training/<method>/`, outside the installable package.
- Anatomical composition belongs here; MyoSuite's `ModelBuilder` and scene recipes consume these APIs to assemble task objects, targets and fixtures. A subject-fitting workflow can live in MyoSuite while delegating anatomical/physical transforms here.
- Search both repositories and relevant dependency APIs before adding helpers or solvers. Cite existing symbols considered/reused and justify new logic; remove superseded code within a compatible migration. Preserve different numerical objectives unless equivalence is demonstrated.
- Export reusable public APIs without importing MyoSuite, its environments or trainers. Consumers must link prerequisite library PRs and require a released version providing the API; do not copy algorithms or hide duplicates behind fallbacks.
- Every applicable PR and review summary must state the owner, reused symbols/paths, necessary new logic, code removed/simplified (or why none), compatibility and checks run/not run. Review the actual diff and imports. Missing ownership evidence requires revision before approval; unavailable cross-repo checks remain unverified. Passing lint/tests or an advisory review hook does not establish ownership approval.

## Quickstart

```bash
# Install
uv sync

# Run tests
uv run pytest tests/ -x -n auto --ignore=tests/test_equivalence.py

# Compose a bilateral model
uv run python -m myo_sim.build.compose --model myoarms

# Run a muscle-symmetry analysis script (standalone, not pytest)
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
cd tests && uv run python debug_muscle_bimanual.py
```

## Commits

- **Never add an AI assistant (e.g. Claude, Anthropic) as a commit co-author** — via `Co-Authored-By` trailers or otherwise. This repo's CLA check requires every commit author/co-author to have signed the CLA, and AI tools cannot sign it, so AI co-author trailers break the check.

## More detail

Everything else — model architecture, file-naming conventions, XML standards, Python style, PR rules, naming conventions, build/composition pipeline, and muscle analysis utilities — is documented in `docs/wiki/`. Start with `docs/wiki/index.md`.
