# Contributing to MyoSim

## Environment setup

```bash
git clone https://github.com/MyoHub/myo_sim.git
cd myo_sim
uv sync --dev
uv run pre-commit install
uv run pytest tests/ -x -n auto \
  --ignore=tests/test_equivalence.py \
  --ignore=tests/test_bimanual_muscle_symmetry.py \
  --ignore=tests/test_muscle_length_params.py
```

## Branch and PR conventions

Fork the repo and push branches to your fork, not to `MyoHub/myo_sim` directly. Open PRs against `dev`, not `main`.

PR body should be plain prose — no structured section headers, no emojis. Describe the problem, what the change does, and any non-obvious tradeoffs. Bullet points are fine; heavy templates are not.

Make one logical change per commit. Run lint before committing:

```bash
uv run ruff check . && uv run ruff format --check .
```

CI will reject lint failures.

## Adding a model part

See `docs/wiki/model-authoring.md` for the full authoring guide. Brief summary:

- XML fragments go in `myo_sim/models/<part>/assets/`.
- Four file types: `*_chain.xml` (bodies, joints, geoms, structural sites), `*_muscle.xml` (actuator/muscle definitions), `*_tendon.xml` (tendon routing), `*_assets.xml` (meshes, materials, defaults).
- The left side of any bilateral part is derived by mirroring the right side in memory via `myo_sim/build/utils.py`. Never create or maintain a hand-edited left-side XML.
- Register public composed models in `myo_sim/build/compose.py` (`MODEL_REGISTRY`).

## Model conversion pipeline

Models are converted from OpenSim (`.osim`) format through three steps (the conversion tooling lives outside this repo):

1. Basic element conversion — bone meshes, joint definitions, muscle paths, and wrapping objects.
2. Moment arm optimization — matching reference moment arms by optimising wrapping object geometry.
3. Muscle force optimization — matching force-length relationships by optimising muscle parameters.

After conversion, any manual adjustments are documented in the relevant model's `README.md`.

## Running tests

```bash
# Fast gate — run before every PR
uv run pytest tests/ -x -n auto \
  --ignore=tests/test_equivalence.py \
  --ignore=tests/test_bimanual_muscle_symmetry.py \
  --ignore=tests/test_muscle_length_params.py

# Pre-release muscle length / FL-bound audit (also run by publish.yml)
uv run pytest tests/test_muscle_length_params.py -v

# Muscle symmetry analysis (standalone scripts, not pytest)
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
```

## Reporting issues

The issue tracker is at `docs/wiki/issue-tracker.md` and mirrored on GitHub at <https://github.com/MyoHub/myo_sim/issues>.

For biomechanical bugs (wrong moment arm, wrong force, incorrect geometry), include:

- The model name and the specific body part or muscle affected.
- Your MuJoCo version (`uv run python -c "import mujoco; print(mujoco.__version__)"`).
- A moment arm plot if you can generate one with `tests/muscle_analysis_utils.py`.
- A reference paper or dataset that shows the expected behaviour.

## Contributing a new model

Open a GitHub issue first describing the model and its anatomical scope before writing any XML. This lets maintainers give early feedback on scope, naming, and where the model fits in the composition pipeline.

PR #73 (contributed left arm, improved muscle wrapping, improved left-right symmetry) is a good example of what an external model contribution looks like and what review will focus on.
