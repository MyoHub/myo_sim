# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`myo_sim` is a packaged library of MuJoCo musculoskeletal (MSK) XML model definitions used by [MyoSuite](https://github.com/facebookresearch/myoSuite). The primary artifacts are `.xml` model files and referenced assets under `myo_sim/models/`, plus Python helpers for registry lookup and MjSpec composition.

## Running Tests

```bash
# Run the full model-loading smoke test
uv run pytest test_sims.py

# Run a single test class
uv run pytest test_sims.py::TestSims::test_sims

# Run a muscle-symmetry analysis script (not pytest — runs as a standalone script)
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
cd tests && uv run python debug_muscle_bimanual.py
```

Requires `mujoco` and `numpy` (plus `matplotlib` for analysis plots). Package metadata lives in `pyproject.toml`.

## Repository Structure

```
myo_sim/              # Installable Python package
  models/             # Packaged XML model tree and assets
    <model>/          # One directory per body segment / assembly
      <model>.xml     # Top-level MuJoCo model (entry point)
      assets/         # Sub-XMLs included via <include> or <compiler meshdir>
        *_assets.xml  # Mesh/texture/material declarations
        *_chain.xml   # Kinematic chain (bodies, joints, geoms)
    meshes/           # Shared .stl mesh files
    scene/            # Scene wrapper XMLs
    contacts/         # Shared contact-pair XMLs for MjSpec composition
  build/              # MjSpec composition helpers and model registry
tests/                # Muscle analysis scripts + utilities
test_sims.py          # Pytest smoke test: loads every model via mujoco.MjModel
```

Key packaged model directories include `arm`, `leg`, `torso`, `head`, `body`, `contacts`, `meshes`, `scene`, and `textures`.

## Model Architecture

Models are composed via MuJoCo's `<include>` and `<compiler meshdir>` mechanisms:

- **Assets XMLs** (`*_assets.xml`) declare meshes, materials, tendons.
- **Body/Chain XMLs** (`*_body.xml`, `*_chain.xml`) define the kinematic tree, joints, muscles, and contact geometries.
- **Top-level XMLs** define directly loadable static models where a part is self-contained; torso-dependent parts are composed through `myo_sim/build/compose.py`.
- **MjSpec models** compose mirrored arms, legs, contacts, and full-body variants in `myo_sim/build/compose.py`.

All meshes live in `myo_sim/models/meshes/` and are shared across models. `scene/` XMLs wrap individual models with environment assets for rendering.

## Design Principles (issue #75)

The architecture separates concerns into four layers, implemented through file-naming conventions within `myo_sim/models/<part>/assets/` and the `myo_sim/build/` package:

1. **Kinematics** (`*_chain.xml`) — immutable skeleton: bodies, joints, and global structural sites. Defined once per body part; never duplicated across assemblies.
2. **Actuation** (`*_muscle.xml`, `*_tendon.xml`) — body-part-local actuation: muscles, tendons, wrapping surfaces, equality constraints, and local via/wrapping sites.
3. **Assets** (`*_assets.xml`) — mesh, material, and texture declarations shared within a body part.
4. **Build** (`myo_sim/build/compose.py`) — deterministic composition pipeline that assembles chains, actuation, and contacts into full models using MjSpec.

When adding or editing model elements, respect this separation: structural geometry belongs in chain files, actuation belongs in muscle/tendon files, and cross-part composition is handled in `myo_sim/build/`.

## Naming Conventions

- Body parts use the `Myo` prefix (e.g., `MyoLeg`, `MyoArm`).
- Bilateral structures use `_r` / `_l` suffixes (right / left) for joints, muscles, bodies, and sites.
- Muscles follow OpenSim naming (e.g., `gaslat_r`, `psoas_l`).
- Sites used for attachment/endpoint markers are placed in group 3 (inactive by default).

## Muscle Analysis Utilities (`tests/muscle_analysis_utils.py`)

Provides helpers for validating MSK model quality:
- `compute_moment_arm_curve` / `compute_force_length_curve` — sweep joint angles and record muscle properties.
- `parse_model_joint_equalities` / `apply_eq_constraints` — resolve MuJoCo joint equality constraints (polynomial coupling between joints) before computing forward kinematics.
- `plot_pair` — compare left/right muscle pairs visually.

Analysis scripts output plots to `tests/output/muscle_analysis/`.

## Model Conversion Pipeline

Models are converted from OpenSim (`.osim`) format via a three-step pipeline (not in this repo):
1. Basic element conversion (bones, joints, muscle paths, wrapping objects)
2. Moment arm optimization (wrapping geometry tuning)
3. Muscle force optimization (force-length parameter fitting)

Post-conversion manual adjustments are documented in each model's `README.md`.

## Development Workflow

- Always use `uv run`, not `python`.
- Always run `uv run pytest -n 8` before creating a PR.
- Run `uv run pre-commit install` after cloning to enable pre-commit hooks (ruff, uv-lock, kernel-analyzer).
- Prefer running individual tests rather than the full test suite to improve iteration speed.

## Commits and PRs

- PR body should be plain, concise prose. Describe the problem, what the change does, and any non-obvious tradeoffs. Bullet points listing changes are fine, but avoid section headers, structured templates, and emojis.
- PR and commit messages are rendered on GitHub, so don't hard-wrap them at 88 columns. Let each sentence flow on one line.
- Push branches to your own fork, not to the MyoHub/myo_sim repo directly.
- Amending commits is fine before a PR has reviewers looking at it. Once a PR is under review, use new commits so reviewers can see what changed.
- When responding to PR review comments: reply to each comment individually confirming what you did (or why you didn't), resolve addressed threads, and add a summary comment on the PR covering what was applied and what was intentionally skipped.

## Code Style

- Line length limit is 128 characters. Docstring length limit is 100 characters.
- Prefer targeted, efficient tests over exhaustive edge-case coverage.
