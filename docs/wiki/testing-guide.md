# Testing Guide

Map of the test suite for agents working in `myo_sim`.

## Test Categories

| File | What it tests | When to run |
|---|---|---|
| `test_sims.py` | Static XML models load without error via `mujoco.MjModel.from_xml_path` | Always before PR |
| `test_build_registry.py` | `BuildStrategy` enum fields, `MODEL_REGISTRY` completeness | After `compose.py` changes |
| `test_contact_paths.py` | Contacts are centralized in `contacts/`, not embedded in `assets/` | After XML or `compose.py` changes |
| `test_mirror_symmetry.py` | Body positions and joint axes are bilateral reflections across the sagittal plane | After arm chain or mirror rule changes |
| `test_chest_ownership.py` | `chest_r` in torso chain, not arm chain; no `chest_l` in bilateral models | After chain XML changes |
| `test_fragment_registry.py` | `FragmentRegistry` names and paths are valid | After `__init__.py` changes |
| `test_passive_torso_build.py` | Passive torso scaffold builds without error | After torso XML or compose changes |
| `test_legs_abdomen_build.py` | `myolegs_abdomen` model builds without error | After legs or abdomen changes |
| `test_leg_muscle_symmetry.py` | Leg muscle moment arms are symmetric between left and right | After `myolegs` XML changes |
| `test_torso_muscle_symmetry.py` | Torso muscle moment arms are symmetric between left and right | After `myotorso` XML changes |
| `test_equivalence.py` | `myofullbody` joint and actuator counts match musclemimic reference spec | Manual only — needs musclemimic reference values |
| `test_bimanual_muscle_symmetry.py` | Bimanual arm muscle moment arms are symmetric | Manual only |

## Fast Gate (Run Before Every PR)

Run this before opening or updating a PR. It skips the two manual-only tests:

```bash
uv run pytest tests/ -x -n auto \
  --ignore=tests/test_equivalence.py \
  --ignore=tests/test_bimanual_muscle_symmetry.py
```

The `-x` flag stops on first failure. The `-n auto` flag runs in parallel across available CPUs.

To run a single test file:

```bash
uv run pytest tests/test_build_registry.py -v
```

## How to Add a Regression Test for a New Model

**New static XML entry point** (a standalone model loaded via `mujoco.MjModel.from_xml_path`): add its path (relative to `MODELS_DIR`) to the `model_paths` list in `tests/test_sims.py`.

**New composed model** (built via `build_model()`): add an entry to the `expected` dict in `test_every_registered_model_has_build_strategy` in `tests/test_build_registry.py`. If the model is bilateral (has left and right arms), also add its name to the `@pytest.mark.parametrize` decorator in `tests/test_mirror_symmetry.py`.

**New muscle symmetry check**: use `discover_bilateral_suffix_pairs()` from `tests/muscle_symmetry_checks.py` to find left/right muscle pairs automatically. Do not hardcode pairs in the test — that will cause the test to silently miss new muscles.

## Analysis Scripts (Not Pytest)

These scripts produce diagnostic plots and are not collected by pytest. Run them manually when investigating muscle mechanics.

```bash
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
cd tests && uv run python debug_muscle_bimanual.py
```

Output goes to `tests/output/muscle_analysis/`. The scripts use helpers from `muscle_analysis_utils.py`:

- `compute_moment_arm_curve` / `compute_force_length_curve` — sweep joint angles and record muscle properties.
- `parse_model_joint_equalities` / `apply_eq_constraints` — resolve MuJoCo joint equality constraints before computing forward kinematics.
- `plot_pair` — compare left/right muscle pairs visually.

## Test Infrastructure Notes

`pyproject.toml` configures pytest with `testpaths = [".", "tests"]` and `python_files = ["test_sims.py", "test_*.py"]`. The test runner discovers both `test_sims.py` at the repo root and all `test_*.py` files under `tests/`.

Tests that call `build_model()` exercise the full MjSpec composition pipeline and require `mujoco >= 3.0`. Tests that parse XML directly (e.g., `test_chest_ownership.py`, `test_contact_paths.py`) have no MuJoCo runtime dependency and run faster.
