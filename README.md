# MyoSim

[![CI](https://github.com/MyoHub/myo_sim/actions/workflows/ci.yml/badge.svg?branch=mm_refactor_mjspec)](https://github.com/MyoHub/myo_sim/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-%3E%3D3.10-blue.svg)](pyproject.toml)

MyoSim is the MuJoCo musculoskeletal model library used by [MyoSuite](https://github.com/facebookresearch/myoSuite).
It provides anatomically detailed XML models of the human arm, leg, torso, and hand,
plus a Python package for loading and composing them.

## Models

| Model | DoF | Muscles | Preview | Version |
|---|---:|---:|---|---|
| [**MyoLeg**](myo_sim/models/leg/README.md) | 29 | 80 | <img src="https://user-images.githubusercontent.com/12837145/236839645-e34eab3f-0358-4ca8-8ae0-68a5c08585e4.png" width="160"> | stable |
| [**MyoArm**](myo_sim/models/arm/README.md) | 38 | 63 | <img alt="myoarm_demo" src="https://github.com/user-attachments/assets/eab77a8b-18af-445c-826f-b050b2a6ed7d" width="160"/> | stable |
| [**MyoTorso**](myo_sim/models/torso/README.md) (MyoBack) | 18 | 210 | <img src="https://github.com/cherylwang20/myo_sim/blob/cec3ce211a516a8798ed2edf9486a0814a0965da/MyoBack.png?raw=true" width="160"> | stable |
| [**MyoHand**](myo_sim/models/arm/README.md) | 23 | 39 | <img src="https://user-images.githubusercontent.com/23240128/232323950-39552200-614b-4c73-aab5-8a78daa0f5f3.png" width="160"> | stable |
| [**MyoFullBody**](docs/wiki/build-and-composition.md) | 123 | 416 | <img src="https://github.com/user-attachments/assets/37976636-1952-48a6-83c4-506722db4c82" width="160"/> | stable |
| [**MyoFinger**](myo_sim/models/legacy/README.md) | 4 | 5 | <img src="https://user-images.githubusercontent.com/23240128/232323930-d1721f87-731b-432d-bafd-8c818ab4bbfe.png" width="160"> | legacy |
| [**MyoElbow**](myo_sim/models/legacy/README.md) | 2 | 6 | <img src="https://user-images.githubusercontent.com/23240128/232323890-6a601a82-1d3c-4e12-901c-0fd9cf232691.png" width="160"> | legacy |

Legacy models ship in the pip package under `myo_sim/models/legacy/` and load via registry names such as `myoelbow`, `myofinger`, and `osl`. They are maintained for backwards compatibility only — see `myo_sim/models/legacy/README.md`.

## Buildable Models

The following registered models can be compiled with `myo_sim.build.compose.build_model("<name>")`:

- `myotorso` — torso scaffold with torso muscles.
- `myotorso_abdomen` — simple abdomen scaffold.
- `myotorso_arm_r` — torso with the right arm only.
- `myotorso_arms` — torso with the right arm plus a mirrored-left arm.
- `myoarm_r` — passive anatomical torso scaffold with the right arm.
- `myoarms` — passive anatomical torso scaffold with mirrored arms.
- `myohand_r` — passive anatomical torso scaffold with the right hand derived from the pruned right arm.
- `myohands` — passive anatomical torso scaffold with right and mirrored-left hands derived from pruned arms.
- `myolegs` — passive anatomical torso scaffold with legs.
- `myolegs_abdomen` — simple abdomen scaffold with legs.
- `myofullbody` — full body with torso, mirrored arms, and legs.

To generate binary MuJoCo model files for the primary composed assemblies, run:

```bash
uv run python -m myo_sim.build.compose --generate
```

This writes `.mjb` files for `myoarms`, `myotorso`, `myolegs`, and `myofullbody` under `myo_sim/models/`. These are GUI-friendly binary artifacts when loaded with a compatible MuJoCo version, but they are version-dependent and not editable source files. For source edits, update the component XML files and `myo_sim/build/compose.py`, then regenerate.

## Install

```bash
pip install git+https://github.com/MyoHub/myo_sim.git@dev
```

Note: the PyPI package `myo-sim` currently points to an older incompatible version.
Use the git install above until a new release is published.

## Quickstart

```python
import mujoco
import myo_sim

# Load a model by registry name
model, data = myo_sim.load("myolegs")
print(f"Joints: {model.njnt}, Muscles: {model.nu}")

# Or compose another registered model
model, data = myo_sim.load("myotorso")

# Legacy models (backwards compatibility)
model, data = myo_sim.load("myoelbow")
```

```python
# Compose a full-body MjSpec model
from myo_sim.build.compose import build_model
model = build_model("myofullbody")
print(f"Full body — joints: {model.njnt}, muscles: {model.nu}")
```

## Development

```bash
git clone https://github.com/MyoHub/myo_sim.git
cd myo_sim
uv sync --dev
uv run pytest tests/ -x -n auto --ignore=tests/test_equivalence.py
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full contributor guide.

## Citation

If you find this repository useful in your research, please cite the following works:

```bibtex
@misc{MyoSuite2022,
  author    = {Caggiano, Vittorio and Wang, Huawei and Durandau, Guillaume
               and Sartori, Massimo and Kumar, Vikash},
  title     = {{MyoSuite}: A contact-rich simulation suite for musculoskeletal motor control},
  year      = {2022},
  doi       = {10.48550/ARXIV.2205.13600},
  url       = {https://arxiv.org/abs/2205.13600},
}
```

```bibtex
@article{Li2026MuscleMimic,
  title={Towards Embodied AI with MuscleMimic: Unlocking full-body musculoskeletal motor learning at scale},
  author={Li, Chengkun and Wang, Cheryl and Ziliotto, Bianca and Simos, Merkourios and Kovecses, Jozsef and Durandau, Guillaume and Mathis, Alexander},
  journal={arXiv preprint arXiv:2603.25544},
  year={2026}
}
```

## License

Apache 2.0 — see [LICENSE](LICENSE).
