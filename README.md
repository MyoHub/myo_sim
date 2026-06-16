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
| **MyoLeg** | 20 | 80 | <img src="https://user-images.githubusercontent.com/12837145/236839645-e34eab3f-0358-4ca8-8ae0-68a5c08585e4.png" width="160"> | stable |
| **MyoArm** | 27 | 63 | <img alt="myoarm_demo" src="https://github.com/user-attachments/assets/eab77a8b-18af-445c-826f-b050b2a6ed7d" width="160"/> | stable |
| **MyoTorso** (MyoBack) | 18 | 210 | <img src="https://github.com/cherylwang20/myo_sim/blob/cec3ce211a516a8798ed2edf9486a0814a0965da/MyoBack.png?raw=true" width="160"> | stable |
| **MyoHand** | 23 | 39 | <img src="https://user-images.githubusercontent.com/23240128/232323950-39552200-614b-4c73-aab5-8a78daa0f5f3.png" width="160"> | stable |
| **MyoFullBody** | 123 | 416 | <img src="https://github.com/user-attachments/assets/37976636-1952-48a6-83c4-506722db4c82" width="160"/> | stable |
| **MyoFinger** | 4 | 5 | <img src="https://user-images.githubusercontent.com/23240128/232323930-d1721f87-731b-432d-bafd-8c818ab4bbfe.png" width="160"> | legacy |
| **MyoElbow** | 2 | 6 | <img src="https://user-images.githubusercontent.com/23240128/232323890-6a601a82-1d3c-4e12-901c-0fd9cf232691.png" width="160"> | legacy |

Legacy models are not included in the current package registry but remain in the repository.

## Install

```bash
pip install git+https://github.com/MyoHub/myo_sim.git@mm_refactor_mjspec
```

Note: the PyPI package `myo-sim` currently points to an older incompatible version.
Use the git install above until a new release is published.

## Quickstart

```python
import mujoco
import myo_sim

# Load a model by registry name (returns MjModel, MjData)
model, data = myo_sim.load("myolegs")
print(f"Joints: {model.njnt}, Muscles: {model.nu}")

model, data = myo_sim.load("myohand_r")
print(f"Hand — joints: {model.njnt}, muscles: {model.nu}")

# Or get the file path directly (works for myolegs, myotorso, myohand_r)
xml_path = myo_sim.get_xml_path("myohand_r")
model = mujoco.MjModel.from_xml_path(str(xml_path))
```

```python
# Compose a full-body MjSpec model (built at runtime from components)
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
