# MyoSim

[![CI](https://github.com/MyoHub/myo_sim/actions/workflows/ci.yml/badge.svg?branch=mm_refactor_mjspec)](https://github.com/MyoHub/myo_sim/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-%3E%3D3.10-blue.svg)](pyproject.toml)

MyoSim is the MuJoCo musculoskeletal model library used by [MyoSuite](https://github.com/facebookresearch/myoSuite).
It provides anatomically detailed XML models of the human arm, leg, torso, and hand,
plus a Python package for loading and composing them.

## Models

| Model | DoF | Muscles | Status |
|---|---|---|---|
| MyoLeg | 20 | 80 | stable |
| MyoArm | 27 | 63 | stable |
| MyoTorso (MyoBack) | 18 | 210 | stable |
| MyoHand | 23 | 39 | stable |

MyoFinger, MyoElbow, and MyoOSL no longer have top-level entry points in the current package. They are legacy models not included in the registry.

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

# Load a model by registry name
model, data = myo_sim.load("myolegs")
print(f"Joints: {model.njnt}, Muscles: {model.nu}")

# Or get the path directly
xml_path = myo_sim.get_xml_path("myotorso")
model = mujoco.MjModel.from_xml_path(str(xml_path))
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

If you find this repository useful in your research, please cite:

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

## License

Apache 2.0 — see [LICENSE](LICENSE).
