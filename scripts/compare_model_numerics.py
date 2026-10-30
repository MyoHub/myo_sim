"""Check that two myo_sim source trees build numerically identical models, names aside.

For a rename-only change: every numeric ``MjModel`` array (qpos0, masses, inertias, actuator and tendon
parameters, geom sizes, ...) and the tendon lengths, actuator lengths, passive forces and body positions at
five random postures must be identical; only the names may differ. Each tree is built in its own process::

    python scripts/compare_model_numerics.py <tree before> <tree after> [model ...]

Exit status 1 if any array differs.
"""

from __future__ import annotations

import argparse
import os
import pickle
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

MODELS = (
    "myohand_r",
    "myohands",
    "myoarm_r",
    "myoarms",
    "myotorso_arms",
    "myofullbody",
    "fragment:myohand_r",
    "fragment:myohand_l",
)
# Arrays that hold names or name addresses only.
NAME_ARRAYS = {"names", "names_map", "name_bodyadr"}


def dump(models: list[str], out: Path) -> None:
    """Numeric arrays and five random postures of each model of the importable myo_sim."""
    import mujoco

    import myo_sim

    result = {}
    for name in models:
        if name.startswith("fragment:"):
            model = myo_sim.FRAGMENT_SPEC_BUILDERS[name.split(":", 1)[1]]().compile()
        else:
            model = myo_sim.load_model(name)
        arrays = {}
        for field in dir(model):
            if field.startswith(("_", "name_")) or field in NAME_ARRAYS:
                continue
            value = getattr(model, field, None)
            if isinstance(value, np.ndarray):
                arrays[field] = value.copy()
        data = mujoco.MjData(model)
        rng = np.random.default_rng(1)
        postures = []
        for _ in range(5):
            data.qpos[:] = model.qpos0
            for j in range(model.njnt):
                if model.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE) and model.jnt_limited[j]:
                    data.qpos[model.jnt_qposadr[j]] = rng.uniform(*model.jnt_range[j])
            mujoco.mj_forward(model, data)
            postures.append(
                {
                    k: getattr(data, k).copy()
                    for k in ("ten_length", "actuator_length", "actuator_moment", "qfrc_passive", "xpos")
                }
            )
        result[name] = {"arrays": arrays, "postures": postures}
    out.write_bytes(pickle.dumps(result))


def compare(before: dict, after: dict) -> list[str]:
    problems = []
    for name, b in before.items():
        a = after[name]
        for field, value in b["arrays"].items():
            other = a["arrays"].get(field)
            if other is None or other.shape != value.shape or not np.array_equal(value, other, equal_nan=True):
                problems.append(f"{name}: {field}")
        for i, (pb, pa) in enumerate(zip(b["postures"], a["postures"])):
            problems += [f"{name}: posture {i} {k}" for k in pb if not np.array_equal(pb[k], pa[k])]
    return problems


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("models", nargs="*", default=list(MODELS))
    if sys.argv[1:2] == ["--dump"]:  # child process: dump the importable myo_sim
        dump(sys.argv[3:], Path(sys.argv[2]))
        return
    args = parser.parse_args()
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for i, tree in enumerate((args.before, args.after)):
            out = Path(tmp) / f"{i}.pkl"
            env = {**os.environ, "PYTHONPATH": str(tree.resolve())}
            subprocess.run([sys.executable, __file__, "--dump", str(out), *args.models], env=env, check=True)
            results.append(pickle.loads(out.read_bytes()))
    problems = compare(*results)
    for name in args.models:
        own = [p for p in problems if p.startswith(f"{name}:")]
        print(f"{name}: {'identical numerics' if not own else 'DIFFERS: ' + ', '.join(p.split(': ', 1)[1] for p in own[:10])}")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
