"""Time myo_sim.diff: one forward pass (lengths, moment arms, velocities, forces, generalised forces) and
one forward and backward pass, for several batch sizes.

    uv run --extra torch python scripts/benchmark_diff.py --models myoarm_r myofullbody --batch 1 512 4096

Defaults to the CPU with 8 threads. On a shared GPU machine, take the machine's GPU lock before a timed
``--device cuda`` run, as the lab's GPU-lock convention asks.
"""

from __future__ import annotations

import argparse
import resource
import statistics
import sys
import time
import warnings
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))

from diff_utils import load, random_state  # noqa: E402

from myo_sim.diff import MuscleTendonModel  # noqa: E402


def _time(fn, device: str, repeats: int) -> float:
    """Median wall time of ``fn`` in milliseconds, after one warm-up call."""
    fn()
    times = []
    for _ in range(repeats):
        if device == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        fn()
        if device == "cuda":
            torch.cuda.synchronize()
        times.append(1e3 * (time.perf_counter() - t0))
    return statistics.median(times)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--models", nargs="+", default=["myohand_r", "myoarm_r", "myolegs", "myofullbody"])
    p.add_argument("--batch", nargs="+", type=int, default=[1, 512, 4096])
    p.add_argument("--dtype", choices=["float32", "float64"], default="float32")
    p.add_argument("--device", default="cpu")
    p.add_argument("--threads", type=int, default=8)
    p.add_argument("--repeats", type=int, default=5)
    p.add_argument("--compile", action="store_true", help="torch.compile the forward pass (inductor)")
    args = p.parse_args()
    torch.set_num_threads(args.threads)
    dtype = getattr(torch, args.dtype)
    warnings.filterwarnings("ignore")
    print(f"torch {torch.__version__}, device {args.device}, {args.dtype}, {args.threads} threads, compile={args.compile}")
    print(f"{'model':<12} {'batch':>6} {'forward ms':>11} {'fwd+bwd ms':>11} {'us/state fwd':>13} {'peak RSS GB':>12}")
    for name in args.models:
        m = load(name)
        mt = MuscleTendonModel(m, device=args.device, dtype=dtype)
        for n in args.batch:
            q, v, a, u = (torch.tensor(x, device=args.device, dtype=dtype) for x in random_state(m, n, seed=0))
            w = torch.tensor(np.random.default_rng(0).normal(0, 1, (n, m.nv)), device=args.device, dtype=dtype)

            def fwd(q=q, v=v, a=a, u=u):
                out = mt.forward(q, v, a, u)
                return out.qfrc_actuator, out.actuator_force, out.ten_length

            step = torch.compile(fwd) if args.compile else fwd

            def run_fwd(step=step):
                with torch.no_grad():
                    step()

            def run_bwd(q=q, v=v, a=a, u=u, w=w):
                qg, ag = q.clone().requires_grad_(), a.clone().requires_grad_()
                out = mt.forward(qg, v, ag, u)
                torch.autograd.grad((out.qfrc_actuator * w).sum(), (qg, ag))

            tf = _time(run_fwd, args.device, args.repeats)
            tb = _time(run_bwd, args.device, args.repeats)
            rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6
            print(f"{name:<12} {n:>6} {tf:>11.2f} {tb:>11.2f} {1e3 * tf / n:>13.2f} {rss:>12.2f}")


if __name__ == "__main__":
    main()
