# Differentiable muscle-tendon model (`myo_sim.diff`)

**Status: steps 1 and 2 of the plan below are in place** (tendon geometry, moment arms, muscle forces,
activation dynamics, with parity tests). Performance work and the first users are still to come. The
discussion happens on the pull request that adds it (#165).

`myo_sim.diff` is an optional PyTorch module, built from a compiled `mujoco.MjModel` of any myo_sim
model or composition. It computes what MuJoCo's `mj_forward` computes for tendons and actuators, as
batched, differentiable functions of `qpos`, `qvel`, `act` and `ctrl`.

## Why

MuJoCo's tendon lengths, moment arms, muscle forces and activation dynamics are exact. But they can't be
differentiated from PyTorch, and they're only available for states MuJoCo has simulated. Several uses
need them as differentiable, batched functions on states MuJoCo hasn't seen:

- **Model-based learning.** World-model methods such as SuperMimic (Liao et al., arXiv:2610.28480)
  backpropagate through imagined rollouts. On muscle bodies, the imagined muscle lengths, velocities and
  forces have to come from a differentiable replica of the model.
- **Learning only what isn't known.** Given exact muscle forces and moment arms, a world model only has
  to learn the contacts and constraint forces.
- **Physiological costs** such as co-contraction need per-muscle, per-joint torques with gradients.
- **Fitting parameters by gradient.** The fitting scripts here use finite differences or derivative-free
  search. Gradients make those fits cheaper, also for scaled bodies.
- **Compliant tendons** (#164) integrate fibre length from the musculotendon length and velocity.

## Install and use

```bash
pip install "myo-sim[torch]"        # or: uv sync --extra torch
```

```python
import torch, myo_sim
from myo_sim.diff import MuscleTendonModel

m = myo_sim.load_model("myoarm_r")
model = MuscleTendonModel(m, device="cuda", dtype=torch.float32)   # float64 for parity with MuJoCo
out = model.forward(qpos, qvel, act)          # (..., nq), (..., nv), (..., na)
out.ten_length, out.ten_velocity              # (..., ntendon)
out.actuator_force, out.qfrc_actuator         # (..., nu), (..., nv)
model.dense_moment(out.actuator_moment)       # (..., nu, nv), MuJoCo's actuator_moment
model.effective_moment(out.actuator_moment, qpos)   # (..., nu, nind), through the joint couplings
act_next, act_mean = model.step_act(act, ctrl, nsteps=10)
```

`import myo_sim` never imports torch. `import myo_sim.diff` raises an `ImportError` that names the extra
when torch is missing.

## What it computes

| Part | Module | Inputs | Outputs | MuJoCo |
|---|---|---|---|---|
| Kinematics | `kinematics.py` | `qpos` | body poses, joint anchors and axes, DOF motion axes, site and geom poses | `mj_kinematics`, `cdof` |
| Wrapping | `wrap.py` | site and geom poses | wrap length, tangent points | `mju_wrap` |
| Tendons | `tendon.py` | poses, `qpos`, `qvel` | lengths, sparse Jacobian `ten_J`, velocities | `mj_tendon` |
| Actuators | `actuator.py` | tendon lengths and Jacobian, `qpos`, `qvel`, `act`, `ctrl` | lengths, sparse moments, velocities, forces, `act_dot`, next activation | `mj_transmission`, `mj_fwdActuation`, `mj_nextActivation` |
| Couplings | `coupling.py` | independent coordinates or `qpos` | full `qpos`/`qvel`, effective moment arms | `mjEQ_JOINT` |
| Facade | `model.py` | all of the above | `MuscleTendonState` (names follow `mjData`) | `mj_forward` |

Supported features:

- **Joints:** hinge, slide, ball and free.
- **Spatial tendons:** site-site segments; sphere and cylinder wraps, with or without side sites,
  including side sites inside the geom (MuJoCo's Newton solve); pulleys (divisors).
- **Fixed tendons** on hinge and slide joints.
- **Transmissions:** tendon, and joint on hinge and slide joints.
- **Gains:** fixed, affine and muscle, including `force < 0` scaled by `acc0`. **Biases:** none, affine
  and muscle.
- **Dynamics:** none, integrator, filter, exact filter and muscle, with `tausmooth`.
- **Activation and limits:** `actearly`; control, activation and force limits; joint actuator-force
  limits.

Anything else raises `NotImplementedError` when the model is built. That includes multi-output
actuators, tendon actuator-force limits, delays, actuator gravity compensation and couplings between two
coupled joints. Mocap bodies keep their model pose, and sleeping isn't modelled.

## Conventions

- **Inputs and shapes.** Inputs are the full MuJoCo state with any leading batch dimensions:
  `qpos (..., nq)`, `qvel (..., nv)`, `act (..., na)`, `ctrl (..., nu)`.
  `couplings.expand(q_ind, qd_ind)` builds a consistent full state from the independent coordinates.
- **Sparse layouts.** `ten_J` uses MuJoCo's row layout (`ten_J_rownnz`, `ten_J_rowadr`,
  `ten_J_colind`). `actuator_moment` uses the layout of `moment_rownnz`, `moment_rowadr` and
  `moment_colind`. `dense_ten_J` and `dense_moment` are `mju_sparse2dense`.
- **Moment arms and torques.** Moment arms are MuJoCo's: `actuator_moment = gear * ten_J`, that is
  dL/dq along every DOF. Ball and free DOFs use MuJoCo's local-frame angular velocities.
  `qfrc_per_actuator` is moment times force for each actuator and DOF, and `qfrc_actuator` sums it over
  actuators. Muscle forces are negative (pulling).
- **Effective moment arms.** Assume the joint couplings hold. Then a muscle's moment arm about an
  independent DOF j is the derivative of its length along j with every coupled joint following:
  `r_eff[a, j] = r[a, j] + sum_c r[a, c] dq_c/dq_j`.
  - It is also the generalised force the muscle exerts on j per unit tension.
  - Joints held by an equality without a second joint contribute nothing.
  - Columns follow `couplings.indep_dofs`.
- **Side-site rule.** MuJoCo 4 changed how a side site picks the wrap.
  - MuJoCo 3: the tangent pair whose mid-direction points at the side site.
  - MuJoCo 4: the path is homotopic to the polyline end -> side site -> end, and its winding picks the
    orientation and the arc.
  - MuJoCo 4 also changed `is_intersect`, `length_circle` and the inside-wrap Newton variable.
  - `side_rule` defaults to the installed MuJoCo's rule (`default_side_rule()`). Pass `"mujoco3"` or
    `"mujoco4"` to pick one.
- **Precision.** Both float64 and float32 work. The inside-wrap Newton solve always runs in float64,
  like MuJoCo's C code. In float32 the solve near z = 1 fails and falls back to the default point, which
  moved some arm muscles (DELT2, DELT3, PECM1, EDC) by up to 3 mm. That is the same failure as
  mujoco_warp#1713.

## Accuracy

`tests/test_diff_parity.py` compares against MuJoCo on random postures, velocities, activations and
controls. Hinges and slides are drawn uniformly in their range, and ball and free joints get random
quaternions. The worst error over 256 states per model, in float64:

| Model | tendon length (m) | tendon velocity (m/s) | moment arm (m) | force (relative) |
|---|---|---|---|---|
| `myohand_r` | 7e-14 | 3e-15 | 1e-15 | 3e-12 |
| `myoarm_r` | 4e-15 | 5e-15 | 3e-15 | 6e-12 |
| `myolegs` | 9e-14 | 1e-13 | 7e-14 | 6e-12 |
| `myofullbody` | 3e-14 | 5e-14 | 2e-14 | 7e-12 |
| `myoarms` (composition) | 3e-14 | 6e-15 | 4e-15 | 8e-12 |
| `tests/data/diff_features.xml` | 1e-15 | 1e-14 | 9e-15 | 5e-13 |

These are MuJoCo 3.11; MuJoCo 4.0 is the same to within a factor of two. The tests assert the targets
of the proposal:

- 1e-9 for length and velocity, 1e-8 for moment arms, 1e-6 relative for force;
- exact `act_dot`, and activation steps against `mj_step` with and without `tausmooth`;
- effective moment arms against MuJoCo's moments times the coupling Jacobian.

States at a wrap onset only get a 1e-5 length check, and they must stay rare. A state is at an onset
when MuJoCo's `wrap_obj` changes under `qpos` perturbations of 1e-7. None of the sampled states was.

In float32, against float64: tendon length within 6e-7 m, moment arms within 6e-6 m, and force within
4e-4 of the largest force.

The feature test model `tests/data/diff_features.xml` exercises what the myo_sim models don't use:

- pulleys and a fixed tendon;
- ball and free joints;
- side sites inside spheres and cylinders;
- affine, filter, exact-filter and integrator actuators, `actearly`;
- control, activation and force limits;
- a coupling and a held joint.

`test_side_rule_matters` is the negative control. With the other MuJoCo version's side-site rule, the
`myoarm_r` lengths move by up to 7 cm.

## Gradients and wrap onsets

- **Moment arms and autograd.** Lengths, velocities, forces and torques are differentiable in `qpos`,
  `qvel`, `act` and `ctrl`. For hinges and slides, autograd's dL/dq equals the analytic moment arms.
- **Inside wraps.** MuJoCo's Jacobian holds the tangent points fixed. The solved point of an inside wrap
  meets the tangent condition only to Newton's tolerance, so there the two differ by about 1e-9 m.
  The Newton solution gets its gradient from the implicit function theorem, so moment arms and
  velocities are differentiable too.
- **Finite differences.** `tests/test_diff_grad.py` checks directional derivatives against central
  differences. It uses only states whose wrap decisions are the same at q - h and q + h.
- **Grazing onset** (the straight segment touches the geom): length and moment arm are continuous, and
  only the slope of the moment arm changes.
- **Side-site switch** (the side-site rule sends the path round the other side of the geom): the length
  jumps by centimetres and the moment arm changes sign. In the onset test it jumps 109 mm under
  MuJoCo 3 and 49 mm under MuJoCo 4.
  - This is the jump that made BIClong 15-20 mm longer past the elbow limit (0.2.4).
  - Autograd returns the derivative of the branch the state is on and never sees the jump; finite
    differences taken across it do.
- **Both kinds are MuJoCo's own.** `myo_sim.diff` takes the same discrete decisions at every point of
  the sweep in `test_wrap_onset_behaviour`.

## Performance

- **Graph tracing.** The whole step traces as one `torch.compile` graph (`fullgraph=True`). Python
  control flow depends only on the model, iteration counts are fixed (Newton: 20) and every branch is a
  `torch.where`. `test_traces_as_one_graph` checks this on the CPU.
- **CUDA.** `test_compiled_step_on_cuda` compiles the step and its gradient with inductor and runs it as
  a captured CUDA graph (`mode="reduce-overhead"`). It skips without CUDA.
- **3x3 products** are batched matmuls. The product-and-sum form crashes inductor's CPU backward in
  torch 2.13.

CPU timings with `scripts/benchmark_diff.py`: float32, 8 threads, eager, torch 2.13. One forward pass
covers lengths, moment arms, velocities, forces and generalised forces; one backward pass covers
`qpos` and `act`.

| Model | batch 1 fwd / fwd+bwd (ms) | batch 512 (ms) | batch 4096 (ms) |
|---|---|---|---|
| `myohand_r` | 2.7 / 5.9 | 28 / 56 | 115 / 260 |
| `myoarm_r` | 3.1 / 6.7 | 36 / 72 | 174 / 448 |
| `myolegs` | 2.7 / 6.7 | 22 / 49 | 97 / 253 |
| `myofullbody` | 3.7 / 7.8 | 87 / 220 | 1005 / 2260 |

For comparison, the same quantities in MuJoCo's C code take 44 µs (`myoarm_r`) and 196 µs
(`myofullbody`) per state on one thread. Batch 1 is dominated by Python and kernel launch overhead. At
large batches on the CPU, the wrap kernels and the sparse Jacobian entries dominate.

### Fused tendon kernel (opt-in)

`MuscleTendonModel(m, "cuda", fused=True)` evaluates the spatial tendons with one Triton kernel
(`myo_sim/diff/fused.py`) instead of some hundred small torch kernels. One program per tendon and block
of 64 states walks the tendon's path (site pairs, sphere and cylinder wraps with side sites, both
side-site rules, the inside wrap in float64) and accumulates the length and the sparse `ten_J` row in
registers. It is CUDA-graph capturable and needs no host synchronisation.

- **Opt-in.** The default stays the pure-torch path. `fused=True` without Triton or a CUDA device warns
  and falls back to it.
- **Gradients.** The kernel has no backward of its own. Under autograd the backward pass recomputes the
  torch path, so gradients are exactly the torch path's.
- **Tests.** `tests/test_diff_fused.py` checks lengths, `ten_J`, velocities and wrap flags against the
  torch path (float64 round-off), forces against MuJoCo, float32 error, and gradients against the torch
  path and finite differences. It skips without Triton and CUDA; `TRITON_INTERPRET=1` runs it on the CPU.
- **Speed.** At 2048 states on an RTX 5090, one tendon refresh (lengths and `ten_J`) takes 0.34 ms,
  against 1.6-1.9 ms for the torch path.

## Implementation notes

- **Kinematics.**
  - Each joint's local transform depends only on its own `qpos`, so all joints are evaluated at once.
  - A body's local transform is its offset followed by its joints, a fixed-length product over joint
    slots.
  - World transforms then take one batched composition per tree level.
- **Tendon pieces.** A spatial tendon splits into pieces:
  - site-site segments;
  - for each site-geom-site wrap, the two tangent segments (wrapped) and the straight segment (not
    wrapped), selected by the wrap state.
  - A piece adds `|x2 - x1| / divisor` to the length.
  - It adds `u . (J(x2) - J(x1)) / divisor` to the Jacobian, with tangent points fixed to the geom's
    body.
- **Jacobian entries.** Each Jacobian entry is a static (piece, DOF, end) triple. The DOF motion axes
  are MuJoCo's `cdof`, shifted to the world origin, so a point's velocity is `v0 + omega x x`.
- **Fallbacks.** The wrap code evaluates every branch of the C code, including its fallbacks (parallel
  end points, failed Newton steps, `iter >= maxiter`), and selects with `torch.where`. Unselected
  branches keep finite gradients.

## Provenance

The wrap, tendon, muscle and activation code started as a port of the torch replica in
[muscle-supermimic](https://github.com/kaibohex/muscle-supermimic) (Apache-2.0, commit 8a6244b),
generalised from its models to any compiled MuJoCo model and changed from MuJoCo-Warp's semantics to the
C engine's. The kinematics is adapted from the same project. The notice is in `myo_sim/diff/NOTICE`, and
each ported file says so in its docstring.

## Plan

1. Tendon geometry and moment arms (spatial and fixed tendons, wrapping, side sites, couplings), with
   parity tests. **Done.**
2. Muscle force and activation dynamics. **Done.**
3. Performance: batched GPU benchmarks, `torch.compile` and CUDA-graph timings, fewer and fused kernels
   for the wraps and the Jacobian entries.
4. Users: MyoSuite's model-based training, and gradient-based fitting scripts here.

## Open questions

- **Name and location.** Keep `myo_sim.diff`, or use something like `myo_sim.torch`? Is a NumPy or JAX
  backend wanted besides torch?
- **Compliant tendons (#164).** How do they attach? Fibre length would be extra state passed in and out
  of the actuator part.
- **CI.** Should the parity tests run in this repository's CI, with torch as a test dependency, or only
  on demand? Today they skip when torch is missing, so CI doesn't run them.
- **GPU numbers.** The CUDA test and the GPU timings still need a run on a free GPU.
