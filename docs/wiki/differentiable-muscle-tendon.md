# Differentiable muscle-tendon model (proposal)

**Status: proposal for discussion. No code yet.** This page describes a component to add to myo_sim; the
discussion happens on its pull request.

## Why

MuJoCo computes tendon lengths, moment arms, muscle forces and activation dynamics inside `mj_forward`.
Those quantities are exact, but they can't be differentiated from PyTorch, and they're only available for
states MuJoCo has simulated. A growing set of uses needs them as differentiable, batched functions of
`qpos`, `qvel` and the activations, on states MuJoCo hasn't seen:

- **Model-based learning.** World-model methods such as SuperMimic (Liao et al., arXiv:2610.28480) train a
  policy by backpropagating through imagined rollouts. On muscle bodies, the imagined muscle lengths,
  velocities and forces have to come from a differentiable replica of the model.
- **Learning only what isn't known.** With exact muscle forces and moment arms, a world model only has to
  learn what MuJoCo's equations make hard to differentiate (contacts and constraint forces) instead of the
  whole muscle-to-motion map.
- **Physiological costs.** Costs such as co-contraction (torque that antagonists cancel) need per-muscle,
  per-joint torques with gradients.
- **Fitting parameters by gradient.** The scripts in this repository fit moment arms, operating ranges and
  passive moments against literature data with finite differences or derivative-free search. A
  differentiable model makes those fits gradient-based and much cheaper, also for scaled bodies.
- **Compliant tendons.** A compliant-tendon muscle (see the elastic-tendon load proposal, #164) integrates
  fibre length from the musculotendon length and velocity, which this component provides.

## What

An optional `myo_sim.diff` module (torch, installed with `myo_sim[torch]`), built from a compiled
`mujoco.MjModel` of any myo_sim model or composition:

| Part | Inputs | Outputs |
|---|---|---|
| Kinematics | `qpos` | body, site and wrap-geom poses |
| Spatial tendons | site and geom poses | lengths, with straight segments, sphere and cylinder wrapping with side sites, pulleys |
| Fixed tendons | `qpos` | lengths |
| Moment arms | tendon lengths | dL/dq through every DOF, including the DOFs a joint drives through `mjEQ_JOINT` couplings |
| Muscles | length, velocity, activation | MuJoCo `<muscle>` force: force-length, force-velocity and passive curves, `lengthrange` scaling |
| Activation | excitation, activation | MuJoCo's muscle activation dynamics, including `tausmooth` |
| Joint torques | moment arms, forces | torque per muscle and DOF, and per DOF |

- **Batched and device-agnostic:** shapes `(..., nq)`, runs on CPU and CUDA, works under `torch.compile`
  and CUDA-graph capture (no host synchronisation, no data-dependent control flow on the host).
- **Generated from the compiled model:** it follows any composition, scaling or edit made through `MjSpec`,
  so there is no second model definition to keep in sync.
- **Same semantics as the installed MuJoCo:** the wrap and side-site rules follow MuJoCo, including the
  MuJoCo 4 side-site rule, selected by the MuJoCo version.

## Accuracy targets

Parity tests against MuJoCo on random postures of every registered model (`myohand_r`, `myoarm_r`,
`myolegs`, `myofullbody` and the compositions):

- tendon length and musculotendon velocity: 1e-9 in float64, round-off in float32;
- moment arms against MuJoCo's `actuator_moment`: 1e-8;
- muscle force against `actuator_force`: 1e-6 relative;
- gradients against central finite differences away from wrap onsets, and the behaviour at onsets
  documented (the path length is continuous there, its derivative is not).

## Prior art and sharing

[muscle-supermimic](https://github.com/kaibohex/muscle-supermimic) (Apache-2.0) already has a torch
replica of MuJoCo's spatial tendons and Hill muscles for MS-Human-700 and MuscleMimic's MyoFullBody,
matching MuJoCo to 1e-9 / 1e-8 / 1e-6 in length, velocity and force. Building on it, as a shared component
maintained here next to the models it mirrors, would serve both projects and MyoSuite's learners. Any code
taken from it keeps its notice.

## Plan

1. Tendon geometry and moment arms (spatial and fixed tendons, wrapping, side sites, couplings), with the
   parity tests above.
2. Muscle force and activation dynamics.
3. Performance: batched GPU benchmarks, `torch.compile` and CUDA-graph capture.
4. Users: MyoSuite's model-based training, and gradient-based fitting scripts here.

## Open questions

- Module name and location (`myo_sim.diff`, `myo_sim.torch`, ...), and whether a NumPy or JAX backend is
  wanted besides torch.
- How compliant tendons (#164) attach: fibre length as extra state passed in and out of the muscle part.
- Whether the parity tests run in this repository's CI (torch as a test dependency) or only on demand.
