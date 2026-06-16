# MyoHead

A simplified rigid kinematic head-and-neck model (no muscle actuators) derived from the HAT (Head-Arms-Trunk) body segment geometry used in OpenSim full-body models.

## Anatomical scope

| Property | Value |
|---|---|
| Degrees of freedom | 2 |
| Actuators (muscles) | 0 — rigid kinematic scaffold only |
| Body segments | neck, head |
| Primary joints | neck_rotation, neck_flexion |

## Known limitations

- [ ] No muscle actuators — the model provides geometry and kinematics only; neck muscle forces are not represented.

## Manual adjustments

- Joint `neck_rotation` and `neck_flexion` were commented out in `myohead_rigid_chain.xml` to produce a fully locked (zero-DOF) rigid variant; `myohead_simple_chain.xml` re-enables both joints for the standard two-DOF configuration.

## Changelog

**2026-06-03** — Refactor model structure and enhance asset management.
**2026-05-26** — Documentation pass; deprecated arm/elbow files removed from sibling directories.
**2025-05-27** — Initial addition of head model alongside body and arm models.

## Citation

See repository README.
