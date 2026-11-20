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

- Joint `neck_rotation` and `neck_flexion` are commented out in `myohead_rigid_chain.xml`, producing a fully locked (zero-DOF) rigid head by default when the torso chain is composed.
- Call `myo_sim.unlock_head_joints(spec)` (or pass `build_kwargs={"unlock_head_joints": True}` to `myo_sim.load_spec`) to restore the 2-DOF (or 3-DOF with `include_roll=True`) cervical joints and optional position servos (`add_actuators=True`), and `myo_sim.add_ocular_rig(spec)` (`build_kwargs={"add_ocular_rig": True}`) to attach binocular/dual-FOV head cameras, gaze sites, and `head_imu` sensors.

## Changelog

**2026-06-03** — Refactor model structure and enhance asset management.
**2026-05-26** — Documentation pass; deprecated arm/elbow files removed from sibling directories.
**2025-05-27** — Initial addition of head model alongside body and arm models.

## Citation

See repository README.
