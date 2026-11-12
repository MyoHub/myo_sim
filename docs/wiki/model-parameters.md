# Model parameters

`myo_sim.physics.muscle_strength` provides model-only strength transforms:

```python
from myo_sim.physics.muscle_strength import peak_force, scale_muscle_strength, scale_muscle_strength_spec

force = peak_force(model)                         # active peak force per actuator
passive = peak_force(model, model.actuator_biasprm)
scale_muscle_strength(model, 0.8)                   # compiled model, in place
scale_muscle_strength_spec(spec, per_actuator)      # spec, in place; returns spec
```

Factors are positive finite scalars or `(nu,)` arrays. Entries for non-muscle
actuators are ignored. Active and native muscle-bias force capacities scale
together; other gain/bias laws, geometry, activation dynamics and clamps stay
unchanged. Automatic forces are resolved using MuJoCo's `scale/acc0` rule before
explicit values are written. The spec variant compiles a copy to preserve the
caller's editable frames, and validates every factor before changing parameters.

These operations do not estimate physiological strength from size or fit
measurements. Subject calibration, torque fitting, task/checkpoint compatibility
and the legacy gain-only sarcopenia convention belong to downstream applications.

## Selected-body mass

```python
from myo_sim.physics.body_mass import scale_body_mass

heavier_arm = scale_body_mass(model, represented_mass=4.5, bodies=["arm"])
subject = scale_body_mass(model, participant_mass=90, reference_mass=60, bodies=["arm"])
```

These are distinct modes. `represented_mass` sets the selected bodies' total
model mass in kg; the participant/reference mode multiplies their original
masses by an explicit ratio. A partial limb's mass is not its participant's
whole-body mass. Supply exactly one mode with positive finite masses.

Only the named bodies are selected, without implicit descendants. `None` selects
every non-world body, including fixed scenery and held objects. Select anatomy
explicitly when those must remain unchanged. Selected inertias scale by the same
factor as mass; geometry, COM, inertial frames and strength are preserved. The
result is an independent compiled-model copy with MuJoCo constants refreshed.
Automatic muscle forces are frozen before mass changes alter their `acc0`.
This does not infer density distributions, resize anatomy or choose a strength
factor from body mass.
