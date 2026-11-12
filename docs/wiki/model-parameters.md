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
