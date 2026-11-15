# Model parameters

`myo_sim.physics.muscle_strength` provides model-only strength transforms and isometric maximal voluntary torque (MVC) calibration:

```python
from myo_sim.physics.muscle_strength import (
    MaxTorqueMeasurement,
    calibrate_muscle_strength,
    max_torque_terms,
    peak_force,
    scale_muscle_strength,
    scale_muscle_strength_spec,
    strength_factor_from_body_mass,
)

force = peak_force(model)                         # active peak force per actuator
passive = peak_force(model, model.actuator_biasprm)
scale_muscle_strength(model, 0.8)                   # compiled model, in place
scale_muscle_strength_spec(spec, per_actuator)      # spec, in place; returns spec

trials = [MaxTorqueMeasurement("r_elbow_flex", qpos_90deg, torque=62.0)]
cal = calibrate_muscle_strength(model, trials, prior=strength_factor_from_body_mass(68.0, 75.2))
scale_muscle_strength(model, cal.factors)
```

Factors are positive finite scalars or `(nu,)` arrays. Entries for non-muscle
actuators are ignored. Active and native muscle-bias force capacities scale
together; other gain/bias laws, geometry, activation dynamics and clamps stay
unchanged. Automatic forces are resolved using MuJoCo's `scale/acc0` rule before
explicit values are written. The spec variant compiles a copy to preserve the
caller's editable frames, and validates every factor before changing parameters.

`max_torque_terms` evaluates each actuator's active and passive torque plus the
joint springs along the joint's coupled motion (via `myo_sim.physics.joint_coupling`)
at steady activation, without gravity, rejecting force clamps, multi-state
actuators, gravity compensation and stateless muscle control clamps that exclude
`[0, 1]`. `calibrate_muscle_strength` fits bounded linear least-squares strength
factors (pure NumPy active-set BVLS, no `scipy` runtime dependency) regularized
weakly toward `prior`. Net joint torques do not identify individual synergists,
so the fitted factors represent effective strength. Task/checkpoint provenance
and the legacy gain-only sarcopenia convention remain in downstream applications.
