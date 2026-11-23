# Model parameters

`myo_sim.physics.muscle_strength` provides model-only strength transforms and isometric maximal voluntary torque (MVC) calibration:

```python
from myo_sim.physics.muscle_strength import (
    MaxTorqueMeasurement,
    calibrate_muscle_strength,
    max_joint_torque,
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
scale_muscle_strength(model, 1.3, muscles=["BIClong", "BRA"])   # selected muscles only
torque, agonists = max_joint_torque(model, qpos_90deg, "r_elbow_flex", max_activation=caps)

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

`muscles` (keyword-only) restricts scaling to distinct, named muscle actuators;
only their gain/bias force parameters are written, every other actuator stays
bit-identical (an unselected automatic force stays automatic), and only the
selected factors are validated. Unknown, duplicate or non-muscle names raise
before any change. `max_joint_torque` returns the signed net maximal torque of
one independent hinge or slide joint (passive muscle and joint-spring torque
plus the active torque of the agonists, whose names it returns) for a
coupling-projected `(nq,)` posture and a direction of `+1` or `-1`. Optional
`max_activation` caps in `[0, 1]` (e.g. `1 - fatigue`) apply to active force
only. It reuses the `max_torque_terms` probe and its model restrictions.

`max_torque_terms` evaluates each actuator's active and passive torque plus the
joint springs along the joint's coupled motion (via `myo_sim.physics.joint_coupling`)
at steady activation, without gravity, rejecting force clamps, multi-state
actuators, gravity compensation and stateless muscle control clamps that exclude
`[0, 1]`. `calibrate_muscle_strength` fits bounded linear least-squares strength
factors (pure NumPy active-set BVLS, no `scipy` runtime dependency) regularized
weakly toward `prior`. Non-muscle actuators remain at zero control/activation;
their zero-input forces (such as a position servo's bias) and the joints' passive
torque are included as fixed contributions rather than fitted strength. Net
joint torques do not identify individual synergists, so the fitted factors
represent effective strength. Task/checkpoint provenance
and the legacy gain-only sarcopenia convention remain in downstream applications.

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
Equality settings, including runtime-edited connect anchors, and geometric
statistics (`center`, `extent`, `meansize`) stay unchanged. Default `meanmass`
and `meaninertia` follow the scaled model; custom values are preserved.
This does not infer density distributions, resize anatomy or choose a strength
factor from body mass.

## Muscle activation time constants

```python
from myo_sim.physics.activation_dynamics import (
    activation_time_constants,
    scale_activation_time_constants,
    scale_activation_time_constants_spec,
)

tau_act, tau_deact = activation_time_constants(model)
scale_activation_time_constants(model, act_factors=1.5, deact_factors=2.0)
scale_activation_time_constants_spec(spec, 1.25)
```

Factors are positive finite scalars or `(nu,)` arrays; `deact_factors` defaults
to `act_factors` when omitted. Only `mjDYN_MUSCLE` actuators are updated
(`actuator_dynprm[:, 0]` and `actuator_dynprm[:, 1]`); motors, stateless muscles,
peak active/passive force capacities, geometry and clamps remain unchanged.

## Passive joint mechanics

```python
from myo_sim.physics.joint_mechanics import scale_joint_mechanics

stiffer = scale_joint_mechanics(model, damping=2.0, stiffness=1.5)
reduced_rom = scale_joint_mechanics(model, range_scale=0.75, joints=["elbow_flexion_r"])
```

`joints=None` selects all independent 1-DOF (`hinge` and `slide`) joints,
skipping `free`/`ball` joints and dependent joints driven by active `mjEQ_JOINT`
equalities. `damping` and `stiffness` accept non-negative finite scalars or
`(n_selected,)` arrays; `range_scale` accepts positive finite values and scales
limited joint intervals `[lo, hi]` around `clip(qpos0, lo, hi)` so reference
postures remain limit-free. Automatic muscle peak forces are frozen before
`mj_setConst` refreshes derived model constants on the returned copy.
Equality settings, including runtime-edited connect anchors, and all model
statistics are preserved because these transforms change neither geometry nor
mass.
