# MyoLeg

MuJoCo musculoskeletal model of the bilateral lower extremity, derived from Rajagopal et al.'s full-body gait model.

## Anatomical scope

| Property | Value |
|---|---|
| Degrees of freedom | 29 |
| Actuators (muscles) | 80 |
| Body segments | calcn_l, calcn_r, femur_l, femur_r, patella_l, patella_r, pelvis, talus_l, talus_r, tibia_l, tibia_r, toes_l, toes_r |
| Primary joints | hip_flexion, hip_adduction, hip_rotation, knee_angle, ankle_angle, subtalar_angle, mtp_angle (bilateral) |

## Reference model

- **Source:** [Rajagopal Full-Body Musculoskeletal Model](https://simtk.org/projects/full_body)
- **Paper:** Rajagopal A, Dembia C, DeMers M, Delp D, Hicks J, Delp S (2016). Full-Body Musculoskeletal Model for Muscle-Driven Simulation of Human Gait. IEEE Transactions on Biomedical Engineering. ([DOI: 10.1109/TBME.2016.2586891](https://doi.org/10.1109/TBME.2016.2586891))

## Fidelity

![glmax1 left/right hip flexion relationship](../../../docs/images/glmax1_r_glmax1_l_hip_flexion_r.png)

Example fidelity check (glmax1 left/right moment-arm and force-length plot) for symmetry and muscle jumping for lower limb. The full leg muscle check can be run with `tests/debug_muscle_leg.py`.


## Known limitations

- [ ] #64 — `myoleg` naming used inconsistently across the repository
- [ ] Endpoints (markers) below the knee joints have approximately 1 cm position differences between the converted MuJoCo and OpenSim model. This may be due to the polynomial approximation of the OpenSim lookup table for knee translation degrees of freedom.
- [ ] Vastus muscle moment arms at the knee joint have relatively large differences (same sign, a few cm). This may be caused by the dependent joint constraints at the knee and also affects knee extensor muscle force.
- [ ] Muscle forces are not identical between the converted MuJoCo and OpenSim models due to differences in muscle model definitions (stiff vs. elastic tendons). Elastic tendon support in MuJoCo is not yet implemented for this model.
- [ ] Muscle moment arms in the reference OpenSim model contain sudden changes (wrapping path jumps), which required the manual adjustments described below.

## Manual adjustments

- Removed wrapping objects for glmax1_l, glmax2_l, glmax1_r, glmax2_r, psoas_l, and psoas_r to prevent wrapping path jumping. Objects removed: Gmax1_at_pelvis_l_wrap, Gmax2_at_pelvis_l_wrap, Gmax1_at_pelvis_r_wrap, Gmax2_at_pelvis_r_wrap, PS_at_brim_l_wrap, PS_at_brim_r_wrap.
- Changed wrapping object type from `cylinder` to `sphere` for iliacus_l and iliacus_r (IL_at_brim_l_wrap, IL_at_brim_r_wrap) to prevent wrapping path jumping.
- Adjusted `lmin` of gaslat_l, gaslat_r, semimem_l, and semimem_r from 0.1 to 0.05 to prevent negative muscle forces.
- Post-conversion adjustments to kinematic and dynamic behaviors, inertial properties, and joint dynamics properties.
- Contact geometries based on the [Yeadon measurement method](https://yeadon.readthedocs.io/en/latest/measurements.html#measurements), slightly adjusted to fit the MuJoCo MSK model. Contact properties were optimized for contact-rich behaviors.

## Changelog

**2026-06-04** — Refactored model loading to use updated leg model XML; added muscle symmetry checks and contact assertion tests.

**2026-06-03** — Refactored model structure and enhanced asset management.

**myoleg_v0.56 (mj237)** — Adjusted height field; migrated to MuJoCo 2.3.7.

**myoleg_v0.53 (mj120)** — Improved collisions; added height field.

**myoleg_v0.52 (mj120)** — Removed extra body that made the torso twice as heavy; body mass is now approximately 80 kg.

**myoleg_v0.51 (mj120)** — Added new keyposes to mark convenient poses.

## MyoLeg26 (reduced 26-muscle, legs-only)

`myolegs26` is a reduced, lower-limb model (no HAT, torso, arms, or head). It is meant as a compact
base for work where the full 80-muscle anatomy
isn't needed.

### Anatomical scope

| Property | Value |
|---|---|
| Degrees of freedom | 18 (46 incl. equality-coupled moving-via-point DoFs) |
| Actuators (muscles) | 26 |
| Body segments | calcn_l, calcn_r, femur_l, femur_r, pelvis, talus_l, talus_r, tibia_l, tibia_r, toes_l, toes_r |
| Primary joints | hip_flexion, hip_adduction, hip_rotation, knee_angle, ankle_angle, mtp_angle (bilateral) |


### Build

Composed at runtime (there is no static `myolegs26.xml`), from
`myo_sim/build/compose.py`:

- **Standalone base** — `build_legs26_base_spec`, exposed via `build_model("myolegs26")`, `build_spec("myolegs26")`, and `load("myolegs26")`. It assembles the component files, adds a **free root joint** (myosuite heading), layers the standard myosuite **scene** (floor + lights), and applies the **`stand` keyframe** loaded from `myolegs26_keyframes.xml`. The stand pose is shipped as a keyframe (not `qpos0`) because the joint couplers cannot be satisfied at `qpos0`; it is fully at-rest (coupler residual ≈ 0, feet on the pedestal).
- **Bare fragment** — `FRAGMENT_SPEC_BUILDERS["myolegs26"]` returns just the chain + muscles/tendons/assets, with **no** free root, scene, or keyframe, for composing into larger models or for downstream consumers (e.g. `assist_sim`) that supply their own root/ground and poses.

```python
import myo_sim

model, data = myo_sim.load("myolegs26")   # compiled MjModel + MjData (standalone base)
spec = myo_sim.build_spec("myolegs26")     # editable MjSpec of the standalone base
```

### Reference & credits

Reduced from the OpenSim gait2392 / gait9dof18 lineage (Ajay Seth, based on
Delp et al. 1990; muscle strengths after Handsfield/Rajagopal, tuned by
Carmichael Ong; planar knee after Yamaguchi & Zajac 1989). Adapted for MyoLeg
by Chun Kwang Tan (sagittal-plane joints, ankle ROM, GRF foot sensors) and
extended by Calder Robbins (toes + mtp joints, EDL/FDL muscles). License:
CC-BY 3.0.

## Citation

See repository README.
