# MyoHead

Head-and-neck geometry from the existing HAT segment, with an opt-in muscular cervical chain derived from the scaled HYOID OpenSim model.

## Anatomical scope

| Property | Value |
|---|---|
| Degrees of freedom | Rigid scaffold: 0 active; `myohead`: 24 hinges, 18 couplings, 6 independent cervical DoFs |
| Actuators (muscles) | Rigid scaffold: 0; `myohead`: 72 (36 right sources + mirrored left) |
| Body segments | Existing rigid neck/head; optional C7–C1 and skull, with welded jaw |
| Primary joints | myohead_pitch1, myohead_roll1, myohead_yaw1, myohead_pitch2, myohead_roll2, myohead_yaw2 |

Use `myo_sim.load("myohead")` for the fixed standalone neck or
`myo_sim.load("myofullbody_neck")` for the opt-in full-body assembly.
`myo_sim.load("myofullbody")` retains the rigid scaffold and 416 actuators.

## Reference model

- **Source:** [Scaled HYOID / Neck6D](https://simtk.org/projects/neckdynamics), via the [pinned MyoConverter distribution](https://github.com/MyoHub/myoconverter/tree/cadf38059367a51239e6dc28c9fbe8b8fbd5149f/models/osim/Neck6D). The model's MIT notice is preserved in `validation/LICENSE.Neck6D`.
- **Paper:** Mortensen, Vasavada and Merryweather, 2018 ([DOI](https://doi.org/10.1371/journal.pone.0199912)).

## Fidelity

Source joint-center offsets, coordinate axes, ranges and couplings are retained.
The right muscle paths are mirrored at compose time using the repository's
mirroring utility. HAT skull/jaw and segmented cervical meshes retain their
neutral placement. The existing default full-body model is unchanged.

See [the measured validation report and reproduction instructions](validation/README.md)
for source agreement, neutral experimental strength, dynamic release comparisons, intentional differences
and the precise scope of acceptance. <!-- TODO: review fidelity against measured report -->

## Known limitations

- [ ] No muscle actuators in the existing rigid scaffold; muscular actuation requires the opt-in assembly.
- Source left attachment asymmetries are replaced by generated right-side symmetry.
- Fixed thoracic/shoulder muscle landmarks, linearized multi-axis bushings, welded jaw, balanced converted inertias and approximate muscle curves limit dynamic fidelity. The calibrated 20–250 ms head-release benchmark passes both directions and participant-separated evaluation; see the validation report for its exploratory scope.
- Cervical mesh component labeling is heuristic and requires anatomical review.

## Manual adjustments

- Joint `neck_rotation` and `neck_flexion` are commented out in `myohead_rigid_chain.xml`, producing a fully locked (zero-DOF) rigid head used when the torso chain is composed. Uncomment both joints there to restore the standard two-DOF neck configuration.
- The optional muscular chain restores source child-frame joint-center offsets missing from cvt3, aligns the skull origin to the existing head and mirrors only muscle sites/paths/actuators, retaining one midline skeleton.
- Source rotational bushing stiffness/damping is restored with native joint springs: exact in sagittal motion and a first-order approximation in multi-axis motion.
- Geniohyoid's negative-slack conversion fit is replaced with source physical length/slack/force values and the standard MuJoCo muscle curve, without fitting to acceptance measurements.

## Changelog

**2026-10-03** — Restore source passive mechanics and anatomical fiber estimation; add graded feedback with a single calibrated gain, participant-separated evaluation, before/after curves and torque diagnostics. Add opt-in source-backed neck composition and reproducible source/neutral-strength acceptance checks; retain default full-body behavior.
**2026-06-03** — Refactor model structure and enhance asset management.
**2026-05-26** — Documentation pass; deprecated arm/elbow files removed from sibling directories.
**2025-05-27** — Initial addition of head model alongside body and arm models.

## Citation

See repository README. The optional neck also uses the model introduced by
Mortensen, Vasavada and Merryweather (2018), DOI 10.1371/journal.pone.0199912.
The dynamic benchmark uses Wochner et al. (2022), DOI 10.1186/s12938-022-00994-9,
and their CC BY 4.0 trajectory data, DOI 10.18419/DARUS-1132.
<!-- TODO: review citation completeness for any subsequent validation publication -->
