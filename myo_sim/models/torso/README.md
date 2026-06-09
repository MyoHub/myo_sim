# MyoTorso

MuJoCo musculoskeletal model of the human torso (lumbar spine and abdomen), derived from the Constrained Lumbar Spine model - 210 on SimTK.

## Anatomical scope

| Property | Value |
|---|---|
| Degrees of freedom | 18 |
| Actuators (muscles) | 210 |
| Body segments | Abdomen, Arm_attachment, cervical_spine, chest_r, head_attach, lumbar1, lumbar2, lumbar3, lumbar4, lumbar5, sacrum, torso |
| Primary joints | L1_L2_AR, L1_L2_FE, L1_L2_LB, L2_L3_AR, L2_L3_FE, L2_L3_LB, L3_L4_AR, L3_L4_FE, L3_L4_LB, L4_L5_AR, L4_L5_FE, L4_L5_LB, flex_extension, lat_bending, axial_rotation (virtual), Abs_r3, Abs_t1, Abs_t2 |

## Reference model

- **Source:** [Constrained Lumbar Spine model - 210](https://simtk.org/projects/lumbarspine)
- **Paper:** Walia et al., 2025 ([DOI](https://doi.org/10.1101/2025.03.13.643057))

## Fidelity

<!-- TODO: review -->

## Known limitations

- [ ] #70 — Myotorso seems asymmetric
- [ ] #51 — Improve passive dynamics of torso

## Manual adjustments

- Adjustments post conversion to optimize for kinematic and dynamic behaviors are detailed in the ICORR 2025 paper (see Citation).
- Wrapping surfaces are stable against flipping tendons at every range of motion.
- Contact geometries manually designed with references; contact properties optimized for contact-rich behaviors.

## Changelog

**2026-06-05** — Moved chest ownership to myotorso; added unit tests.

**2026-06-04** — Removed deprecated myolegs_abdomen from fragment catalog and registry; introduced passive torso model loading functions; refactored myotorso to use one model and remove joints.

**2026-06-03** — Refactored model structure and enhanced asset management.

## Citation

```bibtex
@article{Walia2025,
  title = {MyoBack: A Musculoskeletal Model of the Human Back with Integrated Exoskeleton},
  url = {http://dx.doi.org/10.1101/2025.03.13.643057},
  DOI = {10.1101/2025.03.13.643057},
  publisher = {Cold Spring Harbor Laboratory},
  author = {Walia, Rohan and Garzon, Kevin and Billot, Morgane and Subramanian, Swathika and Wang, Huiyi and Refai, Mohamed Irfan and Durandau, Guillaume},
  year = {2025},
  month = mar
}
```
