# MyoTorso

## General:

The myoTorso mujoco musculoskeletal (MSK) model is generated from: Constrained Lumbar Spine model - 210 [https://simtk.org/projects/lumbarspine] from Opensim.

This generated mujoco MSK model has almost identical kinematics, and very similar muscle kinematics (moment arms) and kinetic (forces) properties.

The model have 210 actuators and 18 joints. The model can be controlled by 3 "virtual joints": Flexion extension, lateral bending, and axial rotation, that maps onto the real joints.

## Conversion process:

The myoTorso model was generated using our developed automatic conversion pipeline (released on June 2023).

Three Conversion steps were taken to generate the myoLeg models from the reference Osim model:

1. Basic element conversion [bone meshes, joint definitions, muscle paths, wrapping objects]
2. Moment arm optimization [matching the moment arm of each muscle by optimizing how muscles wrap over wrapping objects]
3. Muscle force optimizaiton [matching the muscle force-length relationship by optimizing muscle parameters]

After the conversion, a manual adjusting process is done to correct the abnormal results.

## Maunal adjustment:
- Adjustments post conversion to optimize for kinematic and dynamic behaviors are detailed in our paper published at ICORR 2025 (see last section for reference).
- Wrapping surfaces are stable against flipping tendons at every RoM.

## Contact Geometries:
- Manually designed with references
- Contact properties optimized for contact rich behaviors

## Issues:
- N/A

## Citation

If you use this repository in your research, please cite the following:

```bibtex
@article{Walia2025,
  title = {MyoBack: A Musculoskeletal Model of the Human Back with Integrated Exoskeleton},
  url = {http://dx.doi.org/10.1101/2025.03.13.643057},
  DOI = {10.1101/2025.03.13.643057},
  publisher = {Cold Spring Harbor Laboratory},
  author = {Walia,  Rohan and Garzon,  Kevin and Billot,  Morgane and Subramanian,  Swathika and WANG,  HUIYI and Refai,  Mohamed Irfan and Durandau,  Guillaume},
  year = {2025},
  month = mar
}
```
