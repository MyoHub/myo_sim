# MyoArm 0.1

## General:

The myoArm mujoco musculoskeletal (MSK) model is generated from: MoBL - human upper
extremity model [https://simtk.org/projects/upexdyn/] from Opensim and the MyoHand model [https://github.com/MyoHub/myo_sim/tree/main/hand].

This generated mujoco MSK model has almost identical kinematics, and very similar muscle kinematics (moment arms) and kinetic (forces) properties.


## Conversion process:

The myoHand model was generated using our developed automatic conversion pipeline (released on June 2023).

Three Conversion steps were taken to generate the myoLeg models from the reference Osim model:

1. Basic element conversion [bone meshes, joint definitions, muscle paths, wrapping objects]
2. Moment arm optimization [matching the moment arm of each muscle by optimizing how muscles wrap over wrapping objects]
3. Muscle force optimizaiton [matching the muscle force-length relationship by optimizing muscle parameters]

After the conversion, a manual adjusting process is done to correct the abnormal results.

## Maunal adjustment:
- Adjustments post conversion to optimize for kinematic and dynamic behaviors

## Contact Geometries:
- Manually designed with references
- Contact properties optimized for contact rich behaviors

## Issues:
- N/A
