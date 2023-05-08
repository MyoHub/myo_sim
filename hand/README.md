# MyoHand 0.1

## General:

The myoHand mujoco musculoskeletal (MSK) model is taking 2 popular OpenSim models: MoBL - human upper
extremity model [https://simtk.org/projects/upexdyn/] and the 2nd-Hand [https://simtk.org/projects/hand_muscle], for hand and fingers MSK model references.

This generated mujoco MSK model has almost identical kinematics, and very similar muscle kinematics (moment arms) and kinetic (forces) properties.


## Conversion process:

The myoHand model was generated using our developed automatic conversion pipeline (will release at June 2023).

Three Conversion steps were taken to generate the myoLeg models from the reference Osim model:

1. Basic element conversion [bone meshes, joint definitions, muscle paths, wrapping objects]
2. Moment arm optimization [matching the moment arm of each muscle by optimizing how muscles wrap over wrapping objects]
3. Muscle force optimizaiton [matching the muscle force-length relationship by optimizing muscle parameters]

After the conversion, a manual adjusting process is done to correct the abnormal results.

## Maunal adjustment:
- Adjustments post conversion to optimize for kitnematic and dynamic behaviors
- Inertial properties
- Dynamics properties of the joints

## Contact Geometries:
- Manually designed with references
- Contact properties optimized for contact rich behaviors

## Issues:
- N/A