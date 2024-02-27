# MyoAmpTP V0

## General:

MyoAmputation TP Model was modified from the OpenLeg Project OpenSim Model:
https://www.epic.gatech.edu/oslmodels/
This is a modified reduced version to match similar number of DoF and muscles than example model Gait2354.

This generated mujoco MSK model has almost identical kinematics, and very similar muscle kinematics (moment arms) and kinetic (forces) properties.

![Screenshot from 2023-11-22 03-38-25.png](..%2F..%2F..%2FPictures%2FScreenshots%2FScreenshot%20from%202023-11-22%2003-38-25.png)
## Conversion process:

The myoTP model was generated using our developed automatic conversion pipeline (released on June 2023).

Three Conversion steps were taken to generate the myoLeg models from the reference Osim model:

1. Basic element conversion [bone meshes, joint definitions, muscle paths, wrapping objects]
2. Moment arm optimization [matching the moment arm of each muscle by optimizing how muscles wrap over wrapping objects]
3. Muscle force optimizaiton [matching the muscle force-length relationship by optimizing muscle parameters]

After the conversion, a manual adjusting process is done to correct the abnormal results.

## Manual adjustment:
- Modification of yR_FootToAnkle position and range of motion.
- Modification of joint position for yR_FootToAnkle and zR_FootToAnkle to match motor location.
- Added manual initial position in all joints.

## Contact Geometries:

## Issues:
- To be Done: A stl file with the tibia and TT prosthesis in the same stl, not as a separated bone.
- To be Done: A model with passive TT prosthesis.
- Algorithm to help changing the xml file by selecting the number of electrical actuators.