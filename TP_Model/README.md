# MyoAmpTP V0

## General:

MyoAmputation TP Model was modified from the OpenLeg Project OpenSim Model:
https://www.epic.gatech.edu/oslmodels/
This is a modified reduced version to match similar number of DoF and muscles than example model Gait2354.

Final model used "ModifiedAmputationModel.xml".
Output of the myoconversion: OSL_TFA_L_Ankle_Modified_cvt3.xml

This generated mujoco MSK model has almost identical kinematics, and very similar muscle kinematics (moment arms) and kinetic (forces) properties.

Model tested in Mujoco, we need to remove collision attribute for it to work fine on Mujoco.
![Screenshot from 2023-11-22 03-38-25.png](..%2F..%2F..%2FPictures%2FScreenshots%2FScreenshot%20from%202023-11-22%2003-38-25.png)
## Conversion process:

The myoHand model was generated using our developed automatic conversion pipeline (released on June 2023).

Three Conversion steps were taken to generate the myoLeg models from the reference Osim model:

1. Basic element conversion [bone meshes, joint definitions, muscle paths, wrapping objects]
2. Moment arm optimization [matching the moment arm of each muscle by optimizing how muscles wrap over wrapping objects]
3. Muscle force optimizaiton [matching the muscle force-length relationship by optimizing muscle parameters]

After the conversion, a manual adjusting process is done to correct the abnormal results.

## Maunal adjustment:
- Modification of yR_FootToAnkle position and range of motion.
- Different muscles were removed to match the example model Gait2354.

## Contact Geometries:
Missing manual adjustments.

## Issues:
- When using the myosuite modified environment WalkV0 is starting in Done State, even when attempting to load keyframe.
- yR_FootToAnkle and zR_FootToAnkle movement de-attach from the main body.
