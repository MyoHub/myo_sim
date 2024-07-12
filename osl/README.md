# MyoProsthesis

## General:

The MyoProsthesis model combineds both the 80-muscle [MyoLeg](https://github.com/MyoHub/myo_sim/tree/main/leg) model and [Open Source Leg](https://neurobionics.robotics.umich.edu/research/wearable-robotics/open-source-leg/), to simulate a trans-feromal amputation model.

## Modifications:

The MyoLeg model was modified as follows:

1. Replaced the right leg with trans-feromal leg, with 50% of the femur remaining
2. Muscles articulating the knee joint were removed, based on [1]. The list of muscles removed are detailed in the section below
3. Added OSL v2 to the right leg
4. Added a 4-DOF socket joint, with damping to model socket movement and dynamics
5. Adjusted the lengths of the thigh and shank pylons of the OSL to match joint rotation centers of the knee and ankle joint
6. Added a force sensor to represent the load cell between the pylon and ankle assembly of OSL leg
7. Added a force sensor at the socket joint


## Included Muscles - 54 Muscles and 2 Electrical Actuators and Primary Function:
| **Name in MyoLeg**        | **Real Name**                  | **Primary Function**                             | **Joint Movement**   |
|---------------------------|--------------------------------|--------------------------------------------------|----------------------|
| addbrev                   | Adductor brevis                | Thigh Adduction                                  | Hip                  |
| addlong                   | Adductor longus                | Thigh Adduction, rotation                        | Hip                  |
| addmagDist                | Adductor magnus distal         | Pelvis stabilizer                                | Hip, Knee            |
| addmagIsch                | Adductor magnus ischial        | Pelvis stabilizer                                | Hip, Knee            |
| addmagMid                 | Adductor magnus mid            | Pelvis stabilizer                                | Hip, Knee            |
| addmagProx                | Adductor magnus proximal       | Pelvis stabilizer                                | Hip, Knee            |
| bflh                      | Bicep Femoral Long Head        | Thigh extension, rotation and Knee flexion       | Hip, Knee            |
| bfsh                      | Bicep Femoral Short Head       | Knee Rotation                                    | Knee                 |
| edl                       | Extensor Digitorum Longus      | Digits movement                                  | Digits               |
| ehl                       | Extensor Hallucis longus       | Digits movement                                  | Digits               |
| fdl                       | Flexor Digitorum Longus        | Digits movement                                  | Digits               |
| fhl                       | Flexor Hallucis Longus         | Digits movement                                  | Digits               |
| gaslat                    | Gastrocnemious Lateral         | Plantar Flexion, knee Flexion                    | Ankle                |
| gasmed                    | Gastrocnemious medial          | Plantar Flexion, knee Flexion                    | Ankle                |
| glmax1                    | Gluteus Maximus 1              | Thigh Rotation                                   | Hip                  |
| glmax2                    | Gluteus Maximus 2              | Thigh Rotation                                   | Hip                  |
| glmax3                    | Gluteus Maximus 3              | Thigh Rotation                                   | Hip                  |
| glmed1                    | Gluteus Medius 1               | Hip Abduction                                    | Hip                  |
| glmed2                    | Gluteus Medius 2               | Hip Abduction                                    | Hip                  |
| glmed3                    | Gluteus Medius 3               | Hip Abduction                                    | Hip                  |
| glmin1                    | Gluteus Minimus 1              | Hip Abduction and Stabilizer                     | Hip                  |
| glmin2                    | Gluteus Minimus 2              | Hip Abduction and Stabilizer                     | Hip                  |
| glmin3                    | Gluteus Minimus 3              | Hip Abduction and Stabilizer                     | Hip                  |
| grac                      | Gracilis                       | Thigh abduction, knee flexion                    | Hip, Knee            |
| iliacus                   | Iliacus                        | Femur Rotation                                   | Hip                  |
| perbrev                   | Peroneus Brevis                | Foot Eversion                                    | Ankle                |
| perlong                   | Peroneus Long                  | Foot Eversion                                    | Ankle                |
| piri                      | Piriformis                     | Thigh Rotation                                   | Hip                  |
| psoas                     | Psoas Iliaco                   | Hip Rotation                                     | Hip                  |
| recfem                    | Rectus Femoris                 | Knee Extension                                   | Knee                 |
| sart                      | Sartorius                      | Hip and Knee Movement                            | Hip, Knee            |
| semimem                   | Semimembranosus                | Hip and Knee Movement                            | Hip, Knee            |
| semiten                   | Semitendinosus                 | Hip and Knee Movement                            | Hip, Knee            |
| soleus                    | Soleus                         | Plantar Flexion                                  | Ankle                |
| tfl                       | Tensor Fasciae Latae           | Knee Rotation                                    | Knee                 |
| tibant                    | tibialis anterior              | Plantar Dorsiflexion                             | Ankle                |
| tibpost                   | tibialis posterior             | Plantar Flexion and inversion                    | Ankle                |
| vasint                    | vastus intermedius             | Knee Extension                                   | Knee                 |
| vaslat                    | vastus lateralis               | Knee Extension                                   | Knee                 |
| vasmed                    | vastus medialis                | Knee Extension                                   | Knee                 |
| osl_knee_torque_actuator  | Knee Motor                     | Knee Flexion                                     | Knee                 |
| osl_ankle_torque_actuator | Ankle Motor                    | Ankle Flexion                                    | Ankle                |


## Joints/DoF Description:
| **Joint** | **Name**                        |
|-----------|---------------------------------|
| 1         | Hip Flexion Right               |
| 2         | Hip Adduction Right             |
| 3         | Hip Rotation Right              |
| 4        | OSL Knee Angle Right             |
| 5        | OSL Ankle Angle Right            |
| 6        | Hip Flexion Left                 |
| 7        | Hip Adduction Left               |
| 8        | Hip Rotation Left                |
| 9        | Knee Angle Left Translation 2    |
| 10        | Knee Angle Left Translation 1   |
| 11        | Knee Angle Left                 |
| 12        | Knee Angle Left Rotation 2      |
| 13        | Knee Angle Left Rotation 3      |
| 14        | Ankle Angle Left                |
| 15        | Subtalar Angle Left             |
| 16        | MTP Angle Left                  |
| 17        | Knee Angle Left Beta Translation 2 |
| 18        | Knee Angle Left Beta Translation 1 |
| 19        | Knee Angle Left Beta Rotation 1 |

## Muscles removed from trans-feromal leg
26 Muscles were removed according to the suggestions made in [1]. The removed muscles are listed in the table below:

| **Name in MyoLeg**        | **Real Name**                  | **Primary Function**                             | **Joint Movement**   |
|---------------------------|--------------------------------|--------------------------------------------------|----------------------|
| sart                      | Sartorius                      | Hip and Knee Movement                            | Hip, Knee            |
| recfem                    | Rectus Femoris                 | Knee Extension                                   | Knee                 |
| vasint                    | vastus intermedius             | Knee Extension                                   | Knee                 |
| vaslat                    | vastus lateralis               | Knee Extension                                   | Knee                 |
| vasmed                    | vastus medialis                | Knee Extension                                   | Knee                 |
| gaslat                    | Gastrocnemious Lateral         | Plantar Flexion, knee Flexion                    | Ankle                |
| gasmed                    | Gastrocnemious medial          | Plantar Flexion, knee Flexion                    | Ankle                |
| bflh                      | Bicep Femoral Long Head        | Thigh extension, rotation and Knee flexion       | Hip, Knee            |
| bfsh                      | Bicep Femoral Short Head       | Knee Rotation                                    | Knee                 |
| semimem                   | Semimembranosus                | Hip and Knee Movement                            | Hip, Knee            |
| semiten                   | Semitendinosus                 | Hip and Knee Movement                            | Hip, Knee            |
| addmagDist                | Adductor magnus distal         | Pelvis stabilizer                                | Hip, Knee            |
| addmagIsch                | Adductor magnus ischial        | Pelvis stabilizer                                | Hip, Knee            |
| addmagMid                 | Adductor magnus mid            | Pelvis stabilizer                                | Hip, Knee            |
| addmagProx                | Adductor magnus proximal       | Pelvis stabilizer                                | Hip, Knee            |
| grac                      | Gracilis                       | Thigh abduction, knee flexion                    | Hip, Knee            |
| tfl                       | Tensor Fasciae Latae           | Knee Rotation                                    | Knee                 |
| tibant                    | tibialis anterior              | Plantar Dorsiflexion                             | Ankle                |
| soleus                    | Soleus                         | Plantar Flexion                                  | Ankle                |
| edl                       | Extensor Digitorum Longus      | Digits movement                                  | Digits               |
| ehl                       | Extensor Hallucis longus       | Digits movement                                  | Digits               |
| perbrev                   | Peroneus Brevis                | Foot Eversion                                    | Ankle                |
| perlong                   | Peroneus Long                  | Foot Eversion                                    | Ankle                |
| tibpost                   | tibialis posterior             | Plantar Flexion and inversion                    | Ankle                |
| fdl                       | Flexor Digitorum Longus        | Digits movement                                  | Digits               |
| fhl                       | Flexor Hallucis Longus         | Digits movement                                  | Digits               |

[1] Raveendranathan, V., Kooiman, V. G. M., and Carloni, R., 2023, “Musculoskeletal Model of Osseointegrated Transfemoral Amputees in OpenSim,” PLOS ONE, 18(9), p. e0288864.


## ChangeLog