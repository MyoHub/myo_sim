# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **Knee joint centre (`myolegs`, `myolegs_abdomen`, `myofullbody`).** The three knee hinges of each leg now sit at Rajagopal's walker_knee joint frame instead of the tibia's origin. Seen from the tibia, the femur's origin was 4.80, 12.75 and 15.53 mm off OpenSim's at 30, 90 and 120 deg of flexion and now matches it within 0.0007 mm at 0, 30, 90 and 120 deg. Downstream: the knee's kinematics change, and so may the moment arms of the muscles crossing it; policies trained on the old knee should be re-evaluated or retrained.
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
