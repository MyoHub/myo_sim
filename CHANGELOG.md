# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **Pelvis counted once (`myofullbody`, `myolegs`, `myolegs26`, `myolegs_abdomen`).** The torso chain's sacrum is now massless, so the pelvis mass is the leg chain's alone; `myofullbody` weighs 76.78 kg instead of 84.26 kg, and the head (unchanged) is 7.6 % of it. Downstream: dynamics of the four models with legs change; policies trained on them should be re-evaluated or retrained. Torso-only models only lose 7.486 kg of reported mass, since their sacrum is welded to the world.
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
