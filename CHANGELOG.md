# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **`LTpT_T12_l` path (all models with the torso chain).** Five path points of the left T12 longissimus had the right muscle's coordinates, so it seems to have run down the right side of the spine; their z is now negated. Downstream: at qpos = 0, with every other coordinate held fixed, this muscle's moment arm about `lat_bending` goes from +26.86 mm to -26.86 mm, so policies trained on the old path should be re-evaluated or retrained.
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
