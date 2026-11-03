# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **Abdomen variants, finger-collision option and mirrored cameras (#150).** Changes by part:
  - `myolegs_abdomen` and `myotorso_abdomen` no longer apply an extra −90° yaw at the root, so they face −y like every other model.
  - `myolegs_abdomen` gets the 26 leg-to-leg contact pairs of `myolegs`.
  - `build_kwargs={"disable_finger_collisions": True}` again removes the explicit pairs on the geoms it disables, as before 53cc333: 33 in `myoarms`, 49 in `myofullbody` and 15 in `myohands`.
  - The mirrored left-arm cameras look at the left hand.
  - `build_kwargs` no longer change later builds of the same model.

  Downstream:
  - Environments and policies built on the abdomen variants see a rotated world frame and new leg contacts.
  - Coarse-collision arm models lose the forearm contacts with the thorax and the other arm.

- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
