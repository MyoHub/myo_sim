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

- `BIClong_tendon` (both arms, all arm models) no longer wraps `Elbow_BIC_BRD_ellipsoid_BIClong`. Within the elbow range the wrap touched the path in 0.05 % of postures and the change is below 0.4 µm in length and 0.04 mm in moment arm. Past the extension limit, where soft limits let the elbow sit, the wrap made BIClong 15-20 mm longer at once when its sidesite flipped sides, and its onset at the limit returned NaN lengths in MuJoCo < 3.16 and a full-circle length in mujoco-warp. Policies that drive the elbow into hyperextension see a different BIClong force there.
