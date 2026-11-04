# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## 0.2.4

The body of 0.2.3 with fixes. Apart from the items below, every `myofullbody` model array is identical to 0.2.3, and tendon lengths and actuator forces match on 6000 random postures.

- **`standing` keyframe (`myofullbody`, `myolegs`, `myolegs_abdomen`).** `myo_sim.load()` and `load_model()` now return `nkey == 1` for these models: the joint couplings hold, the feet are on the floor, and in `myofullbody` the arms clear the thorax by 6.2 mm. `qpos0` and every model parameter are unchanged. `load_spec()` and `build_spec()` leave the keyframe out unless asked, so joints can still be deleted from their specs. Downstream: code that reads `key_qpos[0]` when `nkey > 0` starts from this pose if its model comes from `load()` or `load_model()`. See `docs/wiki/standing-keyframe.md`.
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

- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
