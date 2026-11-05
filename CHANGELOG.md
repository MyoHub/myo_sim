# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **Side sites for MuJoCo 4 (`myoarm_r`, `myoarms`, `myotorso_arm_r`, `myotorso_arms`, `myofullbody`).** MuJoCo 4 chooses a wrap's side differently. The side sites of TMAJ's and LAT2's cylinder move, and SUP's moves from the radius to the ulna, which gives the same path on MuJoCo 3 and 4 there. No path changes on MuJoCo 3 within the joint ranges; past the supination limit SUP stays on its arc where it used to flip by 2.8 mm. Other shoulder wraps still differ on MuJoCo 4 because their MuJoCo 3 path flips sides inside the range: `build/data/side_sites.json`, `kept`, and `docs/wiki/model-authoring.md`.

## 0.2.4

The body of 0.2.3 with fixes. Apart from the items below, every `myofullbody` model array is identical to 0.2.3, and tendon lengths and actuator forces match on 6000 random postures.

- **`standing` keyframe (`myofullbody`, `myolegs`, `myolegs_abdomen`).** `myo_sim.load()` and `load_model()` now return `nkey == 1` for these models: the joint couplings hold, the feet are on the floor, and in `myofullbody` the arms clear the thorax by 6.2 mm. `qpos0` and every model parameter are unchanged. `load_spec()` and `build_spec()` leave the keyframe out unless you pass `build_kwargs={"postures": True}`, so joints can still be deleted from their specs. `myo_sim.list_postures()` and `add_posture_keyframes()` read the posture table, which holds only `standing` in this release. Downstream: code that reads `key_qpos[0]` when `nkey > 0` starts from this pose if its model comes from `load()` or `load_model()`. See `docs/wiki/postures.md`.
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

- **mujoco-warp version warning.** `myo_sim.load_spec()`, and with it `load()` and `load_model()`, warns once per process when mujoco-warp older than 3.16 is installed and the model has arm or hand tendons, whose lengths those versions can get wrong by up to about 3 mm (google-deepmind/mujoco_warp#1713). No model change.
- **Motor finger gears (`legacy/finger/motorfinger_v0.xml`, #136).** The gears of `extn`, `adabR`, `adabL`, `mflx` and `dflx` go from 20, 5, 5, 10, 10 to 160, 80, 80, 160, 160, to match the capabilities of the muscle finger. Downstream: anything that scales these gears multiplies the new values. MyoSuite's `motor_finger` recipe scales them by 4, so it drops that scale when it moves to this release, and its motorFinger checkpoints, made with 80, 20, 20, 40, 40, need retraining.
