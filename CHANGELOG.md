# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **Complete imported keyframe states.** `build.utils.add_keyframes()` now preserves muscle activation, actuator control and mocap position/orientation (`act`, `ctrl`, `mpos`, `mquat`) alongside time, position and velocity. Resetting an imported keyframe previously silently used defaults for these fields. Unnamed keyframes are accepted, and omitted fields and partial state arrays retain MuJoCo's native defaults.

- **`myo_sim.physics`: joint couplings and 3CC-r muscle fatigue (from MyoSuite).** `joint_coupling` puts coupled joints on their polynomials (NumPy or torch, in place or differentiable), `fatigue` holds the 3CC-r model with its cited per-group parameters (`CumulativeFatigue` in NumPy, `TorchFatigueState` batched in torch and fused with `torch.compile` on CUDA), `fatigue_jax` the same step in JAX, and `muscle_groups` the functional muscle groups. NumPy is enough to import them; torch and JAX are optional (`myo-sim[torch]`, `myo-sim[jax]`). No model file changes.
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.

## 0.2.5

The body of 0.2.4: no model file changes, so every model array is identical to 0.2.4. This release adds runtime code.

- **`myo_sim.physics`: joint couplings and 3CC-r muscle fatigue (from MyoSuite).** `joint_coupling` puts coupled joints on their polynomials (NumPy or torch, in place or differentiable), `fatigue` holds the 3CC-r model with its cited per-group parameters (`CumulativeFatigue` in NumPy, `TorchFatigueState` batched in torch and fused with `torch.compile` on CUDA), `fatigue_jax` the same step in JAX, and `muscle_groups` the functional muscle groups. NumPy is enough to import them; torch and JAX are optional (`myo-sim[torch]`, `myo-sim[jax]`). No model file changes.


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

- **`mujoco<4`.** MuJoCo 4 changes how spatial tendons use side sites, which moves some muscle paths in these models, so this release stays on MuJoCo 3.
