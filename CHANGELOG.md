# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- `BIClong_tendon` (both arms, all arm models) no longer wraps `Elbow_BIC_BRD_ellipsoid_BIClong`. Within the elbow range the wrap touched the path in 0.05 % of postures and the change is below 0.4 µm in length and 0.04 mm in moment arm. Past the extension limit, where soft limits let the elbow sit, the wrap made BIClong 15-20 mm longer at once when its sidesite flipped sides, and its onset at the limit returned NaN lengths in MuJoCo < 3.16 and a full-circle length in mujoco-warp. Policies that drive the elbow into hyperextension see a different BIClong force there.
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
