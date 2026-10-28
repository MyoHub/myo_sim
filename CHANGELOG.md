# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- `BIClong_tendon` (both arms, all arm models) no longer wraps `Elbow_BIC_BRD_ellipsoid_BIClong`. Within the elbow range the wrap touched the path in 0.05 % of postures and the change is below 0.4 µm in length and 0.04 mm in moment arm. Past the extension limit, where soft limits let the elbow sit, the wrap made BIClong 15-20 mm longer at once when its sidesite flipped sides, and its onset at the limit returned NaN lengths in MuJoCo < 3.16 and a full-circle length in mujoco-warp. Policies that drive the elbow into hyperextension see a different BIClong force there.
