# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **Leg and arm muscle maps (myolegs, myoarm, and every composed model that includes them).** Adopted muscles use a rigid-tendon map fitted to the published OpenSim fibre and tendon lengths over the full joint range (`scripts/fit_leg_rigid_maps.py`, `scripts/fit_arm_rigid_maps.py`), then a Silder 2007 passive slack/`fpmax` fit on the legs (`scripts/fit_leg_passive.py`). `semimem` standing \(\ell_n\) moves from ~1.86 to ~0.86; `vasint` \(L_0\) from 17.3 cm to 9.95 cm. `recfem`, `vaslat` and `glmax3` keep their previous maps. Downstream: fibre-length observations and passive/active forces change; policies trained on #123 or earlier plants should be retrained.
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
