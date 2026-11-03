# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

- **Model scaling.** `myo_sim.scale_spec` scales a model to known factors with the rules of OpenSim's Scale Tool and returns a new spec; `scale_report`, `segment_factors`, `fit_length_ranges`, `check_scaled_model` and `model_digest` go with it. See `docs/wiki/model-scaling.md`. No model changes: nothing runs at load time, and registered models, downstream environments and trained policies are unaffected unless a caller scales a model. `check_scaled_model` holds operating range and moment-arm signs to a bound per muscle, so factors other than a similarity usually fail `operating_range` before `fit_length_ranges`, and a changed wrap path fails after it too.
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of `myofullbody` is unchanged at 84.26 kg and the whole-body centre of mass rises by 1.4 cm. The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
