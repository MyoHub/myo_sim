# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

### Changed (breaking)
- **Proposal, open for discussion: the torso models carry the muscular neck too.** `myotorso`, `myotorso_arms` and `myotorso_arm_r` get the same HYOID neck as `myofullbody`, attached last; the scaffold part models (`myotorso_abdomen`, `myoarms`, `myohands`, `myolegs`, ...) keep the rigid head.
  - Each gains 24 joints, 72 actuators, 18 equalities and 8 bodies: `myotorso` 18 → 42 joints and 210 → 282 actuators, `myotorso_arms` 94 → 118 and 336 → 408, `myotorso_arm_r` 56 → 80 and 273 → 345.
  - **Actions:** the neck actuators are appended after all existing ones; existing actuator indices are unchanged.
  - **Observations:** the neck joints follow the lumbar joints, so every arm `qpos`/`qvel` index moves by 24. Keyframes indexed by position break without an error.
  - **Mass:** head and neck carry the same masses as in `myofullbody`; the totals drop by 0.27 kg (`myotorso` 37.15 → 36.88 kg).
  - **Cost:** `mj_step` time (CPU, one core) +86 % for `myotorso` (44 → 82 µs), +19 % for `myotorso_arms` (218 → 260 µs), +37 % for `myotorso_arm_r` (126 → 173 µs).
- **The muscular neck is part of `myofullbody`.** The HYOID cervical chain (24 joints realizing 6 independent DOFs, 18 coupling equalities, 72 Mortensen 2018 muscles) replaces the rigid head scaffold. The `include_neck` build flag and the `myofullbody_neck` model are removed; `myohead` stays as the standalone neck. All part models (`myotorso`, `myotorso_arms`, `myotorso_arm_r`, `myotorso_abdomen`, `myoarms`, `myoarm_r`, `myohands`, `myohand_r`, `myolegs`, `myolegs26`, `myolegs_abdomen`) keep the rigid head and their joint and actuator layout.
  - `myofullbody`: 123 → 147 joints, nq 129 → 153, nv 128 → 152, 416 → 488 actuators, 424 → 496 tendons, 51 → 69 equalities, 104 → 112 bodies; total mass 84.26 → 83.99 kg.
  - **Actions:** the 72 neck actuators are appended after all existing actuators; existing actuator, tendon and equality indices are unchanged. Action spaces grow by 72.
  - **Observations:** the 24 neck joints follow the lumbar joints, so every arm and leg `qpos`/`qvel` index moves by 24 (`myofullbody`: neck at qpos 25-48). Sensors, contact pairs and the `head` body pose at qpos0 are unchanged.
  - **Mass and inertia:** head and neck carry the de Leva (1996) head-and-neck share of 6.94 % of body mass (skull 4.30 kg as in the source, cervical segments scaled from 3.42 to 1.53 kg), and the torso body drops from 18.619 to 14.897 kg because the OpenSim trunk mass already contained the head. Cervical inertias come from a cylinder-slice approximation (1.7–4.7·10⁻⁴ kg·m² instead of 0.04 kg·m²), and the neck joints use armature 0.01, as the torso and leg joints do, which keeps the 2 ms Euler step free of ringing.
  - **Rigid-head models** (every part model: `myotorso`, `myotorso_arms`, `myotorso_arm_r`, `myotorso_abdomen`, `myoarms`, `myoarm_r`, `myohands`, `myohand_r`, `myolegs`, `myolegs26`, `myolegs_abdomen`): same torso mass, and the `head` body carries 5.848 kg (de Leva share of the 84.26 kg rigid-head body; the convention of MyoHub/myo_sim#138). Total masses are unchanged; the mass moves from the trunk to the head.
  - **Downstream:** environments built on `myofullbody` (e.g. MyoSuite's full-body tasks) need their action/observation sizes, `qpos` index maps, init poses and keyframes updated, and trained policies must be retrained. Step cost: `mj_step` on `myofullbody` +14-17 % (CPU, one core), model memory +4 %.

### Changed
- **Head and trunk mass (all models with the torso chain).** The head body now carries 5.848 kg (6.94 % of body mass, de Leva 1996) and the torso 14.897 kg instead of 18.619 kg; the total of the rigid-head full body is unchanged at 84.26 kg and its whole-body centre of mass rises by 1.4 cm; with the neck, `myofullbody` totals 83.99 kg (see above). The OpenSim torso mass already contained the head, so the head was previously about 2.5 % of body mass. Downstream: dynamics of every model with the torso and head change slightly; policies trained on the old masses (e.g. MyoSuite and MuscleMimic `myofullbody` checkpoints) should be re-evaluated or retrained, and MyoSuite parity baselines regenerated when it moves to this release.
