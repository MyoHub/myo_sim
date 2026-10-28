# Changelog

Model changes that alter actuators, joints, sites or dynamics are breaking for downstream environments and trained policies. List them here; they ship together in the next release tag.

## Unreleased

### Changed (breaking)
- **The muscular neck is part of `myofullbody`.** The HYOID cervical chain (24 joints realizing 6 independent DOFs, 18 coupling equalities, 72 Mortensen 2018 muscles) replaces the rigid head scaffold. The `include_neck` build flag and the `myofullbody_neck` model are removed; `myohead` stays as the standalone neck. All part models (`myotorso`, `myotorso_arms`, `myotorso_arm_r`, `myotorso_abdomen`, `myoarms`, `myoarm_r`, `myohands`, `myohand_r`, `myolegs`, `myolegs26`, `myolegs_abdomen`) keep the rigid head and their joint and actuator layout.
  - `myofullbody`: 123 → 147 joints, nq 129 → 153, nv 128 → 152, 416 → 488 actuators, 424 → 496 tendons, 51 → 69 equalities, 104 → 112 bodies; total mass 84.26 → 83.99 kg.
  - **Actions:** the 72 neck actuators are appended after all existing actuators; existing actuator, tendon and equality indices are unchanged. Action spaces grow by 72.
  - **Observations:** the 24 neck joints follow the lumbar joints, so every arm and leg `qpos`/`qvel` index moves by 24 (`myofullbody`: neck at qpos 25-48). Sensors, contact pairs and the `head` body pose at qpos0 are unchanged.
  - **Mass and inertia:** head and neck carry the de Leva (1996) head-and-neck share of 6.94 % of body mass (skull 4.30 kg as in the source, cervical segments scaled from 3.42 to 1.53 kg), and the torso body drops from 18.619 to 14.897 kg because the OpenSim trunk mass already contained the head. Cervical inertias come from a cylinder-slice approximation (1.7–4.7·10⁻⁴ kg·m² instead of 0.04 kg·m²), and the neck joints use armature 0.01, as the torso and leg joints do, which keeps the 2 ms Euler step free of ringing.
  - **Rigid-head models** (every part model: `myotorso`, `myotorso_arms`, `myotorso_arm_r`, `myotorso_abdomen`, `myoarms`, `myoarm_r`, `myohands`, `myohand_r`, `myolegs`, `myolegs26`, `myolegs_abdomen`): same torso mass, and the `head` body carries 5.848 kg (de Leva share of the 84.26 kg rigid-head body; the convention of MyoHub/myo_sim#138). Total masses are unchanged; the mass moves from the trunk to the head.
  - **Downstream:** environments built on `myofullbody` (e.g. MyoSuite's full-body tasks) need their action/observation sizes, `qpos` index maps, init poses and keyframes updated, and trained policies must be retrained. Step cost: `mj_step` on `myofullbody` +14-17 % (CPU, one core), model memory +4 %.
