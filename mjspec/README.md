# MjSpec Prototypes

This folder contains prototype utilities for composing MuJoCo models with
`mujoco.MjSpec` instead of relying only on static MJCF includes.

The main entry point is:

```bash
python mjspec/prototype_mjspec_attach.py --model myotorso_arms --view
```

Available registered models:

- `myotorso_arms`: full torso plus right arm and a mirrored-left arm.
- `myotorso_right_arm`: full torso plus right arm only.
- `myofullbody`: torso, mirrored arms, and legs.
- `myoarms`: lightweight torso base plus mirrored arms.
- `myohand_r`: torso base plus a right hand derived by pruning `myoarm_r`.
- `myohands`: torso base plus right and mirrored-left hands derived from arm specs.

`utils.py` contains shared XML and mirroring helpers. `hand.py` contains the
hand-specific pruning logic used to derive hand-only specs from arm specs.

