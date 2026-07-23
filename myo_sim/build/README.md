# MjSpec Prototypes

This folder contains prototype utilities for composing MuJoCo models with
`mujoco.MjSpec` instead of relying only on static MJCF includes.

The main entry point is:

```bash
uv run python -m myo_sim.build.compose --model myotorso_arms --view
```

Available registered models:

- `myotorso_arms`: full torso plus right arm and a mirrored-left arm.
- `myoarm_r`: passive anatomical torso scaffold plus right arm only.
- `myotorso_arm_r`: full torso plus right arm only.
- `myofullbody`: torso, mirrored arms, and legs.
- `myolegs26`: passive anatomical torso scaffold with reduced 26-muscle legs.
- `myolegs_abdomen`: simple abdomen scaffold plus legs.
- `myoarms`: passive anatomical torso scaffold plus mirrored arms.
- `myohand_r`: passive anatomical torso scaffold plus a right hand derived by pruning `myoarm_r`.
- `myohands`: passive anatomical torso scaffold plus right and mirrored-left hands derived from arm specs.

`utils.py` contains shared XML and mirroring helpers. `hand.py` contains the
hand-specific pruning logic used to derive hand-only specs from arm specs.
