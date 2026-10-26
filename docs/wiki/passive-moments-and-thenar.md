# Passive joint moments and thenar muscles (opt-in)

Two optional build steps; with both off (the default) the model is unchanged.

```python
from myo_sim import load_spec
spec = load_spec(..., passive_moments=True, thenar_muscles=True)        # measured rows only
spec = load_spec(..., passive_moments="assumed", thenar_muscles=True)   # plus assumed rows
```

## Passive joint moments (`build/passive.py`, `build/data/passive_joint_moments.json`)

Each row is one joint. Every applied row cites its source and keeps the source curve;
`scripts/fit_passive_moments.py` refits the stored springs from it. A spring is a
one-joint fixed tendon with a polynomial spring beyond an onset angle (MuJoCo and
mujoco-warp evaluate it natively).

| Status | Applied | Rows |
| --- | --- | --- |
| measured | `passive_moments=True` | wrist flexion and deviation (Formica 2012); thumb CMC flexion and abduction (Domalain 2010); index MCP flexion, intrinsic component only (Knutson 2000) |
| assumed | only `passive_moments="assumed"` | **MCP 3-5 flexion reuse the index MCP curve** (no digit-specific data); **thumb MP reuses the index MCP curve**, as Barry et al. 2018 did |
| covered_by_muscles | never | forearm pro/supination: MyoHand's muscles already supply it (posture-dependent) |
| validation_only | never | index PIP (Li 2006): total moment, used to check the model |
| gap | never | thumb IP, all DIP, PIP 3-5, all MCP abduction |

Caveats to carry into any PR or paper:

- Assumed rows are transfers, not measurements of those joints.
- Formica's wrist rows are neutral-centred linear stiffness, valid within about +-17 deg.
- Knutson's extrinsic component is not added: it should come from the extrinsic muscles,
  but MyoHand's FDS/FDP/EDC/EI give at most 1.2 N cm at the index MCP (Knutson: up to
  30 N cm), so the model lacks tenodesis.
- Known model issues found on the way: an RI2 path jump between 10 and 15 deg MCP
  flexion, and no MCP hyperextension (range starts at 0 deg).

## Thenar muscles (`build/thenar.py`, `build/data/thenar_muscles.json`)

Adductor pollicis, flexor and abductor pollicis brevis on every hand. Sites and peak
forces from MUSIC (Xu et al. 2026, MIT code); the other muscle parameters are copied
from opponens pollicis (a placeholder until Jacobson et al. 1992 / Stamenkovic et al.
2014 or the Barry et al. 2018 model values are available).

## Flags

Both are independent post-process steps and order-insensitive with respect to other
build flags (e.g. mass correction, hip synergists).
