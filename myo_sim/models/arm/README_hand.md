# MyoHand

MyoHand is a MuJoCo musculoskeletal model of the right wrist and hand, derived from the MoBL upper-extremity OpenSim model and the 2nd-Hand model.

## Anatomical scope

| Property | Value |
|---|---|
| Degrees of freedom | 23 |
| Actuators (muscles) | 45 (39 + intrinsic AdPt, AdPo, FPBs, FPBd, APB, DI1s; RI2 is DI1d) |
| Body segments | radius, ulna, carpals (scaphoid, lunate, triquetrum, pisiform, trapezium, trapezoid, capitate, hamate), metacarpals (1-5), proximal/middle/distal phalanges (digits 2-5), thumb phalanges |
| Primary joints | forearm pronation/supination, wrist flexion/deviation, thumb CMC/MCP/IP, finger MCP/PIP/DIP (digits 2-5) |

## Reference model

- **Source:** [MoBL Upper Extremity Dynamic Model](https://simtk.org/projects/upexdyn/) and [2nd-Hand](https://simtk.org/projects/hand_muscle)
- **Paper:** Holzbaur et al., 2005 ([DOI](https://doi.org/10.1007/s10439-005-3320-7))

## Fidelity

![DELT1 left/right shoulder elevation relationship](../../../docs/images/RI2_RI2_l_mcp2_flexion_r.png)

Example fidelity check (RI2, now DI1d, left/right moment-arm and force-length plot) for symmetry and muscle jumping for hand model. The full hand arm muscle check can be run with `tests/debug_muscle_bimanual.py`.


## Known limitations

None currently tracked.

## Manual adjustments

- Adjustments post conversion to optimize for kinematic and dynamic behaviors.
- Inertial properties.
- Dynamics properties of the joints.
- Contact geometries manually designed with references; fingertip and palm pads fitted to Dallard et al. 2018 and to the fist, and 144 self-contact pairs per hand (`docs/wiki/hand-contacts-and-pads.md`).
- Contact properties optimized for contact-rich behaviors.

## Changelog

**2026-10-29** — Thumb and first-dorsal-interosseous heads: `AdP` -> `AdPt` + `AdPo`, `FPB` -> `FPBs` + `FPBd`, `RI2` -> `DI1d` + `DI1s`; OP, APB, FPB and DI1 sites fitted to published moment arms, thumb intrinsic passive scales and springs refitted (+3 actuators per hand). See `docs/wiki/passive-moments-and-thenar.md`.

**2026-10-29** — PIP flexion limit of digits 2-5 raised from 90 to 115 deg (Leijnse et al. 2010), palm pads refitted (palm 8.0 mm), EDM without the `Fifthpm_wrap_r` wrap. See `docs/wiki/passive-moments-and-thenar.md`.

**2026-10-28** — Finger MCP, PIP and DIP damping set to Kamper et al. 2002 (0.0142, 0.0105, 0.0081 N m s/rad; was 0.05), and RI2 routed without the second-metacarpal wrap its insertion entered (its length jumped by 7 mm near 10 deg MCP flexion). Both change the dynamics of every model with a hand. See `docs/wiki/passive-moments-and-thenar.md`.

**2026-06-17** — Moved the hand README under the arm model folder and refreshed it to the standard model README format.

**2026-06-16** — Added MyoHand equivalence coverage and documented MjSpec hand composition.

**2026-06-04** — Initial MyoHand README described the MoBL and 2nd-Hand reference models and conversion process.

## Citation

See repository README.
