# Hand self-contacts and pads

How the right hand's soft-tissue pads and self-contact pairs are built (the left hand is mirrored), and
how they are checked. Numbers against the literature: `hand-literature-comparison.md` (S1).

| What | Where | Written by |
| --- | --- | --- |
| Fingertip pads (`distph{2,3,4}_coll_2_r`, `5distph_coll_2_r`, `distal_thumb_coll_2_r`), palm, thenar and hypothenar pads (`palm_pad_r`, `thenar_pad_r`, `hypothenar_pad_r`), distal capsule positions | `myoarm_r_chain.xml` | `scripts/fit_hand_pads.py` from `build/data/hand_pads.json` |
| Self-contact pairs of both hands | `myo_sim/models/contacts/myohand_self_contacts.xml` (generated block) | `scripts/generate_hand_contacts.py` from `build/data/hand_contact_pairs.json` |

`compose.py` adds the self-contact file to each arm fragment (`add_hand_self_contacts`), so every model
with a hand carries the pairs: `myoarm_r`, `myoarms`, `myotorso_arms`, `myotorso_arm_r`, `myofullbody`,
and, through the pruning, `myohand_r`, `myohands` and the bare hand fragments. The pads are collision
geoms only (group 4, contact through explicit pairs); the hand bodies have explicit inertials, so masses
and inertias are unchanged.

## Fingertip pads

**Finding.** The five ellipsoids that #123 added as fingertip pads sat on the dorsal (nail) side of the
distal phalanges: 1.5-5 mm dorsal of the bone centroid, with palmar defined by the direction FDP (FPL for
the thumb) moves the tip. The palmar soft tissue came from the distal capsules alone.

**Target.** Dallard et al. 2018 (J Biomech 67:166, MRI of the index fingertip, 5 men): soft tissue normal to
the pad D_N0 = 6.4 ± 0.8 mm, measured from the distal palmar corner of the phalanx perpendicular to its
long axis (their Fig. 2). Only the index is measured; the other digits get the same value scaled by
their distal-phalanx length (assumption, status `transferred`).

**Measurement** (`fingertip_soft_tissue_mm`). Long axis from the DIP (IP) centre to the farthest bone
vertex; palmar direction from the long flexor; the ray through the bone's mid-line at the corner station
(2 mm, half of Dallard's 4 mm phalanx height, proximal of the tip), from the outermost collision geom of
the phalanx to the bone mesh. Before the change: 6.4 / 8.7 / 9.5 / 6.8 / 4.9 mm (index, middle, ring,
little, thumb).

**Fit.** Each pad becomes a palmar ellipsoid (semi-axes 6 × 8 × 2.5 mm for the index across, along and
normal to the phalanx, scaled with the digit), its distal end at the bone tip, its palmar position solved
so the skin at the corner station is at the target. A distal capsule that alone exceeds the target is
moved dorsally by the excess: middle 2.05 mm, ring 2.78 mm, little 0.73 mm (their capsules sat 2 mm
palmar of the index's). Result: 6.40 / 6.63 / 6.71 / 6.07 / 7.89 mm, equal to the targets.

**Not fitted: compliance.** Dallard's pad deflects 3.1 ± 0.2 mm at 4 N. MuJoCo's soft-contact
penetration at a given force scales with the effective inverse mass of each contact, so one `solref`
cannot reproduce a force-deflection curve across contacts; the pads use the same stiff contact
parameters as the other pairs (below).

## Palm, thenar and hypothenar pads

No palmar soft-tissue thickness is published: ultrasound gives muscle thickness only (hypothenar
8.4 ± 1.9 mm, whole thenar 26.7 ± 3.9 mm, Misirlioglu & Taskiran 2018; OP 9.7, APB 6.0, FPB 6.2 mm,
Karademir et al. 2022), which are lower bounds. The model has no skin or surface mesh that covers the
palm (only bone meshes), so the thickness cannot be read from the model either. It is therefore fitted
to the fist:

- Each pad is an ellipsoid on its metacarpal body, its palmar axis along the palmar direction of the
  digit (the way MCP flexion moves the proximal phalanx), its centre on the palmar bone surface, so
  the soft tissue under it equals its palmar semi-axis. Shapes and anchors are assumed
  (`hand_pads.json`).
- `palm_pad_r` (on MC3, over MC2-4, 40 × 36 mm): the thickness at which, with MCP, PIP and DIP of digits
  2-4 at their flexion limits, the nearest fingertip just touches it: **8.0 mm** (19.1 mm while the PIP
  flexion limit was 90 deg, before S1b).
- `hypothenar_pad_r` (on MC5, proximal two thirds): the same with digit 5: **16.8 mm** (17.0 mm before
  S1b), above the hypothenar muscle thickness (8.4 mm).
- `thenar_pad_r` (on MC1): no fingertip reaches the thenar eminence in the fist, so it takes the lower
  bound, the whole-thenar muscle thickness: **26.7 mm**.

**PIP range (S1b).** The fist criterion depends on the flexion limits. With the PIP limit at 90 deg (S1)
the fingertips landed farther from the bones than in a real fist and the palm pad took 19.1 mm. Since S1b
the PIP limit is the measured maximum passive flexion, 115 deg (Leijnse et al. 2010; see
`passive-moments-and-thenar.md`), and the palm pad is 8.0 mm, close to the hypothenar muscle thickness
(8.4 ± 1.9 mm). Rerun `scripts/fit_hand_pads.py` whenever a finger flexion limit changes.

## Self-contact pairs

**Scan** (`generate_hand_contacts.py --scan`): `myohand_r`, 3000 uniform random postures of the 20 thumb
and finger joints (seed 0) plus the full fist; signed distance (`mj_geomDistance`) for every pair of hand
collision geoms on bodies that are neither the same nor parent and child.

**Selection** (`hand_contact_pairs.json` → `selection`):

- not paired when the geoms overlap at rest, except the 13 pairs of #123;
- finger-finger and thumb-finger pairs that penetrate more than 2 mm in at least 2 % of the postures;
- thumb pulp against every fingertip pad (pinch);
- fingers and thumb against a palm geom (metacarpal capsules and pads) that they penetrate in the scan or
  come within 1 mm of in the fist;
- always: the fingertip pads, distal and middle phalanges of digits 2-5, and the distal and proximal
  thumb against the palm and hypothenar pads.

Result: **144 pairs per hand** (288 for two hands): the 13 existing, 47 finger-finger, 30 thumb-finger,
4 pulp-to-pulp and 50 against the palm. The earlier 71-pair proposal was made with the dorsal pads; with
the pads palmar the scan selects 81 new pairs that do not involve the palm, against the proposal's 59.

**Rest overlaps.** `proxph3_coll_r`–`proxph4_coll_r` (−1.5 mm) and `proxph4_coll_r`–`5proxph_coll_r`
(−0.2 mm) overlap at q = 0. They keep their geometry and get `margin = -0.002`: the contact starts only
beyond 2 mm of penetration (side-by-side fingers touch and compress slightly at rest). MuJoCo clamps
`margin - gap` at zero, so a `gap` cannot do this.

**Parameters.** `condim` 3, 4 for the pulp-to-pulp pairs (torsional friction, so a pinched pad does not
spin). Friction is MuJoCo's default (1, 0.005, 0.0001), as in the earlier finger pairs; no skin-on-skin
coefficient is sourced. `solref 0.004 1`, `solimp 0.95 0.99 0.001 0.5 2`: the stiffest contact MuJoCo
keeps stable at the 2 ms step, as for the grip dynamometer (`hand_function.json`). With the default
`solref 0.02` the fist at full FDS and FDP activation pushes the fingertips 3-8 mm into the palm.

## Checks

| Check | Result |
| --- | --- |
| Active hand contacts at q = 0 (`myohand_r`, `myohands`, `myoarm_r`, `myoarms`, `myofullbody`) | none |
| Active fist (FDS, FDP at 1, 1 s, wrist neutral, thumb locked) | penetration beyond each pair's margin ≤ 0.74 mm; digits 2, 4, 5 touch the palm (within 0.8 mm); digit 3 stops 14.4 mm short (18.5 mm before S1b), held by its neighbours |
| 20 random-activation rollouts of `myohand_r` (1.5 s each) | deepest paired penetration beyond the margin 1.6 mm; up to 9 contacts; finite |
| Unpaired penetrations in those rollouts | thumb proximal phalanx against the little finger's distal and middle phalanges (up to 14 mm: thumb-to-little opposition, below the 2 % scan threshold), the thenar pad against the MC2 capsule and the palm pad against the ring proximal phalanx near the MCP joint (pad-to-palm overlaps of one soft-tissue mass) |
| CPU vs mujoco-warp, states from random-activation rollouts | same contacts, distances within 0.008 mm |
| CPU vs mujoco-warp, random kinematic postures | same contact sets; capsule-ellipsoid depths differ by up to 5 mm at about 10 mm penetration (different deep-penetration estimates), none at realistic depths |

**mujoco-warp contact budget.** In random kinematic postures `myofullbody` produced 29 contacts on the CPU
and overflowed mujoco-warp's default `nconmax` guess (48) on the CPU device, which crashed instead of
reporting. Callers of `put_data` / `make_data` need `nconmax` of at least 64 for these models; MyoSuite's
mjlab tasks set it explicitly.

**Hand-function scenes.** `scripts/validate_hand_function.py` removes the model's own contact pairs in its
scenes (the protocol has collisions off except the dynamometer), so tenodesis and the lateral pinch are
unchanged. The grip changes: the dynamometer is placed against the palm, now the palm pad, 11 mm further
palmar (see `hand-function-validation.md`).

**Left and right hands.** The left pairs mirror the right ones in order, parameters and geom roles
(`tests/test_hand_contacts.py`). Where fingers touch at rest (both wrists extended 60 deg in
`myofullbody`), the two hands still settle up to 0.15 deg apart (MCP3, MCP4): static friction keeps
whatever posture the contact solver reaches, and at MuJoCo's default solver tolerance (1e-8, warm-started)
the solution of the two hands differs slightly. At tolerance 1e-12 the difference is 0.015 deg, without
warm start 0.010 deg, with frictionless pairs 0.013 deg and without contacts 0.002 deg. The solver options
are shared by every model, so they stay at their defaults; `test_relaxed_hand_rests_off_its_limits`
allows 0.2 deg for these two joints in that case only.
