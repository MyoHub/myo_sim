# Passive joint moments and thenar muscles

Every composed model with an arm or hand (`myofullbody`, `myotorso_arms`, `myoarms`,
`myohands`, `myoarm_r`, `myohand_r`, ...) carries, by default, the thenar muscles and the
measured passive joint moments of the hand. Both are XML fragments of the right arm,
merged by `build/compose.py` into the right-arm child XML and, mirrored, into the left
one (`right_arm_optional_fragments()`); hand models keep them through the arm-to-hand
pruning. Opt-out flags select which fragments are merged:

| Fragment (`models/arm/assets/`) | Content | Merged when |
| --- | --- | --- |
| `myoarm_r_chain.xml` | thenar attachment sites (inert) | always |
| `myoarm_r_thenar_tendon.xml`, `myoarm_r_thenar_muscle.xml` | AdP, FPB, APB | `thenar_muscles=True` (default) |
| `myoarm_r_passive_tendon.xml` | fixed tendons of the `measured` rows | `passive_moments=True` (default) or `"assumed"` |
| `myoarm_r_passive_assumed_tendon.xml` | fixed tendons of the `assumed` rows | `passive_moments="assumed"` |

```python
from myo_sim import load_model, load_spec
model = load_model("myofullbody")                                   # thenar muscles + measured rows
model = load_model("myofullbody", passive_moments="assumed")        # plus assumed transfers
model = load_model("myofullbody", thenar_muscles=False, passive_moments=False)  # previous model
spec = load_spec("myofullbody", subject_scale={"digit2_pip_flexion": 1.2, "AdP.force": 0.8})
```

**Downstream changes.** The thenar muscles add 3 actuators per hand (`AdP`, `FPB`,
`APB`; `_l` on the left, `_r` in hand-only models), appended after each arm's other
muscles: `myoarm_r` 63 -> 66, `myoarms` 126 -> 132, `myohand_r` 39 -> 42, `myohands`
78 -> 84, `myofullbody` 416 -> 422 (the left-arm and leg actuators shift by 3 or 6
indices). Policies and observation layouts trained on the previous models need
`thenar_muscles=False`, which restores the previous action space; with both flags off
the compiled model equals the previous one except for the 8 inert thenar sites per hand.
The polynomial tendon stiffness needs MuJoCo >= 3.7 (the package floor).

The XML is the model; the JSON tables in `build/data/` are its provenance and fitting
input. `scripts/fit_passive_moments.py` refits the springs and writes both passive
fragments (`--xml-only` rewrites them from the JSON as is);
`scripts/generate_thenar_fragments.py` writes the thenar tendon and muscle fragments.
`tests/test_passive_thenar.py` checks that the fragments on disk are what the scripts
write. `myo_sim.build.hand_data` reads the tables, names the passive tendons and applies
`subject_scale`.

## Passive joint moments (`myoarm_r_passive*_tendon.xml`, `build/data/passive_joint_moments.json`)

Each row is one joint. A spring segment is a one-joint fixed tendon
(`passive_<row>_<pos|neg>[i]_r`, mirrored to `_l`) whose spring acts
beyond an onset angle (absolute, joint coordinates) with restoring moment
`-(k0 x + k1 x^2 + k2 x^3)`; MuJoCo and mujoco-warp evaluate it natively
(`tendon_lengthspring`, `tendon_stiffnesspoly`). Segments can be stacked per side.
Every applied row cites its source and keeps the source curve (published model and
parameters) under `source_curve`; `scripts/fit_passive_moments.py` refits the stored
springs from it and records the fit error. Where a source reports a *total* passive
moment, the model's own muscle passive moment at the measured posture is subtracted
first (`subtract_model_muscles`), and where the model's muscles already exceed the
measured total the spring adds nothing (`model_excess_max_Nm`).

| Row | Status | Source | Segments (+/-) | RMSE (N m) | Max err (N m) | Source peak (N m) | Model excess (N m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| wrist flexion, deviation | measured | Formica 2012 | 1+1 | linear, exact | | | |
| forearm pro/supination | covered_by_muscles | Formica 2012 (reference only) | | | | | |
| thumb CMC flexion | measured | Domalain 2010 | 3+0 | 0.0295 | 0.0756 | 1.498 | 0 |
| thumb CMC abduction | measured | Domalain 2010 | 3+1 | 0.0694 | 0.1543 | 1.384 | 0 |
| thumb MP flexion | **assumed** | Barry 2018 (= index MCP, Kamper 2002) | 1+1 | 0.0187 | 0.0422 | 0.054 | 0.031 |
| thumb IP flexion | **assumed** | Barry 2018 (= index PIP, Kamper 2002) | 4+4 | 0.0007 | 0.0019 | 0.052 | 0 |
| index MCP flexion | measured | Knutson 2000, intrinsic only | 1+1 | 0.0003 | 0.0014 | 0.040 | 0 |
| index PIP flexion | measured | Kamper 2002 | 4+4 | 0.0011 | 0.0023 | 0.047 | 0 |
| index DIP flexion | measured | Kamper 2002 | 4+0 | 0.0031 | 0.0090 | 0.080 | 0 |
| MCP 3-5 flexion | **assumed** | index curve (Knutson 2000) | 1+1 | 0.0003 | 0.0014 | 0.040 | 0 |
| PIP 3-4 flexion | **assumed** | index curve (Kamper 2002) | 4+4 | 0.0011 | 0.0023 | 0.052 | 0 |
| PIP 5 flexion | **assumed** | index curve (Kamper 2002) | 1+1 | 0.0205 | 0.0345 | 0.041 | 0.125 |
| DIP 3-5 flexion | **assumed** | index curve (Kamper 2002) | 4+0 | 0.0031 | 0.0090 | 0.080 | 0 |
| all MCP abduction | gap | no measured passive data | | | | | |

**Assumed rows stay opt-in.** With `act = 0`, other joints at 0 and the wrist at -30 / 0 /
+30 deg, the total passive moment (muscles + springs) of each assumed row was compared
with its source curve over the measured range. Nine rows match it within the fit error
(<= 0.011 N m) and add nothing where the model's muscles already act (thumb IP, MCP 3-5,
PIP 3-4, DIP 3-5). Two do not: at thumb MP flexion the thumb muscles (FPB, AdP) alone
give up to 0.091 N m against a 0.061 N m source peak, and the spring still adds up to
0.028 N m on the same side (total 0.119 N m); at PIP 5 the digit-5 extensors give 0.166 N m
against 0.052 N m and the spring adds 0.019 N m. So the assumed fragment is not merged by
default.

Mappings and caveats (details in each row's `notes`):

- **Formica 2012** (healthy, n = 10): direction-specific linear stiffness about neutral,
  valid within about +-17 deg; no end-range stiffening.
- **Domalain 2010** (healthy, n = 10): Mp is the *external* moment (internal = -Mp), in
  Chao et al. TMC angles relative to each subject's maxima; aligned with MyoHand's
  extension / abduction limits. Left out: the F-E abduction term (averaged coefficients
  give ~11 N m at the resting adduction vs <= 1.9 N m measured) and the A-A flexion term
  (single-joint springs). The adduction side is linear: a cubic extrapolates to 5.5-6.8
  N m at MyoHand's adduction limit, 26 deg past the measured span.
- **Hollister 1992**: thumb CMC axes 72.7 +- 8.4 deg apart, flexion-extension in the
  trapezium, abduction-adduction in the metacarpal, not intersecting. MyoHand's are 80.9
  deg apart and intersect (both on `firstmc`); the Domalain mapping is by joint limits.
- **Knutson 2000** (healthy, n = 8, Table 1 medians): only the intrinsic component; the
  extrinsic component belongs to the extrinsic muscles. MyoHand's FDS/FDP/EDC/EI give at
  most 1.2 N cm at the index MCP (Knutson: up to 30 N cm), so tenodesis is missing until
  their passive tension is restored (a separate change).
- **Kamper 2002** (Table 2 static torque, wrist neutral, other finger joints fixed;
  operating points -10..60 deg): total moments, so the model's muscle moment is
  subtracted. Outside -10..60 deg the torque is held at its edge values: the PIP cubic
  turns back toward flexion above 45.5 deg (+0.064 N m at 90 deg), the DIP cubic reaches
  -0.25 N m at 90 deg. Kamper's simulation used the angle relative to the proximal joint;
  measured with that joint at neutral, which is what a single-joint spring reproduces.
  The DIP cubic crosses zero near -14 deg although the paper states 7 deg as the
  zero-torque DIP angle; used as published. Damping B (0.81 / 1.05 / 1.42 x 1e-2 N m s/rad
  for DIP / PIP / MCP) is recorded but not applied (MyoHand: 0.05 N m s/rad).
- **Li 2006** (index PIP, MCP 0 / 60 deg): kept as a validation target in the PIP row.
- **Assumed rows** are transfers, not measurements of those joints: Barry et al. 2018 set
  the thumb MCP / IP equal to the index MCP / PIP; McFarland et al. 2023 scaled the index
  PIP/DIP curves for digits 3-5 (scale factors not available, so transferred unscaled).
- Model issues found on the way: an RI2 path jump between 10 and 15 deg MCP flexion; no
  MCP hyperextension (range starts at 0 deg; measured passive extension reaches -65 deg,
  Binder-Markey 2021 data); the digit-5 extensors and, with the thenar architecture, AdP
  already exceed the measured totals (model excess column).
- **Thumb CMC with the thenar muscles**: the CMC rows were fitted without subtracting
  muscle moments (MyoHand's own muscles give <= 2.1 N cm there), but the AdP's passive
  force adds up to 0.62 N m at the CMC extension limit and 0.55 N m at the abduction
  limit (Domalain peaks: 1.50 / 1.38 N m), so the default model exceeds the measured CMC
  totals near those limits. A refit with `subtract_model_muscles` (thenar on) or a better
  AdP path would remove it.

## Thenar muscles (`myoarm_r_thenar_*.xml`, `build/data/thenar_muscles.json`)

Adductor pollicis (AdP), flexor and abductor pollicis brevis (FPB, APB) on every hand,
in the arm's `myoarm_forearm_muscle` class (the class of OP).

| Muscle | PCSA (cm2) | Peak force (N) | Optimal fibre length (mm) | Pennation (deg) | MUSIC force (N) |
| --- | --- | --- | --- | --- | --- |
| AdP (Jacobson "AddP") | 1.94 | 167.5 | 34.0 | 17.3 | 100.0 |
| FPB | 0.66 | 57.0 | 41.5 | 6.2 | 80.0 |
| APB (Jacobson "AbPB") | 0.68 | 58.7 | 41.6 | 4.6 | 42.7 |

- **Sites**: MUSIC (Xu et al. 2026, ACM TOG, doi:10.1145/3811402; code MIT, see
  `build/data/LICENSE-MUSIC.txt`), on MyoHand's own bodies (frames identical to MUSIC's;
  note MUSIC's `cmc_flexion` / `cmc_abduction` names are swapped relative to MyoHand).
- **Architecture**: Jacobson et al. 1992 Table I (n = 9; the text quotes AddP PCSA 1.10
  cm2, the table 1.94, used). Stamenkovic et al. 2014 (two AdP heads, combined PCSA 3.19
  cm2) is recorded as an alternative; the MUSIC path is a single head.
- **Force** = PCSA x 86.3 N/cm2: the least-squares specific tension of MyoHand's OP,
  EPB and APL against Jacobson's PCSA (individually 176.5, 97.9, 60.5 N/cm2, so
  MyoHand implies no single value).
- **Length range**: static `lengthrange` written by the generator, a rigid tendon of
  slack length `(L_MT(q = 0) - L_fo cos(alpha)) / 1.033` (Barry et al. 2018, Eq. 1), so
  the class's normalised fibre-length range 0.75-1.05 spans the same range of L_fo.
  MUSIC's fixed values (`source_lengthrange_m`, which do not cover MUSIC's own tendon
  lengths) are kept in the table for reference only.
- **Known issue**: MUSIC's straight AdP path gives ~47 mm excursion over the thumb range
  against 34 mm fibres, so its normalised length spans ~0.2-1.55 (no active force when
  short, strong passive force when long). Barry et al. 2018 used two heads with wrapping
  surfaces; their model (simtk, MIT) would give a better path.
- Activation dynamics and force-length shape come from the class, as for OP.

## Subject scaling (`subject_scale`)

Values are absolute SI. Thenar rows tag `force_N` and `pcsa_cm2` with `scales_with:
mass`, `optimal_fiber_length_m` with `length` (reference segment: hand); passive rows are
`scales_with: none` (measured stiffness is kept unless a subject measurement exists).
`subject_scale` takes `{row_name: factor}` (`"digit2_pip_flexion"`, `"AdP.force"`,
`"AdP.length"`) and applies the factors to the built spec by element name
(`hand_data.apply_subject_scale`, a spec-level step like the sensor and keyframe
injection), without editing the fragments: a row factor multiplies the stiffness of that
row's tendons on both sides, `.force` the peak force, `.length` the optimal fibre length
(the length range is re-derived with the same slack-length rule). Keys for elements
absent from the model are ignored.

## Licences and sources

Papers are cited only; no figure, table or text is copied. Numbers come from: Formica et
al. 2012 (J Neurophysiol 108:1158), Knutson et al. 2000 (J Biomech 33:1675), Kamper et al.
2002 (J Biomech 35:1581), Li et al. 2006 (J Orthop Res 24:407), Domalain et al. 2010 (J
Biomech Eng 132:071009), Hollister et al. 1992 (J Orthop Res 10:454), Barry et al. 2018 (J
Biomech 77:206), Jacobson et al. 1992 (J Hand Surg Am 17:804), Stamenkovic et al. 2014
(Muscle Nerve 49:405), Binder-Markey et al. 2021 (Front Neurol 12:687624; range-of-motion
data for validation), McFarland et al. 2023 (IEEE TBME 70:1424; cited for its approach
only: the ARMS model is non-commercial and nothing is taken from it). MUSIC's site
coordinates are used under its MIT licence.
