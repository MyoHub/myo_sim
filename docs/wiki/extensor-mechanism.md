# Finger extensor mechanism (option a)

The dorsal aponeurosis of digits 2-5 in every model with a hand: how extensor and intrinsic force reaches
the middle and distal phalanges, and the oblique retinacular ligament (ORL). Option (a) of the
extensor-mechanism proposal: fixed force fractions with MuJoCo pulley branches; no new joints or actuators.

| What | Where | Written by |
| --- | --- | --- |
| Fractions, junction geometry, ORL parameters, fit targets and results | `myo_sim/build/data/extensor_mechanism.json` | `scripts/fit_extensor_mechanism.py --fit` |
| Junction sites and band wraps (`EM{d}_*_r`), one labelled block per phalanx | `myoarm_r_chain.xml` | `scripts/fit_extensor_mechanism.py` |
| Branched tendons of EDC2-5, EIP, EDM, LU_RB2-5, UI_UB2-4 (labelled blocks where they stood, so the tendon order is unchanged) and the ligaments `lig_ORL{d}_r` (main block) | `myoarm_r_tendon.xml` | `scripts/fit_extensor_mechanism.py` |
| Length ranges of LU_RB2-5 and UI_UB2-4 | `myoarm_r_muscle.xml` | `scripts/fit_extensor_mechanism.py` |
| Slack lengths and passive scales of the extrinsic extensors | `myoarm_r_muscle.xml`, `build/data/muscle_passive.json` | `scripts/fit_muscle_passive.py` |

The left hand is mirrored from these files like every other right-arm element.

## Topology

Per digit d, on the proximal phalanx: the hood `EM{d}_H_r` (where the extensor's main path ends, at the old
EDC{d}-P5 site), a central-slip via site `EM{d}_CSP_r` (old EDC{d}-P6), the radial and ulnar band sites
`EM{d}_RB_r` / `EM{d}_UB_r`, and a band wrap cylinder `EM{d}_band_wrap_r` coaxial with the PIP; on the middle
phalanx the central-slip insertion `EM{d}_CS_r` (old P7), band via sites `EM{d}_RBD_r` / `EM{d}_UBD_r` just
over the PIP, and the terminal convergence `EM{d}_T_r` (old P8); on the distal phalanx the terminal insertion
`EM{d}_TI_r` (old P9).

Each tendon keeps its path up to its junction (EDC, EIP, EDM: the hood; LU_RB: the radial band site; UI_UB:
the ulnar band site) and continues in pulley branches. A branch with divisor D carries the force F / D, so
its divisor is 1 / fraction:

- central slip: junction -> `CSP` -> PIP wrap (the EDC's) -> `CS`;
- lateral band: junction -> band site -> band wrap -> band via site -> `T` -> DIP wrap (the EDC's) -> `TI`.

Whatever the branches do not carry stays on the proximal phalanx at the junction (for the extensors: the
sagittal bands and capsule; for the interossei: the bony part).

| Muscle | Central slip | Each lateral band | Proximal phalanx | Source |
| --- | --- | --- | --- | --- |
| EDC2-5, EIP, EDM | c (1 - r) | c r / 2 (both bands) | 1 - c | r = 0.63 (Lee et al. 2008a, Table 5); c fitted (below) |
| LU_RB2-5 | 1/3 | 2/3 (radial band) | 0 | Chao & An 1978, as reproduced by Brook et al. 1995 (Eqs. 12a-d) |
| UI_UB2-4 | 1/3 | 1/3 (ulnar band) | 1/3 | as above |

The RI bundles and ADM (S4) stay bony. The interosseous identities and their hood shares (Salsbury 1937)
follow in S5, which ends the renamed intrinsics on the same band sites.

**Band geometry.** Garcia-Elias et al. 1991a (p. 1134) place the lateral bands 1.9 mm dorsal of the PIP
centre in extension (central band 7.1 mm dorsal, bands 5.2 mm palmar of it). The band sites sit 1.9 mm dorsal
of the PIP axis, 4 mm proximal of it and half a bone width to each side (from the bone mesh), and the band
wrap is a 1.9 mm cylinder on the PIP axis. The band via sites on the middle phalanx (2.2 mm from the axis,
60 deg from distal toward dorsal) make the bands wrap that cylinder dorsally over the whole PIP range:
without them MuJoCo drops the wrap once the straight segment passes palmar of the cylinder, and the bands
became PIP flexors beyond about 95 deg. The bands therefore keep a 1.9 mm extension arm at every PIP
angle; Garcia-Elias measured them moving palmar of the axis in full flexion, where they carry little
tension, which a constant fraction cannot represent.

## Extensor share c

The DIP and PIP moment arms of a branched extensor are linear in c (r fixed). c is fitted by weighted least
squares (1 / SD^2) to the index EDC2 and EIP whole-tendon arms of An et al. 1983 (Table 3, averaged over
PIP 0-90 and DIP 0-60 deg) and the effective static moment arms of Lee et al. 2008b (Table 4, mean of their
nine postures), with the model's own geometric arms, and applied to every extensor. The unbounded optimum
is 1.09; c = 1.0 (no extensor force left on the proximal phalanx), against Lee et al. 2008a's nine-posture
mean of 0.39 (0.72 with the finger extended). With c = 0.39 the IP joints would get about a third of the
measured extension arm.

## Oblique retinacular ligament

One tension-only fixed tendon per finger, `lig_ORL{d}_r`, of length r (q_DIP - q_PIP): it lengthens with DIP
flexion and shortens with PIP flexion (Landsmeer 1949: palmar of the PIP axis, part of the terminal tendon
at the DIP). r is the model's terminal-tendon DIP arm (no source gives the ORL's arms; equal arms make its
slack a pure angle difference). The slack length is r x 30 deg: Harris & Rutledge 1972 (Tables I-II) found it
slack in unresisted flexion, with PIP-DIP differences up to 30 deg. The stiffness gives Ueba et al. 2011's
ORL share of the DIP resistance at PIP 0, DIP 45 deg, MCP 0 with 1.96 N on the extensor (25 %); the total
there is Kamper et al. 2002's DIP static torque plus the preload. The ORL is a fixed tendon, the cheapest
form (variant 2 of the proposal): a spatial ORL needs extra wraps to stay palmar of the PIP, and its arms
are not measured either.

Fitted per digit (`orl.result` in the table): arms 3.6 / 3.1 / 5.5 / 3.0 mm, stiffness 3756 / 5095 / 1644 /
5332 N/m (digits 2-5). With the refitted springs the index ORL carries 23.7 % of the DIP resistance in
Ueba's posture.

**The two sources conflict.** With the slack set by Harris & Rutledge, the ORL is slack at Ueba's PIP 30,
60 and 90 deg (DIP 45 deg is less than 30 deg beyond the PIP), so it reproduces Ueba's 25 % at PIP 0 but
not 31 / 18 / 3 % at 30 / 60 / 90 deg. Harris & Rutledge is kept as the constraint: it keeps free finger
motion unaffected, as Leijnse & Spoor 2012's coupling fits also found the ORL unnecessary in 67 of 68
fingers.

The ORL counts as part of the model's own passive moment when the residual joint springs are fitted
(`scripts/fit_passive_moments.py`, ligaments `lig_`).

## Refit order

1. Fractions and ORL: `uv run python scripts/fit_extensor_mechanism.py --fit`.
2. Extrinsic slack lengths and passive scales: the extensors' slack lengths in `muscle_passive.json` were
   shifted once by their tendon-length change at q = 0 (`reanchored_m`), then
   `uv run python scripts/fit_muscle_passive.py`.
3. Residual joint springs: `uv run python scripts/fit_passive_moments.py`.

## Limitations

- Constant fractions: no posture dependence of c or of the slip ratio (Lee et al. 2008a; Valero-Cuevas et
  al. 2007), no release of the distal phalanx with the PIP flexed 80-90 deg (Landsmeer 1949; Harris &
  Rutledge 1972; Ueba et al. 2011).
- The lumbrical's 2/3 terminal share gives a DIP extension arm of about 2.4 mm, against 0.7 +- 0.6 mm (An et
  al. 1983) and an effective 0.53 mm (Lee et al. 2008b); listed under the open checks.
- The extensors' MCP moment arms are unchanged (the hood is distal of the MCP wrap).

Results against the literature: `hand-literature-comparison.md` (EM-a).
