# Extrinsic hand muscles: juncturae, FCU, FDS5

Revisions of the extrinsic hand muscles from the hand rebuild (S7). Data, sources and fit results are in
`myo_sim/build/data/extrinsic_muscles.json`; `scripts/generate_extrinsic_muscles.py` writes them into the
right-arm XML (`--fit` refits). Tests: `tests/test_extrinsic_muscles.py`. Model values against the
literature: `hand-literature-comparison.md` (S7).

The fit was run once more after the passive refits below; the table gives the final model's values
(`hand-literature-comparison.md`, S7). The middle-finger share depends on the contact-driven transfer
without links (25.7-27.0 deg over the last refits), so it moves by about a degree with small changes.

## Juncturae tendinum

**Anatomy and choice.** The juncturae link the EDC tendons on the dorsum of the hand. The link between
the index and middle tendons is usually a thin type 1 band (2nd interspace: 63 % prevalence, 95 % type 1),
the ulnar links are frequent and thick (3rd interspace 92.7 %, 4th 87.4 % with 90.7 % type 3; Yammine
2015, Table 5). Von Schroeder & Botte 1993 find the radial coupling carried by the intertendinous fascia
and the web, while on the ulnar side the juncturae carry about half of it. The model has the two ulnar
links, EDC4-EDC3 (`lig_JT34_r`) and EDC4-EDC5 (`lig_JT45_r`).

**Primitive.** Each of EDC3, EDC4 and EDC5 carries a junction node, the primitive of the lumbrical nodes
(`passive-moments-and-thenar.md`, *Lumbrical junction nodes*): a light body on a slide joint
`node_JT{3,4,5}_r` in its metacarpal, on the EDC path segment from the metacarpal via site to the MCP
wrap, tied by a tendon equality `node_JT{d}_tie_r` to the tendon's branched path distal of the node
(`node_JT{d}_edc_r`, which includes the extensor-mechanism pulley branches). Mass, armature, damping and
`solref` are the lumbrical nodes' (0.03 kg, 5 N s/m, 0.004 1). The links are tension-only spatial tendons
from the ring tendon's node (mid-metacarpal) obliquely distally to the neighbours' nodes (at the
metacarpal necks): the juncturae arise from the ring tendon and insert more distally on the neighbours.
The node positions along the tendons are a judgement (no measured junctura coordinates were used). Each
link is slack up to its length in the relaxed resting posture of the cadaver protocol (MP 56, PIP 35,
DIP 17 deg), so it carries no force at rest or with the fingers extended.

**Fit.** Von Schroeder & Botte 1993 pulled one EDC tendon with 1.5 kg (palm down, wrist pinned neutral)
and measured the MP extension of the neighbours before and after cutting the juncturae and the
intertendinous fascia. The junctura share (intact minus cut) is the target; the model has no web or
fascia coupling, so the model value is the extension with the links minus without them. Protocol in the
model: `myohand_r` palm down under gravity, wrist, forearm and thumb held, settled from the resting
posture, then 14.7 N along the tendon for 1.5 s.

| Traction on | Neighbour | Junctura share (deg), von Schroeder & Botte 1993 | Model, links minus none (deg) |
| --- | --- | --- | --- |
| EDC-ring (EDC4) | middle MP | 18 (38 -> 20) | 6.2 |
| EDC-ring (EDC4) | small MP | 22 (44 -> 22) | 22.3 |
| EDC-long (EDC3) | ring MP | 8 (26 -> 18) | -0.9 |
| EDC-long (EDC3) | small MP | 7 (16 -> 9) | 0.2 |

Stiffness: `lig_JT45_r` 263 N/m (fitted); `lig_JT34_r` 1000 N/m, at its upper bound. The bound is
numerical: tendon springs are explicit in MuJoCo's Euler step, and with EDC4 and the middle-finger flexors
fully active the link force changes from step to step by 0.17 N at 1000 N/m (5.5 N mean), 0.6 N at
3000 N/m and 3.5 N at 10000 N/m. The transfer saturates anyway: 10000 N/m gives 6.1 deg and an
almost rigid link (100000 N/m) 6.7 deg instead of 18, limited by the link's obliquity. The links are softer than the hood segments of Garcia-Elias et al.
1991b (40-294 N/mm). The pulled finger's own extension changes by at most 1.2 deg with the links.

**Limits.**
- Traction on the middle-finger tendon does not reach the ring and little fingers: a tension-only link
  from the ring tendon slackens when the middle tendon moves proximally. Von Schroeder & Botte find 7-8
  deg there.
- Without the links, ring traction already extends the middle MP by about 26 deg in the model (contact between
  the proximal phalanges); the cadaver value after cutting the juncturae is 20 deg (fascia and web).
- No index-middle link and no web: the radial coupling (32 % of the middle MP's range with EDC-index
  traction) is absent.

## FDP inter-tendon links (not added)

The deep flexor tendons are coupled by passive connections that are slack over a range and then taut,
in addition to fibres of one belly inserting on the neighbouring tendon (Leijnse 1997). Neural enslaving
(Zatsiorsky et al. 2000: 2-49 % of a finger's own maximal force) bounds the mechanical transfer from
above. The links would sit between the lumbrical nodes (no extra joints), but their slack range and
stiffness need population values that were not checked for this change, so they are not added yet.

## Flexor carpi ulnaris

FCU runs over the pisiform (a sesamoid in its tendon) and on along the pisohamate ligament past the hamate
hook to the base of MC5 (pisometacarpal ligament). The path now has a via point on the hamate hook
(`FCU-P5_r`, the hamate mesh vertex nearest the pisiform; the MC5 site is renamed `FCU-P6_r`), and the
pisiform via point is fitted within the pisiform's bounding box (the tendon on the bone's palmar surface)
to Horii et al. 1993's neutral moment arms. The force split between the two ligaments is not modelled.
The path proximal of the pisiform stays on the radius, so the moment arms do not depend on forearm
rotation (Horii 1993: no significant change). FCU's length range is shifted by its length change at q0,
so its normalised fibre length at q0 is unchanged.

| Quantity | Before | After | Literature |
| --- | --- | --- | --- |
| Flexion moment arm (mm) | 18.4 | 14.7 | 16 ± 3 (Horii 1993, Table II); 14 (Loren 1996, neutral) |
| Ulnar-deviation moment arm (mm) | 10.6 | 12.2 | 14 ± 3 (Horii 1993); 23 (Loren 1996) |
| CMC5 flexion moment arm (mm) | 4.6 | 3.1 | – |

## Extensor carpi ulnaris (tried, not changed)

Horii 1993 (excursion over 100 deg of flexion-extension: 10 / 10 / 4 mm in supination / neutral /
pronation) and Loren 1996 (extension moment arm 7.8 / 7.2 / 0.9 mm) show the ECU's extension arm falling
in pronation, because the tendon is held in the groove of the ulnar head, which does not turn with the
radius. In the model the ECU's wrist-crossing via point and wrap sphere sit on the radius, so its arms
(7.5 mm extension, 27.8 mm ulnar deviation) do not change with rotation. Moving them to the ulna at their
q0 position gives the right trend but far too much of it: at full pronation the ECU becomes a 12.6 mm
flexor, and fitting the groove point (within 1 cm) to both sources still gives -4.7 mm in pronation and
10 mm of ulnar arm in supination (Loren 21 mm). The cause is the forearm-rotation axis of the model, which
passes about 20 mm from the ECU groove (anatomically it runs through the ulnar head), so the hand swings
far around the groove. The ECU stays as it was; the rotation axis is a separate question.

## FDS5

MyoHand's extrinsic forces are Holzbaur et al. 2005's peak forces (45 N/cm2) scaled by muscle-specific
factors: 2.66 / 2.84 / 2.96 for FDS2-4 (162.5 / 258.8 / 171.2 N against 61.2 / 91.0 / 57.9 N). FDS5 had
4.56 (75.3 against 16.5 N), out of line with its group and with Lieber et al. 1992's small FDS5 (PCSA
0.40 ± 0.05 cm2, a quarter of FDS4's 1.61). Its force is now Holzbaur's 16.5 N times the mean FDS2-4
factor, 46.5 N (27 % of FDS4). Its optimal fibre length, 51.5 mm, is already Lieber's 42 mm renormalised
to a 2.7 um sarcomere (Holzbaur: 5.2 cm) and is kept. FDS2's 83.5 mm likewise equals Lieber's combined
bellies (32 + 38 mm, 68 mm) renormalised; Matsuzawa et al. 2021's 32.9 mm is one belly of the digastric
muscle, so FDS2 is unchanged.

## Refits

Extrinsic slack lengths and passive scales (`fit_muscle_passive.py`), then the joint springs
(`fit_passive_moments.py`); numbers in `hand-literature-comparison.md` (S7) and the CHANGELOG.

## References

- Garcia-Elias M et al. Extensor mechanism of the fingers. II. Tensile properties of components. J Hand Surg Am 16(6):1136-1140, 1991.
- Holzbaur KRS, Murray WM, Delp SL. A model of the upper extremity for simulating musculoskeletal surgery and analyzing neuromuscular control. Ann Biomed Eng 33(6):829-840, 2005 (Table 1). Cited only; no model file used.
- Horii E, An KN, Linscheid RL. Excursion of prime wrist tendons. J Hand Surg Am 18(1):83-90, 1993.
- Leijnse JNAL. Measuring force transfers in the deep flexors of the musician's hand: theoretical analysis, clinical examples. J Biomech 30:873, 1997.
- Lieber RL et al. Architecture of selected muscles of the arm and forearm: anatomy and implications for tendon transfer. J Hand Surg Am 17(5):787-798, 1992 (Table I).
- Loren GJ et al. Human wrist motors: biomechanical design and application to tendon transfers. J Biomech 29(3):331-342, 1996 (appendix).
- Matsuzawa K et al. The origin structure of each finger in the flexor digitorum superficialis muscle. Surg Radiol Anat 43:3-10, 2021.
- von Schroeder HP, Botte MJ. The functional significance of the long extensors and juncturae tendinum in finger extension. J Hand Surg Am 18(4):641-647, 1993.
- Yammine K. The anatomy and prevalence of the juncturae tendinum in the hands. Eur J Anat 19(4):351-359, 2015.
- Zatsiorsky VM, Li ZM, Latash ML. Enslaving effects in multi-finger force production. Exp Brain Res 131:187-195, 2000.
