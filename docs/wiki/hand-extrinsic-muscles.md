# Extrinsic hand muscles: juncturae, FCU, FDS5, wrist wrap sides, APL at the thumb CMC, ECU

Revisions of the extrinsic hand muscles from the hand rebuild (S7 and the steps after it) and the ECU re-route
that followed the rebuild. Data, sources and fit results are in `myo_sim/build/data/extrinsic_muscles.json`;
`scripts/generate_extrinsic_muscles.py` writes them into the right-arm XML (`--fit` refits, `--fit-ecu` only the
ECU site). Tests: `tests/test_extrinsic_muscles.py`. Model values against the literature:
`hand-literature-comparison.md` (S7 and later steps, ECU).

The fit was run once more after the passive refits below, again after the EDC2 wrap change and again
after the JT3/JT5 tie fix and the ECU re-route; the table gives the model's values (`hand-literature-comparison.md`, S7, EDC2
wrap, JT3/JT5 ties, ECU). The middle-finger share depends on the contact-driven transfer without links
(25.7-27.0 deg over the S7 refits, 13.8 deg after the EDC2 wrap change, 13.1 deg after the tie fix, 13.9 deg after the ECU re-route), so it
moves with small changes.

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
wrap, tied by a tendon equality `node_JT{d}_tie_r` to the tendon's branched path from its via point
`EDC{d}-P4_r`, proximal of the node (`node_JT{d}_edc_r`, which includes the extensor-mechanism pulley
branches). The helper starts where the EDC's own segment to the MCP wrap starts, so it wraps the MCP as the
tendon does and the node moves with the tendon material over the whole MCP range (rigid tendon: the
material's excursion is the same anywhere along the segment). Until the tie fix the helpers of JT3 and JT5
started at the node's rest point near the MC head, missed the MCP wrap the EDC takes from P4 and moved
those nodes 6-11 mm more than the tendon at MCP 90 deg; `tests/test_extrinsic_muscles.py` checks node
against tendon excursion over the MCP range, with that construction as its negative control. Mass, armature, damping and
`solref` are the lumbrical nodes' (0.03 kg, 5 N s/m, 0.004 1). The links are tension-only spatial tendons
from the ring tendon's node (mid-metacarpal) obliquely distally to the neighbours' nodes (at the
metacarpal necks): the juncturae arise from the ring tendon and insert more distally on the neighbours.
The node positions along the tendons are a judgement (no measured junctura coordinates were used). Each
link is slack up to the longer of its lengths with straight fingers and in the relaxed resting posture of
the cadaver protocol (MP 56, PIP 35, DIP 17 deg), so it carries no force in either. With the nodes on
their tendons, `lig_JT45_r` is 1.1 mm longer with straight fingers than in the resting posture (before the
tie fix the resting posture was the longer one for both links).

**Fit.** Von Schroeder & Botte 1993 pulled one EDC tendon with 1.5 kg (palm down, wrist pinned neutral)
and measured the MP extension of the neighbours before and after cutting the juncturae and the
intertendinous fascia. The junctura share (intact minus cut) is the target; the model has no web or
fascia coupling, so the model value is the extension with the links minus without them. Protocol in the
model: `myohand_r` palm down under gravity, wrist, forearm and thumb held, settled from the resting
posture, then 14.7 N along the tendon for 1.5 s.

| Traction on | Neighbour | Junctura share (deg), von Schroeder & Botte 1993 | Model, links minus none (deg): S7 / after the EDC2 wrap / after the tie fix / after the ECU re-route |
| --- | --- | --- | --- |
| EDC-ring (EDC4) | middle MP | 18 (38 -> 20) | 6.2 / 17.1 / 17.0 / 16.5 |
| EDC-ring (EDC4) | small MP | 22 (44 -> 22) | 22.3 / 21.7 / 21.6 / 20.3 |
| EDC-long (EDC3) | ring MP | 8 (26 -> 18) | -0.9 / -1.4 / 0.0 / 0.0 |
| EDC-long (EDC3) | small MP | 7 (16 -> 9) | 0.2 / 0.2 / 0.0 / 0.0 |

Stiffness after the refit that followed the JT3/JT5 tie fix: both links at their upper bound of 1000 N/m
(`lig_JT34_r` 1000, `lig_JT45_r` 1000 N/m; after the EDC2 wrap change 993 and 222 N/m, in S7 999.9 and
263 N/m). With the nodes on their tendons, ring-finger traction pulls JT4 further from the neighbours'
nodes than the over-moving JT3/JT5 nodes allowed, so a stiffer EDC4-EDC5 link is needed for the same
transfer. The middle finger's share rose after the EDC2 wrap change because its contact-driven extension
without links fell (below). The bound is
numerical: tendon springs are explicit in MuJoCo's Euler step, and with EDC4 and the middle-finger flexors
fully active the link force changes from step to step by 0.17 N at 1000 N/m (5.5 N mean), 0.6 N at
3000 N/m and 3.5 N at 10000 N/m. In S7 the transfer saturated: 10000 N/m gave 6.1 deg and an
almost rigid link (100000 N/m) 6.7 deg instead of 18, limited by the link's obliquity. The links are softer than the hood segments of Garcia-Elias et al.
1991b (40-294 N/mm). The pulled finger's own extension changes by at most 1.2 deg with the links.

**Limits.**
- Traction on the middle-finger tendon does not reach the ring and little fingers: a tension-only link
  from the ring tendon slackens when the middle tendon moves proximally. Von Schroeder & Botte find 7-8
  deg there.
- Without the links, ring traction already extends the middle MP in the model (contact between the proximal
  phalanges): about 26 deg in S7, 13.8 deg after the EDC2 wrap change and its refits; the cadaver value after
  cutting the juncturae is 20 deg (fascia and web). This contact-driven part moves with small model changes.
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

## Extensor carpi ulnaris through the ulnar groove

**Anatomy.** The ECU tendon runs in the groove on the dorsal surface of the ulnar head, radial to the
styloid, held there by its own subsheath, which is fixed to the ulna. The ulna does not turn in forearm
rotation, so the tendon changes its position relative to the wrist axes: Horii et al. 1993 found it the only
wrist motor whose excursion changes with forearm rotation (flexion-extension excursion 10 / 10 / 4 mm in
supination / neutral / pronation, radioulnar 14 / 16 / 17 mm), and Loren et al. 1996 measured its extension
moment arm falling from 7.8 / 7.2 mm to 0.9 mm in pronation while its ulnar arm rose from 21 to 28 mm. In
pronation the ECU is an ulnar deviator rather than an extensor.

**Before.** The ECU's wrist-crossing via point and a wrap sphere sat on the radius, 20 mm from the rotation
axis and 10 mm outside the ulnar head, so its arms did not change with rotation (7.5 mm extension, 27.8 mm
ulnar deviation, against Horii's 6 ± 1 and 17 ± 3 mm at neutral). The wrap's side site sat at the sphere's
centre. A trial in S7 put the groove on the ulna at that position and overshot (16 to -12.6 mm), which
was then blamed on the rotation axis; the axis passes through the ulnar head (next section), and it was the
groove's distance from the axis, 20 mm instead of about 10, that swung the arm.

**Now.** `ECU-P5_r` sits on the ulna at the distal end of the groove; distal of it the tendon runs straight
to the MC5 base (`ECU-P6_r`). The site is fitted in spherical coordinates about a sphere fitted to the ulnar
head's mesh vertices (distal 22 mm of the ulna; radius 9.6 mm, rms 1.8 mm): on the posterior to lateral
side of the head (between straight posterior and 60 deg toward the radioulnar joint), 0.5 to 3 mm outside
the sphere, to Horii's and Loren's extension and ulnar arms at forearm -90 / 0 / +90 deg (SD 1 and 3 mm,
Horii's Table II). It lands 55 deg from posterior toward lateral, 23 deg from the distal pole and 3 mm
outside the sphere (a bound; across the groove's surface the fit's rms z ranges only 2.0-2.7): at the level
of the ulna's distal end, 5.2 mm from the rotation axis (3.5 mm posterior, 3.8 mm lateral), where the tendon
leaves the groove over the distal edge of the head. The old via point was 19.6 mm from the axis. The wrap
sphere `ECU_torus_wrap_r`, its side site and the unused duplicates `ECU_torus_r` and `ECU_sidesite_r` are
removed: with the groove on the ulna the tendon never reaches a sphere on the radius, and none on the ulna
would be touched either (the groove site is the last point on the ulna). Over the forearm, flexion and
deviation ranges (100 postures) the path stays outside the bones' convex hulls except at full flexion with
30 deg radial deviation: up to 1.2 mm into the ulna's hull in pronation and 3.3 mm into the triquetrum's in
supination (before: 2.8 mm into the ulna's). The ECU's passive parameters are refitted with the other
extrinsics.

| Quantity (forearm supination / neutral / pronation, wrist neutral) | Before | After | Literature |
| --- | --- | --- | --- |
| Extension moment arm (mm) | 7.5 / 7.5 / 7.5 | 5.7 / 6.1 / 0.1 | 7.8 / 7.2 / 0.9 (Loren 1996); 5.7 / 5.7 / 2.3 (Horii 1993, mean over 100 deg); 6 ± 1 at neutral (Horii, Table II) |
| Ulnar-deviation moment arm (mm) | 27.8 / 27.8 / 27.8 | 11.1 / 17.4 / 17.8 | 21 / 28 / 28 (Loren); 16.0 / 18.3 / 19.5 (Horii, mean over 50 deg); 17 ± 3 at neutral (Horii, Table II) |
| Fit to both sources (rms of (model - mean) / SD, 12 values) | 3.11 | 2.02 | 0 |
| Supination moment arm, elbow 90 deg (mm) | -2.6 / 0.7 / 2.6 | 0.2 / -1.2 / -0.2 | -4.7 / -2.1 / 6.4 ± 1.7 / 2.0 / 4.8 (Haugstvedt 2001, wrist free) |

**Limits.** The extension arm now falls in pronation and stays in supination, and the neutral ulnar arm is
inside Horii's band, but a groove fixed to the ulna still overshoots the trend: the fit moves the groove's
exit to the distal edge of the head, as close to the axis as its bounds allow, and in supination the ulnar
arm still falls to 11 mm (Horii 16, Loren 21). In vivo the ulna also translates dorsally by 3.9 mm during
pronation (Matsuki et al. 2010) and the subsheath lets the tendon move a little, neither of which a hinge
with a fixed via point has. The ECU's pronation-supination arm near
the end positions (Haugstvedt et al. 2001: a pronator in full supination, a supinator in full pronation,
measured with the wrist free) is lost: with the groove near the axis the tendon's line passes close to it.
Neither source states the angles of its supinated and pronated positions; the model uses ±90 deg.

## Forearm rotation axis (checked, not moved)

The radius turns about `pro_sup_r`, an axis through the centre of the radial head and the distal ulna (the
MoBL-ARMS definition after Holzbaur et al. 2005 and Murray et al. 1995, cited only). In vivo the axis passes
through the centre of the radial head and, distally, the ulnar head 1.9 ± 0.7 mm posterior of its geometric
centroid (Matsuki et al. 2010, 10 forearms); Akhbari et al. 2021 place the distal centre of rotation within
0.5-2 mm of the ulnar fovea (0.6 ± 0.8 mm volar at maximum pronation, 2.0 ± 0.5 mm volar at maximum
supination). On the model's meshes (`generate_extrinsic_muscles.axis_offsets`):

| Quantity | Model | Literature |
| --- | --- | --- |
| Axis to the radial head centre (circle through the head rim, 7 mm; radius 9 mm) (mm) | 1.0 | through the centre (Matsuki 2010) |
| Axis to the ulnar head centre (sphere through the distal 22 mm; rms 1.8 mm) (mm) | 0.7 (0.7 anterior, 0.2 lateral) | 1.9 ± 0.7 posterior (Matsuki 2010) |

Both are within the meshes' resolution (about 210 vertices per bone; the fits leave 1.8-2 mm rms). Moving
the distal end 2.6 mm posterior to Matsuki's mean would change every arm model's forearm rotation for an
offset below that resolution, and it helps the ECU only a little (refitted groove: rms z 2.02 -> 1.68, ulnar
arm in supination 11.1 -> 13.5 mm), so the axis is unchanged. The rotators' moment arms about it are
recorded against Haugstvedt et al. 2001 in `hand-literature-comparison.md` (ECU step): the pronator quadratus and FCU follow the measured curves; the
pronator teres turns into a supinator (6.4 mm) and the biceps into a pronator (-4.7 mm) in full pronation,
where the measured arms are -0.6 and 9.3 mm, which comes from their paths (no wrap around the radius), not
from the axis.

## FDS5

MyoHand's extrinsic forces are Holzbaur et al. 2005's peak forces (45 N/cm2) scaled by muscle-specific
factors: 2.66 / 2.84 / 2.96 for FDS2-4 (162.5 / 258.8 / 171.2 N against 61.2 / 91.0 / 57.9 N). FDS5 had
4.56 (75.3 against 16.5 N), out of line with its group and with Lieber et al. 1992's small FDS5 (PCSA
0.40 ± 0.05 cm2, a quarter of FDS4's 1.61). Its force is now Holzbaur's 16.5 N times the mean FDS2-4
factor, 46.5 N (27 % of FDS4). Its optimal fibre length, 51.5 mm, is already Lieber's 42 mm renormalised
to a 2.7 um sarcomere (Holzbaur: 5.2 cm) and is kept. FDS2's 83.5 mm likewise equals Lieber's combined
bellies (32 + 38 mm, 68 mm) renormalised; Matsuzawa et al. 2021's 32.9 mm is one belly of the digastric
muscle, so FDS2 is unchanged.

## EDC2 wrist wrap

EDC2 crosses the wrist over the sphere `EDCI_torus_wrap_r` (MyoHand's stand-in for MoBL's EDC index
torus, in `lunate_r`). Its side site `EDCI_torus_site_EDC2_side_r` sat at the sphere's centre. A side site
inside the geom makes MuJoCo apply the inside wrap: the tendon is held within the sphere's circle, like a
ring. Over the wrist range the ring held EDC2 from its palmar edge in flexion (a floor) and from its ulnar
edge in extension, never dorsally. The inside wrap is solved iteratively, and MuJoCo (float64) and
mujoco-warp (float32) give different answers where the deflection is small: up to 0.79 mm of EDC2 length
in 32 of 512 random arm postures (`myoarm_r`). Any side site inside the sphere gives the same result, so
moving it off the centre but inside changes nothing.

The side site now lies outside the sphere on its dorsal side (+z of `lunate_r`, two radii from the centre;
`edc2_wrap_side` in the data file). The tendon wraps over the sphere's dorsal surface, as the extensor
tendons glide over the dorsal carpus, and both engines resolve this ordinary side wrap alike (below 1e-6 m
in the same 512 postures; `tests/test_extrinsic_muscles.py`). The sphere is unchanged; a fit of its centre,
radius and side direction to the ring's moment arms collapses to a 1 mm sphere (a via point) and still
misses them by 1.2 mm RMS, so it was not used. EDC3 already wraps its torus sphere from outside.

| EDC2, `myoarm_r` | Side site at the centre (inside wrap) | Dorsal side site |
| --- | --- | --- |
| Wrist extension moment arm at 60 / 30 deg extension, neutral, 30 / 60 deg flexion (mm) | 20.2 / 22.1 / 17.3 / 12.0 / 6.2 | 24.0 / 22.9 / 18.7 / 15.2 / 9.5 |
| Ulnar-deviation moment arm at neutral (mm) | 2.3 | 4.9 |
| MCP2 / PIP / DIP extension moment arms at rest (mm) | 8.73 / 3.13 / 2.25 | 8.73 / 3.13 / 2.25 |
| Length at q0 (mm) | 406.54 | 407.92 |

The finger moment arms, and with them the An 1983 and Lee 2008b comparisons, do not change: the wrap
spans the wrist only. EDC2's wrist arms now lie with EDC3's and EDC4's (24-25 mm at 60 deg extension).
### The other forearm wraps

The same fix for the other forearm wraps whose side site sat inside the geom (`wrap_sides` in the data
file; CPU-warp length differences before: PQ 1.40, ECRL 1.01, EDC5 0.72, EDM 0.65, EDC4 0.36, ECRB 0.16,
FCR 0.07 mm). Each side site goes two radii out, on the side where the tendon runs: dorsal (+z of the
lunate) for the extensors, palmar for FCR and PQ. Where the arm curves then jump or leave Horii 1993's
bands, the geom's centre and radius were fitted (bounded: centre within 5 mm per axis, radius 0.5-2
times) to Horii's neutral arms and to the arm curves of the inside wrap. Arms in mm, + = extensor /
ulnar deviator (PQ: + = supinator), at wrist and forearm neutral unless stated:

| Muscle | Change | Before | After | Literature |
| --- | --- | --- | --- | --- |
| PQ | palmar side site; geometry unchanged | pronation arm -2.0 / -7.0 / -4.9 (pronation 80 deg / neutral / supination 80 deg) | -3.0 / -7.3 / -4.9 | – |
| EDC5 | dorsal side site; geometry unchanged | 21.1 / 11.1 / 1.4 (extension 60 / neutral / flexion 60); ulnar 9.9 | 21.2 / 15.2 / 5.4; ulnar 8.0 | – |
| EDC4 | dorsal; sphere moved 1.0 mm, radius 3.0 -> 2.8 mm (continuous arms) | 24.2 / 17.3 / 5.4; ulnar 9.1 | 25.4 / 17.8 / 8.0; ulnar 8.7 | – |
| EDM | dorsal; cylinder radius 14 -> 23.6 mm, centre 0.7 mm (reproduces the inside wrap's curves) | 17.9 / 9.5 / -1.9; ulnar 14.5 | unchanged | – |
| ECRB | dorsal; sphere centre 5.2 mm, radius 2.0 -> 1.6 mm | 15.3 / -14.2 | 14.0 / -12.6 | 12 ± 2 / -13 ± 2 (Horii); 16 / -16 (Loren) |
| FCR | palmar; sphere centre 6.7 mm, radius 1.5 -> 1.2 mm | -18.5 / -4.0 | -16.4 / -6.4 | -15 ± 3 / -8 ± 2 (Horii); -15 / -5.7 (Loren) |
| ECRL | not changed (inside wrap kept) | 11.7 / -23.6 | – | 7 ± 2 / -19 ± 2 (Horii); 10 / -23 (Loren) |

ECRL keeps its inside wrap: with the side site outside, no sphere within 8 mm of its place and 1/3-3 times
its radius keeps ECRL an extensor beyond 45 deg of wrist extension (the tendon leaves the sphere and its
straight path passes palmar of the axis); its CPU-warp difference (about 1 mm) stays. For the others CPU
and mujoco-warp agree within 3e-7 m in 128 random arm postures (test with the inside wrap as negative
control). EDM's cylinder is large for an anatomical surface; it is a wrapping surface only (no contact).
ECU's wrap sphere is gone with the ECU re-route (above); the deltoid and pectoral heads (up to 3.3 mm) are a
later step.

## APL at the thumb CMC (S2c; FPL tried, not adopted)

Barry et al. 2018's OpenSim model of the index finger and thumb (2020 revision on
simtk.org/projects/thumbindexdynam, MIT Use Agreement, `build/data/LICENSE-Barry2018.txt`) uses MyoHand's
bone meshes and body frames, so its path points apply in MyoHand's bodies unchanged (`thumb_extrinsics`
in the data file).

- **APL:** from its MC1-base via point `APL-P6_r` over a cylinder on the trapezium (`APL_cmc_wrap_r`) to
  Barry's insertion on the MC1 base (`APL-P7_r`; `APL-P8_r` removed). MuJoCo has no ellipsoid wrap, so the
  cylinder takes the centre and orientation of Barry's `THM_CMC1_APL` ellipsoid and its 6 mm semi-axis as
  radius; Barry's trapezium point lies inside it and is not used. The side site sits two radii from the
  axis, in the middle of the band of side directions (180-270 deg about the cylinder's axis) over which the
  tendon wraps at every Smutz posture. The forearm and wrist part of the path is unchanged (wrist arms
  unchanged). APL was a weak CMC flexor; it is now an extensor and abductor.
- **FPL (not adopted):** Barry's trapezium point and first MC1 point raise FPL's CMC flexion arm from 4.2
  to 14.3 mm (Smutz 14.3 ± 4.0; RMS z 2.50 -> 1.70), but the lateral pinch then has no solution within its
  off-axis limit (best 3.6 N at 14 times the limit); the trapezium point alone (13.8 mm) fails the same way.
  FPL keeps MyoHand's path (open check in `hand-literature-comparison.md`).

| At Smutz et al. 1998's neutral (mm; + = flexor / adductor) | Before | After | Smutz 1998 (mean ± SD) |
| --- | --- | --- | --- |
| APL CMC flexion / CMC abduction | 1.4 / -2.6 | -7.1 / -11.3 | -7.1 ± 3.4 / -10.5 ± 3.3 |
| APL RMS z (3 postures per joint) | 2.34 | 0.71 | 0 |
| FPL CMC flexion / CMC abduction / MP flexion (unchanged) | 4.2 / -1.7 / 7.1 | 4.2 / -1.7 / 7.1 | 14.3 ± 4.0 / -0.2 ± 4.9 / 13.6 ± 2.2 |

Barry's adductor-pollicis transverse-head insertion is used too (`passive-moments-and-thenar.md`); his
oblique-head path is not (it makes the lateral pinch infeasible).

## Refits

Extrinsic slack lengths and passive scales (`fit_muscle_passive.py`), then the joint springs
(`fit_passive_moments.py`); numbers in `hand-literature-comparison.md` (S7, EDC2 wrap) and the CHANGELOG.

After the ECU re-route the same chain was run again (passive muscles, extrinsic fits, springs). The ECU's
smaller ulnar arm leaves the muscles a larger deficit in radial-deviation stiffness, so a new spring segment
(`passive_wrist_deviation_pos_r`) covers it: muscles and springs now match Formica's wrist deviation
stiffness with 0.010 N m rms (0.054 before), wrist flexion 0.010 (0.011). The ECU's passive scale falls 0.83
-> 0.72 (slack +2.1 -> +0.4 %); the joint-fit error of the muscles alone rises 866 -> 1128 (without the
release step 690 -> 908), the deficit at the wrist; the release bound of EDC3-5 and EDM 0.0261 -> 0.0259.

## References

- Akhbari B et al. Biomechanics of the distal radioulnar joint during in vivo forearm pronosupination. J Wrist Surg 10(3):208-215, 2021.
- Garcia-Elias M et al. Extensor mechanism of the fingers. II. Tensile properties of components. J Hand Surg Am 16(6):1136-1140, 1991.
- Haugstvedt JR, Berger RA, Berglund LJ. A mechanical study of the moment-forces of the supinators and pronators of the forearm. Acta Orthop Scand 72(6):629-634, 2001 (Fig. 3, read from the figure).
- Holzbaur KRS, Murray WM, Delp SL. A model of the upper extremity for simulating musculoskeletal surgery and analyzing neuromuscular control. Ann Biomed Eng 33(6):829-840, 2005 (Table 1). Cited only; no model file used.
- Horii E, An KN, Linscheid RL. Excursion of prime wrist tendons. J Hand Surg Am 18(1):83-90, 1993.
- Leijnse JNAL. Measuring force transfers in the deep flexors of the musician's hand: theoretical analysis, clinical examples. J Biomech 30:873, 1997.
- Lieber RL et al. Architecture of selected muscles of the arm and forearm: anatomy and implications for tendon transfer. J Hand Surg Am 17(5):787-798, 1992 (Table I).
- Loren GJ et al. Human wrist motors: biomechanical design and application to tendon transfers. J Biomech 29(3):331-342, 1996 (appendix).
- Matsuki KO et al. In vivo 3D kinematics of normal forearms: analysis of dynamic forearm rotation. Clin Biomech 25(10):979-983, 2010.
- Matsuzawa K et al. The origin structure of each finger in the flexor digitorum superficialis muscle. Surg Radiol Anat 43:3-10, 2021.
- Murray WM, Delp SL, Buchanan TS. Variation of muscle moment arms with elbow and forearm position. J Biomech 28(5):513-525, 1995. Cited only (its forearm axis definition).
- von Schroeder HP, Botte MJ. The functional significance of the long extensors and juncturae tendinum in finger extension. J Hand Surg Am 18(4):641-647, 1993.
- Yammine K. The anatomy and prevalence of the juncturae tendinum in the hands. Eur J Anat 19(4):351-359, 2015.
- Zatsiorsky VM, Li ZM, Latash ML. Enslaving effects in multi-finger force production. Exp Brain Res 131:187-195, 2000.
