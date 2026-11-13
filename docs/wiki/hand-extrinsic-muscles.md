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
- Since the MCP hyperextension step the little finger's share is below the target (12.1 deg in the pulley step, 9.4 deg after
  the intrinsic passive fit; von Schroeder & Botte 22 deg), with JT45 at its bound. The ring finger now hyperextends under
  1.5 kg on its EDC to -44 deg (their Fig. 2: -10 ± 14 deg), but that is not the cause: holding the ring MCP at 0 / -10 / -30 deg
  in the same trial lowers the little finger's share to 2.1 / 3.4 / 6.5 deg, since a ring tendon that moves less stretches the
  links less. With the EDC via sites moved onto the metacarpal neck (MCP pulleys) the JT45 link is 2.5 mm shorter (18.2 mm)
  and the little finger rests 5 deg more flexed in the trial (76 against 71 deg; their resting MP 56 deg), so the link reaches
  2.4 N instead of 3.7 N at the same stiffness. Middle-finger traction now reaches the ring finger (6.2 deg; von Schroeder &
  Botte 8 deg; 0 before), since the ring MCP is no longer held by the interossei's passive excess. The node positions along the tendons are a judgement (no measured junctura
  coordinates), so no cited lever is left; open check in `hand-literature-comparison.md`.

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

Superseded in the wrist range step: EDC2-5 now cross the wrist through via sites (Paths on MuJoCo 4, below). The history:

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
**Sphere moved (cupping and ORL step).** The sphere sits 11 mm dorsal of the carpus (it was MoBL's EDC index torus), so with the side site outside it the tendon ran over its top, 15.6 mm clear of the bone at wrist flexion: a straight line clear of the dorsal wrist and hand, which the earlier inside wrap had hidden. The centre moves 6 mm palmar, back to the earlier path height. Wrist arm at neutral / 39 deg flexion 18.7 / 13.6 -> 17.0 / 10.8 mm (`dev` 17.3 / 10.3, Holzbaur et al. 2005's average 16 mm). EDC4 and EDC5 lie 11 mm clear, ECRL 12 mm, EPL 14 mm, also on `dev`.

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

- **APL:** until the MuJoCo 4 step it ran from its MC1-base via point `APL-P6_r` over a cylinder on the trapezium to
  Barry's insertion on the MC1 base (`APL-P7_r`; `APL-P8_r` removed), the cylinder taking the centre and
  orientation of Barry's `THM_CMC1_APL` ellipsoid. It went round the far side of the cylinder, 30-48 mm longer
  than straight, and under MuJoCo 4 snapped back by 44 mm. It now runs over a via site on the trapezium (below,
  `APL_via_r`) to the same insertion. APL was a weak CMC flexor; it is now an extensor and abductor.
- **FPL (not adopted):** Barry's trapezium point and first MC1 point raise FPL's CMC flexion arm from 4.2
  to 14.3 mm (Smutz 14.3 ± 4.0; RMS z 2.50 -> 1.70), but the lateral pinch then has no solution within its
  off-axis limit (best 3.6 N at 14 times the limit); the trapezium point alone (13.8 mm) fails the same way.
  FPL keeps MyoHand's path (open check in `hand-literature-comparison.md`).

| At Smutz et al. 1998's neutral (mm; + = flexor / adductor) | Before | After | Smutz 1998 (mean ± SD) |
| --- | --- | --- | --- |
| APL CMC flexion / CMC abduction, cylinder | 1.4 / -2.6 | -7.1 / -11.3 | -7.1 ± 3.4 / -10.5 ± 3.3 |
| APL CMC flexion / CMC abduction, via site (MuJoCo 4 step) | -7.1 / -11.3 | -4.9 / -7.4 | -7.1 ± 3.4 / -10.5 ± 3.3 |
| APL RMS z (3 postures per joint), cylinder | 2.34 | 0.71 | 0 |
| FPL CMC flexion / CMC abduction / MP flexion (unchanged) | 4.2 / -1.7 / 7.1 | 4.2 / -1.7 / 7.1 | 14.3 ± 4.0 / -0.2 ± 4.9 / 13.6 ± 2.2 |

Barry's adductor-pollicis transverse-head insertion is used too (`passive-moments-and-thenar.md`); his
oblique-head path is not (it makes the lateral pinch infeasible).

## Paths on MuJoCo 4 (wrist and thumb)

MuJoCo 4 chooses a wrap's side by the winding of endpoint, side site, endpoint; it is the reference for these paths
(`model-authoring.md`). Fits were rerun on it. Arms in mm from dL/dq, + = extensor / ulnar deviator; `build/data/side_sites.json`
holds the positions, targets and residuals, `scripts/migrate_side_sites.py fit-vias` rewrites them.

**FDS2-5 and FDP2-5.** Both heads wrap a 13 mm cylinder on the lunate. Below about -58 to -66 deg of wrist flexion the old
side sites sent the tendon to the dorsal side, where it extended the wrist (FDS2 +16 mm) and the length jumped by 3-6 mm on
MuJoCo 4 and 6-12 mm on MuJoCo 3. Each site now sits in the middle of the side directions that keep the palmar arc: 170-180 deg
of directions qualify on MuJoCo 4 and a 42-46 deg part of them gives the same path on MuJoCo 3, and that part is used. The path
is identical on both; FDS2's arm goes from -7.3 at -90 deg to -14.1 at 0 and -30.5 at 80 deg. No moment arm of these muscles
at the wrist is in the literature used here, so this rests on sign and on Knutson et al. 2000's passive moment at the MCP
(below). `fit_muscle_passive.py` gives the same RMSE on the index, 3.287 N cm: Knutson's wrist range ends at 60 deg, where the
new path equals the old MuJoCo 3 path.

**FCR, ECRB, ECRL, EDM.** Small spheres (FCR 1.2 mm, ECRB 1.6 mm) acted as one-sided pegs: no side direction keeps both the
arms and continuity on MuJoCo 4 (the few without jumps turn FCR into an extensor at wrist extension). EDM's 23.6 mm cylinder
ended the wrap when via point `EDM-P4_r` came inside it at the flexion limit, a 44 mm jump. ECRL's inside wrap turned it into a
flexor below -65 deg. Each is now a via site on the lunate fitted to the tabled arms with the sign held over the whole range.

| Muscle | Before, on MuJoCo 4: jump; arm at neutral | Via site: arm at neutral (RMS z) | Literature at neutral | Garland 2018 mean over range |
| --- | --- | --- | --- | --- |
| FCR | 18.9 mm; -16.4 / -6.4 | -15.8 / -7.7 (0.44) | -15 ± 3 / -8 ± 2 (Horii); -15 / -5.7 (Loren); -16.2 ± 1.5 / -8.6 ± 4.0 (Garland MRI) | -12.2 ± 2.1 / -10.2 ± 2.5 |
| ECRB | 12.2 mm; 14.0 / -12.6 | 11.8 / -12.5 (0.60) | 12 ± 2 / -13 ± 2 (Horii); 16 / -16 (Loren); 9.2 ± 1.6 / -11.1 ± 2.6 (Garland MRI) | 10.8 ± 2.8 / -10.0 ± 2.2 |
| ECRL | none; 11.7 / -23.6, flexor below -65 deg | 7.6 / -18.3, extensor to -90 deg (0.57) | 7 ± 2 / -19 ± 2 (Horii); 10 / -23 (Loren); 4.8 ± 1.3 / -16.7 ± 2.4 (Garland MRI) | 6.6 ± 3.0 / -17.2 ± 3.6 |
| EDM | 44.1 mm; 9.5 / 14.5 | 9.3 / 14.3 (0.21) | none used | none used |
| APL (trapezium, CMC flexion / abduction at Smutz's neutral) | 44.8 mm; -7.1 / -11.3 | -4.9 / -7.4 (0.24) | -7.1 ± 3.4 / -10.5 ± 3.3 (Smutz) | – |

On MuJoCo 3 all of these are the same paths: a via site has no wrap rule. EDC2, EDC4 and EDC5 keep their sphere wraps; on both
versions their lengths still jump by 2.2 / 3.0 / 5.8 mm in range (`jumps_mm` in the data file), and they were not moved here.

The wrist fits keep their arms over -90 to 80 deg of flexion and -30 to 40 deg of deviation, past today's limits, and the MCP side
sites of EDC2 and EIP were moved so that their paths stay continuous to 0.55 rad of hyperextension; neither range is applied here.

**APL fibre length on the new path.** On MuJoCo 3 the old CMC cylinder took the APL most of the way round at most postures:
39 mm of extra length at q0, a path of 224-251 mm (5-95 % of random in-range postures) where the via path runs 175-201 mm.
The length range had been fitted to the longer path, so on the via path the fibres ran at 0.55-0.88 optimal length and
carried no passive force over the measured wrist range. The APL now takes its optimal fibre length from the literature and
its slack length from this path (`physiological` in `build/data/muscle_passive.json`, `physiological_maps` in
`scripts/fit_muscle_passive.py`):

| | Value | Source |
| --- | --- | --- |
| Optimal fibre length | 71 mm (at 2.7 um), pennation 8 deg | Jacobson et al. 1992, as renormalised in Holzbaur et al. 2005 Table 1 |
| Tendon slack length | 120.2 mm: fibres at optimal length with the wrist and thumb in neutral | rule; not measurable (Holzbaur et al. 2005, p. 832) |
| MuJoCo map | L0 71.8 mm, slack 118.7 mm, `lengthrange` 172.5-194.0 mm (was L0 78.8, slack 131.6 mm) | rigid-tendon map of `build/rigid_conversion.py` |
| Passive fit | MuJoCo slack +7.7 % (127.9 mm), `lengthrange` 181.7-203.2 mm, `fpmax` 1.3 | `fit_muscle_passive.py` |
| Fibre length, 5-95 % of random in-range postures | 0.79-1.14 at the reference slack; 0.66-1.01 after the passive fit (0.55-0.88 before; 0.84-1.20 on the old path on MuJoCo 3) | |

For comparison, Mirakhorlo et al. 2016 measured 50.1 mm optimal fibre length, 12 deg pennation and a 156 mm tendon in one
specimen; that tendon on this path would leave the fibres at 0.5-1.0 optimal length. Holzbaur et al. chose 130 mm for their
own path. No other re-routed muscle needed this: EDM, FCR, ECRB, ECRL, BIClong, DELT1, TMAJ, LAT1-3, SUP, FDS2-5, FDP2-5 and
LTpT keep their median length within 3 mm and their 5th percentile within 0.08 L0; only the tails that came from jumps of
the old wraps are gone (EDM 0.77-1.71 -> 0.78-1.22, BIClong 0.64-1.45 -> 0.64-1.29). No EDM moment arm at the wrist was
found in the literature used here.

**Wrist deviation stiffness.** The muscles' share of Formica et al. 2012's passive deviation moment fell from 0.090 to
0.136 N m rms with the new paths. Cause: on the old path the APL alone gave up to 0.24 N m against ulnar deviation. Without
it the fit loaded EPB, EPL and FPL instead; EPL's extra passive extension moment at neutral raised the wrist-flexion excess,
which capped ECU's passive scale (0.63 -> 0.46) and so left a larger deficit in radial deviation (muscles 0.20 against
0.51 N m at 17 deg). The fit is one-sided on purpose: a deficit goes to the joint springs, an excess cannot be removed. With
the springs, the model matches Formica with 0.010 N m rms (0.014 before the new paths). A second fit started from the old
thumb and ECU scales reaches a lower muscle cost (deviation 0.062 N m rms) but puts ECU and ECRB near their full passive
scale; at the springs' posture (elbow 90 deg, fingers extended) the muscles then exceed Formica by 0.19 N m in radial
deviation, no spring can remove that, and the total error is 0.087 N m. The fit as run is kept. Since the springs are
fitted at Formica's own posture (`passive-moments-and-thenar.md`, springs at the posture of their data) the total is 0.016 /
0.023 N m in flexion / deviation there, and the alternative still loses (0.063 / 0.028 N m).

**EDC2-5 (wrist range step).** The four EDC tendons crossed the wrist over small spheres on the lunate
(`EDCI`, `EDCM`, `EDCR`, `EDCL_torus_wrap_r`, MyoHand's stand-ins for MoBL's extensor tori). With the ulnar deviation
limit at 40 deg their lengths jumped where the straight line crossed the sphere in extension with ulnar deviation: EDC3 by
up to 10 mm (from 23 deg ulnar deviation at 22-71 deg extension), EDC5 by 7.3 mm; EDC2, EDC4 and EDC5 already jumped by
2-6 mm inside the old range (`jumps_mm`). `migrate_side_sites.py`'s side search found no side direction without a jump on
either rule. Each sphere is replaced by a via site `EDC{d}_via_r` on the lunate (`vias` in `side_sites.json`, path in
`extensor_mechanism.json`), fitted to the previous path's arms along single-joint sweeps from neutral, where that path was
continuous (flexion -60..60, deviation -20..20 deg; sign: extensor from 80 deg extension to 30 deg flexion). The spheres,
their side sites and the `wrap_sides` entries of EDC2, EDC4 and EDC5 are gone. On a 0.5 deg grid over the whole wrist
range no wrist-crossing tendon jumps now.

| Muscle | Before: flexion arm at -60 / 0 / 60 deg; deviation arm at 0 | After | Fit (RMS z) |
| --- | --- | --- | --- |
| EDC2 | 23.9 / 16.6 / 7.1; -2.0 | 22.7 / 18.2 / 5.3; -1.7 | 0.40 |
| EDC3 | 25.1 / 17.4 / -1.2; 2.0 | 24.9 / 15.9 / 1.6; 2.1 | 0.47 |
| EDC4 | 25.5 / 16.8 / 8.0; 8.1 | 25.3 / 18.7 / 6.7; 8.7 | 0.39 |
| EDC5 | 20.8 / 15.1 / 5.4; 7.8 | 20.3 / 15.0 / 5.5; 7.9 | 0.12 |

Holzbaur et al. 2005 give 16 mm for EDC2 at neutral. The EDC vias were refitted to Brand & Hollister 1993 in the wrist
path step (next section).

## Wrist paths over the whole range

With the flexion limit at 80 deg (wrist range step) FCU, PL and FPL turned into wrist extensors past about 72 deg of
flexion, and EDC3, EDM and EPL turned flexor near the limits. Cause: each path is a chord from its last forearm site to its
first hand site. A forearm site a few mm proximal of the axis (FCU 19, PL 3, FPL 10 mm) lets the hand site swing past it in
flexion, and the arm crosses zero when the two lie on one line through the axis. Brand & Hollister 1993 (Ch. 12, cadaver,
forearm midprone, -30..30 deg, read from the figures) measured the wrist flexion arm of every wrist-crossing tendon: the
flexor arms stay or rise towards flexion (PL, FPL, FDS and FDP by bowstringing), the extensor arms fall slowly.

**Fits.** `fit-vias` in `scripts/migrate_side_sites.py` fits each site to Brand's curve and the other cited arms, and holds
the sign of the flexion arm over flexion -75..80 deg at deviation -36, -18, 0, 11 and 22 deg (margin 2 mm for flexors, 1 mm
for extensors). Existing path sites are listed under `path_sites` in `build/data/side_sites.json` (the fit moves them; `write`
puts them into the chain):

- **FCU-P3_r** (radius): fitted to Brand, Loren et al. 1996 and Horii et al. 1993 (FCU has no retinaculum; it lies superficial
  to it and inserts on the pisiform). The fit puts the site 4 mm from FCU-P2_r, so the tendon runs straight from the forearm
  to the pisiform. FCU's `lengthrange` follows its length change at q0 (+3.4 mm), as before (`generate_extrinsic_muscles.py`).
- **PL-P2_r** (radius): 8 mm more proximal (11 mm proximal of the axis, 27 mm palmar). Elbow arm unchanged (-4.6 mm),
  pronation arm -3.2 -> -1.8 mm.
- **FPL-P4_r** (radius), kept within 10 mm of its radial position (FPL is the radial-most tendon of the carpal tunnel), and
  **FPL-P5_r** (capitate, the carpal-tunnel site) slid 13 mm distally along its own segment towards its tangent point on
  `FPL_ellipsoid_wrap_r`. The thumb arms at neutral are unchanged; with the CMC flexed and abducted (0.5 / 0.4 rad) the wrap
  plane follows the moved site and FPL's CMC flexion arm rises from 1.1 to 3.8 mm (Smutz et al. 1998: 14.3 mm at neutral).
- **EPL-P5_r** (radius, Lister's tubercle) and **EPL-P6_r** (capitate), each kept in a box near its old place (no detour
  between the tubercle and the trapezium). EPL turned flexor in full extension with radial deviation (-15 mm); its arm now
  stays an extension arm there, but it is about twice Brand's.
- Vias `FCR_via_r`, `ECRB_via_r` (Brand added to Horii, Loren and Garland), `EDM_via_r` and `EDC2-5_via_r` (now fitted to
  Brand's extensor digitorum curves, Figs 12-76 and 12-77, instead of the previous path) get the same sign rows. `ECRL_via_r`
  keeps its single-joint rows only: Loren's ECRL polynomial crosses zero at 59 deg flexion, and holding ECRL an extensor at
  full flexion with ulnar deviation moved its neutral radial-deviation arm from -18 to -12 mm (Horii -19).
- **EIP_via_r** (lunate) replaces the ellipsoid `ExtensorEllipse_ellipsoid_wrap_r`, whose arm stepped by up to 30 mm (a
  0.3 mm length kink) in full extension with ulnar deviation; fitted to the previous path's arms along single-joint sweeps
  (no EIP wrist arm in the literature used here).

Flexion arms in mm, + = flexion (deviation 0), and the smallest arm over the whole 2-D range (flexors as flexion arm,
extensors as extension arm; negative = the muscle acts the other way somewhere). Identical on MuJoCo 4.0 and 3.17:

| Muscle | Before: -30 / 0 / 30 / 80 deg | After | Brand & Hollister 1993: -30 / 0 / 30 | Smallest arm over the 2-D range, before -> after |
| --- | --- | --- | --- | --- |
| FCR | 12.1 / 15.8 / 17.0 / 2.4 | 12.6 / 16.8 / 18.5 / 4.0 | 17.5 / 17.8 / 16.5 | 0.6 -> 1.3 |
| FCU | 11.8 / 14.7 / 15.0 / -4.5 | 12.2 / 15.9 / 16.0 / 4.2 | 16.0 / 19.1 / 18.5 | -15.6 -> 1.1 |
| PL | 19.9 / 25.1 / 27.2 / -12.5 | 15.9 / 21.9 / 25.3 / 9.7 | 16.0 / 21.8 / 25.6 | -18.3 -> 3.2 |
| FPL | 8.7 / 11.2 / 12.7 / -7.0 | 11.3 / 15.9 / 18.7 / 9.2 | 12.1 / 15.2 / 21.6 | -9.7 -> 1.8 |
| FDS2 | 10.4 / 14.1 / 22.9 / 30.5 | 10.4 / 14.1 / 22.9 / 30.5 | 11.7 / 14.4 / 19.2 | 7.3 -> 7.3 |
| FDS3 | 10.7 / 14.6 / 23.3 / 30.5 | 10.7 / 14.6 / 23.3 / 30.5 | – | 7.5 -> 7.5 |
| FDS4 | 11.4 / 14.9 / 23.5 / 30.4 | 11.4 / 14.9 / 23.5 / 30.4 | – | 7.7 -> 7.7 |
| FDS5 | 11.6 / 15.3 / 24.0 / 30.9 | 11.6 / 15.3 / 24.0 / 30.9 | – | 7.8 -> 7.8 |
| FDP2 | 8.1 / 11.6 / 20.1 / 27.9 | 8.1 / 11.6 / 20.1 / 27.9 | 10.1 / 12.5 / 18.0 | 4.4 -> 4.4 |
| FDP3 | 8.0 / 11.7 / 20.5 / 29.0 | 8.0 / 11.7 / 20.5 / 29.0 | – | 4.0 -> 4.0 |
| FDP4 | 7.8 / 11.8 / 20.3 / 28.2 | 7.8 / 11.8 / 20.3 / 28.2 | – | 3.7 -> 3.7 |
| FDP5 | 7.7 / 11.6 / 19.8 / 26.9 | 7.7 / 11.6 / 19.8 / 26.9 | – | 3.4 -> 3.4 |
| ECRL | -8.9 / -7.6 / -5.3 / -0.5 | -9.4 / -8.0 / -5.6 / -0.6 | -11.5 / -10.5 / -8.8 | -6.3 -> -7.3 |
| ECRB | -14.0 / -11.8 / -8.3 / -0.9 | -14.6 / -12.3 / -8.7 / -1.1 | -13.8 / -13.6 / -12.5 | -0.9 -> -0.8 |
| ECU | -8.6 / -6.1 / -2.7 / 3.4 | -8.6 / -6.1 / -2.7 / 3.4 | – | -7.4 -> -7.4 |
| EDM | -13.8 / -9.3 / -4.2 / 4.8 | -13.4 / -10.6 / -6.9 / 0.3 | – | -6.1 -> -1.6 |
| EIP | -15.8 / -12.8 / -10.1 / -9.1 | -16.6 / -14.1 / -9.9 / -1.0 | – | 5.8 -> 0.3 |
| EPL | -15.2 / -13.4 / -10.4 / -3.7 | -16.1 / -13.8 / -9.7 / -1.0 | -7.0 / -7.6 / -7.4 | -15.2 -> -3.0 |
| EPB | -3.1 / -1.0 / 1.1 / 4.4 | -3.1 / -1.0 / 1.1 / 4.4 | 3.5 / 3.8 / 4.1 | -9.3 -> -9.3 |
| APL | 2.0 / 5.7 / 8.7 / 10.8 | 2.0 / 5.7 / 8.7 / 10.8 | 5.3 / 7.4 / 9.5 | -7.3 -> -7.3 |
| EDC2 | -22.4 / -18.2 / -12.3 / -0.3 | -18.3 / -16.7 / -12.5 / -2.9 | -17.8 / -15.8 / -15.2 | 0.5 -> 0.8 |
| EDC3 | -21.5 / -15.9 / -9.1 / 3.5 | -18.7 / -15.9 / -11.6 / -2.5 | -17.8 / -15.8 / -14.5 | -3.5 -> 0.9 |
| EDC4 | -22.9 / -18.7 / -13.1 / -2.1 | -18.6 / -15.2 / -10.5 / -1.2 | -17.5 / -14.9 / -13.3 | -1.4 -> 0.7 |
| EDC5 | -18.4 / -15.0 / -10.6 / -1.9 | -14.7 / -12.2 / -8.6 / -1.1 | -15.0 / -11.5 / -10.2 | -0.4 -> 0.9 |

FDS, FDP, ECU, EPB and APL are unchanged. The FDS and FDP arms rise faster than Brand's towards flexion; ECRL, ECRB and
ECU turn flexor at full flexion with ulnar deviation, which Loren's polynomials allow; EPB and APL run close to the axis.
These, EPL's large extension arm and the two corner sign changes of EPL and EDM are open checks
(`hand-literature-comparison.md`). On a 0.5 deg grid over flexion and deviation no wrist-crossing tendon length jumps and
no arm changes by more than 1.5 mm between neighbouring cells; the wrap jump table is re-recorded.

## MCP pulleys and hyperextension

**Before.** FDS and FDP 2-5 (all but FDP2, which wraps `2ndmcp_ellipsoid_wrap_r`) crossed the MCP on via sites only. Past 0 deg
the PP site swung dorsal of the line to the MC site, so the flexion arm fell from 6-10 mm at 0 deg to 0-2 mm at -60 deg (FDS3 an
extensor of 2 mm). EDC2 and EDC3 ran from a via site on top of the MC head (82-90 deg from distal about the MCP axis) to the hood
site `EM{d}_H_r`; past about -45 deg the hood site passed the via site and both turned flexors. With the old paths the MCP could
not leave 0 deg: the passive index MCP moment at -60 deg was 6 N cm against Knutson et al. 2000's 48 N cm (wrist neutral).

**Change** (`mcp_pulleys` in `build/data/extrinsic_muscles.json`, written by `generate_extrinsic_muscles.py`, chain blocks
`MCP pulleys 2-5`). One cylinder per tendon on the MCP flexion axis in the metacarpal (`mcp{d}_{muscle}_wrap_r`), with its side
site two radii out on the palmar (flexors) or dorsal (extensors) side (`mcp{d}_{muscle}_side_r`), the middle of the wrapped arc:

- Flexors (Landsmeer's model III in flexion, as An et al. 1983 Fig. 4 and Brook et al. 1995): the via sites still set the arm in
  flexion, rising with the angle; the cylinder, inserted after the last metacarpal site, holds the arm in hyperextension, where
  the tendons lie on the palmar plate. Its radius is the path's own arm at 0 deg, rounded down to 0.1 mm, so the flexion arms do
  not change. FDP2 keeps its wrap (already 8.8-9.6 mm in hyperextension).
- Extensors (Landsmeer's model I, constant arm: Brook et al. 1995; about constant in An et al. 1983 Fig. 4): EDC2 8.6 mm and EIP
  9.0 mm (An et al. 1983 Table 3), EDC3-5 and EDM scaled from EDC2 by An et al. 1979's long-extensor moment potentials and the
  model's middle phalanges (8.6 / 6.7 / 7.5 mm). The via site before the hood (`EDC{d}-P4_r`, `EIP-P5_r`, `EDM-P6_r`) moves about
  the MCP axis to 135 deg from distal, 1 mm outside the cylinder, on the MC neck. Past about -30 deg the tendon leaves the cylinder
  and its arm rises by up to 1.7 mm (bowstringing); it stays an extensor.
- The `3rdmcp` and `5thmcp` ellipsoid wraps and eleven side sites lost their last user and were removed (two of them were in
  `side_sites.json`). On MuJoCo 4 and 3.17 the paths are the same: every new wrap has its side site outside the cylinder and no
  endpoint crosses the ray opposite it in -61..90 deg; no tendon length jumps along the MCP sweeps (`tests/test_side_sites.py`).

MCP flexion moment arms in mm (+ = flexion), other joints at 0, MuJoCo 4 (3.17 identical); -60 / -30 / 0 / 90 deg and the mean over
0-85 deg (An et al. 1983's protocol):

| Muscle | Before | After | Literature |
| --- | --- | --- | --- |
| FDS2 | 1.5 / 5.2 / 8.6 / 13.0; 11.8 | 8.6 / 8.6 / 8.6 / 13.0; 11.8 | 11.9 ± 0.7 mean (An 1983); 9.6 at 0, 14.2 at 90, 6.6 at -60 extrapolated (Brook 1995); 15.2 neutral (An 1979 x model phalanx) |
| FDP2 | 8.8 / 9.5 / 9.6 / 14.4; 11.7 | unchanged | 11.1 ± 1.1 (An 1983); 8.3 at 0, 13.1 at 90, 5.3 at -60 extrapolated (Brook); 13.0 (An 1979) |
| EIP | -5.2 / -9.4 / -8.7 / -8.3; -7.1 | -10.0 / -9.0 / -9.0 / -9.0; -9.0 | -9.0 ± 1.3 (An 1983); -8.8 constant (Brook); -7.5 ± 3.4 (Lee 2008b) |
| EDC2 | +5.3 / -6.7 / -8.7 / -8.4; -7.6 | -9.4 / -8.6 / -8.6 / -8.6; -8.6 | -8.6 ± 1.6 (An 1983); -8.3 (Brook); -7.0 ± 3.2 (Lee 2008b); -12.0 (An 1979); -9.7 (Ketchum) |
| FDS3 / FDP3 | -2.0 / 3.9 / 9.5 / 15.7 and 0.7 / 4.7 / 8.5 / 13.7 | 9.5 / 9.5 / 9.5 / 15.7 and 8.4 / 8.4 / 8.5 / 13.7 | 14.8 / 11.8 neutral (An 1979) |
| EDC3 | +4.2 / -6.2 / -9.4 / -6.7; -7.9 | -8.8 / -9.5 / -8.6 / -8.6; -8.6 | -11.9 (An 1979); -12.3 (Ketchum) |
| FDS4 / FDP4 | 0.4 / 3.7 / 6.8 / 10.2 and 0.3 / 3.6 / 6.6 / 10.4 | 6.6 / 6.6 / 6.8 / 10.2 and 6.5 / 6.4 / 6.6 / 10.4 | 11.4 / 10.2 (An 1979) |
| EDC4 | -1.4 / -7.5 / -7.9 / -8.2; -7.2 | -6.6 / -7.5 / -6.6 / -6.7; -6.7 | -9.3 (An 1979); -10.5 (Ketchum) |
| FDS5 / FDP5 | 0.0 / 3.4 / 6.5 / 10.4 and -0.4 / 2.8 / 5.9 / 10.9 | 6.4 / 6.4 / 6.5 / 10.4 and 5.8 / 5.8 / 5.9 / 10.9 | 11.1 / 10.3 (An 1979) |
| EDC5 / EDM | -4.3 / -6.2 / -5.9 / -7.2 and -4.4 / -6.6 / -6.3 / -6.4 | -8.3 / -8.1 / -7.5 / -7.5 (both) | -10.5 (An 1979, one long extensor); -9.8 (Ketchum, EDC5) |

An 1979's moment potentials are normalised by the middle phalanx and taken at neutral; times this model's phalanges they lie
1.2-1.4 times above An 1983's range means, as in An 1983 itself, so they rank the digits rather than set values. Ketchum's values
are one posture of six cadavers (Brand & Hollister 1993, Table 14-1). No measured MCP moment arm in hyperextension was found:
there the model rests on continuity and on Knutson's passive moment (below). The flexion arms of the flexors are unchanged, and
the index means stay within one SD of An 1983 (FDP2 within 0.6 mm).

**Extensor arms in flexion (checked, not changed).** An et al. 1983 state that the extension arms of EI and EC "were fairly
constant throughout the range of motion" (p. 422); their single-specimen Fig. 4a reads about 7.4 / 7.6 / 6.8-7.0 mm at 0 / 45 /
90 deg (a fall of under 1 mm past 45 deg), their range means are 8.6 ± 1.6 (EC) and 9.0 ± 1.3 mm (EI), Brook et al. 1995 fit a
constant arm (8.3 / 8.8 mm), and Lee et al. 2008b find no effect of MCP posture on the effective extensor arms (p = 0.42 / 0.33).
The model's constant 8.6 / 9.0 mm over 0-90 deg match the means, and Fig. 4a's specimen lies within one SD of them, so no
angle-dependent radius is introduced. In the active fist the passive EDC3 gives -0.04 N m at the middle MCP against +5.4 N m
from FDS3 and FDP3, so the extensor arm did not hold the fist open (below and `hand-literature-comparison.md`).

## PIP pulleys

**Before.** FDS and FDP 2-5 crossed the PIP on via sites only: a last site in the proximal phalanx (PP), 4-7 mm proximal
of the joint (FDP3 and FDP4: their mid-shaft site), and a first site in the middle phalanx (MP). In flexion the MP site
swings around the joint toward the PP site; once the two lie on one ray from the axis (at 180 deg minus their angles from
the bones' long axes) the segment's line passes dorsal of the axis. FDS2 and FDS3 turned extensor past about 105 and
100 deg, FDP2 flipped from +10.4 to -10.4 mm between 95 and 100 deg, and FDP4 and FDP5 fell below 1 mm at 115 deg. The PIP
flexes to 115 deg, so a fist reaches this zone.

**Why not a cylinder alone.** The MCP pulleys work because a flexor wraps its cylinder in hyperextension, on the convex
side. In PIP flexion the flexors lie on the concave side: MuJoCo wraps a cylinder only when the straight segment cuts it
(or when the side site's homotopy class differs), and here the segment between the PP and MP sites never comes closer to
the axis than the PP site, so a cylinder on the axis never engages. A side site inside the cylinder (inside wrap) holds the
tendon on the circle and gives an arm of about r cos(theta / 2), falling with flexion; via sites far from the joint
(A2 and A4 mid-shaft) keep the arm but bowstring 3-5 mm above the literature in midrange. Neither was kept.

**Change** (`pip_pulleys` in `build/data/extrinsic_muscles.json`, written by `generate_extrinsic_muscles.py`, chain blocks
`PIP pulleys 2-5`; fit `--fit-pip`). Per tendon with the defect (FDS2-4, FDP2-5; FDS5 keeps 2.8-10.2 mm over the range):

- a cylinder on the PIP axis in the PP (`pip{d}_{muscle}_wrap_r`, side site two radii out on the palmar side), inserted
  after the last PP site, with the path's own arm at 0 deg as radius (rounded down to 0.1 mm): it holds that arm near
  extension and in hyperextension, where the tendon lies on the volar plate;
- the pulley points next to it: the PP site moves toward the previous PP site on the path (fraction 0-0.6; the A2
  pulley's distal edge) and the first MP site up to 3 mm dorsally (toward the bone) and up to 6 mm distally (A4). A grid
  search keeps the arm at least the cylinder's radius from 60 to 120 deg (5 deg past the limit) and takes the smallest RMS
  change of the arm over 0-90 deg, where the old path agreed with the literature.

FDP2's 3 mm distal move, FDP4's 5 mm and FDS3's 2 mm are the largest; every MP site moved 3 mm dorsally except FDP5's
(1 mm). The DIP arms do not change (the FDP sites past the first MP site are untouched): FDP2 4.3 / 6.3 / 5.1 mm at DIP 0 /
60 / 90 deg (An et al. 1983: 4.1 ± 1.4 mm mean; Brook et al. 1995: 3.0 / 4.4 / 5.2), continuous and flexor.

PIP flexion moment arms in mm (+ = flexion), other joints at 0, MuJoCo 4 (3.17 identical); 0 / 30 / 60 / 90 / 100 / 110 / 115 deg
and the mean over 0-90 deg (An et al. 1983's protocol):

| Muscle | Before | After | Literature |
| --- | --- | --- | --- |
| FDS2 | 4.8 / 6.1 / 6.8 / 4.9 / 2.3 / -1.5 / -3.1; 6.0 | 4.7 / 5.9 / 7.6 / 8.6 / 8.8 / 8.7 / 8.4; 6.7 | 6.2 ± 1.0 mean (An 1983); 4.1 / 5.3 / 6.6 / 8.0 / 8.5 / 9.0 / 9.3, mean 6.0 (Brook 1995; past 90 extrapolated); 7.2 neutral (An 1979 x model phalanx); 8.3 (Ketchum) |
| FDP2 | 6.9 / 8.7 / 9.9 / 10.4 / -7.2 / -10.4 / -10.4; 9.1 | 6.8 / 8.3 / 10.3 / 11.6 / 11.8 / 11.9 / 11.7; 9.2 | 7.9 ± 1.1 mean (An 1983); 5.8 / 7.1 / 8.5 / 10.0 / 10.6 / 11.2 / 11.6, mean 7.8 (Brook); 9.0 (An 1979); 9.8 (Ketchum) |
| FDS3 | 7.0 / 8.9 / 9.9 / 6.9 / 1.2 / -5.4 / -7.4; 8.8 | 7.0 / 9.0 / 11.3 / 12.5 / 12.5 / 11.7 / 10.7; 9.9 | 6.8 (An 1979); 8.7 (Ketchum) |
| FDP3 | 7.5 / 10.9 / 12.5 / 10.7 / 9.0 / 6.4 / 4.9; 11.0 | 7.5 / 9.5 / 12.1 / 12.3 / 11.4 / 9.9 / 8.9; 10.5 | 9.1 (An 1979); 10.7 (Ketchum) |
| FDS4 | 3.1 / 4.6 / 5.3 / 4.7 / 4.0 / 2.9 / 2.3; 4.6 | 3.0 / 4.1 / 5.2 / 5.2 / 4.8 / 4.1 / 3.7; 4.5 | 5.8 (An 1979); 8.5 (Ketchum) |
| FDP4 | 7.1 / 9.2 / 9.3 / 6.1 / 4.2 / 1.9 / 0.7; 8.5 | 7.1 / 9.3 / 12.0 / 12.3 / 11.5 / 10.2 / 9.2; 10.3 | 7.1 (An 1979); 10.4 (Ketchum) |
| FDS5 | 3.9 / 6.9 / 9.3 / 10.2 / 9.9 / 9.2 / 8.6; 7.8 | unchanged (no pulley) | 6.1 (An 1979); 7.4 (Ketchum) |
| FDP5 | 4.8 / 6.6 / 7.6 / 6.6 / 5.0 / 2.2 / 0.4; 6.7 | 4.7 / 6.0 / 7.6 / 8.0 / 7.6 / 6.7 / 5.9; 6.6 | 7.7 (An 1979); 8.5 (Ketchum) |

An 1979's values are neutral moment potentials times this model's middle phalanges (25.1 / 29.1 / 24.9 / 21.0 mm); Ketchum's are one
posture of six cadavers (Brand & Hollister 1993, Table 14-1). No measured PIP arm past about 90 deg was found; Brook's curve is
Landsmeer's model III fitted to An 1983 over 0-90 deg and rises further when extrapolated. The index means stay within one SD of An
1983 for FDS2 (6.7 against 6.2 ± 1.0) and FDP2 is 1.2 SD above (9.2 against 7.9 ± 1.1; 9.1 before). In deep flexion the arms now
peak and ease off instead of collapsing: the smallest arm from 90 to 115 deg over digits 2-5 is 3.7 mm for FDS (FDS4) and 5.9 mm
for FDP (FDP5), against -7.4 and -10.4 mm before. The ring and little FDS and FDP stay below An 1979 and Ketchum, as before.

The paths are the same on MuJoCo 4 and 3.17 and have no tendon length jumps along the PIP, MCP and wrist sweeps
(sweeps of the MCP step); the lumbrical node tie tendons copy the FDP path, wrap included.

Refits: intrinsic (the lumbrical nodes ride the FDP paths), extrinsic passive, springs, postures and pads; the FCU, ECU and
junctura fits are untouched (their paths do not cross the PIP). The extrinsic passive fit now also starts its search from the
tabled values of the last fit: from its fixed starts alone it landed in a worse optimum under the new paths (joint error 1482
against 1170 at the previous values), and the wrist-flexion spring then missed Formica et al. 2012 by 0.119 N m. Effects on the
fist, grip and tenodesis: `hand-literature-comparison.md` (PIP pulley step).

## Refits

Extrinsic slack lengths and passive scales (`fit_muscle_passive.py`), then the joint springs
(`fit_passive_moments.py`); numbers in `hand-literature-comparison.md` (S7, EDC2 wrap) and the CHANGELOG.

After the ECU re-route the same chain was run again (passive muscles, extrinsic fits, springs). The ECU's
smaller ulnar arm leaves the muscles a larger deficit in radial-deviation stiffness, so a new spring segment
(`passive_wrist_deviation_pos_r`) covers it: muscles and springs now match Formica's wrist deviation
stiffness with 0.006 N m rms (0.054 before), wrist flexion 0.010 (0.011). The ECU's passive scale falls 0.83
-> 0.75 (slack +2.1 -> +0.3 %); the joint-fit error of the muscles alone rises 866 -> 1053 (without the
release step 690 -> 860), the deficit at the wrist; the release bound of EDC3-5 and EDM 0.0261 -> 0.0267.

## References

- An KN, Chao EY, Cooney WP, Linscheid RL. Normative model of human hand for biomechanical analysis. J Biomech 12(10):775-788, 1979.
- An KN, Ueba Y, Chao EY, Cooney WP, Linscheid RL. Tendon excursion and moment arm of index finger muscles. J Biomech 16(6):419-425, 1983 (Table 3, Fig. 4).
- Brand PW, Hollister AM. Clinical Mechanics of the Hand, 2nd ed. Mosby, 1993 (Table 14-1, after Ketchum & Thompson; Ch. 12, wrist moment arms, Figs 12-7, 12-9, 12-18, 12-42, 12-49, 12-53, 12-58, 12-65, 12-76, 12-77).
- Brook N, Mizrahi J, Shoham M, Dayan J. A biomechanical model of index finger dynamics. Med Eng Phys 17(1):54-63, 1995.
- Akhbari B et al. Biomechanics of the distal radioulnar joint during in vivo forearm pronosupination. J Wrist Surg 10(3):208-215, 2021.
- Garcia-Elias M et al. Extensor mechanism of the fingers. II. Tensile properties of components. J Hand Surg Am 16(6):1136-1140, 1991.
- Haugstvedt JR, Berger RA, Berglund LJ. A mechanical study of the moment-forces of the supinators and pronators of the forearm. Acta Orthop Scand 72(6):629-634, 2001 (Fig. 3, read from the figure).
- Holzbaur KRS, Murray WM, Delp SL. A model of the upper extremity for simulating musculoskeletal surgery and analyzing neuromuscular control. Ann Biomed Eng 33(6):829-840, 2005 (Table 1). Cited only; no model file used.
- Formica D et al. The passive stiffness of the wrist and forearm. J Neurophysiol 108(4):1158-1166, 2012 (Table 2).
- Horii E, An KN, Linscheid RL. Excursion of prime wrist tendons. J Hand Surg Am 18(1):83-90, 1993.
- Leijnse JNAL. Measuring force transfers in the deep flexors of the musician's hand: theoretical analysis, clinical examples. J Biomech 30:873, 1997.
- Knutson JS, Kilgore KL, Mansour JM, Crago PE. Intrinsic and extrinsic contributions to the passive moment at the metacarpophalangeal joint. J Biomech 33(12):1675-1681, 2000.
- Lee SW, Chen H, Towles JD, Kamper DG. Estimation of the effective static moment arms of the tendons in the index finger extensor mechanism. J Biomech 41(7):1567-1573, 2008.
- Jacobson MD et al. Architectural design of the human intrinsic hand muscles. J Hand Surg Am 17(5):804-809, 1992 (APL via Holzbaur et al. 2005, Table 1).
- Lieber RL et al. Architecture of selected muscles of the arm and forearm: anatomy and implications for tendon transfer. J Hand Surg Am 17(5):787-798, 1992 (Table I).
- Loren GJ et al. Human wrist motors: biomechanical design and application to tendon transfers. J Biomech 29(3):331-342, 1996 (appendix).
- Matsuki KO et al. In vivo 3D kinematics of normal forearms: analysis of dynamic forearm rotation. Clin Biomech 25(10):979-983, 2010.
- Mirakhorlo M et al. Anatomical parameters for musculoskeletal modeling of the hand and wrist. Int Biomech 3(1):40-49, 2016 (Table 2; CC BY 4.0).
- Matsuzawa K et al. The origin structure of each finger in the flexor digitorum superficialis muscle. Surg Radiol Anat 43:3-10, 2021.
- Murray WM, Delp SL, Buchanan TS. Variation of muscle moment arms with elbow and forearm position. J Biomech 28(5):513-525, 1995. Cited only (its forearm axis definition).
- von Schroeder HP, Botte MJ. The functional significance of the long extensors and juncturae tendinum in finger extension. J Hand Surg Am 18(4):641-647, 1993.
- Smutz WP et al. Mechanical advantage of the thumb muscles. J Biomech 31(6):565-570, 1998.
- Yammine K. The anatomy and prevalence of the juncturae tendinum in the hands. Eur J Anat 19(4):351-359, 2015.
- Zatsiorsky VM, Li ZM, Latash ML. Enslaving effects in multi-finger force production. Exp Brain Res 131:187-195, 2000.
