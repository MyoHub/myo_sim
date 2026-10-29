# Proposal: explicit ligaments as passive spatial tendons

Status: draft for discussion. Nothing in the models changes with this page. It describes
what explicit ligaments would add, how they would fit the model policy and the passive
joint-moment fits, how we would validate them, and what a prototype says about their
cost.

## 1. The issue

Passive joint behaviour in the reference models is lumped. The draft MyoHub/myo_sim#139
adds one-joint `<fixed>` tendon springs to the hand and wrist. Each spring is fitted to
the residual of a measured whole-joint passive moment, i.e. the measured moment minus the
passive moment of the model's own muscles at zero activation:

- wrist flexion and deviation: Formica et al. 2012;
- thumb CMC flexion and abduction: Domalain et al. 2010;
- index MCP (intrinsic part): Knutson et al. 2000;
- PIP and DIP: Kamper et al. 2002.

The question here is whether to model ligaments explicitly, as passive spatial tendons
between bone sites, and if so how.

**What ligaments cannot add here.** In the body, ligaments hold the bones together and
limit off-axis motion. Our joints are ideal hinges, ball joints or kinematically coupled
joints, so the joint itself already does both. An explicit ligament in this model can
only add a generalized force about the degrees of freedom that exist. For a one-DOF hinge
(PIP, DIP, elbow flexion, the coupled knee), that is one more nonlinear one-joint spring.
The fitted residual spring already provides this from in-vivo whole-joint data, and it
does so with fewer assumptions. Adding ligaments to such joints changes how the moment is
parameterised, not what the model can do.

**Where explicit ligaments do pay off:**

- **Joints with several DOFs, where the passive moment depends on the combination of
  angles.** One-joint springs cannot represent this coupling.
  - MCP: the dorsal portions of the collaterals lengthen with flexion and the volar
    portions with extension, so different portions restrain the joint at the two ends of
    the range (Minami et al. 1984). Ab/adduction laxity falls with flexion (Minami et al.
    1985), which couples abduction to flexion.
  - Thumb CMC (two DOFs). #139 had to leave out Domalain's cross terms because single-joint
    springs cannot hold them.
  - The CMC4/5 joints now being added.
  - PIP lateral deviation, if that DOF is added.
  - The knee's coupled rotations, if those are freed.
  - The glenohumeral capsule across elevation, plane of elevation and rotation.
  - The elbow collaterals, if a varus-valgus DOF exists.
- **Ligament forces as outputs.** Injury, surgery and rehabilitation studies ask for the
  force in a named structure (e.g. a collateral, the ACL or the inferior glenohumeral
  ligament), not for a lumped joint moment.
- **A passive moment derived from anatomy rather than from a curve fit.** Attachment
  geometry and a force-strain law can generalise to postures and DOF combinations nobody
  measured. A polynomial fitted over one measured span does not. #139 documents exactly
  this failure: a cubic that turns back toward flexion outside Kamper's range, and a CMC
  adduction cubic extrapolated 26 deg past the data.
- **Directions with no measured passive data.** MCP abduction is a "gap" row in #139's
  table: no data, no spring. Collaterals with literature properties would give it a
  passive moment with a stated provenance.

**How ligaments coexist with the residual springs.** The decomposition gains a term:

`residual spring = measured passive moment - muscle passive moment - ligament moment`

The ligaments are fitted first, from anatomy and tissue data, never to the joint-moment
curve. The residual springs are then refitted to whatever is left. A ligament must never
contribute more than the residual it replaces, at any angle inside the measured span.

## 2. Literature

DOIs were checked against PubMed / Europe PMC. Numbers are quoted as published. Where a
value is not in an abstract or an open full text, it is listed as to be confirmed from
the cited paper.

### Hand: MCP, PIP, thumb

- **MCP collaterals: geometry and length change.**
  - Minami A, An KN, Cooney WP, Linscheid RL, Chao EY (1984). Ligamentous structures of
    the metacarpophalangeal joint: a quantitative anatomic study. J Orthop Res
    1(4):361-368. doi:10.1002/jor.1100010404.
  - Specimens: index MCP joints of 10 hands (6 donors, mean age 52 years), 7 of them
    measured by biplanar radiography with markers at the origin and insertion of the
    dorsal, middle and volar thirds of each collateral, under a 20 N joint compression.
  - Each collateral has a superficial and a deep layer. The radial collateral is thicker
    and wider and attaches closer to the joint margin; its origin is further distal than
    the ulnar one's, and the ulnar collateral is less oblique (pp. 363-364). Both arise
    from a comma-shaped area dorsoproximal to the rotation axis on the metacarpal head and
    insert progressively more volarly on the phalanx base, about a quarter of the base
    width onto its volar aspect. The accessory collateral runs from just proximal and
    volar of the proper origin to the distal two thirds of the volar plate's lateral
    border, so its insertion moves with the plate, which bends 10-55 deg against the
    metacarpal during flexion (p. 365).
  - Attachment coordinates, n = 7, mean ± SD in mm, in a metacarpal frame (origin at the
    centre of the head's lateral-view profile, x along the shaft) and a proximal-phalanx
    frame: Table 2, p. 366. The proper-collateral origins lie 6.0-7.9 mm dorsal of the
    head centre (y; accessory collaterals 4.8-5.3 mm) and their insertions 3.1-5.3 mm
    volar of the phalanx-base origin; the radial collateral sits closer to the shaft
    centre line (z) than the ulnar one.
  - Index MCP flexed from 0 to 80 deg (abstract; text p. 365; Fig. 7, p. 367):
    - the dorsal portions of both collaterals lengthened by 3-4 mm;
    - the middle portions by 0.4-1.0 mm;
    - the volar portions shortened by 1-2 mm.
  - The dorsal portions lengthen from 0 to 50 deg, stay constant to 70 deg and shorten
    slightly beyond; the volar portions shorten from 20 to 80 deg (p. 367). The apparent
    lengths in Fig. 7 lie between roughly 15 and 25 mm.
  - In hyperextension the dorsal portions shortened by 2-3 mm, the middle portions
    slightly, and the volar portions lengthened slightly.
  - These are apparent lengths (marker distances). They follow the true length only while
    the ligament is under tension; a slack ligament crimps (p. 367).
  - Implication: one strand per collateral cannot reproduce this cam behaviour. Use at
    least a dorsal and a volar strand, preferably three.
- **MCP collaterals: sectioning.** Minami A et al. (1985). Ligament stability of the
  metacarpophalangeal joint: a biomechanical study. J Hand Surg Am 10(2):255-260.
  doi:10.1016/s0363-5023(85)80117-9.
  - 15 fresh fingers (7 index, 4 long, 4 ring; donors aged 40-70), load-displacement
    tests with sequential sectioning at 0 and 75 deg flexion, n = 3 per condition.
    Contributions are shares of the total capsuloligamentous restraining load, mean ± SE.
  - The proper collaterals are the primary stabilizers in all four modes (distraction,
    dorsopalmar, ab/adduction, pronation-supination). Together they carry about 85 % of
    the distraction restraint at 0 deg and about 72 % at 75 deg (Table I, p. 256).
  - The accessory collaterals act mainly in ab/adduction, and only in extension: the
    ulnar accessory carries 37.3 ± 6.3 % of the abduction restraint at 0 deg and
    8.4 ± 5.2 % at 75 deg; the radial accessory 31.2 ± 14.3 % and 9.4 ± 6.9 % of the
    adduction restraint. The proper collateral on the tension side carries the rest
    (57-58 % at 0 deg, 88 % at 75 deg; Table IV, p. 257).
  - The palmar plate prevents dorsal dislocation only at full extension (19.5 ± 6.4 % at
    0 deg, 0 at 75 deg; Table III). The dorsal capsule carries about 20 % of the
    distraction restraint in flexion (Table I).
  - Laxity in every mode except distraction falls with flexion, most of it between 0 and
    45 deg (p. 258).
- **Thumb MCP UCL elongation.** Sircar K, Hägermann C, Scaal M, Müller LP, Dargel J,
  Wegmann K (2024). Thumb metacarpophalangeal joint kinematics and elongation of the
  ulnar collateral ligament. J Hand Surg Am 49(1):8-14. doi:10.1016/j.jhsa.2023.10.001.
  - 28 fresh-frozen thumbs (donors 64-81 years, mean 71), MCP loaded by 10 N on each of
    EPL and FPL. Radiopaque markers at the UCL origin and insertions, lateral radiographs
    at 0, 20, 40 and 60 deg flexion. Joints split by head sphericity into round
    (n = 15) and flat (n = 13).
  - Four marker distances from the common origin: to the dorsal (L1), middle (L2) and
    palmar (L3) insertion of the proper UCL, and to the dorsal insertion of the
    accessory UCL (L4).
  - Elongation relative to neutral (Table 3, p. 12): flat joints peak at 111 % (L1,
    40 deg) and fall back to 106 % at 60 deg; round joints are longest near 60 deg
    (L1 117 %, L2 112 %, L3 107-108 %, L4 113 %). The abstract attributes the 117 % to the accessory
    UCL; the table gives it to the dorsal proper portion.
  - The flexion axis lies at 29 % (flat) or 40 % (round) of the head's dorsopalmar
    depth and at 96 % or 86 % of metacarpal length (Table 1, p. 11). Absolute lengths
    are in the online supplement (Table S1).
  - Like Minami's, these are apparent lengths (marker distances).
- **PIP collaterals and volar plate**
  - Allison DM (2005). Anatomy of the collateral ligaments of the proximal
    interphalangeal joint. J Hand Surg Am 30(5):1026-1031.
    doi:10.1016/j.jhsa.2005.05.015.
    - Proper collateral fibres run parallel to the middle phalanx in all joint positions.
    - The accessory collateral lies between the proper collateral and the volar plate.
  - Bowers WH, Wolf JW, Nehil JL, Bittinger S (1980). The proximal interphalangeal joint
    volar plate. I. An anatomical and biomechanical study. J Hand Surg Am 5(1):79-88.
    doi:10.1016/s0363-5023(80)80049-9.
    - Anatomy in 56 PIP joints of 14 hands. The plate averages 1.5 mm thick (0.5-2.5 mm),
      about a third of the condylar radius (p. 81). Its main distal attachments are
      lateral, confluent with the proper collateral insertion; the accessory collateral
      slings it from the proper collateral and the condyles.
    - Sequential sectioning under a static hyperextension load (p. 81-82): accessory
      collateral alone +5 deg, central 80 % of the distal attachment +5 deg, proximal
      attachments +10 deg; with the lateral distal attachments also cut, 25-35 deg.
    - Hyperextension to failure in 20 further digits (Table I, p. 85): 7-59 kgf at the
      volar complex (lever-corrected). At 20 cm/min the index averaged 39.5 kgf and the
      little finger 22.5 kgf (about 390 and 220 N; p. 84-85). Fast loading ruptured the
      distal attachment, slow loading (2 cm/min) attenuated the proximal check
      ligaments.
  - Rhee RY, Reading G, Wray RC (1992). A biomechanical study of the collateral
    ligaments of the proximal interphalangeal joint. J Hand Surg Am 17(1):157-163.
    doi:10.1016/0363-5023(92)90134-b.
    - 68 fresh fingers, donors over 60 (mean 67). The PIP was held at 0 deg and bent
      sideways to failure; the force at the ligament was computed from the lever arms
      (Fig. 2, p. 158).
    - Failure force 176.9 ± 5.9 N (mean ± SE, n = 58), after about 7 mm of loading-head
      travel (p. 161). By loading rate (Table III, p. 160): 184 ± 8 N at 1 mm/s,
      171 ± 10 N at 4 mm/s, 165 ± 16 N at 10 mm/s, not significantly different; the rate
      changes the rupture site (Table IV).
    - Radial 197.3 ± 9.7 vs ulnar 164.9 ± 12.9 N, not significant (Table I). The little
      finger is about 27 % weaker: 142 ± 8 N against 185-203 N for index, long and ring
      (Table II).
    - The lateral collateral is the primary restraint to medial-lateral stress: removing
      the extensor hood and palmar plate left 144.8 ± 19 N against 171.0 ± 10 N intact
      (not significant), while halving the collateral cut it to 118.6 ± 5 N (Table VI,
      p. 162).
    - No stiffness is reported.
  - Dzwierzynski WW, Pintar F, Matloub HS, Yoganandan N (1996). Biomechanics of the
    intact and surgically repaired proximal interphalangeal joint collateral ligaments.
    J Hand Surg Am 21(4):679-683. doi:10.1016/s0363-5023(96)80026-8. Native ligament
    strength in axial distraction: 162.5 N.
- **Thumb CMC (trapeziometacarpal) anatomy**
  - Bettinger PC, Linscheid RL, Berger RA, Cooney WP, An KN (1999). An anatomic study of
    the stabilizing ligaments of the trapezium and trapeziometacarpal joint. J Hand Surg
    Am 24(4):786-798. doi:10.1053/jhsu.1999.0786. Sixteen ligaments and their
    attachments.
  - Bettinger PC, Berger RA (2001). Functional ligamentous anatomy of the trapezium and
    trapeziometacarpal joint (gross and arthroscopic). Hand Clin 17(2):151-168.
    doi:10.1016/s0749-0712(21)00236-5.
  - Ladd AL, Lee J, Hagert E (2012). Macroscopic and microscopic analysis of the thumb
    carpometacarpal ligaments: a cadaveric study of ligament anatomy and histology.
    J Bone Joint Surg Am 94(16):1468-1477. doi:10.2106/jbjs.k.00329.
    - 30 hands (10 fresh-frozen, 20 embalmed) from 19 donors, mean age 76 years.
    - Seven principal ligaments, present in all 30 hands:
      - three dorsal (dorsal radial, dorsal central, posterior oblique), a "deltoid"
        complex fanning from the dorsal tubercle of the trapezium to the dorsal MC1 base;
      - two volar (anterior oblique, ulnar collateral, also called the first volar
        trapeziometacarpal ligament);
      - two ulnar (first dorsal trapeziometacarpal, intermetacarpal).
    - Attachments (pp. 1471-1474): dorsal radial from the radial side of the dorsal
      tubercle to the dorsoradial MC1; dorsal central from its centre to the central
      dorsal MC1 edge; posterior oblique from its ulnar side to the ulnar MC1, at the
      first dorsal interosseous origin; anterior oblique from the volar trapezium edge to
      the volar MC1 "beak", ulnar to the APL insertion; ulnar collateral from the distal
      edge of the transverse carpal ligament and the volar-ulnar trapezial ridge to the
      volar-ulnar MC1; first dorsal trapeziometacarpal from the dorsal-ulnar trapezium
      edge to next to the posterior oblique insertion; intermetacarpal from the dorsal
      radial edge of MC2 to the volar-ulnar MC1.
    - Taut positions under manual passive load (pp. 1472-1474): the dorsal complex in
      flexion; the anterior oblique in extension and adduction; the ulnar collateral in
      extension and abduction; the first dorsal trapeziometacarpal in abduction and
      pronation; the intermetacarpal at the extremes of abduction.
    - Dimensions, measured with each ligament at maximum passive tension (Table I,
      p. 1473): lengths 9.2-13.0 mm (posterior oblique longest, 13.03 ± 1.58 mm; dorsal
      central shortest, 9.18 ± 1.29 mm), widths 2.2-6.0 mm, thicknesses 0.71-1.95 mm.
    - The dorsal ligaments are thicker (1.71-1.95 mm) than the volar ones (p < 0.05).
    - The anterior oblique ligament is thin (0.71 ± 0.11 mm), variable, and histologically
      closer to capsule than to a proper ligament.
  - Hagert E, Lee J, Ladd AL (2012). Innervation patterns of thumb trapeziometacarpal
    joint ligaments. J Hand Surg Am 37(4):706-714.e1. doi:10.1016/j.jhsa.2011.12.038.
- **Thumb CMC mechanics**
  - Bettinger PC, Smutz WP, Linscheid RL, Cooney WP, An KN (2000). Material properties
    of the trapezial and trapeziometacarpal ligaments. J Hand Surg Am 25(6):1085-1095.
    doi:10.1053/jhsu.2000.18487.
    - Bone-ligament-bone tests of seven ligaments.
    - The dorsoradial ligament has the greatest ultimate load.
    - The anterior oblique ligament has the least stiffness.
    - No stiffness or ultimate-load value is taken from it. The measured thumb CMC
      stiffness comes from D'Agostino et al. (2014), below.
  - D'Agostino P, Kerkhof FD, Shahabpour M, Moermans JP, Stockmans F, Vereecke EE
    (2014). Comparison of the anatomical dimensions and mechanical properties of the
    dorsoradial and anterior oblique ligaments of the trapeziometacarpal joint. J Hand
    Surg Am 39(6):1098-1107. doi:10.1016/j.jhsa.2014.02.025.
    - 13 fresh-frozen thumbs from 9 donors (mean age 74 years, Eaton stage I-II;
      Table 2, p. 1100). Lengths between the deep bony attachments on MRI (9 hands),
      width and thickness by caliper at maximal passive tension (pp. 1100-1101).
    - Dimensions, mean ± SD (Tables 3-5, pp. 1103-1105): DRL length 10.0 ± 1.0 mm on
      MRI (12.9 ± 2.1 mm by dissection), thickness 1.1 ± 0.2 mm, width 12.1 ± 1.6 mm;
      AOL 12.2 ± 2.1 mm (13.0 ± 2.6 mm), 0.6 ± 0.1 mm, 13.6 ± 2.6 mm. The DRL appeared
      as one sheet with no separate dorsal central ligament (Fig. 2), so its width
      covers what Ladd et al. 2012 split into the dorsal radial and dorsal central
      ligaments.
    - Bone-ligament-bone complexes, cyclic displacement-controlled loading at 2 Hz up to
      150 N or failure. Stiffness is the slope of the linear part in cycle 20, the
      highest value over the subsequent tests (p. 1101).
    - **Stiffness: DRL 89 ± 21 N/mm, AOL 65 ± 30 N/mm** (p. 1103; Tables 4-5, whose
      column header prints the unit as MPa). Hysteresis 25 ± 3 % and 21 ± 4 %. The DRL
      is significantly stiffer, shorter and thicker than the AOL.
    - These are the only measured thumb CMC ligament stiffnesses used here. With the
      MRI dimensions they correspond to an effective modulus k L / A of about 67 MPa
      (DRL) and 97 MPa (AOL).
  - Colman M, Mass DP, Draganich LF (2007). Effects of the deep anterior oblique and
    dorsoradial ligaments on trapeziometacarpal joint stability. J Hand Surg Am
    32(3):310-317. doi:10.1016/j.jhsa.2006.12.002. In lateral pinch, mean 3-D translation
    increased by 1.3 mm after DRL transection and by 0.6 mm after dAOL transection.
  - Najima H, Oberlin C, Alnot JY, Cadot B (1997). Anatomical and biomechanical studies
    of the pathogenesis of trapeziometacarpal degenerative arthritis. J Hand Surg Br
    22(2):183-188. doi:10.1016/s0266-7681(97)80058-7.
    - 32 hands, donors 71-95 years (mean 84).
    - Five ligaments after Pieron: anterior oblique, ulnar collateral, first
      intermetacarpal, posterior oblique and dorsoradial. All were present in every hand
      except the ulnar collateral (31/32). Seven hands had two intermetacarpal ligaments,
      an anterior one to the palmar-ulnar MC1 tubercle and a posterior one to the
      dorsoulnar MC1 (p. 185).
    - Attachments (p. 185): the anterior oblique, posterior oblique, ulnar collateral and
      intermetacarpal ligaments all insert on or near the palmar-ulnar tubercle of the MC1
      base; the posterior oblique arises from the dorsoulnar trapezial tubercle, the
      dorsoradial from the dorsoradial tubercle (to the dorsal MC1 edge), the ulnar
      collateral from the flexor retinaculum, the intermetacarpal from the dorsoradial
      MC2 base.
    - Excised ligaments pulled to failure at 5 mm/min, 31 mm between the clamps (p. 184).
      Maximum load, all cartilage grades (Table 2, p. 186): anterior oblique 5.19,
      ulnar collateral 5.63, intermetacarpal 4.46, posterior oblique 7.06, dorsoradial
      7.74 N (SEM 0.4-0.7 N). Maximum strain 104-110 %, dorsoradial 137 % (Fig. 5).
    - Strength of the anterior oblique, intermetacarpal and posterior oblique falls with
      age and with cartilage degeneration.
    - No stiffness is reported. The loads are one to two orders of magnitude below the
      PIP collateral and wrist ligament values above, and the strain is grip-to-grip on
      excised, very old tissue. Use them for the strength ranking (dorsoradial and
      posterior oblique strongest, intermetacarpal weakest), not for k.
  - Esplugas M et al. (2016). J Wrist Surg 5(4):277-283. doi:10.1055/s-0036-1581052.
  - Lin JD, Karl JW, Strauch RJ (2014). Trapeziometacarpal joint stability: the evolving
    importance of the dorsal ligaments. Clin Orthop Relat Res 472(4):1138-1145.
    doi:10.1007/s11999-013-2879-9.
  - Norose M et al. (2022). Sci Rep 12:20790. doi:10.1038/s41598-022-25355-3. Argues
    that the dorsal "ligaments" belong to a capsulo-aponeurotic complex with the first
    dorsal interosseous.
- **How much of the passive moment is ligament.**
  - Kuo PH, Deshpande AD (2012). Muscle-tendon units provide limited contributions to
    the passive stiffness of the index finger metacarpophalangeal joint. J Biomech
    45(15):2531-2538. doi:10.1016/j.jbiomech.2012.07.034.
    - 10 healthy adults (6 men, 4 women, 23 ± 3.7 years), index MCP driven through its
      range in vivo. Peak passive moment 438 ± 185 N mm in full extension and
      288 ± 71 N mm in full flexion (p. 2533).
    - The muscle-tendon share is computed, not measured: a subject-scaled model of the
      seven MCP muscles (Holzbaur-based parameters) is subtracted from the measured
      total, and the rest is assigned to the capsule-ligament complex.
    - The muscle-tendon units do 24.9 ± 10.6 % of the passive work (Fig. 7, p. 2536);
      the flexors 31.8 ± 11.7 % in extension and the extensors 19.9 ± 12.9 % in flexion
      (Fig. 8, p. 2537). The intrinsic muscles contribute almost nothing.
  - Knutson JS, Kilgore KL, Mansour JM, Crago PE (2000). Intrinsic and extrinsic
    contributions to the passive moment at the metacarpophalangeal joint. J Biomech
    33(12):1675-1681. doi:10.1016/s0021-9290(00)00159-7.
    - 8 subjects (6 men, 2 women, 21-39 years), index MP at seven wrist angles. The
      extrinsic part is whatever changes with wrist angle; the intrinsic part (capsule,
      ligaments, cartilage and inactive intrinsic muscles) is a function of MP angle
      only (Eqs. 1-4, p. 1677).
    - Near the MP extension limit (40 N cm) the median extrinsic share is 94 % with the
      wrist extended 60 deg, 57 % at wrist 0 and 14 % with the wrist flexed 60 deg; near
      the flexion limit it rises from 40 % to 88 % from wrist extension to flexion
      (p. 1678; Fig. 5, p. 1680).
    - The intrinsic component rises faster than the extrinsic one in every subject.
      Median intrinsic slack angles: -18 deg (volar tissues, hyperextension) and 79 deg
      (dorsal tissues, flexion) (Table 1, p. 1678).
  - Together these are the closest thing to a published partition, and they disagree on
    the muscle share: Kuo's modelled muscles take a quarter of the work, while
    Knutson's wrist-dependent part takes more than half at the extension limit with the
    wrist neutral. Kuo finds the intrinsic muscles negligible, so Knutson's single-joint
    part is mostly capsule and ligament. So ligaments may take most of the
    single-joint residual that #139 fits from Knutson, minus a share for the capsule;
    not most of the total passive moment.
- **The sources the residual springs use**
  - Formica D, Charles SK, Zollo L, Guglielmelli E, Hogan N, Krebs HI (2012). The
    passive stiffness of the wrist and forearm. J Neurophysiol 108(4):1158-1166.
    doi:10.1152/jn.01014.2011.
  - Domalain M, Vigouroux L, Berton E (2010). Determination of passive moment-angle
    relationships at the trapeziometacarpal joint. J Biomech Eng 132(7):071009.
    doi:10.1115/1.4001397.
    - 10 men (26.0 ± 5.2 years), in vivo, wrist neutral, thumb MCP and IP free.
    - Each moment depends on both angles (Eqs. 2-3, Table 1, p. 071009-3): the
      flexion-extension moment has an abduction-angle exponential, and the
      ab/adduction moment a flexion-angle one. These are the cross terms a one-joint
      spring cannot hold.
    - Flexion beyond the resting posture was not measured, so the fit does not hold
      there (p. 071009-4).
    - The measured moment includes the passive opponens pollicis and APL. The
      multi-joint thumb muscles were kept slack by posture (p. 071009-4), so the
      residual has to be taken at that posture.
  - Kamper DG, Hornby TG, Rymer WZ (2002). Extrinsic flexor muscles generate concurrent
    flexion of all three finger joints. J Biomech 35(12):1581-1589.
    doi:10.1016/s0021-9290(02)00229-4.
  - Esteki A, Mansour JM (1996). An experimentally based nonlinear viscoelastic model of
    joint passive moment. J Biomech 29(4):443-450. doi:10.1016/0021-9290(95)00081-x.
    Index MCP of 3 subjects. The hysteresis loop barely depends on speed: the
    dissipative part scales with joint speed to the power n = 0.12-0.25 (Table 3,
    p. 448), and no significant friction was found. Neither a linear tendon damper nor
    MuJoCo's joint damping reproduces this.
- **Finger models with ligaments.** Sancho-Bru JL, Pérez-González A, Vergara-Monedero M,
  Giurintano D (2001). A 3-D dynamic model of human finger for studying free movements.
  J Biomech 34(11):1491-1500. doi:10.1016/s0021-9290(01)00106-3. Includes the passive
  action of the MCP ligaments, which is needed to reproduce ab/adduction.
- **Hand models without ligaments.** Mirakhorlo M, Van Beek N, Wesseling M, Maas H,
  Veeger HEJ, Jonkers I (2018). A musculoskeletal model of the hand and wrist: model
  definition and evaluation. Comput Methods Biomech Biomed Engin 21(9):548-557.
  doi:10.1080/10255842.2018.1490952. An OpenSim hand and wrist with no ligaments or
  capsule; the authors name the absent carpal and ligamentous structures as one source
  of its wrist moment-arm errors (p. 554).

### Wrist and elbow

- **Wrist**
  - Berger RA, Imeada T, Berglund L, An KN (1999). Constraint and material properties of
    the subregions of the scapholunate interosseous ligament. J Hand Surg Am
    24(5):953-962. doi:10.1053/jhsu.1999.0953. Yield strength by region:
    - dorsal 260.3 ± 118.1 N;
    - palmar 117.9 ± 21.3 N;
    - proximal 62.7 ± 32.2 N.
  - Nowalk MD, Logan SE (1991). Distinguishing biomechanical properties of intrinsic and
    extrinsic human wrist ligaments. J Biomech Eng 113(1):85-93. doi:10.1115/1.2894089.
  - Majors BJ, Wayne JS (2011). Development and validation of a computational model for
    investigation of wrist biomechanics. Ann Biomed Eng 39(11):2807-2815.
    doi:10.1007/s10439-011-0361-y.
    - A rigid-body wrist: 15 bones free in six DOFs, articular contact from CT surfaces,
      46 ligaments as 75 straight linear tension-only springs (p. 2809).
    - Carpal ligament stiffness from published tests or earlier models, 40-220 N/mm per
      ligament (Table 1, p. 2809). Every carpometacarpal and intermetacarpal ligament was
      assumed to be 100 N/mm.
    - No in-situ strain (a uniform 2 % deformed the carpus) and no toe region, which the
      authors note overestimates the ligaments at low strain (p. 2814).
  - Gislason MK et al. (2009). Proc Inst Mech Eng H 223(7):849-861.
    doi:10.1243/09544119jeim527. A finite-element wrist with ligaments.
- **Elbow**
  - Regan WD, Korinek SL, Morrey BF, An KN (1991). Biomechanical study of ligaments
    around the elbow joint. Clin Orthop Relat Res (271):170-179.
    doi:10.1097/00003086-199110000-00023.
    - The anterior MCL and the radial collateral are taut through most of the flexion
      arc.
    - The posterior MCL is taut only in flexion.
    - The anterior MCL is the strongest and stiffest, failing at 260 N on average.
    - Table 1 (p. 175; mean, SD), anterior MCL / posterior MCL / radial collateral:
      stiffness 1528 (478) / 861 (240) / 924 (470) N per unit strain, resting length
      21.1 / 16.5 / 19.9 mm, which gives about 72 / 52 / 46 N/mm; modulus 118 (37) /
      97 (27) / 54 (28) MPa; cross-section 12.9 / 8.9 / 17.0 mm²; n = 6-7.
    - The 54-118 MPa moduli bracket the transfer modulus of about 80 MPa used for the
      hand ligaments above.
  - Spratley EM, Wayne JS (2011). Computational model of the human elbow and forearm:
    application to complex varus instability. Ann Biomed Eng 39(3):1084-1091.
    doi:10.1007/s10439-010-0224-y.
    - Ligaments as straight, linear tension-only springs, most in fan-shaped arrays
      (p. 1085); no toe region, no wrapping (p. 1090).
    - Collateral stiffness and tension-free flexion angles taken from Regan et al. 1991
      (Table 1, p. 1086): anterior MCL bundle 72.3 N/mm (tension-free at 22 deg),
      posterior MCL fibres 26.1 N/mm (80 and 110 deg), radial collateral fibres
      15.5 N/mm (30-85 deg), lateral ulnar collateral 57.0 N/mm (107 deg), annular
      ligament 28.5 N/mm at 2 % in-situ strain.
  - Rahman M, Cil A, Stylianou AP (2018). Medial collateral ligament deficiency of the
    elbow joint: a computational approach. Bioengineering 5(4):84.
    doi:10.3390/bioengineering5040084.
    - Blankevoort-type ligaments with ε_l = 0.03 and a parallel damper of 0.5 N s/mm.
    - The lateral ulnar collateral and annular ligaments wrap over bone.

### Knee

- **Ligament models in musculoskeletal simulation**
  - Wismans J, Veldpaus F, Janssen J, Huson A, Struben P (1980). A three-dimensional
    mathematical model of the knee-joint. J Biomech 13(8):677-685.
    doi:10.1016/0021-9290(80)90354-1.
    - Ligaments and capsule as seven tension-only springs with a purely quadratic law,
      f = k (L - L0)² for L > L0 (Eq. 12), k = 10-35 N/mm² from Trent et al. 1976
      (mean of six specimens), and an initial strain in extension of -3 to +5 % that
      sets L0 (Eq. 13, Table 1). The linear branch came later, with Blankevoort et al.
      1991.
    - Doubling k changed the anterior-posterior laxity little; small changes in the
      initial strains changed it a lot (sensitivity section).
  - Blankevoort L, Kuiper JH, Huiskes R, Grootenboer HJ (1991). Articular contact in a
    three-dimensional model of the knee. J Biomech 24(11):1019-1031.
    doi:10.1016/0021-9290(91)90019-j. Ligaments as non-linear elastic line elements:
    - force law, Eqs. 6a-c (p. 1021): f = k ε² / (4 ε_l) for 0 ≤ ε ≤ 2 ε_l,
      f = k (ε - ε_l) for ε > 2 ε_l, f = 0 for ε < 0, with ε = (L - L0) / L0 (Eq. 7)
      and k the stiffness in N (newtons per unit strain);
    - ε_l = 0.03 for all ligaments, after Butler et al. 1986 (p. 1021);
    - slack length from a reference strain ε_r at the reference (extended) posture:
      L0 = L_r / (ε_r + 1) (Eqs. 8a-b);
    - k per line element, the ligament's stiffness divided equally over its elements:
      ACL bundles 5000 N, PCL 9000 N, LCL 2000 N, MCL 2750 N, medial capsule 1000 N
      (Table 1, p. 1022);
    - reference strains from -0.25 to +0.10 (Table 1), not measured but tuned so that
      the model reproduced the specimen's measured internal-external rotation under
      ±3 N m (p. 1022);
    - Appendix B (p. 1030): k as elastic modulus × cross-sectional area, with
      E ≈ 345 MPa from fascicle-bone units of three donors aged 20-30 (Butler et al. 1986);
      Table 2 compares the resulting k with earlier knee models.
  - Blankevoort L, Huiskes R (1991). Ligament-bone interaction in a three-dimensional
    model of the knee. J Biomech Eng 113(3):263-269. doi:10.1115/1.2894883. Wrapping
    the MCL over the tibial edge changes how it resists valgus moments: valgus rotation
    under 24 N m fell by 0.9-1.7 deg (Fig. 6). Its effect on internal/external rotation
    laxity is negligible (abstract).
  - Shelburne KB, Pandy MG (1997). A musculoskeletal model of the knee for evaluating
    ligament forces during isometric contractions. J Biomech 30(2):163-176.
    doi:10.1016/s0021-9290(96)00119-4.
    - A sagittal-plane knee with 11 straight-line bundles (ACL, PCL, MCL, LCL,
      posterior capsule) and the Blankevoort-Huiskes force law (p. 165).
    - Bundle stiffness 1500-4500 N (per unit strain), started from Blankevoort and
      Huiskes 1991 and tuned to cadaver anterior-posterior drawer stiffness; reference
      strains -0.21 to +0.06 (Table B1, p. 176).
    - In maximal isometric extension the resultant ACL force reaches about 500 N at
      20 deg of flexion (p. 167).
  - Shelburne KB, Pandy MG, Anderson FC, Torry MR (2004). Pattern of anterior cruciate
    ligament force in normal walking. J Biomech 37(6):797-805.
    doi:10.1016/j.jbiomech.2003.10.010. Peak ACL force 303 N.
- **Lenhart et al. 2015.** Lenhart RL, Kaiser J, Smith CR, Thelen DG. Prediction and
  validation of load-dependent behavior of the tibiofemoral and patellofemoral joints
  during movement. Ann Biomed Eng 43(11):2675-2685. doi:10.1007/s10439-015-1326-3. A
  12-DOF OpenSim knee with 14 ligament bundles of 76 strands in all (Fig. 1, p. 2677):
  - non-linear toe region below 6 % strain, linear above (p. 2677);
  - linear stiffness = cross-sectional area × 125 MPa, divided equally over a bundle's
    strands;
  - reference strains in an extended posture taken from the literature; the per-bundle
    values are in the paper's supplementary material;
  - Monte Carlo spread (n = 2000): SD 30 % in stiffness and 0.02 in reference strain
    (p. 2678);
  - wrap objects for the MCL, the posterior capsule and the patellar tendon (p. 2677).
- **Smith et al. 2016 (two papers)**
  - Smith CR, Lenhart RL, Kaiser J, Vignos MF, Thelen DG (2016). Influence of ligament
    properties on tibiofemoral mechanics in walking. J Knee Surg 29(2):99-106.
    doi:10.1055/s-0035-1558858.
    - l0 = l_ref / (1 + ε_ref).
    - The MCL and LCL reference strains mattered more than stiffness for tibial
      rotation.
  - Smith CR, Vignos MF, Lenhart RL, Kaiser J, Thelen DG (2016). The influence of
    component alignment and ligament properties on tibiofemoral contact forces in total
    knee replacement. J Biomech Eng 138(2):021017. doi:10.1115/1.4032464.
    - Same framework (quadratic toe, linear above; 125 MPa × area), 11 bundles of 1-20
      strands plus the patellar tendon, in an 83-year-old subject with a knee
      replacement (pp. 021017-1 to -2).
    - Table 1 (p. 021017-2): stiffness 800-5700 N per unit strain per bundle; reference
      strains in the relaxed extended posture from -0.06 (posteromedial PCL) to +0.08
      (posterior capsule); MCL 0.03, LCL 0.06. Monte Carlo SD 30 % and 0.02, as
      Lenhart et al.
- **Other knee models**
  - Marra MA, Vanheule V, Fluit R, Koopman BHFJM, Rasmussen J, Verdonschot N, Andersen
    MS (2015). A subject-specific musculoskeletal modeling framework to predict in vivo
    mechanics of total knee arthroplasty. J Biomech Eng 137(2):020904.
    doi:10.1115/1.4029258.
    - AnyBody force-dependent kinematics: an 11-DOF knee with 17 ligament bundles that
      wrap geometric shapes (Fig. 3).
    - The Blankevoort law with ε_l = 0.03 and L0 = L_r / (ε_r + 1) at upright standing
      (Eqs. 2-3, p. 020904-5).
    - Table 1 (p. 020904-5): stiffness 1000-6000 N per unit strain; reference strains
      from -0.25 (anterior LCL) to +0.12 (superior medial patellofemoral ligament).
  - Bloemker KH, Guess TM, Maletsky L, Dodd K (2012). Computational knee ligament
    modeling using experimentally determined zero-load lengths. Open Biomed Eng J
    6:33-41. doi:10.2174/1874230001206010033.
    - Stiffness per bundle (N per unit strain): ACL 5000, PCL 9000, LCL 2000, MCL 2750,
      the per-element values of Blankevoort et al. 1991, Table 1.
    - Kinematics are highly sensitive to zero-load length.
  - Kia M et al. (2016). J Biomech Eng 138(5):051010. doi:10.1115/1.4032850.
  - Guess TM, Razu S, Jahandar H (2016). J Knee Surg 29(2):126-137.
    doi:10.1055/s-0036-1571954.
- **Tissue properties**
  - Woo SL-Y, Hollis JM, Adams DJ, Lyon RM, Takai S (1991). Tensile properties of the
    human femur-anterior cruciate ligament-tibia complex: the effects of specimen age
    and orientation. Am J Sports Med 19(3):217-225. doi:10.1177/036354659101900303.
    - 27 pairs of knees in three age groups (22-35, 40-50, 60-97 years), tested at 30 deg
      of flexion; one knee of each pair loaded along the ACL axis (anatomical), the other
      along the tibial axis (p. 218).
    - Linear stiffness (slope between 3 and 7 mm of elongation, p. 220), anatomical
      orientation: 242 ± 28, 220 ± 24 and 180 ± 25 N/mm; ultimate load 2160 ± 157,
      1503 ± 83 and 658 ± 129 N for the young, middle and older groups (mean ± SEM;
      Table 3, pp. 221-222).
  - Noyes FR, Grood ES (1976). The strength of the anterior cruciate ligament in humans
    and Rhesus monkeys. J Bone Joint Surg Am 58(8):1074-1082.
    doi:10.2106/00004623-197658080-00006. Bone-ACL-bone units at a fast strain rate:
    donors 16-26 years (n = 6) 182 ± 56 N/mm stiffness, 1730 ± 660 N maximum force,
    111 ± 26 MPa modulus; donors 48-86 years (n = 20) 129 ± 39 N/mm, 734 ± 266 N,
    65.3 ± 24.0 MPa (Tables II-III, pp. 1077-1078).
  - Trent PS, Walker PS, Wolf B (1976). Ligament length patterns, strength, and
    rotational axes of the knee joint. Clin Orthop Relat Res (117):263-270.
    doi:10.1097/00003086-197606000-00034. The source of Wismans's stiffness values;
    Woo et al. 1991 (p. 217) quote its ACL stiffness as 141 N/mm and its ultimate load
    as 633 N (donors 25-55 years).
  - Butler DL, Kay MD, Stouffer DC (1986). Comparison of material properties in
    fascicle-bone units from human patellar tendon and knee ligaments. J Biomech
    19(6):425-432. doi:10.1016/0021-9290(86)90019-9.
    - Fascicle-bone units from three donors aged 21-30 (Table 1, p. 426), at a strain
      rate of 100 %/s (p. 429).
    - Ligament (ACL, PCL, LCL) modulus 315 ± 22 MPa across donors (mean ± SEM,
      n = 23 units; per-donor means 278-447 MPa); patellar tendon 643 ± 53 MPa; strain
      at maximum stress 15 % for the ligaments (Table 3, p. 431).
  - Chandrashekar N, Mansouri H, Slauterbeck J, Hashemi J (2006). J Biomech
    39(16):2943-2950. doi:10.1016/j.jbiomech.2005.10.031.

### Shoulder

- **Tissue properties**
  - Bigliani LU, Pollock RG, Soslowsky LJ, Flatow EL, Pawluk RJ, Mow VC (1992). Tensile
    properties of the inferior glenohumeral ligament. J Orthop Res 10(2):187-197.
    doi:10.1002/jor.1100100205.
    - 16 shoulders, 48 bone-ligament-bone specimens (superior band, anterior and
      posterior pouch), pulled at 0.004-0.04 mm/s (abstract; p. 191).
    - Mean ± SD over all regions (Table 1, p. 190): thickness 2.28 ± 0.66 mm, width
      12.3 ± 2.9 mm, length 40.7 ± 4.7 mm; failure stress 5.5 ± 2.2 MPa; strain to
      failure 27.0 ± 8.9 % for the whole specimen and 10.9 ± 5.5 % for the midsubstance,
      so most of the strain occurs near the insertions.
    - Modulus 36.9 ± 14.6 MPa (anterior pouch 30.3, posterior pouch 41.9; Table 2,
      p. 192). Modulus × mean area gives about 1 kN per unit strain for the whole
      ligament.
  - Boardman ND 3rd, Debski RE, Warner JJ, Taskiran E, Maddox L, Imhoff AB, Fu FH,
    Woo SL (1996). Tensile properties of the superior glenohumeral and coracohumeral
    ligaments. J Shoulder Elbow Surg 5(4):249-254. doi:10.1016/s1058-2746(96)80050-4.
    - 10 of each ligament, donors 55-76 years (p. 250).
    - Stiffness: coracohumeral 36.7 ± 5.9 N/mm, superior glenohumeral 17.4 ± 1.5 N/mm;
      ultimate load 359.8 ± 40.3 N and 101.9 ± 11.5 N; mid-portion area 53.7 ± 3.2 and
      11.3 ± 1.6 mm² (abstract; Table 1, p. 252).
  - Voycheck CA et al. (2010). J Appl Physiol 108(6):1711-1718.
    doi:10.1152/japplphysiol.01175.2009.
- **Capsule as a sheet vs strands**
  - Moore SM, Ellis B, Weiss JA, McMahon PJ, Debski RE (2010). The glenohumeral capsule
    should be evaluated as a sheet of fibrous tissue: a validated finite element model.
    Ann Biomed Eng 38(1):66-76. doi:10.1007/s10439-009-9834-7. Strain error was below
    5 % for the continuous capsule and 20 % for a discrete-ligament model. This is the
    main caveat against representing the capsule with a few strands.
  - Ellis BJ et al. (2007). J Biomech 40(3):603-612.
    doi:10.1016/j.jbiomech.2006.01.024.
- **Musculoskeletal models with a capsule**
  - Menze J, Croci E, Andersen MS et al. (2025). Advancing musculoskeletal shoulder
    modeling: reflecting glenohumeral translation with bony, ligamentous, and muscular
    stability constraints. Front Bioeng Biotechnol 13:1441530.
    doi:10.3389/fbioe.2025.1441530.
    - AnyBody model with SGHL, MGHL, IGHL and CHL modelled as 2, 2, 8 and 5 strands
      wrapped on a sphere at the humeral head, with the force law of Marra et al. 2015
      (section 2.2.3).
    - Stiffness 320-1099 N per unit strain (IGHL posterior, inferior and anterior 320,
      388 and 582; MGHL 375; SGHL 550; CHL 1099), based on Bigliani et al. 1992 and
      Boardman et al. 1996 among others (Table 1).
    - Slack lengths calibrated per subject from reference strains of 5-11 % at stated
      glenohumeral positions, e.g. MGHL 7 % and CHL 5 % at 0 deg abduction, the IGHL
      regions 10-11 % at 60 deg abduction with rotation (Table 1).
  - Dalman MJ, Saul KR (2026). Assessing sensitivity of predictive shoulder simulations
    to uncertain soft tissue properties. J Biomech 195:113114.
    doi:10.1016/j.jbiomech.2025.113114.
    - OpenSim, Blankevoort ligaments, stiffness 600-1500 N per unit strain.
    - Slack lengths were chosen so that the ligaments engage at 30 deg abduction.
    - Slack length was the most influential parameter.

### Ligament mechanics and precedent

- **Reviews**
  - Weiss JA, Gardiner JC (2001). Computational modeling of ligament mechanics. Crit Rev
    Biomed Eng 29(3):303-371. doi:10.1615/critrevbiomedeng.v29.i3.20. From its review
    of one-dimensional ligament models (section V.A):
    - quadratic and linear force-elongation laws with a zero-force reference length are
      the norm, the quadratic-then-linear form being Blankevoort's;
    - in Mommersteeg et al.'s comparison, three or fewer line elements per knee
      ligament made the model very sensitive to the geometry, while seven or more were
      redundant;
    - reference strains optimised to reproduce measured kinematics, as Blankevoort et
      al. did (-25 % to +10 %), are often physiologically unrealistic and not unique.
  - Butler DL, Grood ES, Noyes FR, Zernicke RF (1978). Biomechanics of ligaments and
    tendons. Exerc Sport Sci Rev 6:125-181. doi:10.1249/00003677-197800060-00005.
- **OpenSim.**
  - `Blankevoort1991Ligament` is the modern ligament element:
    - quadratic toe region up to a transition strain (default 0.06), linear above;
    - damping c·ε̇, applied only while stretching (default c = 0.003 per unit strain per
      second).
  - The older `Ligament` class uses a normalised spline scaled by a force constant.
- **Notation trap.** Blankevoort et al. 1991 (Eqs. 6a-c) and the papers that follow them
  write the parameter as ε_l = 0.03, with the toe region ending at 2ε_l. That is the same
  curve as OpenSim's transition strain of 0.06: k ε² / (4 ε_l) = k ε² / (2 ε_t) and
  k (ε - ε_l) = k (ε - ε_t / 2) with ε_t = 2 ε_l.
- **Spine tissue data.**
  - Chazal J, Tanguy A, Bourges M, Gaurel G, Escande G, Guillot M, Vanneuville G (1985).
    Biomechanical properties of spinal ligaments and a histological study of the
    supraspinal ligament in traction. J Biomech 18(3):167-176.
    doi:10.1016/0021-9290(85)90202-7.
    - 43 whole ligaments from 12 fresh cadavers (30-80 years) and surgery, pulled at
      1 mm/min between bone clamps (p. 167).
    - Sigmoid curves (intertransverse ligament excepted): the toe ends at strains of
      0.06-0.12 (point A), the linear part at 0.16-0.44 (point B), failure at 0.21-0.57
      (point C). Failure stress: posterior longitudinal
      20.8, flavum 15.2, anterior longitudinal 10.6, supra- and interspinous 8.7 MPa
      (Table 2, p. 170). Failure loads 437 N (anterior) and 323 N (posterior
      longitudinal; p. 170).
  - Pintar FA, Yoganandan N, Myers T, Elhagediab A, Sances A Jr (1992). Biomechanical
    properties of human lumbar spine ligaments. J Biomech 25(11):1351-1356.
    doi:10.1016/0021-9290(92)90290-H.
    - 132 samples from 38 cadavers, T12-S1 (abstract).
    - Linear-region stiffness by ligament type, mean ± SD (Table 3, p. 1353): anterior
      longitudinal 33.0 ± 15.7, posterior longitudinal 20.4 ± 11.9, capsules
      33.9 ± 10.7, flavum 27.2 ± 9.2, interspinous 11.5 ± 6.6, supraspinous
      23.7 ± 10.9 N/mm. Failure strain from 11-16 % (posterior longitudinal) to
      71-115 % (supraspinous) (p. 1352).
  - Christophy M, Faruk Senan NA, Lotz JC, O'Reilly OM (2012). A musculoskeletal model
    for the lumbar spine. Biomech Model Mechanobiol 11:19-34.
    doi:10.1007/s10237-011-0290-6. Open access. Its lumbar joints follow constraint
    functions, and ligaments and facets are not modelled; the authors propose a joint
    stiffness matrix for them (pp. 30-31).
- **Spine precedent.** Meszaros-Beller L, Hammer M, Schmitt S, Pivonka P (2023). Effect
  of neglecting passive spinal structures. Front Physiol 14:1135531.
  doi:10.3389/fphys.2023.1135531.
  - An OpenSim model with explicit spinal ligaments and discs.
  - Adding them reduced the compressive load and anterior torque attributed to net muscle
    forces.
- **No MuJoCo precedent.** We found no published MuJoCo musculoskeletal model with
  anatomical ligaments. The MyoBack model is the nearest relative: it uses passive spring
  tendons for its exoskeleton bands. Walia R et al. (2025), IEEE ICORR 2025:128-135,
  doi:10.1109/icorr66766.2025.11063132.

### Gaps in the data

- No tensile or stiffness study of the finger (digits 2-5) MCP collaterals was found.
- For the PIP there are failure loads (collateral 162.5 N in distraction, about 177 N in
  lateral bending; volar plate about 220-390 N in hyperextension) but no stiffness.
- For the thumb CMC, stiffness is measured for two of the seven ligaments only: the
  dorsoradial (89 ± 21 N/mm) and anterior oblique (65 ± 30 N/mm) ligaments, from donors
  with a mean age of 74 (D'Agostino et al. 2014). The isolated-ligament failure loads of
  Najima et al. 1997 are a few newtons from very old donors.
- No in-vivo length or strain measurement of the thumb CMC ligaments was found.
- The CMC2-5 ligaments have no data of their own; a published wrist model assumes
  100 N/mm for all of them (Majors and Wayne 2011).

Finger ligament stiffness would therefore be transferred: either from cross-sectional
area times a modulus, as Lenhart et al. do, or from related ligaments. Those rows would
carry the status `transferred`. The thumb CMC dorsoradial and anterior oblique rows can
carry D'Agostino's measured values; the dorsal central ligament is part of D'Agostino's
dorsoradial sheet and shares its value.

An example of the area route for the thumb CMC: width × thickness from Ladd et al. 2012
(Table I) gives about 1.7 mm² (intermetacarpal) to 10.2 mm² (posterior oblique).
- With the knee-ligament modulus of 345 MPa that Blankevoort et al. 1991 take from
  Butler et al. 1986 (Appendix B; Butler's own cross-donor ligament mean is 315 MPa), k
  is about 0.6-3.5 kN per unit strain, or k / L of about 6·10⁴ to 3.6·10⁵ N/m at Ladd's
  lengths.
- D'Agostino's measured thumb CMC stiffness implies an effective modulus of only about
  65-100 MPa (section 2, thumb CMC mechanics), close to the 65 MPa of older ACLs (Noyes
  and Grood 1976) and below Lenhart's 125 MPa. With it, k is about 0.1-1.0 kN per unit
  strain, or k / L of about 1·10⁴ to 1·10⁵ N/m, which brackets the measured
  6.5·10⁴-8.9·10⁴ N/m.
- **Proposal:** transfer hand ligament stiffness with D'Agostino's effective modulus
  (about 80 MPa, from donors of the same age as the hand-anatomy series), and keep
  345 MPa as the upper sensitivity bound. The rectangle overestimates the area of a
  tapering ligament, which adds to the margin.

## 3. MuJoCo mechanics

Installed versions: MuJoCo 3.11.0 and mujoco-warp 3.11.0. The latest MuJoCo release is
3.17.0.

**Passive spatial tendon.** A `<spatial>` tendon through two or more sites, optionally
wrapping a sphere or cylinder geom with a sidesite. No actuator references it. MuJoCo
wraps only spheres and cylinders, so cam-shaped heads must be approximated by those or by
extra strands.

**Tension-only spring.** `springlength="0 L0"` gives a dead band: zero force below the
slack length L0, a spring above it. The two-value form exists since MuJoCo 2.3.1.

**Stiffness and damping.**
- **Polynomial stiffness** (since 3.7.0): `stiffness="a b c"` gives
  f(x) = -(a x + b x² + c x³), where x is the stretch beyond L0. #139 already relies on
  this for its fixed tendons.
- **Polynomial damping**: f(v) = -(a v + b v|v| + c v³).
- **Damping is not gated by the dead band.** A tendon damper acts at any length, so a
  slack ligament with damping would add viscous drag across the whole range. OpenSim's
  ligament damps only while stretching.

**Tendon limit.** `limited="true" range="0 Lmax"` with `solreflimit` / `solimplimit`
(and `margin`) makes a one-sided stop.
- The constraint solver enforces it, softly, through the constraint impedance.
- The limit force appears in `efc_force`.
- It is not a force-strain law: its stiffness comes from `solref`, not from tissue data.

**Tendon armature** adds inertia along the tendon length. It can stabilise stiff tendons
but is non-physical for a ligament.

**Integrators.** All CPU models use Euler.
- Euler integrates only joint damping implicitly. Tendon stiffness and damping are
  explicit, so a stiff ligament is subject to the explicit stability limit.
- Linearised along the tendon, that limit is k < 4 / (h² · invweight), where
  `tendon_invweight0` is the inverse effective mass along the tendon.
- MuJoCo 3.13.0 added a `discrete` integrator that puts positive position stiffness into
  the effective inertia. It is not in the installed 3.11.0, and switching integrators
  would be a model-wide change.

**mujoco-warp.** Version 3.11.0 implements all of the above natively:
- the polynomial tendon spring and damper with the dead band (`passive.py`);
- sphere and cylinder wrapping (`smooth.py`);
- tendon limit constraints (`constraint.py`).

Its Euler step, like the CPU one, makes only `dof_damping` implicit.

## 4. Proposed model changes

### Which ligaments first

**Pilot: the hand.** The pilot covers the MCP and PIP collaterals of digits 2-5 and the
thumb CMC.

- **MCP.** Per collateral, a dorsal and a volar strand of the proper collateral (Minami
  1984), plus the accessory collateral. This is 3 strands per side, 6 per MCP.
- **PIP.** Per side, a proper and an accessory collateral, 4 per PIP.
- **Thumb CMC.** The seven ligaments that Ladd et al. 2012 found in every specimen:
  - dorsoradial;
  - dorsal central;
  - posterior oblique;
  - anterior oblique (thin and capsular, so a low-stiffness strand);
  - ulnar collateral (first volar trapeziometacarpal);
  - first dorsal trapeziometacarpal;
  - intermetacarpal (second metacarpal base to the volar-ulnar first metacarpal; double
    in 7 of 32 hands in Najima et al. 1997).

That is 24 + 16 + 7 = 47 strands per hand.

**Why these joints.**
- The MCP and the thumb CMC are the two-DOF joints where coupling matters and where #139
  either has a gap (MCP abduction) or had to drop cross terms (CMC).
- The CMC4/5 joints being added now are the natural next members.
- The PIP is included because, if the PIP lateral-deviation DOF is added, it is the
  collaterals that restrain it. While PIP stays a hinge, its collaterals add nothing
  the residual spring does not.

**Later candidates.**
- **Knee.** Today it is one DOF with polynomial couplings of the translations and
  rotations, so ligaments would only add flexion moment. They pay off once the coupled
  DOFs are freed and articular contact defines the kinematics, as in Lenhart et al.
  2015. That is a separate, larger change.
- **Glenohumeral capsule.** Combined elevation-rotation end range. Moore et al. 2010 is
  the caveat on strands.
- **Elbow collaterals.** Only with a varus-valgus DOF. Spratley and Wayne 2011 give a
  per-fibre stiffness and tension-free angle set to start from.
- **Lumbar spine.** Only with intervertebral DOFs. Per-ligament stiffness and failure
  strains exist (Pintar et al. 1992; Chazal et al. 1985), while Christophy et al. 2012's
  lumbar model leaves the ligaments out.
- **DIP joints and the thumb MP/IP.** These are hinges and stay with the residual
  springs.

### XML roles

The four file roles are unchanged.
- **`*_chain.xml`** holds the ligament attachment sites on the bones they belong to
  (`lig_<joint>_<strand>_o_r` / `_i_r`) and any ligament wrap geoms with their
  sidesites.
- **`*_tendon.xml`** holds the ligament `<spatial>` tendons, inside a generated block:

  ```xml
  <!-- BEGIN generated by scripts/fit_ligaments.py; do not edit by hand -->
  <spatial name="lig_mcp2_rcl_dorsal_r" class="myoarm_ligament"
           springlength="0 0.0152" stiffness="0 2.1e5 0">
      <site site="lig_mcp2_rcl_dorsal_o_r"/>
      <geom geom="lig_mcp2_head_wrap_r" sidesite="lig_mcp2_head_side_r"/>
      <site site="lig_mcp2_rcl_dorsal_i_r"/>
  </spatial>
  <!-- END generated by scripts/fit_ligaments.py -->
  ```

  The numbers are placeholders.
- **`*_assets.xml`** gets a `myoarm_ligament` default class for group and colour.

**Site positions are hand-edited.** They come from landmarking on our own bone meshes,
guided by the published descriptions, so they sit outside the generated block. Their
provenance is the landmarking record (which description, which mesh feature) in
`myo_sim/build/data/ligaments.json`.

**Policy text.** The policy (model-authoring.md in #140) currently maps "passive joint
moment or ligament" to a one-joint fixed tendon. It would gain a row for explicit
ligaments as passive spatial tendons. The role table's `*_tendon.xml` entry would gain
"passive spatial ligaments".

### Force law and parameters

Strain is ε = (L - L0) / L0. The force law is the Blankevoort curve (Blankevoort et al.
1991, Eqs. 6a-c; Wismans et al. 1980 used its quadratic part alone) used by OpenSim's
`Blankevoort1991Ligament`, Lenhart et al. 2015 and Marra et al. 2015, with
stiffness k in newtons per unit strain and transition strain ε_t (0.06 by default, i.e.
Blankevoort's ε_l = 0.03):

- F = 0 for ε ≤ 0;
- F = k ε² / (2 ε_t) for 0 < ε ≤ ε_t;
- F = k (ε - ε_t / 2) for ε > ε_t.

**Mapping to MuJoCo.** Let x = L - L0.

- The toe region is the quadratic term b = k / (2 ε_t L0²) on a tendon with
  `springlength="0 L0"`.
- The linear branch is exact if a second tendon on the same path carries -b from
  `springlength="0 L0(1+ε_t)"`. Beyond the transition the sum is k (x/L0 - ε_t/2), the
  linear branch.
- That doubles the path cost. The cheaper default is therefore one tendon with (a, b)
  fitted to the curve over the strain range the strand actually reaches in the joint's
  range. The exact pair is used only for strands that go well past ε_t.

**Parameters per strand**, one JSON row each:
- the attachment sites and wrap object;
- k: measured bone-ligament-bone stiffness where it exists, otherwise cross-sectional
  area × modulus, status `transferred`;
- the reference strain ε_ref at a stated reference posture, which sets the slack length
  L0 = L_ref / (1 + ε_ref);
- ε_t;
- the citation;
- the status.

Slack length dominates the result in every knee and shoulder study above, so it gets the
most scrutiny.

**No damping** on ligament tendons, because MuJoCo's tendon damper is not gated by the
dead band. Viscous behaviour stays in the joint damping (#139, from Kamper et al. 2002).
The measured whole-joint dissipation is nearly rate-independent anyway (Esteki and
Mansour 1996: speed to the power 0.12-0.25), which no MuJoCo damper reproduces.

**Springs, not tendon limits.** Limits remain an option for a strand that must act as a
hard stop. Their force is set by `solref` rather than tissue data, and the prototype
shows they cost more (below).

### Fitting and refitting the residual springs

`scripts/fit_ligaments.py` runs first:

1. It reads `build/data/ligaments.json`.
2. It sweeps each strand over its joints' ranges.
3. It sets L0 from the reference strain.
4. It checks the length-angle curves against the cadaver data (validation, below).
5. It writes the generated block.

It does not fit to joint-moment curves. That is deliberate: the ligament must stay an
anatomical prediction, not a second curve fit.

`scripts/fit_passive_moments.py` (#139) then subtracts the ligament moment along with the
muscle moment, at the source posture:

`spring target = source curve - muscle passive moment (act = 0) - ligament moment`

The ligament moment at a joint is Σ J_tendonᵀ F_ligament.

**A ligament never exceeds the residual it replaces.** If its moment exceeds the
residual anywhere in the measured span, in the restoring direction:
1. its reference strain is lowered, which means more slack, within the published spread
   (Lenhart et al. use 0.02 SD);
2. if that is not enough, the row is marked and the strand is left out, with the reason
   in the row.

A side whose residual becomes zero or negative gets no spring, as #139 already handles.
Both scripts are rerun whenever muscles, joint axes or ligaments crossing the joint
change.

**The muscle model is a refit trigger too.** The residual is taken over the muscles'
passive moment, so changing the muscle model changes it even with every path and
parameter kept: compliant tendons (the tendon stretch moves each fibre's operating point),
different passive force-length curves, or fibres that add damping of their own. After such
a change the ligament reference strains are checked again against the new residual
(steps 1-2 above) and the residual springs are refitted.

**One element per tissue.** Ligaments, residual springs and any additional joint-limit
stop forces (for example coordinate-limit springs near the range ends, as some OpenSim
models carry) must not be stacked for the same tissue. Each passive structure is counted
by exactly one element: where a ligament models a collateral or a capsule region, neither
a residual spring nor a stop force may also represent it, and a stop force added later is
subtracted from the residual like a ligament. MuJoCo's own joint range limits stay the
hard ends of the range, not a tissue model.

### Mirroring and hand pruning

**Mirroring.** Everything is authored on the right side with `_r` names. The left side
comes from `build_mirrored_child_xml()` and the `MirrorRules`, as for muscle paths. Sites,
wrap geoms and spatial tendons already mirror today.

**Hand pruning.** `build/hand.py` deletes every tendon not referenced by an actuator, and
removes group-3 wrap geoms that no remaining tendon uses. #139 had to exempt its fixed
tendons from this. The pruning would also have to keep tendons whose name starts with
`lig_`, together with their sites and wraps.

**Symmetry tests.** These are extended to the ligament tendons (lengths at mirrored
postures).

## 5. Validation

1. **Length vs joint angle.** Per strand, against the cadaver data:
   - MCP collateral portions vs Minami et al. 1984 (dorsal +3 to 4 mm, middle +0.4 to
     1.0 mm, volar -1 to -2 mm from 0 to 80 deg; dorsal -2 to -3 mm in hyperextension;
     the dorsal length plateau from 50 to 70 deg). The middle-portion target applies if a
     third proper-collateral strand is used. These are apparent lengths, so they only
     bound a strand while it is taut;
   - thumb MCP UCL elongation vs Sircar et al. 2024 (if the thumb MP is added later):
     dorsal proper portion at most 111 % (flat heads, 40 deg) to 117 % (round heads,
     60 deg) of its neutral length;
   - near-isometric PIP proper collaterals per Allison 2005.

   The tolerances are stated in the JSON row.
2. **Total passive moment.** Muscles + ligaments + residual springs reproduce the source
   curves at least as well as #139's table (RMS per row). A test asserts that the ligament
   moment never exceeds the residual inside each measured span.
3. **End-feel and laxity.**
   - Moment-angle sweeps at the end of range in each DOF.
   - MCP abduction laxity at 0 vs 75 deg flexion; the abduction range should shrink
     with flexion, mostly between 0 and 45 deg (Minami et al. 1985, p. 258).
   - MCP ab/adduction restraint shares at 0 and 75 deg flexion: accessory collaterals
     about a third in extension and under 10 % in flexion, the rest from the proper
     collateral (Minami et al. 1985, Table IV).
   - CMC laxity in flexion-abduction combinations.
   - No new posture where the passive moment changes sign unexpectedly.

   Translational laxity data (e.g. Colman et al. 2007) do not apply while the CMC cannot
   translate.
4. **Ligament forces.** Where published, compare against them.
   - In the hand, the published values are failure loads (e.g. 162.5 N in distraction
     and about 177 N in lateral bending for the PIP collateral, Rhee et al. 1992), and
     they serve as upper bounds in the grip and pinch tests.
   - Knee, elbow and shoulder forces become relevant with those joints.
5. **CPU vs mujoco-warp.** Compare ligament lengths, `qfrc_passive` and `qacc` at random
   postures. The prototype already does this (section 6).
6. **Hand function.** The McFarland et al. 2023 tenodesis, grip and lateral pinch tests
   (`scripts/validate_hand_function.py`, `docs/wiki/hand-function-validation.md` on the
   hand-function validation branch) still pass. Tenodesis is the most sensitive, since
   it depends directly on the MCP passive moment.
7. **Stability.** A random-activation stress test at the model timestep, with each
   strand's stiffness at its upper published bound, plus the analytic margin
   4 / (h² · invweight) per strand.
8. **Cost.** `mj_step` time and memory for the affected registered models against `dev`,
   in the PR (validation checklist item 5).

## 6. Cost: prototype

**What was built.** A throwaway MjSpec script, not committed, added passive spatial
ligaments to the composed `myohands`, `myoarms` and `myofullbody` from `dev` (9405437).
It used 62 strands per hand, 124 in total:
- 4 per joint at each MCP, PIP and DIP of digits 2-5;
- 4 each at the thumb MP and IP;
- 6 at the thumb CMC, including an intermetacarpal strand to the second metacarpal.

42 strands wrap: cylinders at the MCP, PIP and thumb MP, and a sphere at the CMC for the
three dorsal ligaments.

**Geometry and parameters.** Attachments were placed from each joint's anchor and axis
(lateral offset 4-7 mm, eccentric dorsal origins). The geometry is plausible, not
anatomical. Slack length was set at 85 % of each strand's length span over its joints'
ranges, with k = 5·10⁴ N/m. The full prototype is a larger set than the 47-strand pilot,
which would cost roughly 60-75 % of the increments below.

**Variants:**
- `lin`: linear tension-only spring.
- `lin_nowrap`: the same without wrap geoms.
- `blank`: the exact Blankevoort pair, i.e. two tendons per strand.
- `limit`: tendon length limits (range upper bound at 95 % of the span, solref 0.02 1)
  instead of springs.

**CPU method.** MuJoCo 3.11.0, single thread pinned to one core.
- Random sparse muscle activation, changed every 100 steps, with the same schedule for
  every variant.
- Per variant: 300 warm-up steps, then 12 blocks of 200 steps.
- Reported value: the median over blocks, then over 3 interleaved rounds.
- Round-to-round spread was below 2 %.

| Model | base (µs/step) | lin | lin_nowrap | blank | limit |
| --- | --- | --- | --- | --- | --- |
| myohands | 62.0 | 75.9 (+22 %) | 71.6 (+15 %) | 87.2 (+41 %) | 77.8 (+26 %) |
| myoarms | 127.9 | 151.4 (+18 %) | 147.6 (+15 %) | 173.7 (+36 %) | 163.5 (+28 %) |
| myofullbody | 336.7 | 369.8 (+10 %) | 366.8 (+9 %) | 406.8 (+21 %) | 394.8 (+17 %) |

**What the CPU numbers say.**
- **Per strand.** Each strand costs about 0.1 µs per step on CPU, and each wrap adds a
  further ~0.1 µs.
- **Limits.** Limits cost more than springs because active limits add constraint rows:
  about 22 strands at their limit on average, with mean `nefc` rising from 23 to 35 on
  `myohands`.
- **Memory.** Model memory grows by about 0.1 MB (`nbuffer` 75.5 → 75.6 MB for
  `myohands`).

**mujoco-warp.** Version 3.11.0 on an RTX 5090, 1024 worlds, graph-captured `step`, median
of 3 interleaved rounds. The GPU was shared with two training runs, so the
spread between rounds was up to ±15 %. Read these as relative numbers.

| Model | base (steps/s) | lin | blank | limit |
| --- | --- | --- | --- | --- |
| myohands | 232 k | 198 k (0.85×) | 170 k (0.73×) | 200 k (0.86×) |
| myofullbody | 64.9 k | 62.6 k (0.96×) | 55.1 k (0.85×) | 57.1 k (0.88×) |

**CPU vs warp parity.** Eight random hand postures, `mj_forward` vs `mjw.forward`:
- ligament lengths agree to 1.2·10⁻⁷ m;
- `qfrc_passive` agrees to 1.5·10⁻⁵ N m, against a peak of 0.46 N m;
- `qacc` agrees to a relative 4·10⁻³ or better, with contacts included.

The check also showed a pre-existing difference of 0.25 mm in the `EDC2_tendon` length
between CPU and warp on `dev`, without ligaments. It should be looked at separately.

**Stability at h = 2 ms** (`myohands`, Euler). Random activation drove the joints into
the ligaments; each run lasted 3 s over 2 seeds. A run counted as diverged on a QACC
warning or a finger speed above 100 rad/s.

| Case | analytic k_crit, min / median (N/m) | lin: stable up to | diverged at |
| --- | --- | --- | --- |
| `dev` (phalanx inertia floor 1e-4 kg m², armature 1e-4) | 2.0·10⁶ / 5.8·10⁷ | 1·10⁷ N/m | 3·10⁷ N/m |
| rod-like phalanx inertia, armature 1e-6 | 1.7·10⁵ / 6.5·10⁵ | 3·10⁵ N/m | 1·10⁶ N/m |

Published bone-ligament-bone stiffness runs from tens of N/mm for small ligaments
(superior glenohumeral 17.4 N/mm, coracohumeral 36.7 N/mm; thumb CMC anterior oblique
65 N/mm and dorsoradial 89 N/mm) to 242 N/mm for a young ACL, i.e. about 10⁴ to
2.4·10⁵ N/m. The area × 345 MPa estimate for the short thumb CMC ligaments (section 2, up
to 3.6·10⁵ N/m) exceeds that range; with the proposed effective modulus of about 80 MPa
they stay below about 1·10⁵ N/m, consistent with D'Agostino's measurements.

**What the stability numbers say.**
- **Current `dev` inertias.** Hand ligaments in that range are stable with a large
  margin.
- **Realistic phalanx inertia.** If the inertia floor and the 1e-4 armature are ever
  reduced to physical values (cf. issue #128), the margin for the stiffest hand strands
  falls to about 1-3×. The analytic bound 4 / (h² · invweight) predicts the empirical
  limit to within a factor of about 2. It would be a test on every strand. A strand at
  the top of the 345 MPa CMC estimate (3.6·10⁵ N/m) would sit at the empirical limit of
  the rod-like-inertia case (stable at 3·10⁵, diverged at 10⁶ N/m). The measured thumb
  CMC values (6.5·10⁴-8.9·10⁴ N/m) leave a margin of about 3× in that case.
- **Damping.** A tendon damper of 500 N s/m, the order used by Rahman et al. and Bloemker
  et al. (0.5 N s/mm), was stable on `dev`. It diverged with realistic inertia, because
  it is explicit. 50 N s/m was stable in both cases. This is another reason for no
  ligament damping.
- **Limits.** Tendon limits never diverged. With solref time constants of 0.004 / 0.01 /
  0.02 s, the peak overshoot past the range was:
  - 0.29 / 0.54 / 0.93 mm on `dev`;
  - 1.3 / 2.5 / 4.5 mm with realistic inertia.

  For 5-15 mm strands, that softness is large.

**One unrelated event.** In 23 random-activation runs of `myofullbody` per variant, one
run of `lin_nowrap` hit a QACC warning. The cause was a NaN in the length of the existing
`BIClong_tendon` path, not a ligament.

## 7. Open questions

1. **Reference model or variant.** The full hand set costs +10 to +22 % CPU step time on
   hand-containing models with single-tendon springs; the pilot costs less. Is that
   "substantially more expensive" under policy 4? Our suggestion:
   - ligaments go into the reference model if the pilot stays within roughly +15 % on
     `myohands`;
   - the exact two-tendon Blankevoort form is used only where strains need it.
2. **Finger ligament stiffness.** There are no tensile data for the digit 2-5 MCP
   collaterals. Options:
   - transfer from cross-sectional area × modulus;
   - transfer from the PIP collateral and thumb CMC data;
   - transfer from the knee and shoulder data.

   Each option is a `transferred` row with the assumption stated. For the thumb CMC,
   D'Agostino et al. 2014 give measured values for the dorsoradial and anterior oblique
   ligaments, which also fix an effective modulus (about 65-100 MPa) for the area route;
   section 2 proposes that modulus for the other hand ligaments, with 345 MPa as the
   upper bound. Is that acceptable for the MCP and PIP collaterals?
3. **Reference strains.** We have no in-vivo strain data for the CMC ligaments. Do we
   set the reference strain from the length-angle data (MCP) and from a "taut at the end
   of range" assumption (CMC)? For the CMC, Ladd et al. 2012 give the posture in which
   each ligament is maximally taut and its length there (Table I), which could fix that
   assumption per strand. Minami's MCP lengths are apparent lengths, valid only while a
   portion is taut. Slack length is the most influential parameter in every published
   sensitivity study. Wismans et al. 1980 already found the knee laxity far more
   sensitive to the initial strains than to doubling k. For scale, the published knee
   models set reference strains per bundle in an extended or standing posture of -0.25 to
   +0.10 (Blankevoort et al. 1991), -0.06 to +0.08 (Smith et al. 2016) and -0.25 to
   +0.12 (Marra et al. 2015); Menze et al. 2025 set 5-11 % at stated shoulder positions.
   Weiss and Gardiner 2001 caution that reference strains tuned to reproduce kinematics
   are often physiologically unrealistic and not unique, which argues for the
   anatomical "taut at the end of range" route here.
4. **How much of the residual the ligaments may take.** Kuo and Deshpande 2012 suggest
   the capsule-ligament complex dominates MCP stiffness; Knutson et al. 2000 find the
   multi-joint muscles dominant at the range ends for most wrist angles. Both leave the
   single-joint part to capsule and ligament. Do we let ligaments take most of that
   single-joint residual, or cap them at a fixed fraction (for the capsule) until there
   is better data?
5. **Wrapping surfaces.** MuJoCo wraps only spheres and cylinders. Are three strands per
   collateral on a cylinder enough to reproduce the MCP cam effect, or do we need more
   strands? In a knee comparison quoted by Weiss and Gardiner 2001, three or fewer line
   elements per ligament made the model very sensitive to the attachment geometry, and
   seven or more were redundant.
6. **Inertia and stability.** The stability margin depends on the phalanx inertia floor
   and armature. Should a ligament PR wait for, or be coordinated with, the inertia work
   (#128)?
7. **Policy text.** The authoring guide and role table (#140) need the passive
   spatial-ligament row. Should that go into #140 or into the first ligament PR?
8. **Knee and shoulder.** Do we want the knee ligaments at all while the knee is a
   one-DOF coupled joint? Is a strand-based glenohumeral capsule acceptable given the
   Moore et al. 2010 result?
