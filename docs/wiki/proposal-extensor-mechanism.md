# Proposal: a finger extensor mechanism for the MyoHand

Status: draft for discussion. Nothing in this page is implemented in the model yet. The numbers in *Prototype and cost* come from a throwaway MjSpec prototype that was built on `dev` (9405437) and is not part of this change.

The proposal follows the model policy in `docs/wiki/model-authoring.md` (MyoHub/myo_sim#140):

- one reference model;
- no feature flags;
- expensive or optional features become separately registered models;
- Python composes, XML defines;
- every value has a source.

## The issue

### What the model has today

The extensor side of each finger in `myo_sim/models/arm/assets/myoarm_r_{chain,tendon,muscle}.xml` uses fixed paths:

- EDC2-5, EIP and EDM are single spatial tendons from the forearm to the distal phalanx. Each crosses the MCP, PIP and DIP and pulls on all three joints with its full force.
- The intrinsics are lumped per digit into three actuators:
  - RI (radial interosseous) ends on the proximal phalanx and crosses only the MCP.
  - LU_RB (lumbrical plus radial band) and UI_UB (ulnar interosseous plus ulnar band) end on the middle phalanx. They cross the MCP and the PIP, but not the DIP.
- Nothing splits extensor force between a central slip and a terminal tendon.
- No oblique retinacular ligament couples the PIP and the DIP.

The table lists the moment arms at the neutral posture (`qpos0`) of `myohand_r` on `dev`. They come from the tendon Jacobian, `r = -dL/dq`, in mm. Positive means flexion for the finger joints and abduction (radial) for `mcp*_abduction`. Wrist entries are omitted.

| Tendon | MCP flexion | MCP abduction | PIP | DIP |
| --- | --- | --- | --- | --- |
| EDC2 | -8.7 | -0.4 | -5.2 | -3.6 |
| EIP | -9.2 | +1.5 | -6.5 | -2.7 |
| EDC3 / EDC4 / EDC5 | -9.4 / -7.9 / -5.9 | +0.9 / +4.4 / +3.2 | -5.4 / -4.0 / -5.3 | -3.1 / -5.5 / -3.0 |
| EDM | -5.7 | -0.8 | -2.9 | -2.3 |
| RI2 | -4.2 | +5.9 | — | — |
| RI3 / RI4 / RI5 | +3.4 / +3.4 / +5.7 | +7.5 / +6.9 / +3.7 | — | — |
| LU_RB2 / 3 / 4 / 5 | +3.0 / +3.0 / +2.4 / +2.8 | +6.6 / +8.4 / +3.8 / +6.5 | -3.5 / -1.3 / -0.6 / -3.8 | — |
| UI_UB2 / 3 / 4 / 5 | +3.6 / +4.6 / +2.5 / +3.2 | -6.7 / -4.1 / -4.5 / -6.4 | -2.5 / -1.3 / -2.9 / -2.8 | — |

"—" means no moment arm (|r| < 0.2 mm).

RI2 has an extension moment arm at MCP 0 deg that becomes +6.4 mm of flexion at 45 deg. This is the RI2 path jump between 10 and 15 deg of MCP flexion found by the hand-function validation (`hand-function-validation.md`, a separate change not yet on `dev`). At 45 deg of index flexion, the extensor moment arms keep the same pattern: no intrinsic crosses the DIP.

### What this gets wrong

**DIP extension comes only from the extrinsic extensors.** In the hand, the lateral bands carry interosseous and lumbrical force to the terminal tendon. The intrinsics therefore extend the DIP as well as the PIP. In the model they cannot.

McFarland et al. (2023) state the same gap for ARMS:

- the intrinsics "create flexion moments about the MCP joints without extending the distal phalanges";
- "100% of the forces produced by the extrinsic finger extensors are transmitted across both interphalangeal joints";
- "of highest priority, future work will need to move towards incorporating an extensor mechanism".

MyoHand is a little better than ARMS, because LU_RB and UI_UB do cross the PIP. The DIP problem is the same.

**There is no PIP-DIP coupling.** Nothing passive links the two interphalangeal joints. Flexing the PIP neither slackens a structure that holds the DIP, nor tensions one.

**The intrinsic-plus and intrinsic-minus postures cannot be reproduced.**
- *Intrinsic-plus* (MCP flexed, PIP and DIP extended): in the prototype test below (from a full fist, then all index intrinsics at full activation), `dev` ends at MCP 91, PIP -2, DIP 90 deg. The DIP stays flexed because no intrinsic crosses it.
- *Intrinsic-minus "claw"* (MCP hyperextended, IPs flexed when the extrinsics act alone): this needs MCP hyperextension. The model's MCP range is 0..90 deg, so the claw cannot appear with any tendon routing.

**Fingertip force directions.** Valero-Cuevas et al. (1998) measured maximal index fingertip forces in five directions: palmar 27.9, distal 24.3, medial 22.9, lateral 14.7 and dorsal 7.5 N. Their model reproduced these only with a Winslow-rhombus extensor mechanism. The dorsal force was most sensitive to the PIP extensor moment arm and to how tension is allocated in the network, and the "floating net" of the intrinsic bands was critical.

Synek and Pahr (2016) compared two index models built on the normative data of An et al. (1979), one with a Winslow-rhombus mechanism and one with a single extensor tendon and the intrinsics ending on the proximal phalanx, over nine postures. With the mechanism, the maximal forces in the palmar, proximal, dorsal and distal directions were 187.6 % larger on average (palmar +40.0 %), while the peak resultant force was 4.3 % smaller (Figs. 4-5, p. 3426). Co-contraction of the intrinsics and the long extensor let the extrinsic flexors reach higher activations. Hu et al. (2014) found that including the network lowers the estimated intrinsic muscle forces by 22-61 % and the MCP contact force by 10-41 % in isometric pressing.

## Anatomy to cover

- **EDC split.** Over the proximal phalanx the EDC tendon splits into:
  - a central slip, inserting on the base of the middle phalanx and extending the PIP;
  - two lateral slips. These join the interosseous and lumbrical contributions as the lateral bands, which converge into the terminal tendon on the base of the distal phalanx and extend the DIP.

  Landsmeer (1949, pp. 34-35) describes the mixing: the lateral band of the communis joins the lateral part of the interosseous tendon to form the lateral tendon, and the medial band of the interosseous joins the middle part of the communis to form the middle tendon (central slip). Harris and Rutledge (1972, p. 724) likewise have the interossei and lumbricals inserting into both the central slip and the lateral bands. The lateral bands leave the mechanism 1-2 cm proximal to the PIP (p. 717).
- **Retinacular and triangular ligaments.** Transverse retinacular ligaments hold the bands at the sides of the PIP. The triangular ligament keeps the bands from drifting apart on the dorsum of the middle phalanx. Harris and Rutledge (1972, pp. 717-719) cut the triangular ligaments and freed the bands from the transverse and oblique retinacular ligaments and from the central slip: the bands still moved palmarly in flexion and dorsally in extension. They attribute the shift to bowstringing between the band origin and the insertion, which moves with the middle phalanx, and to the shape of the proximal-phalanx head, which is wider palmarly.
- **Oblique retinacular ligament (ORL).** Landsmeer (1949, pp. 35-36) describes it as the oblique part of the retinacular ligament: the most lateral fibres of the terminal tendon, attached to the lateral border of the proximal phalanx and partly to the flexor sheath, passing closely palmar to the PIP axis. In his account, DIP flexion tenses it and flexes the PIP, and PIP extension extends the DIP through it (pp. 38, 41-42). Its role is disputed (see *PIP-DIP coupling*).
- **Sagittal bands.** They centre the extensor tendon over the MCP. They also carry part of the EDC force into the proximal phalanx. Harris and Rutledge (1972, pp. 719-722) found that the MCP is extended mainly by these fibres encircling the joint to the flexor sheath. The direct slip to the base of the proximal phalanx is lax from 0 to 90 deg of MCP flexion and tightens only in extreme hyperextension. Landsmeer (1949, p. 32) found no tendinous insertion there.
- **A posture-dependent network.** Together these form Winslow's rhombus. The force split depends on posture, because the bands translate as the PIP flexes.
  - Forced DIP flexion pulls the aponeurosis about 3 mm distally and slackens the central slip until the PIP has flexed 60-70 deg (Landsmeer 1949, pp. 39, 42).
  - With the PIP flexed 80-90 deg, the distal phalanx escapes the extensor (Landsmeer 1949, pp. 37, 41). Harris and Rutledge (1972, p. 724) attribute this to the central slip holding the mechanism distally: after the central slip was cut, the extensor extended the DIP in any PIP position.

## Literature

**Force transmission measured in cadavers.**
- Lee, Chen, Towles and Kamper (2008a) loaded EDC and EIP separately (11.8 N, about 30 % of their estimated maximal force) in five index fingers in nine postures (MCP 0/30/60 deg crossed with PIP/DIP 0/0, 30/20 and 45/30 deg; Table 1). Terminal slip force = r·c·F and central slip force = (1−r)·c·F (Eq. 1). Here c is the fraction of tendon force that reaches the two slips and r the terminal share of it. Both come from a sagittal-plane inverse-statics fit to the measured fingertip force and moment, with slip moment arms of half the measured DIP and PIP joint thickness (Appendix, p. 8).
  - Means (SD) over postures and specimens (Table 5): EDC r = 0.63 (0.10), c = 0.39 (0.19); EIP r = 0.64 (0.09), c = 0.40 (0.21).
  - The terminal-to-central ratio stays near 1.7:1 (= 0.63 / 0.37) across postures. r has no IP effect (p = 0.60) and a weak MCP effect (p = 0.02); c falls significantly with IP flexion (p < 0.001; Table 4).
  - c = 0.39 is a mean over all nine postures. The mean slip forces (Table 3) give c ≈ 0.72 and r ≈ 0.60 with all joints extended (MCP 0), and c ≈ 0.25-0.31 with the IPs flexed. With the IPs extended, MCP flexion to 60 deg lowers both slip forces by 30-50 % (p. 7).
  - The lost share (1−c) goes into the sagittal bands, the MCP capsule and the palmar proximal phalanx, into soft-tissue connections between the hood and the dorsum of the proximal and middle phalanges (strongest near the PIP capsule), and into friction in the carpal tunnel (pp. 2, 6-7).
  - r = 0.63 contradicts the classical 50:25:25 split, which gives a terminal share of 0.5 (p. 6).
- Micks and Reswick (1981) showed in three cadaver long fingers, MCP fixed in extension, that the central fibres are tensed in flexion and the lateral fibres in extension, with a distribution over all fibres at mid-range.
  - They measured the effective PIP moment arm of the whole communis force (measured proximal to the hood), with the passive tissue moments cancelled by using force increments: 2.3-5.5 mm over 5-90 deg; in two of the fingers lowest at mid-flexion (2.7 mm at 50 deg, 2.3 mm at 45 deg), and 4.3-5.5 mm at 90 deg in all three (Table I, p. 466).
  - Passive PIP moments become substantial beyond about ±25 deg from the balanced position. Excising the oblique fibres of the extensor complex shifted the passive moment-angle curve by about 15 deg; at 95 deg flexion they carried an extension moment of about 100 g·cm (about 10 N·mm) (pp. 465-466).
- Valero-Cuevas et al. (2007) loaded the network of cadaver middle fingers (MCP 45, PIP 45, DIP 10 deg). The ratio of central-slip to terminal-slip tension reversed as the input ratio between intrinsics and extensor changed. They note that a network with invariant tension propagation "by definition cannot display this behavior".
- Lee et al. (2008b) estimated effective static moment arms (ESMAs: joint moment per unit tendon force, passive moments removed) for five tendons of the index extensor mechanism: five fresh-frozen hands, the same nine postures, loads up to 11.8 N for EDC and EIP, 8.8 N for FDI and FPI and 5.9 N for LUM (Table 1).
  - Means across postures and specimens, in mm (Table 4, p. 1571): EDC DIP 2.26 (1.08), PIP 3.10 (1.18), MCP 7.03 (3.21), MCP abduction about 0; EIP 2.41, 3.42, 7.53; FDI DIP 0.34, PIP -0.38, MCP -2.02 (flexion), abduction 3.10; FPI DIP 0.90, PIP 0.13, MCP -4.63, adduction 3.91; LUM DIP 0.53, PIP -0.84, MCP -6.99, abduction 1.77.
  - Posture effects (Table 3): the EDC MCP ESMA rises from 4.29 to 8.92 mm as the IPs flex from 0/0 to 45/30 deg; the DIP ESMAs of FPI (1.75 to 0.38 mm) and LUM (0.83 to 0.08 mm) fall with IP flexion; the abduction ESMAs fall with MCP flexion (FDI 4.69 to 1.24 mm).
  - The ESMAs are smaller than kinematic moment arms at the MCP (EDC 7.0 mm against 8.6 mm in An et al. 1983). The authors attribute the difference to force distribution within the hood, dissipation into the capsule and the ORL, and the reaction at the fixed fingertip (pp. 1571-1572).
  - EDC's DIP and PIP ESMAs show no posture effect (Table 2: p = 0.93 and 0.45 for IP posture), although the slip forces of Lee et al. (2008a) fall with IP flexion.
- An et al. (1983) measured tendon excursions in seven index fingers under a 5 N load and averaged the moment arms over the range of motion (Table 3; the values are printed in cm under an "mm" header): EC DIP 2.2 ± 0.4, PIP 2.8 ± 1.1, MCP 8.6 ± 1.6 mm; EI 1.9 ± 0.5, 2.6 ± 1.1, 9.0 ± 1.3 mm; LU DIP 0.7, PIP 1.8 mm; UI DIP 1.6, PIP 2.6 mm (extension). At the DIP and PIP these kinematic values agree with the ESMAs of Lee et al. (2008b).

**Geometry and material.**
- Garcia-Elias et al. (1991a) measured the 3-D geometry of the mechanism by stereophotogrammetry in seven fingers in five tendon-loaded configurations (intrinsic-minus, extension, mid-flexion, full flexion, intrinsic-plus).
  - Segment lengths change little, while their orientation varies considerably (Table II). The largest length change is the proximal lateral band, 37.1 ± 2.6 mm in extension against 31.6 ± 1.3 mm in full flexion.
  - The lateral bands migrate palmarly by 2.6 mm on average as the PIP flexes. The central band lies 7.1 mm (6.9-7.5) dorsal of the PIP centre of rotation. The lateral bands lie 5.2 mm palmar of the central band in extension, 6.1 mm in mid-flexion and 5.5 mm in intrinsic-plus, i.e. still dorsal of the axis. In intrinsic-minus and full flexion they lie 7.7 and 7.8 mm palmar of the central band, i.e. palmar of the axis (p. 1134). The authors note that the bands carry little tension there, so this does not make them PIP flexors.
  - The angle between the two intercrossing bundles ranges from 34.5 ± 3.7 deg in extension to 42.1 ± 4.1 deg in intrinsic-minus.
- Garcia-Elias et al. (1991b) measured structural stiffness per segment in seven middle fingers (ages 40-75), loaded to 100 N, where strains stayed below 4 % (Table I, N/mm, mean ± SD):
  - central band: proximal 256 ± 62, middle 126 ± 26, distal 294 ± 57;
  - intercrossing fibres: central-to-lateral 40 ± 12, lateral-to-central 75 ± 21;
  - lateral band: proximal 43 ± 7, distal (terminal tendon) 121 ± 24.

  The abstract's "275 N/mm" is the mean of the proximal and distal central band. The paper gives no force distribution and no failure loads.
- Qian et al. (2014) tested coupons from 19 hoods and give modulus, stiffness and failure load per region. For example: central slip 101 MPa, terminal slip 90 MPa, lateral band 105 MPa, sagittal band 55 MPa. Passive flexion alone strains the hood by 2-3 %.
- Schultz et al. (1981) found no elastic tissue in the hood over the PIP (histology, pp. 494-496). The central tendon and the lateral bands are linked by criss-cross fibres in two separate, non-interdigitating layers. The authors suggest that these fibres govern how far the lateral bands move palmarly in flexion and how they return in extension; the study is qualitative (fresh and preserved cadaver fingers, no counts or forces).
- Canoso et al. (2021) measured the band shift in vivo by ultrasound (7 subjects aged 23-55, middle and ring fingers, MCP at 0 deg, PIP actively flexed 0 to 90 deg). At mid-proximal phalanx the ulnar lateral band moved palmarly by a median of 1.8 mm (range 0.7-3.0) in the middle finger and 1.1 mm (0.3-2.7) in the ring finger (Table 1). Palpation suggested 5-10 mm, which the authors attribute to a haptic illusion. Dissection of 5 embalmed hands confirmed the direction of the shift but could not measure it. This is smaller than the 2.6 mm Garcia-Elias et al. (1991a) report as the PIP flexes, and is measured further proximally.

**PIP-DIP coupling.**
- Landsmeer (1949) described the dorsal aponeurosis and the retinacular ligament (*Anatomy to cover*); his later papers (1958, 1963) treat the coordination of the interphalangeal joints.
- Harris and Rutledge (1972) found the ORL thin and, in fresh fingers, slack in normal motion.
  - With the PIP at 0 deg it tightened only beyond about 70 deg of DIP flexion, and with the PIP at 30, 60 or 90 deg it did not tighten at all (Table II, p. 715).
  - Cutting it on both sides caused no perceptible loss of flexion or extension (p. 714). Cutting the lateral bands while sparing the ORL left the DIP hanging at 70 deg, without active extension (p. 715).
  - In high-speed films of one hand, the PIP and DIP angles differed by at most 30 deg in unresisted flexion and 25 deg in extension, too little to tighten the ORL (Table I, pp. 715-716). They conclude that the lateral bands alone extend the DIP and that the ORL is a stay ligament (p. 716).
- Ueba et al. (2011) dissected 40 fresh-frozen fingers from 10 hands (age 57 ± 14 years).
  - The ORL was present on both sides in 38 fingers; 2 of the 80 sides lacked it. It was thin on average (3 on a 5-point scale) and most robust on the ulnar side of the ring finger. It arose from the flexor sheath near the A2 pulley and merged with the lateral band over the proximal third of the middle phalanx (pp. 1961-1962).
  - Protocol: the extrinsic extensor was preloaded with 1.96 N, and the DIP was cycled passively from 0 to 45 deg (it would flex only to 45 deg with the PIP extended). The outcome was the peak force at a screw in the distal-phalanx tuft (pp. 1960-1961).
  - The intact peak force was highest at PIP 30 deg (3.1 ± 2.0 N) and lowest at PIP 90 deg (1.5 ± 1.6 N). The MCP at 0, 45 or 90 deg, tested with the PIP extended, had no effect (p. 1962).
  - Cutting the ORL lowered the force by 25 % at PIP 0 deg, 31 % at 30 deg, 18 % at 60 deg and 3 % at 90 deg, with the MCP extended (p. 1962). These are shares of the peak force, not moments; the paper does not give the lever arm.
  - The low force at PIP 90 deg does not depend on the ORL. Cutting the central slip raised it markedly, which the authors attribute to the central slip pulling the bands distally and relaxing the terminal tendon (pp. 1962, 1964).
- Leijnse, Quesada and Spoor (2010) measured coupled IP trajectories in vivo in 68 fingers (17 hands, 9 subjects aged 33 ± 11 years), unloaded, flexed by the deep flexor with the MCP at 0 deg.
  - The DIP-versus-PIP trajectories are sigmoid and consistent within a finger but vary between fingers. The mean slope dDIP/dPIP of the extension trajectories peaks at 1.0 ± 0.31 at PIP 70 deg and falls to 0.53 ± 0.28 at 100 deg, 0.25 ± 0.25 at 0 deg and -0.11 ± 0.25 at -13 deg (p. 2386, Fig. 4A).
  - Passive PIP hyperextension ranges from 0 to almost 50 deg; greater hyperextensibility goes with flatter slopes near extension (correlation C1, p. 2386). Mean passive ranges per digit are in Table 2 (e.g. PIP flexion 110-119 deg, PIP extension -11 to -21 deg).
- Leijnse and Spoor (2012) fitted a 2-D kinematic model of inextensible strings that go slack or taut to these 68 trajectories: a central string M (PIP moment arm 5 mm), n lateral strings joining a terminal tendon (DIP moment arm 3.5-4 mm), one retinacular ligament from the PIP volar plate, and the oblique retinacular ligament (Table 1, Fig. 3).
  - The coupling follows dθ_DIP = (r_M − r_T) / r_T,DIP · dθ_PIP (Eq. 1): the lateral parts move palmar of the central slip at the PIP as it flexes. It needs only a two-segment extensor insertion and is active only while both parts are taut, i.e. with the deep flexor active (pp. 570-571).
  - Fits needed 4.9 ± 2 strings (2 to 10). For PIP below 25-45 deg the fits were within 0.5 deg RMS; over the full range, about half of the fingers stayed within 3 deg and the rest had maximum errors of 3-12 deg in the upper mid-range (p. 576).
  - The retinacular ligament went slack with IP flexion; one sufficed in 66 of 68 fingers. The oblique retinacular ligament improved the fit in only one finger, by 2 deg (p. 577), so it does not drive unloaded coupling.
  - Inextensible strings sufficed, which suggests that hood strain is not essential to unloaded coupling (p. 577).
- **The ORL sources disagree.** Landsmeer (1949) gives the ORL an active coupling role. Harris and Rutledge (1972) and Leijnse and Spoor (2012) find it slack or unnecessary in unloaded motion. Ueba et al. (2011) measure a passive share of up to 31 % with the PIP extended and the DIP within 45 deg, where Harris and Rutledge saw it slack. Their protocols differ: a preloaded extensor and peak force over 0-45 deg in Ueba, visual tautness in fresh fingers in Harris and Rutledge. Also, Ueba cut all tissue at the ORL's location where the ligament was absent. We therefore treat the ORL as a passive element fitted to Ueba's shares, and check that it stays slack in unresisted flexion.

**Models with constant fractions.** Most musculoskeletal models that include the mechanism distribute force with constant coefficients:
- Chao et al. (1976): the first 3-D force analysis of finger joints, from biplanar radiographs of six cadaver fingers. It assumes that the terminal tendon carries the sum of the two band forces and that the central slip carries the full long-extensor force (assumptions 3-4, p. 391). The band forces are otherwise free unknowns. The redundancy was removed by setting four of the nine tendon forces to zero in turn (126 combinations, p. 392). The extensors were treated as passive and capped at 0.5 of the applied force at the DIP and PIP (p. 392).
- An et al. (1979);
- Chao and An (1978), as reproduced by Brook et al. (1995, Eqs. 12a-d, p. 59): F_TE = F_RB + F_UB, F_RB = 2/3 F_LU + 1/6 F_LE, F_UB = 1/3 F_UI + 1/6 F_LE, F_ES = 1/3 F_LU + 1/6 F_LE + 1/3 F_RI + 1/3 F_UI. Half of the long-extensor force thus reaches the slips, two thirds of it the terminal tendon (c = 0.5, r = 2/3 in the notation of Lee et al. 2008a);
- Valero-Cuevas et al. (1998): 50 % of EDC/EIP to the central slip and 25 % to each lateral offshoot nominally, plus a floating net for the intrinsic bands;
- Hu et al. (2014): the same coefficients, citing Chao and An (1978) and Brook et al. (1995);
- Synek and Pahr (2016): central-to-terminal ratios of 50:50 for the lumbrical and ulnar interosseous and 60:40 for the long extensor, following Qiu and Kamper (2014). The radial interosseous ends on bone, and the mechanism is not deformable (p. 3424);
- MacIntosh and Keir (2017), an OpenSim index finger. Seven muscles are divided into 14 muscle components; each extensor and intrinsic muscle has one component per insertion, with its share of the maximal force (Table 1, p. 1376):
  - EDC: 12.18 N to the central slip and 3.05 N to each band (terminal share 0.33);
  - EI: 7.23 N each to the central slip and the two bands (0.67);
  - FDI: 54.0 N to the proximal phalanx and 13.5 N to the terminal slip;
  - FPI: 22.45 N central, 11.31 N terminal;
  - lumbrical: 1.81 N central, 3.15 N terminal.

  The shares were chosen to minimise the deviation between predicted activations and EMG in two postures at 5 and 30 N (p. 1374). The model then predicted terminal-to-central slip forces of 0.59:0.41 (± 0.05) in static pressing (0.58 at 5 N to 0.68 at 30 N, not significant pairwise) and 60:40 averaged over unloaded movements (pp. 1377-1378);
- Synek et al. (2019): transmission fractions identified from cadaver fingertip forces.

The terminal share of the long extensor ranges across these models from 0.3 to 0.67, against r = 0.63 measured by Lee et al. (2008a).

**Models where the split is solved with the muscle forces.**
- Brook et al. (1995): a 3-D dynamic index model. They keep the form of Chao and An's equations with variable coefficients (Eqs. 14a-d, p. 59):
  - F_TE = χ_RB F_RB + χ_UB F_UB, with χ ≈ 0.985 for a convergence angle of about 10 deg;
  - F_RB = α_LU F_LU + α_LE F_LE − K ε_RB, and F_UB likewise with α_UI F_UI;
  - the central slip takes the remainder plus K(ε_RB + ε_UB), where ε is the relative displacement of a band and the central slip and K an elasticity taken from an earlier model.

  All of the long-extensor force reaches the slips, the radial interosseous ends on bone, and the retinacular ligaments are omitted. The α are optimised together with the muscle forces. In a simulated pinch, α_LU stayed near 0.67, while α_LE settled at about 0.15 instead of its initial 0.33. This sends 70 % of the long-extensor force to the central slip, a terminal share of about 0.3 (p. 61). Excursion coefficients fitted to An et al. (1983) are in Table 2 (p. 59): terminal tendon 1.88 mm at the DIP, central slip 2.92 mm at the PIP.
- Sancho-Bru et al. (2001): a 3-D inverse-dynamic index model. The mechanism is a net of straight branches with a force balance at each connection point (Fig. 4, p. 1494). The 4 equations of motion and 8 balances (12 equations) face 18 unknowns (6 muscle forces and 12 branch forces), which are found by minimising Σ(F/PCSA)², bounded by Hill-type muscle limits (pp. 1495-1496). EDC and EI are lumped; the radial interosseous ends on bone; the lumbrical arises from a floating point on the FDP. The MCP collateral ligaments are quadratic springs, and no retinacular ligament is included (pp. 1492-1495).

**Models where the split emerges from the network.**
- Valero-Cuevas et al. (2007): a 3-D string network solved quasi-statically.
- Saxena et al. (2012): inferred network topology and parameters from cadaver data.
- Dogadov et al. (2017, 2025): elastic string networks with retinacular and triangular ligaments, solved by energy minimisation. Posture significantly changed the forces in the intercrossing fibres.
- Jadelis et al. (2023): a finite-element shell model of the index hood co-simulated with a multibody model.
- Sachdeva et al. (2015): simulated the tendon network, including the oblique ligament, with Eulerian-on-Lagrangian strands. Pulling FDP alone produced PIP-DIP coupling, and lengthening the ORL produced a swan-neck. The strands needed a dedicated quasistatic solver because branching inextensible loops lock under bilateral constraints. They report about 10 s of computation per simulated second for the index finger at a 3 ms step.
- Sueda et al. (2008) simulated hand musculotendons with strands but no explicit hood.

**OpenSim models.**
- Holzbaur et al. (2005) and Saul et al. (2015) have no intrinsics and no extensor mechanism.
- Mirakhorlo et al. (2018) model the medial bands as separate paths that share via points. They omit the lateral bands, because an OpenSim muscle-tendon unit cannot have several insertions. The model is published under CC BY-NC-ND 4.0.
- MacIntosh and Keir (2017) published their index finger (above) on SimTK (simtk.org/projects/finger). The paper calls it open-source but names no licence. The SimTK download carries a "MIT Use Agreement" whose text is the MIT licence, with a copyright line naming Stanford University (2016). That permits reuse with the notice. This proposal uses no values from the model files; the shares quoted above are the published Table 1, cited for comparison only.

**AnyBody model.** Engelhardt et al. (2021, CC BY) built a detailed AnyBody hand from a 16-specimen dataset. It has no extensor mechanism: the DIP is driven from the PIP by a finger rhythm when motion data lack the distal phalanx, and the passive tension of the hood is omitted (pp. 779, 785). The lumbricals arise from a massless substitute segment between two via points on the FDP tendon (p. 779).

**MuJoCo models.**
- MS-Human-700's manipulation model (Apache-2.0) has separate lumbricals and palmar and dorsal interossei. Their tendon Jacobians at the neutral posture show that all of them cross only the MCP, while EDCI crosses MCP, PIP and DIP.
- We found no public MuJoCo implementation of the extensor mechanism.

**Interosseous insertions.** Salsbury (1937) dissected the interossei of 30 hands (28 specimens per muscle) and estimated the share of each tendon that goes to the extensor expansion rather than to bone (Table II, p. 400):
- palmar interossei: PI1 (index) 98.8 %, PI2 (ring) 100 %, PI3 (little) 89.1 %;
- first dorsal interosseous: 0 %, inserting wholly into the proximal phalanx in every hand;
- other dorsal interossei: DI2 48.4 %, DI3 93 %, DI4 66.3 %.

Where a dorsal interosseous inserts into both, the two insertions usually come from functionally separate fleshy parts (p. 401). Salsbury also describes a transverse fascial sheet over the MCP that gives the common extensor an extensive insertion, which he considers the main reason it does not act as an IP extensor (p. 402).

**What this means for us.** An et al. (1983) and Lee et al. (2008b) give whole-tendon moment-arm targets. Lee et al. (2008a) give slip-force checks for r and c across the nine postures. Ueba (2011) gives a passive target for the ORL, and Harris and Rutledge (1972) a slack range for it. Leijnse (2010) gives a free-flexion coupling target. Valero-Cuevas (1998) gives fingertip-force targets. The material data (Garcia-Elias 1991b, Qian 2014) bound the band stiffness.

## Proposed model changes

### What MuJoCo can and cannot do

MuJoCo tendons cannot have a free branch point:
- one actuator drives one tendon;
- a tendon force never splits by equilibrium at a moving node.

A spatial tendon can, however, have several branches started by `pulley` elements. The tendon length is the sum of the branch lengths, each divided by its `divisor`. Each branch therefore carries tension F / divisor. This is a native, exact constant-fraction split of one muscle force.

A split that depends on posture needs a body at the branch point. This gives three ways to build the mechanism.

### Option (a): fixed split with pulley branches

Each extensor and intrinsic muscle keeps its actuator and its tendon. The tendon's path ends at a hood (or band) site on the proximal phalanx. Branches then continue to the insertions. For the EDC/EIP/EDM tendon:

- **main path:** forearm → MCP wrap → hood site on the proximal phalanx (divisor 1);
- **central slip:** hood site → PIP wrap → base of the middle phalanx, divisor 1 / (c(1−r));
- **two lateral slips:** hood site → band site at the PIP → terminal convergence site on the middle phalanx → DIP wrap → base of the distal phalanx, divisor 2 / (c·r) each.

The main path ends on the proximal phalanx, so the remaining (1−c)·F acts on the proximal phalanx at the hood. It stands in for the share Lee et al. (2008a) attribute to the sagittal bands, the capsule and the hood's connections to the phalanges; the part of it that they attribute to carpal-tunnel friction has no counterpart in the model. Lee's mean values (r = 0.63, c = 0.39) would give the central slip 0.14 F and each lateral slip 0.12 F. The fit below gives two to two and a half times that.

**Checking the fractions against joint moments.** In option (a) the EDC's DIP moment arm is c·r·r_TE and its PIP moment arm c(1−r)·r_CS + c·r·r_LB, with the geometric moment arms of the terminal tendon (r_TE), central slip (r_CS) and lateral bands (r_LB). Lee's r and c are not tissue fractions that transfer on their own: they come from an inverse model with its own slip moment arms (half the joint thickness, no lateral-band arm at the PIP). What transfers is the joint moment per unit tendon force, and two independent measurements agree on it for the EDC:
- the kinematic moment arms of An et al. (1983): DIP 2.2 ± 0.4 mm, PIP 2.8 ± 1.1 mm;
- the ESMAs of Lee et al. (2008b): DIP 2.26 ± 1.08 mm, PIP 3.10 ± 1.18 mm, with no IP-posture effect.

Micks and Reswick's (1981) effective PIP moment arm of 2.3-5.5 mm is consistent with these.

Taking the geometry of Garcia-Elias et al. (1991a) as an illustration (r_CS ≈ 7.1 mm; r_LB ≈ 1.9 mm in extension, 7.1 − 5.2):
- **PIP.** With r = 0.63 the PIP moment arm is c·(0.37 · 7.1 + 0.63 · 1.9) ≈ 3.8·c mm, so c ≈ 0.73-0.81 matches 2.8-3.1 mm.
- **DIP.** The DIP target of 2.2-2.3 mm then needs r_TE ≈ 4.5-4.9 mm. With the terminal-tendon arm of the Leijnse and Spoor (2012) model (3.5-4 mm) it needs c ≈ 0.87-1.0.
- **Lee's own data.** Lee et al. (2008a) find c ≈ 0.72 with the finger extended (from Table 3), at the low end of this range.
- **The nine-posture mean.** It is lower (0.39) because c falls with IP flexion, but the EDC's IP joint moments per unit force do not (Lee et al. 2008b, Table 2). A constant split therefore loses less than the trend of c suggests.
- **c = 0.39 is too low.** With the same geometry it gives the IP joints about half the measured extension moment (PIP 1.5 mm, DIP 0.9-1.2 mm).

**Proposal:** keep r = 0.63 and fit c to the EDC and EIP DIP and PIP targets above, using the model's own geometric moment arms. With the illustrative geometry, c comes out at about 0.75-1.0:

| Branch | Share of F |
| --- | --- |
| Central slip | 0.28-0.37 |
| Each lateral slip | 0.24-0.32 |
| Hood (proximal phalanx) | 0-0.25 |

The slip forces of Lee et al. (2008a) and the ESMAs of Lee et al. (2008b) at the nine postures are reported as checks. The MCP is not a target for c, because the MCP ESMA (7.0 mm) is smaller than the kinematic moment arm (8.6 mm) and rises with IP flexion.

**Intrinsics.** LU_RB and UI_UB end at their band site and branch to the terminal tendon and to the central slip. Their fractions would come from the constant coefficients of Chao and An (1978) as used by Hu et al. (2014) and reproduced by Brook et al. (1995, Eqs. 12a-d): LU 2/3 to the radial band and 1/3 to the central slip; UI 1/3 to the band, 1/3 to the central slip, and the rest to the proximal phalanx. With r_TE = 3.5-4 mm, the lumbrical's 2/3 terminal share gives a DIP moment arm of 2.3-2.7 mm, against an ESMA of 0.53 ± 1.33 mm that falls towards zero with IP flexion (Lee et al. 2008b, Tables 3-4) and a kinematic moment arm of 0.7 ± 0.6 mm (An et al. 1983, Table 3). The intrinsic fractions are therefore checked against the same ESMAs.

**Muscle lengths.** The muscle sees the divisor-weighted length. Its `lengthrange` must therefore be recomputed, and the slack-length and `fpmax` fit of the extrinsic extensors (`passive-muscles-tenodesis.md`, Vittorio-Caggiano/myo_sim#2) rerun.

**Pros:**
- no new DOFs or actuators, so the action and observation spaces stay as they are;
- the step cost barely changes (below);
- it runs at the model timestep without any change;
- it behaves identically on CPU and mujoco-warp (pulleys are supported natively);
- every fraction is one XML number with a citation.

**Cons:** the split is constant by construction. It cannot show the posture dependence of c, the band translation, or the switching of Valero-Cuevas et al. (2007). Nor can it release the distal phalanx when the PIP is flexed 80-90 deg. In the hand the central slip then holds the mechanism distally, and the extensor loses the DIP (Landsmeer 1949; Harris and Rutledge 1972; Ueba et al. 2011, central-slip section). In option (a) the EDC keeps c·r of its force on the terminal tendon in every posture.

A tendon `equality` constraint is the other fixed option. It couples tendon *lengths*, not forces, so it is a kinematic constraint. It is not a force split, and we do not recommend it for the bands.

### Option (b): a network of junction bodies

The branch points become small bodies:
- a hood node H and a central node C on the proximal phalanx, each on one slide joint along the bone;
- radial and ulnar band nodes RB and UB at the PIP, each with an axial and a dorso-volar slide;
- a terminal node T on the middle phalanx, with one axial slide.

Passive tendons link the nodes:
- the central slip, in two segments, H→C and C→middle phalanx;
- the lateral slips H→RB and H→UB;
- the intercrossing medial fibres RB→C and UB→C;
- the lateral bands RB→T and UB→T;
- the terminal tendon T→distal phalanx;
- a soft triangular ligament RB–UB.

The transverse retinacular ligaments are the dorso-volar slide stiffness. EDC, EIP and EDM end on H, LU_RB on RB and UI_UB on UB. Each band is tension-only. The force split then follows from geometry and posture.

**Numerical requirements.** The prototype found three; details are in *Prototype and cost*.

1. **Elastic bands.** Bands as tendon springs with a dead band (`springlength="0 L"`) and stiffness 1e4-2.5e5 N/m (the range of Qian 2014 and Garcia-Elias 1991b, whose segments span 40-294 N/mm) are mostly unstable at 2 ms with the semi-implicit Euler integrator, unless each junction slide carries enough joint damping (about 500 N s/m for 5e4 N/m). MuJoCo integrates joint damping implicitly in the Euler integrator. With 50 N s/m, 1e4 N/m still runs at 2 ms, 5e4 N/m needs 1 ms and 2.5e5 N/m needs 0.25-0.5 ms (armature 1e-4 to 1e-2 kg); with 5 N s/m or less, even 1e4 N/m needs 0.25-1 ms. `implicitfast` stabilised no configuration that Euler could not run, because spring stiffness stays explicit in both.
2. **A muscle on a light node is an explicit damper.** With `vmax` = 1.5 L0/s, the slope of MuJoCo's force-velocity curve at zero velocity is 2 F0 / (vmax L0): about 330 N s/m for EDC2, 420 for EIP and 1800-2500 for the finger intrinsics (L0 of 3-5 cm). An explicit damper c on a mass m is stable only if c·dt/m < 2. A free node at 2 ms would need about 2.5 kg of armature under LU_RB2. In the spring prototype, LU_RB2's force alternated between 0 and its maximum on every step while the finger was held still.
3. **Bands as tendon length limits are the practical choice.** A tendon limit is a unilateral constraint solved by the constraint solver, so the band acts as a cable with stiffness set by `solref`/`solimp`. This keeps the node attached to the finger's inertia when the band is taut.
   - Settings that work: `solref="0.004 1"` (time constant 2 dt), `solimp="0.99 0.999 0.001 0.5 2"`, node armature 1e-3 kg and node damping 50 N s/m. The network then runs stably at 2 ms with Euler.
   - Effective band stiffness: 9-36 N/mm, close to the coupon values of Qian et al. (2014). This is below the softest segments of Garcia-Elias et al. (1991b): 40 N/mm for the central-to-lateral fibres and 43 N/mm for the proximal lateral band.
   - MuJoCo's default `solref` gives bands of only 0.2-3 N/mm, and the muscle force chatters.

**Armature.** Armature, not mass, should give the nodes their inertia. Armature stabilises without adding weight under gravity. At 1e-3 kg it is small next to the tendon forces at finger accelerations (a few mN at 5 m/s²).

**Damping.** The node damping is an artefact: it resists band translation. Going from 5 to 50 N s/m changed the index joint trajectories by 1.1 deg RMS (6.5 deg max) at a 0.1 ms step. The finger joint damping itself (0.05 N m s/rad, about 2000 N s/m at a 5 mm moment arm) is much larger.

**Topology and geometry must admit the full range of motion.**
- In the prototype, the central node sat at the PIP and the band nodes slid along the bone. Under FDP + FDS at full activation, PIP flexion stopped at 76-78 deg, while `dev` and option (a) reach 93 deg.
- In the locked state the central slip carried 77 N and the terminal tendon 47 N with the extensors almost silent. Removing the intercrossing fibres and the ORL did not unlock it. The likely cause (not yet confirmed) is geometric: near the PIP the central slip leaving C runs almost perpendicular to C's slide, so its tension cannot move the node.
- The node positions, slide directions and rest lengths therefore have to be identified together. The free-flexion coupling data (Leijnse 2010) and a passive-ROM test should check that no band carries more than a small force when the muscles are off. Leijnse and Spoor (2012) is the model to follow for strings that go slack or taut.

**Cost.** The network adds 7 DOFs per finger: 56 in `myofullbody`, which changes `nq`/`nv` and therefore downstream observations.

**Pros:** a posture-dependent split, band translation, and the intrinsic-plus posture emerging without tuning.

**Cons:** high cost; geometry that has to be identified; a new numerical-parameter family (node armature and damping, `solref`/`solimp`) that needs its own provenance (method page, not a measurement).

### Option (c): coupling the PIP and DIP through the ORL

Three ways:

1. **A joint equality** `md = poly(pm)`. This imposes a rigid kinematic coupling, which the ORL is not. The DIP must still be able to move against the PIP under load (Leijnse 2010 reports large individual variation). Not recommended.
2. **A passive fixed tendon over PIP and DIP** with coefficients equal to the ORL moment arms (volar at the PIP, dorsal at the DIP) and a tension-only dead-band spring (`springlength="-1e1 0"`; only lengthening is resisted). It is cheap: no geometry evaluation and no DOF. Its moment arms are constant.
3. **A passive spatial tendon** from the proximal phalanx (volar and lateral, about 8 mm proximal to the PIP), past a lateral via site on the middle phalanx, to the terminal tendon insertion. It has posture-dependent moment arms. In the prototype it had 1.6-3.5 mm (flexion) at the PIP and 2.5 mm (extension) at the DIP at the neutral posture. Without a wrap object it crossed to the volar side of the DIP at 45 deg flexion, so it needs a wrap or a second via site.

Variant 3 fits the explicit-ligament framework of MyoHub/myo_sim#142 (see *Interaction with other work*). Variant 2 or 3, radial and ulnar, would be fitted to the ORL share of the passive DIP resistance in Ueba et al. (2011): 25/31/18/3 % at PIP 0/30/60/90 deg. These are shares of the peak force over DIP 0-45 deg with the MCP extended and 1.96 N on the extrinsic extensor, so the fit has to reproduce that protocol. Its stiffness and slack length have no direct measurement in the sources above, so they are fitted parameters. The slack length is constrained from the other side by Harris and Rutledge (1972): in unresisted flexion the ORL should not become taut (Tables I-II, pp. 715-716). The fit has to go together with the residual DIP and PIP springs (below). The ORL is a passive element, not the source of the active coupling: in Leijnse and Spoor's (2012) fits to 68 fingers it improved the unloaded FDP-driven trajectory in only one finger, by 2 deg. The coupling target of validation 2 is therefore met by the extensor branches, and the ORL is fitted to the passive data only.

### Sagittal bands

- **Option (a):** the sagittal bands are the (1−c) share that ends at the hood site on the proximal phalanx. The EDC path's existing MCP wrap keeps the tendon centred.
- **Option (b):** H slides only along the proximal phalanx, which is the centring, and a passive band from H to the proximal-phalanx base can carry the sagittal share.

Ulnar or radial subluxation of the tendon (sagittal band rupture) is not modelled in either option.

### The existing intrinsics

The actuators stay as they are: RI2-5, LU_RB2-5 and UI_UB2-5, the same names and the same count. Their paths change:
- LU_RB and UI_UB end at the band site on the proximal phalanx (option a) or the band node (option b). From there the force reaches the terminal tendon and the central slip. Their PIP moment arm becomes one band moment arm, no longer a path straight to the middle phalanx, and they gain a DIP moment arm.
- RI keeps its MCP-only insertion on the index finger. This assumes the index radial interosseous (the first dorsal interosseous) inserts mainly into the proximal phalanx. Lee et al. (2008b) support it functionally: the FDI's PIP and DIP ESMAs were near zero (-0.38 and 0.34 mm, the PIP value not different from zero) and its main action was MCP abduction (3.10 mm; Table 4, p. 1571). Salsbury (1937) found the first dorsal interosseous inserting wholly into the proximal phalanx in every hand. MacIntosh and Keir (2017) give their FDI a terminal-slip component (13.5 of 67.5 N), but that share was fitted to EMG, not dissected.
- The other radial bundles are not all bony:
  - RI3 (DI2) sends about half its tendon to the expansion (48 %);
  - RI4 and RI5 (palmar interossei of the ring and little fingers) send 89-100 % (Salsbury 1937, Table II).

  Their hood shares are open question 4.

The lumbrical's moving origin on the FDP tendon is a separate topic and is not part of this proposal.

### XML roles and generated blocks

**`myoarm_r_chain.xml`** gets:
- the hood, band, terminal-convergence and ORL sites on the phalanges they belong to;
- for option (b), the junction bodies and their slide joints, as children of the proximal and middle phalanges.

**`myoarm_r_tendon.xml`** gets:
- the pulley branches of EDC2-5, EIP, EDM, LU_RB2-5 and UI_UB2-5;
- the ORL tendons;
- for option (b), the limited passive band tendons.

The branch divisors, the ORL coefficients and stiffness, and the recomputed `lengthrange` values are written by a generator, `scripts/fit_extensor_mechanism.py`, into delimited blocks (`<!-- BEGIN generated by scripts/fit_extensor_mechanism.py; do not edit by hand -->`). A test compares each block with the script's output.

**`myoarm_r_muscle.xml`:** only the regenerated `lengthrange` values change.

**`myo_sim/build/data/extensor_mechanism.json`** holds, one row per element:
- the source fractions (Lee 2008a r and c per posture; the Chao and An coefficients as used by Hu 2014);
- the ORL target (Ueba 2011);
- the stiffness bounds (Garcia-Elias 1991b, Qian 2014);
- the validation targets.

**`docs/wiki/extensor-mechanism.md`** is the method page.

Python changes nothing: `compose.py` attaches and mirrors the new elements like any other site, tendon or body. The left hand is mirrored, never edited. For option (b), `tests/test_mirror_symmetry.py` must also cover the slide-joint axes of the nodes.

### Recommendation

**1. Option (a) plus ORL variant 2 or 3 goes into the reference model** (policy 2). It is more realistic and costs 5-8 % per step on CPU in `myofullbody` (table below). It keeps the action and observation spaces, and it runs at the existing 2 ms step on CPU and mujoco-warp.

**2. Option (b) becomes a separately registered variant** (policy 4) after the pilot, and only if it beats option (a) on the validation below. It costs 30-60 % more per step on CPU and has about 35 % lower mujoco-warp throughput in `myofullbody`, and it adds 56 DOFs. The name is open, for example `myohands_extnet`.

**3. Index first.** The pilot implements option (a) and option (b) on the index finger of `myohand_r` only: EDC2, EIP, LU_RB2 and UI_UB2 are re-routed, and RI2 is unchanged. It runs the validation below against `dev` and the literature targets, and only then extends to digits 3-5. Digit 5 with EDM and digits 3-4 with their interosseous hood shares carry the most open questions.

## Validation

Each check has a target from the data file and a tolerance stated there.

1. **Force transmission (Lee 2008a protocol).**
   - Setup: lock the index at the nine postures and load EDC and EIP separately at 11.8 N; with option (b), let the nodes settle.
   - Output: terminal and central slip tensions.
   - Targets: r = 0.63 ± 0.10 with no IP-posture effect; c falling from about 0.72 with the finger extended to 0.25-0.31 with the IPs flexed (Lee et al. 2008a, Tables 3-5). Option (a) is constant here by construction, and option (b) has to earn its cost.
   - Joint-moment targets at the same postures: the ESMAs of Lee et al. (2008b) per tendon and DOF (Table 4) and their posture trends (Table 3: EDC MCP ESMA rising with IP flexion, FPI and LUM DIP ESMAs falling with it); the effective PIP moment arm of the communis, 2.3-5.5 mm with the MCP extended (Micks and Reswick 1981, Table I).
   - Intrinsic loading, compared with Valero-Cuevas et al. (2007): the slip ratio should reverse as the intrinsic-to-extensor ratio grows.
2. **PIP-DIP coupling in free flexion.**
   - Setup: drive the finger with FDP alone and with FDS alone, slowly and from extension.
   - Targets: the DIP-PIP trajectories of Leijnse et al. (2010) with FDP driving: slope dDIP/dPIP of 1.0 ± 0.31 at PIP 70 deg, 0.53 ± 0.28 at 100 deg and 0.25 ± 0.25 at 0 deg (p. 2386); and the passive ORL share against Ueba et al. (2011), under Ueba's protocol (1.96 N on the extrinsic extensor, DIP 0-45 deg).
   - ORL slack: in unloaded flexion, with the PIP-DIP differences of Harris and Rutledge (1972, Table I; at most 30 deg), the ORL carries no more than a small force.
   - Distal-phalanx release: with the PIP flexed 90 deg, the passive DIP resistance falls (Ueba et al. 2011: 1.5 ± 1.6 N against 3.1 ± 2.0 N at PIP 30 deg). Option (a) is expected to miss this (see its cons); option (b) should show it.
   - Full passive range: with all muscles off, every band force stays below a small bound over the joint ranges (the locking check above).
3. **Fingertip force.**
   - Setup: index posture of Valero-Cuevas et al. (1998); maximal forces in the five directions from an optimisation over the index muscles, as in `hand-function-validation.md`'s pinch.
   - Targets: direction and magnitude. Compare intrinsic-only, extrinsic-only and combined activation.
4. **Intrinsic-plus and claw.**
   - Intrinsic-plus: from a fist, intrinsics on and extrinsics off, PIP and DIP should extend while the MCP flexes.
   - Claw: intrinsics silenced and EDC on. This needs MCP hyperextension in the joint range, which `dev` does not have, so it depends on a separate joint-range change (Leijnse 2010 reports passive hyperextension up to almost 50 deg at the PIP; MCP data to be added).
5. **Tenodesis and the McFarland et al. (2023) hand-function tests.** `scripts/validate_hand_function.py` and `tests/test_hand_function.py` (`hand-function-validation.md`, separate change) must still pass:
   - tenodesis ROM, monotonicity and digit spread;
   - grip magnitude and digit shares;
   - lateral pinch (thumb, unaffected).

   The extrinsic extensor slack lengths and `fpmax` are refitted first.
6. **CPU against mujoco-warp.** Passive and actuator forces and one-step state agree within float32 tolerance for the new elements, as for the rest of the model (`model-authoring.md` check 4).
7. **Left-right symmetry.** `tests/test_mirror_symmetry.py` and the `debug_muscle_*.py` scripts cover the new sites, tendons and, for option (b), bodies and joints.

## Interaction with other work

**Elastic tendons.** MyoHub/myo_sim#124 (draft) adds series-elastic tendons by inserting a one-DOF slide joint, with its own body, between a muscle's tendon and its bony attachment. A compliant tendon can instead be a massless series element with the normalised fibre length as a state, as in Bolt (Wang et al., SIGGRAPH Asia 2026; github.com/willwng/bolt, Apache-2.0) and the usual Hill-type formulations (e.g. De Groote et al. 2016). Each step balances fibre and tendon force and applies the tendon force through the muscle's existing path Jacobian. This adds no DOF and no node mass, unlike the slide joint of #124, so neither its armature nor its fixed-axis approximation is needed. For this proposal:

- **Option (a) works with it.** A pulley-branched path is still one tendon with one length, so each muscle carries one lumped compliance over its whole branched path, and the branches share the tendon force by their divisors. The branches themselves stay inextensible.
- **Option (b) still needs node handling.** A series element serves one muscle on one path: it provides no tendon networks, no branching and no junction nodes. The bands, nodes, armature and damping of option (b) stay as described above, and the series element and the network compliance have to be identified together, not stacked.
- **A compliant muscle on a light node is a stiff spring on a small mass.** The node sees the tendon stiffness, of the order of F_max over the tendon stretch at F_max (strain at F_max times slack length): tens of kN/m for the hand intrinsics at a strain of about 5 %. Explicit Euler needs k < 4 m / dt², about 1 kN/m for a 1e-3 kg node and 10 kN/m for 1e-2 kg at 2 ms. Option (b) therefore needs node armature sized for this, or rigid treatment of short-tendon muscles (Bolt treats a tendon as rigid when its slack length is shorter than the optimal fibre length). A rigid muscle on a node keeps the force-velocity damping problem of requirement 2.

If the slack lengths and peak forces of the finger muscles are refitted once elastic tendons exist, that refit and the one this proposal needs for the extensors should be a single fit, because the extensor mechanism changes the extensors' paths.

**Explicit ligaments (MyoHub/myo_sim#142).** That proposal adds ligaments as passive spatial tendons with a Blankevoort force law mapped onto MuJoCo's tension-only dead band, generated by `scripts/fit_ligaments.py` from `build/data/ligaments.json`. The ORL is a ligament, and option (c) variant 3 should be one more strand in that framework: same force law, same data file, same `lig_` naming and pruning rules. It should not get a separate mechanism here.

The refit order then becomes:
1. ligaments, including the ORL;
2. the extensor-mechanism fractions;
3. the extrinsic slack-length and `fpmax` fit;
4. the residual joint springs.

#142 avoids tendon limits by default, because their force comes from `solref` rather than tissue data. Option (b) is the exception, for the numerical reason in requirement 2: a muscle inserting on a light node needs the implicit constraint coupling. Its `solref`/`solimp` would then be chosen to match the band stiffness of Garcia-Elias et al. (1991b) and Qian et al. (2014), and documented as such.

**Residual joint springs (MyoHub/myo_sim#139).** The PIP and DIP springs are fitted to the measured passive moment minus the muscles' passive moment (`model-authoring.md`, "fit the residual"). The ORL and the re-routed extensors change the muscle (and ligament) part of that moment, so the PIP and DIP springs have to be refitted on top of this change. The ORL's passive DIP share (Ueba 2011) should come out of the same fit, not be added afterwards. If the finger joint damping is lowered from 0.05 N m s/rad toward the 0.008-0.014 N m s/rad of Kamper et al. (2002), quoted in `hand-function-validation.md`, the node damping of option (b) becomes relatively larger, and its artefact has to be measured again.

## Prototype and cost

Built with MjSpec in a script on `dev` (not committed). Geometry is derived from the existing EDC, LU_RB, UI_UB and FDP sites; all values are placeholders, not identified parameters.

- **Option (a):** pulley branches with 50/25/25 for EDC/EIP and the Chao-An coefficients for LU_RB and UI_UB, plus the ORL as a fixed tendon.
- **Option (b):** five nodes and seven slides per finger, ten band tendons plus two ORL spatial tendons.

The default option (b) settings are the constraint bands above. "Springs" means option (b) with dead-band springs at 5e4 N/m and 500 N s/m of node damping.

**CPU:** one thread on an AMD Ryzen 9 9950X, MuJoCo 3.11.0, Euler at 2 ms, random muscle activations, best of three runs of 6000 steps.

| Model | Variant | Fingers | nv | Tendons | µs per step | vs base |
| --- | --- | --- | --- | --- | --- | --- |
| `myohand_r` | base | — | 23 | 39 | 35.1 | |
| `myohand_r` | (a) | index | 23 | 41 | 36.2 | +3 % |
| `myohand_r` | (b) | index | 30 | 51 | 37.5 | +7 % |
| `myohand_r` | (b) springs | index | 30 | 51 | 37.1 | +6 % |
| `myohand_r` | (a) | 4 | 23 | 47 | 38.0 | +8 % |
| `myohand_r` | (b) | 4 | 51 | 87 | 52.8 | +50 % |
| `myohands` | base | — | 46 | 78 | 63.4 | |
| `myohands` | (a) | 8 | 46 | 94 | 70.8 | +12 % |
| `myohands` | (b) | 8 | 102 | 174 | 94.5 | +49 % |
| `myofullbody` | base | — | 128 | 424 | 348 | |
| `myofullbody` | (a) | 8 | 128 | 440 | 364-375 | +5-8 % |
| `myofullbody` | (b) | 8 | 184 | 520 | 548-558 | +57-60 % |
| `myofullbody` | (b) springs | 8 | 184 | 520 | 452 | +30 % |

**Where the time goes.** In `myofullbody`, option (b)'s extra time is mostly in the constraint solver: 85 → 234 µs (137 µs with springs). The band limits add about 50 active constraint rows, and the solver works on 184 instead of 128 DOFs. Kinematics and tendons add about 30 µs.

**Extrapolation.** Scaling the index-only cost linearly underestimates the full cost: 8 × 2.4 µs predicts +20 µs, but +200 µs was measured. Any cost estimate for option (b) therefore has to be measured on the full model.

**mujoco-warp** (mujoco_warp with Warp 1.18, RTX 5090, 1024 worlds, CUDA graph of 10 steps). The GPU was shared with other jobs at over 90 % utilisation, so only the ratios are meaningful.

| Model | base | (a) | (b) | (b) springs |
| --- | --- | --- | --- | --- |
| `myohands` steps/s | 361k | 310k (-14 %) | 255k (-29 %) | 251k (-30 %) |
| `myofullbody` steps/s | 75.1k | 68.5k (-9 %) | 48.3k (-36 %) | 51.2k (-32 %) |

**Parity.** All variants load in mujoco-warp: pulley branches, dead-band springs, tendon limits and the junction slides are all supported. After one step from the same state:
- qpos agrees with CPU within 2e-7;
- tendon lengths agree within 4e-7 m;
- `qfrc_passive` agrees within 1e-4 relative (float32; the largest difference is in the stiff dead-band springs).

All runs stayed finite over the benchmark.

**Timestep.** Errors are index MCP/PIP/DIP RMS (max) over the deterministic phases of a 3 s protocol (extensors, fist, intrinsics, FDP alone), against the same model at 0.1 ms.

| Variant | 2 ms | 1 ms |
| --- | --- | --- |
| base | 0.6 (2.2) deg | 0.3 (0.9) deg |
| (a) | 0.5 (2.2) deg | 0.2 (0.9) deg |
| (b), constraint bands, node damping 50 | 1.7 (6.3) deg | 1.0 (5.0) deg |
| (b), dead-band springs 5e4 N/m, node damping 500 | 0.6 (2.2) deg, but LU_RB2 force chatters | 0.3 (1.1) deg |
| (b), dead-band springs 5e4 N/m, node damping 50 | unstable | 2.3 (8.9) deg |

**Timestep summary:**
- Option (a) changes nothing.
- Option (b) runs at the model's 2 ms step only with constraint bands. These need `solref` time constant 2 dt, `solimp` near 1, node armature 1e-3 kg and node damping 50 N s/m, and they give about twice the timestep error of the base model.
- Option (b) with elastic springs at the measured band stiffness needs 0.25-1 ms, or very heavy node damping (about 500 N s/m), and even then the intrinsic muscle forces chatter at 2 ms.
- `implicitfast` does not help in either case.

**Behaviour, uncalibrated.**
- *Intrinsic phase of the protocol* (MCP / PIP / DIP in deg):
  - `dev`: 91 / -2 / 90;
  - option (a): 92 / 90 / 27;
  - option (b): 91 / -1 / 25, close to the intrinsic-plus posture.
- *Force split at the Lee et al. (2008a) postures* (option (b), constraint bands, EDC2 at full activation):
  - terminal share r = 0.17-0.25 (target 0.63);
  - c ≈ 1 (target 0.39), because the prototype has no sagittal or capsular path.
- *Force split with LU_RB2 driving:* r = 0.62-0.65 with the IPs extended and 0.27-0.28 at PIP 45 / DIP 30.

The split is posture dependent, as it should be, but the geometry has to be identified against the data before any number means anything.

## Open questions

1. **Index pilot first?** Is an index-only pilot of both options, compared on the validation list, acceptable before any change to the reference model?
2. **Fractions for option (a).** Should they come from Lee et al. (2008a) (r = 0.63, c = 0.39, with c's posture dependence lost) or from the nominal 50/25/25 of Valero-Cuevas et al. (1998)? Lee's c also includes force lost to the joint capsule, the hood's connections to the phalanges and carpal-tunnel friction, and its mean of 0.39 includes the flexed postures. Our suggestion (see *Checking the fractions against joint moments*): keep r = 0.63 and fit c to the whole-tendon DIP and PIP moment arms, on which An et al. (1983) and Lee et al. (2008b) agree. This gives c of about 0.75-1.0, because c = 0.39 gives about half the measured IP extension moment.
3. **ORL: fixed or spatial?** Fixed tendon (constant moment arms, cheapest) or spatial tendon (posture-dependent arms, needs a wrap)? Its stiffness and slack length are fitted to the shares of Ueba et al. (2011), with the slack range of Harris and Rutledge (1972) as a constraint. The two disagree on when the ORL tightens (see *PIP-DIP coupling*); if both cannot be met, which should win?
4. **Interosseous hood shares** of digits 3-5, and EDM's share at the little finger. Salsbury's (1937) expansion shares are a starting point. In today's bundle names:
   - RI3 (DI2) 0.48;
   - UI_UB3 (DI3) 0.93;
   - UI_UB4 (DI4) 0.66;
   - RI4 (PI2) 1.0;
   - RI5 (PI3) 0.89;
   - UI_UB2 (PI1) 0.99;
   - RI2 (DI1) 0.

   Should the bundles take these values?
5. **MCP hyperextension.** A range change is needed for the claw test. Is it in scope here or a separate PR?
6. **Option (b) identification.** Node placement and slide directions should be identified from the 3-D geometry of Garcia-Elias et al. (1991a). The in-vivo band shift of Canoso et al. (2021), 1-2 mm at mid-proximal phalanx from PIP 0 to 90 deg, is a check on the band nodes' slide range. What tolerance on passive band force over the full range should count as "admits the range of motion"?
7. **Elastic tendons.** Should compliant tendons (#124's slide joints or a massless series element) land before or after this change? The slack-length refit of the extensors should happen once, after both.
8. **Variant name and registration** for option (b), if it is pursued.
9. **Effective moment arms.** Lee et al. (2008b) report effective moment arms smaller than kinematic estimates. We suggest using them (Table 4) as the fit targets for option (a) and as validation targets for option (b), with the kinematic moment arms of the paths reported next to them.

## References

- An KN, Chao EY, Cooney WP, Linscheid RL (1979). Normative model of human hand for biomechanical analysis. J Biomech 12(10):775-788. doi:10.1016/0021-9290(79)90163-5
- An KN, Ueba Y, Chao EY, Cooney WP, Linscheid RL (1983). Tendon excursion and moment arm of index finger muscles. J Biomech 16(6):419-425. doi:10.1016/0021-9290(83)90074-X
- Brook N, Mizrahi J, Shoham M, Dayan J (1995). A biomechanical model of index finger dynamics. Med Eng Phys 17(1):54-63. doi:10.1016/1350-4533(95)90378-O
- Canoso JJ, Naredo E, Martínez-Estupiñán L, Mérida-Velasco JR, Pascual-Ramos V, Murillo-González J (2021). Palpation of the lateral bands of the extensor apparatus of the fingers. J Anat 239(3):663-668. doi:10.1111/joa.13446
- Chao EY, Opgrande JD, Axmear FE (1976). Three-dimensional force analysis of finger joints in selected isometric hand functions. J Biomech 9(6):387-396. doi:10.1016/0021-9290(76)90116-0
- Chao EY, An KN (1978). Graphical interpretation of the solution to the redundant problem in biomechanics. J Biomech Eng 100:159-167. doi:10.1115/1.3426207
- Dogadov A, Alamir M, Servière C, Quaine F (2017). The biomechanical model of the long finger extensor mechanism and its parametric identification. J Biomech 58:232-236. doi:10.1016/j.jbiomech.2017.04.030
- Dogadov AA, Valero-Cuevas FJ, Servière C, Quaine F (2025). The bundles of intercrossing fibers of the extensor mechanism of the fingers greatly influence the transmission of muscle forces. Int J Numer Methods Biomed Eng 41(7):e70068. doi:10.1002/cnm.70068
- Engelhardt L, Melzner M, Havelkova L, Fiala P, Christen P, Dendorfer S, Simon U (2021). A new musculoskeletal AnyBody detailed hand model. Comput Methods Biomech Biomed Engin 24(7):777-787. doi:10.1080/10255842.2020.1851367
- Garcia-Elias M, An KN, Berglund L, Linscheid RL, Cooney WP, Chao EYS (1991a). Extensor mechanism of the fingers. I. A quantitative geometric study. J Hand Surg Am 16(6):1130-1136. doi:10.1016/S0363-5023(10)80079-6
- Garcia-Elias M, An KN, Berglund LJ, Linscheid RL, Cooney WP, Chao EYS (1991b). Extensor mechanism of the fingers. II. Tensile properties of components. J Hand Surg Am 16(6):1136-1140. doi:10.1016/S0363-5023(10)80080-2
- Harris C, Rutledge GL (1972). The functional anatomy of the extensor mechanism of the finger. J Bone Joint Surg Am 54(4):713-726. doi:10.2106/00004623-197254040-00003
- Holzbaur KRS, Murray WM, Delp SL (2005). A model of the upper extremity for simulating musculoskeletal surgery and analyzing neuromuscular control. Ann Biomed Eng 33(6):829-840. doi:10.1007/s10439-005-3320-7
- Hu D, Howard D, Ren L (2014). Biomechanical analysis of the human finger extensor mechanism during isometric pressing. PLoS One 9(4):e94533. doi:10.1371/journal.pone.0094533
- Jadelis CT, Ellis BJ, Kamper DG, Saul KR (2023). Cosimulation of the index finger extensor apparatus with finite element and musculoskeletal models. J Biomech 157:111725. doi:10.1016/j.jbiomech.2023.111725
- Landsmeer JMF (1949). The anatomy of the dorsal aponeurosis of the human finger and its functional significance. Anat Rec 104(1):31-44. doi:10.1002/ar.1091040105
- Landsmeer JMF (1958). A report on the coordination of the interphalangeal joints of the human finger and its disturbances. Acta Morphol Neerl Scand 2:59-84.
- Landsmeer JMF (1963). The coordination of finger-joint motions. J Bone Joint Surg Am 45:1654-1662.
- Lee SW, Chen H, Towles JD, Kamper DG (2008a). Effect of finger posture on the tendon force distribution within the finger extensor mechanism. J Biomech Eng 130(5):051014. doi:10.1115/1.2978983
- Lee SW, Chen H, Towles JD, Kamper DG (2008b). Estimation of the effective static moment arms of the tendons in the index finger extensor mechanism. J Biomech 41(7):1567-1573. doi:10.1016/j.jbiomech.2008.02.008
- Leijnse JNAL, Quesada PM, Spoor CW (2010). Kinematic evaluation of the finger's interphalangeal joints coupling mechanism: variability, flexion-extension differences, triggers, locking swanneck deformities, anthropometric correlations. J Biomech 43(12):2381-2393. doi:10.1016/j.jbiomech.2010.04.021
- Leijnse JNAL, Spoor CW (2012). Reverse engineering finger extensor apparatus morphology from measured coupled interphalangeal joint angle trajectories: a generic 2D kinematic model. J Biomech 45(3):569-578. doi:10.1016/j.jbiomech.2011.11.002
- MacIntosh AR, Keir PJ (2017). An open-source model and solution method to predict co-contraction in the finger. Comput Methods Biomech Biomed Engin 20(13):1373-1381. doi:10.1080/10255842.2017.1364732
- McFarland DC, Binder-Markey BI, Nichols JA, Wohlman SJ, de Bruin M, Murray WM (2023). A musculoskeletal model of the hand and wrist capable of simulating functional tasks. IEEE Trans Biomed Eng 70(5):1424-1435. doi:10.1109/TBME.2022.3217722
- Micks JE, Reswick JB (1981). Confirmation of differential loading of lateral and central fibers of the extensor tendon. J Hand Surg Am 6(5):462-467. doi:10.1016/S0363-5023(81)80104-9
- Mirakhorlo M, Van Beek N, Wesseling M, Maas H, Veeger HEJ, Jonkers I (2018). A musculoskeletal model of the hand and wrist: model definition and evaluation. Comput Methods Biomech Biomed Engin 21(9):548-557. doi:10.1080/10255842.2018.1490952
- Qian K, Traylor K, Lee SW, Ellis B, Weiss J, Kamper D (2014). Mechanical properties vary for different regions of the finger extensor apparatus. J Biomech 47(12):3094-3099. doi:10.1016/j.jbiomech.2014.06.035
- Qiu D, Kamper DG (2014). Orthopaedic applications of a validated force-based biomechanical model of the index finger. Proc 36th Annu Int Conf IEEE Eng Med Biol Soc, pp. 4013-4016. doi:10.1109/EMBC.2014.6944504
- Sachdeva P, Sueda S, Bradley S, Fain M, Pai DK (2015). Biomechanical simulation and control of hands and tendinous systems. ACM Trans Graph 34(4):42. doi:10.1145/2766987
- Salsbury CR (1937). The interosseous muscles of the hand. J Anat 71(Pt 3):395-403.
- Sancho-Bru JL, Pérez-González A, Vergara-Monedero M, Giurintano D (2001). A 3-D dynamic model of human finger for studying free movements. J Biomech 34(11):1491-1500. doi:10.1016/S0021-9290(01)00106-3
- Saul KR, Hu X, Goehler CM, Vidt ME, Daly M, Velisar A, Murray WM (2015). Benchmarking of dynamic simulation predictions in two software platforms using an upper limb musculoskeletal model. Comput Methods Biomech Biomed Engin 18(13):1445-1458. doi:10.1080/10255842.2014.916698
- Saxena A, Lipson H, Valero-Cuevas FJ (2012). Functional inference of complex anatomical tendinous networks at a macroscopic scale via sparse experimentation. PLoS Comput Biol 8(11):e1002751. doi:10.1371/journal.pcbi.1002751
- Schultz RJ, Furlong J, Storace A (1981). Detailed anatomy of the extensor mechanism at the proximal aspect of the finger. J Hand Surg Am 6(5):493-498. doi:10.1016/S0363-5023(81)80110-4
- Sueda S, Kaufman A, Pai DK (2008). Musculotendon simulation for hand animation. ACM Trans Graph 27(3). doi:10.1145/1360612.1360682
- Synek A, Pahr DH (2016). The effect of the extensor mechanism on maximum isometric fingertip forces: a numerical study on the index finger. J Biomech 49(14):3423-3429. doi:10.1016/j.jbiomech.2016.09.004
- Synek A, Lu SC, Vereecke EE, Nauwelaerts S, Kivell TL, Pahr DH (2019). Musculoskeletal models of a human and bonobo finger: parameter identification and comparison to in vitro experiments. PeerJ 7:e7470. doi:10.7717/peerj.7470
- Ueba H, Moradi N, Erne HC, Gardner TR, Strauch RJ (2011). An anatomic and biomechanical study of the oblique retinacular ligament and its role in finger extension. J Hand Surg Am 36(12):1959-1964. doi:10.1016/j.jhsa.2011.09.033
- Valero-Cuevas FJ, Zajac FE, Burgar CG (1998). Large index-fingertip forces are produced by subject-independent patterns of muscle excitation. J Biomech 31(8):693-703. doi:10.1016/S0021-9290(98)00082-7
- Valero-Cuevas FJ, Yi JW, Brown D, McNamara RV, Paul C, Lipson H (2007). The tendon network of the fingers performs anatomical computation at a macroscopic scale. IEEE Trans Biomed Eng 54(6):1161-1166. doi:10.1109/TBME.2006.889200
