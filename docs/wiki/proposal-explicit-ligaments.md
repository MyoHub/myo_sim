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
  - MCP: the collaterals are slack in extension and taut in flexion (Minami et al. 1984),
    which couples abduction to flexion.
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
  - Index MCP flexed from 0 to 80 deg:
    - the dorsal portions of both collaterals lengthened by 3-4 mm;
    - the middle portions by 0.4-1.0 mm;
    - the volar portions shortened by 1-2 mm.
  - In hyperextension the dorsal portions shortened by 2-3 mm.
  - Implication: one strand per collateral cannot reproduce this cam behaviour. Use at
    least a dorsal and a volar strand, preferably three. The origin and insertion
    coordinates are to be confirmed from the paper.
- **MCP collaterals: sectioning.** Minami A et al. (1985). Ligament stability of the
  metacarpophalangeal joint: a biomechanical study. J Hand Surg Am 10(2):255-260.
  doi:10.1016/s0363-5023(85)80117-9.
  - The proper collaterals are the primary stabilizers in all directions.
  - The accessory collaterals act mainly in ab/adduction.
  - The palmar plate prevents dorsal dislocation only at full extension.
- **Thumb MCP UCL elongation.** Sircar K, Hägermann C, Scaal M, Müller LP, Dargel J,
  Wegmann K (2024). Thumb metacarpophalangeal joint kinematics and elongation of the
  ulnar collateral ligament. J Hand Surg Am 49(1):8-14. doi:10.1016/j.jhsa.2023.10.001.
  Four UCL portions measured over 0-60 deg flexion; maximum elongation is 111-117 % of
  neutral length, depending on portion and joint shape.
- **PIP collaterals and volar plate**
  - Allison DM (2005). Anatomy of the collateral ligaments of the proximal
    interphalangeal joint. J Hand Surg Am 30(5):1026-1031.
    doi:10.1016/j.jhsa.2005.05.015.
    - Proper collateral fibres run parallel to the middle phalanx in all joint positions.
    - The accessory collateral lies between the proper collateral and the volar plate.
  - Bowers WH, Wolf JW, Nehil JL, Bittinger S (1980). The proximal interphalangeal joint
    volar plate. I. An anatomical and biomechanical study. J Hand Surg Am 5(1):79-88.
    doi:10.1016/s0363-5023(80)80049-9.
  - Rhee RY, Reading G, Wray RC (1992). A biomechanical study of the collateral
    ligaments of the proximal interphalangeal joint. J Hand Surg Am 17(1):157-163.
    doi:10.1016/0363-5023(92)90134-b. The lateral collateral is the primary restraint to
    medial-lateral stress. Failure loads by loading rate are to be confirmed from the
    paper.
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
    - Seven principal ligaments:
      - three dorsal (dorsal radial, dorsal central, posterior oblique);
      - two volar (anterior oblique, ulnar collateral);
      - two ulnar (dorsal trapeziometacarpal, intermetacarpal).
    - The dorsal ligaments are thicker.
    - The anterior oblique ligament is thin, variable, and closer to capsule than to a
      proper ligament.
  - Hagert E, Lee J, Ladd AL (2012). Innervation patterns of thumb trapeziometacarpal
    joint ligaments. J Hand Surg Am 37(4):706-714.e1. doi:10.1016/j.jhsa.2011.12.038.
- **Thumb CMC mechanics**
  - Bettinger PC, Smutz WP, Linscheid RL, Cooney WP, An KN (2000). Material properties
    of the trapezial and trapeziometacarpal ligaments. J Hand Surg Am 25(6):1085-1095.
    doi:10.1053/jhsu.2000.18487.
    - Bone-ligament-bone tests of seven ligaments.
    - The dorsoradial ligament has the greatest ultimate load.
    - The anterior oblique ligament has the least stiffness.
    - The stiffness and ultimate-load values are to be confirmed from the paper.
  - D'Agostino P, Kerkhof FD, Shahabpour M, Moermans JP, Stockmans F, Vereecke EE
    (2014). Comparison of the anatomical dimensions and mechanical properties of the
    dorsoradial and anterior oblique ligaments of the trapeziometacarpal joint. J Hand
    Surg Am 39(6):1098-1107. doi:10.1016/j.jhsa.2014.02.025. The DRL is shorter, thicker
    and stiffer than the AOL; values are to be confirmed from the paper.
  - Colman M, Mass DP, Draganich LF (2007). Effects of the deep anterior oblique and
    dorsoradial ligaments on trapeziometacarpal joint stability. J Hand Surg Am
    32(3):310-317. doi:10.1016/j.jhsa.2006.12.002. In lateral pinch, mean 3-D translation
    increased by 1.3 mm after DRL transection and by 0.6 mm after dAOL transection.
  - Najima H, Oberlin C, Alnot JY, Cadot B (1997). J Hand Surg Br 22(2):183-188.
    doi:10.1016/s0266-7681(97)80058-7.
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
    45(15):2531-2538. doi:10.1016/j.jbiomech.2012.07.034. The muscle-tendon units do
    less than 50 % of the passive work, and the capsule-ligament complex dominates the
    joint stiffness throughout the flexion-extension range.
  - Knutson JS, Kilgore KL, Mansour JM, Crago PE (2000). Intrinsic and extrinsic
    contributions to the passive moment at the metacarpophalangeal joint. J Biomech
    33(12):1675-1681. doi:10.1016/s0021-9290(00)00159-7.
  - These two papers are the closest thing to a published partition. They suggest that
    at the MCP, ligaments may legitimately take most of the residual.
- **The sources the residual springs use**
  - Formica D, Charles SK, Zollo L, Guglielmelli E, Hogan N, Krebs HI (2012). The
    passive stiffness of the wrist and forearm. J Neurophysiol 108(4):1158-1166.
    doi:10.1152/jn.01014.2011.
  - Domalain M, Vigouroux L, Berton E (2010). Determination of passive moment-angle
    relationships at the trapeziometacarpal joint. J Biomech Eng 132(7):071009.
    doi:10.1115/1.4001397.
  - Kamper DG, Hornby TG, Rymer WZ (2002). Extrinsic flexor muscles generate concurrent
    flexion of all three finger joints. J Biomech 35(12):1581-1589.
    doi:10.1016/s0021-9290(02)00229-4.
  - Esteki A, Mansour JM (1996). An experimentally based nonlinear viscoelastic model of
    joint passive moment. J Biomech 29(4):443-450. doi:10.1016/0021-9290(95)00081-x.
- **Finger models with ligaments.** Sancho-Bru JL, Pérez-González A, Vergara-Monedero M,
  Giurintano D (2001). A 3-D dynamic model of human finger for studying free movements.
  J Biomech 34(11):1491-1500. doi:10.1016/s0021-9290(01)00106-3. Includes the passive
  action of the MCP ligaments, which is needed to reproduce ab/adduction.

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
    doi:10.1007/s10439-011-0361-y. A rigid-body wrist with ligaments as linear springs.
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
  - Spratley EM, Wayne JS (2011). Computational model of the human elbow and forearm:
    application to complex varus instability. Ann Biomed Eng 39(3):1084-1091.
    doi:10.1007/s10439-010-0224-y.
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
  - Blankevoort L, Kuiper JH, Huiskes R, Grootenboer HJ (1991). Articular contact in a
    three-dimensional model of the knee. J Biomech 24(11):1019-1031.
    doi:10.1016/0021-9290(91)90019-j. Ligaments as non-linear elastic line elements.
  - Blankevoort L, Huiskes R (1991). Ligament-bone interaction in a three-dimensional
    model of the knee. J Biomech Eng 113(3):263-269. doi:10.1115/1.2894883. Wrapping
    the MCL over the tibial edge changes how it resists valgus moments. Its effect on
    internal/external rotation laxity is negligible.
  - Shelburne KB, Pandy MG (1997). A musculoskeletal model of the knee for evaluating
    ligament forces during isometric contractions. J Biomech 30(2):163-176.
    doi:10.1016/s0021-9290(96)00119-4.
  - Shelburne KB, Pandy MG, Anderson FC, Torry MR (2004). Pattern of anterior cruciate
    ligament force in normal walking. J Biomech 37(6):797-805.
    doi:10.1016/j.jbiomech.2003.10.010. Peak ACL force 303 N.
- **Lenhart et al. 2015.** Lenhart RL, Kaiser J, Smith CR, Thelen DG. Prediction and
  validation of load-dependent behavior of the tibiofemoral and patellofemoral joints
  during movement. Ann Biomed Eng 43(11):2675-2685. doi:10.1007/s10439-015-1326-3. An
  OpenSim knee with 14 multi-strand ligament bundles:
  - non-linear toe region below 6 % strain;
  - linear stiffness = cross-sectional area × 125 MPa;
  - reference strains from the literature;
  - Monte Carlo spread: 30 % in stiffness, 0.02 in reference strain;
  - wrap objects for the MCL and the posterior capsule.
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
- **Other knee models**
  - Marra MA et al. (2015). A subject-specific musculoskeletal modeling framework to
    predict in vivo mechanics of total knee arthroplasty. J Biomech Eng 137(2):020904.
    doi:10.1115/1.4029258.
  - Bloemker KH, Guess TM, Maletsky L, Dodd K (2012). Computational knee ligament
    modeling using experimentally determined zero-load lengths. Open Biomed Eng J
    6:33-41. doi:10.2174/1874230001206010033.
    - Stiffness per bundle (N per unit strain): ACL 5000, PCL 9000, LCL 2000, MCL 2750.
    - Kinematics are highly sensitive to zero-load length.
  - Kia M et al. (2016). J Biomech Eng 138(5):051010. doi:10.1115/1.4032850.
  - Guess TM, Razu S, Jahandar H (2016). J Knee Surg 29(2):126-137.
    doi:10.1055/s-0036-1571954.
- **Tissue properties**
  - Woo SL, Hollis JM, Adams DJ, Lyon RM, Takai S (1991). Tensile properties of the
    human femur-anterior cruciate ligament-tibia complex. Am J Sports Med
    19(3):217-225. doi:10.1177/036354659101900303. Young specimens: 242 ± 28 N/mm
    linear stiffness and 2160 ± 157 N ultimate load.
  - Noyes FR, Grood ES (1976). J Bone Joint Surg Am 58(8):1074-1082.
    doi:10.2106/00004623-197658080-00006.
  - Trent PS, Walker PS, Wolf B (1976). Clin Orthop Relat Res (117):263-270.
    doi:10.1097/00003086-197606000-00034.
  - Butler DL, Kay MD, Stouffer DC (1986). J Biomech 19(6):425-432.
    doi:10.1016/0021-9290(86)90019-9.
  - Chandrashekar N, Mansouri H, Slauterbeck J, Hashemi J (2006). J Biomech
    39(16):2943-2950. doi:10.1016/j.jbiomech.2005.10.031.

### Shoulder

- **Tissue properties**
  - Bigliani LU et al. (1992). Tensile properties of the inferior glenohumeral ligament.
    J Orthop Res 10(2):187-197. doi:10.1002/jor.1100100205. Strain to failure averages
    27 % for the whole specimen and 11 % for the midsubstance.
  - Boardman ND et al. (1996). Tensile properties of the superior glenohumeral and
    coracohumeral ligaments. J Shoulder Elbow Surg 5(4):249-254.
    doi:10.1016/s1058-2746(96)80050-4. Stiffness: coracohumeral 36.7 ± 5.9 N/mm,
    superior glenohumeral 17.4 ± 1.5 N/mm.
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
      wrapped on the humeral head.
    - Stiffness 320-1099 N per unit strain.
  - Dalman MJ, Saul KR (2026). Assessing sensitivity of predictive shoulder simulations
    to uncertain soft tissue properties. J Biomech 195:113114.
    doi:10.1016/j.jbiomech.2025.113114.
    - OpenSim, Blankevoort ligaments, stiffness 600-1500 N per unit strain.
    - Slack lengths were chosen so that the ligaments engage at 30 deg abduction.
    - Slack length was the most influential parameter.

### Ligament mechanics and precedent

- **Reviews**
  - Weiss JA, Gardiner JC (2001). Computational modeling of ligament mechanics. Crit Rev
    Biomed Eng 29(3):303-371. doi:10.1615/critrevbiomedeng.v29.i3.20.
  - Butler DL, Grood ES, Noyes FR, Zernicke RF (1978). Biomechanics of ligaments and
    tendons. Exerc Sport Sci Rev 6:125-181. doi:10.1249/00003677-197800060-00005.
- **OpenSim.**
  - `Blankevoort1991Ligament` is the modern ligament element:
    - quadratic toe region up to a transition strain (default 0.06), linear above;
    - damping c·ε̇, applied only while stretching (default c = 0.003 per unit strain per
      second).
  - The older `Ligament` class uses a normalised spline scaled by a force constant.
- **Notation trap.** Several papers write the transition as ε_l = 0.03, with the toe
  region ending at 2ε_l. That is the same curve as a transition strain of 0.06.
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
- No in-vivo length or strain measurement of the thumb CMC ligaments was found.

Finger ligament stiffness would therefore be transferred: either from cross-sectional
area times a modulus, as Lenhart et al. do, or from related ligaments. Those rows would
carry the status `transferred`.

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
- **Thumb CMC.** Six ligaments, following the consensus set of Ladd et al. 2012:
  - dorsoradial;
  - dorsal central;
  - posterior oblique;
  - anterior oblique;
  - ulnar collateral;
  - intermetacarpal (first to second metacarpal base).

That is 24 + 16 + 6 = 46 strands per hand.

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
- **Elbow collaterals.** Only with a varus-valgus DOF.
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

Strain is ε = (L - L0) / L0. The force law is the Blankevoort / Wismans curve used by
OpenSim's `Blankevoort1991Ligament`, with stiffness k in newtons per unit strain and
transition strain ε_t (0.06 by default):

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
     1.0 mm, volar -1 to -2 mm from 0 to 80 deg; dorsal -2 to -3 mm in hyperextension);
   - thumb MCP UCL elongation vs Sircar et al. 2024 (if the thumb MP is added later);
   - near-isometric PIP proper collaterals per Allison 2005.

   The tolerances are stated in the JSON row.
2. **Total passive moment.** Muscles + ligaments + residual springs reproduce the source
   curves at least as well as #139's table (RMS per row). A test asserts that the ligament
   moment never exceeds the residual inside each measured span.
3. **End-feel and laxity.**
   - Moment-angle sweeps at the end of range in each DOF.
   - MCP abduction laxity at 0 vs 90 deg flexion; the abduction range should shrink
     with flexion.
   - CMC laxity in flexion-abduction combinations.
   - No new posture where the passive moment changes sign unexpectedly.

   Translational laxity data (e.g. Colman et al. 2007) do not apply while the CMC cannot
   translate.
4. **Ligament forces.** Where published, compare against them.
   - In the hand, the published values are failure loads (e.g. 162.5 N for the PIP
     collateral), and they serve as upper bounds in the grip and pinch tests.
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
ranges, with k = 5·10⁴ N/m. The full prototype is a larger set than the 46-strand pilot,
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
(coracohumeral 36.7 N/mm, superior glenohumeral 17.4 N/mm) to 242 N/mm for a young ACL,
i.e. about 10⁴ to 2.4·10⁵ N/m.

**What the stability numbers say.**
- **Current `dev` inertias.** Hand ligaments in that range are stable with a large
  margin.
- **Realistic phalanx inertia.** If the inertia floor and the 1e-4 armature are ever
  reduced to physical values (cf. issue #128), the margin for the stiffest hand strands
  falls to about 1-3×. The analytic bound 4 / (h² · invweight) predicts the empirical
  limit to within a factor of about 2. It would be a test on every strand.
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

   Each option is a `transferred` row with the assumption stated. The CMC stiffness
   values are to be confirmed from Bettinger et al. 2000 and D'Agostino et al. 2014.
3. **Reference strains.** We have no in-vivo strain data for the CMC ligaments. Do we
   set the reference strain from the length-angle data (MCP) and from a "taut at the end
   of range" assumption (CMC)? Slack length is the most influential parameter in every
   published sensitivity study.
4. **How much of the residual the ligaments may take.** Kuo and Deshpande 2012 suggest
   the capsule-ligament complex dominates MCP stiffness. Do we let ligaments take most of
   the MCP residual, or cap them at a fixed fraction until there is better data?
5. **Wrapping surfaces.** MuJoCo wraps only spheres and cylinders. Are three strands per
   collateral on a cylinder enough to reproduce the MCP cam effect, or do we need more
   strands?
6. **Inertia and stability.** The stability margin depends on the phalanx inertia floor
   and armature. Should a ligament PR wait for, or be coordinated with, the inertia work
   (#128)?
7. **Policy text.** The authoring guide and role table (#140) need the passive
   spatial-ligament row. Should that go into #140 or into the first ligament PR?
8. **Knee and shoulder.** Do we want the knee ligaments at all while the knee is a
   one-DOF coupled joint? Is a strand-based glenohumeral capsule acceptable given the
   Moore et al. 2010 result?
