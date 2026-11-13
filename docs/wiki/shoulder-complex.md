# Shoulder complex (scapulothoracic)

Work in progress on `feat/scapulothoracic`. Before it, the arm models used the MoBL-ARMS shoulder: the sternoclavicular,
acromioclavicular and "unrotate" joints followed `shoulder_elv_r` through linear joint equalities, and there were no
scapular muscles. This page records the port of the thoracoscapular shoulder of Seth et al. (2019) step by step.

## Source and licence

Seth A, Dong M, Matias R, Delp SL. Muscle contributions to upper-extremity movement and work from a musculoskeletal
model of the human shoulder. Front Neurorobot 13:90, 2019 (doi:10.3389/fnbot.2019.00090). Materials: SimTK project
`thoracoscapular`, release FrontiersPubMaterials, under CC BY 4.0. The release asks users to cite the paper and to
share improvements. `myo_sim/build/data/LICENSE-Seth2019.txt` holds the attribution and the list of changes;
`myo_sim/build/data/scapulothoracic.json` holds the derived values with their provenance. No file of the release is
committed; `scripts/fit_scapulothoracic.py` reads the published archive directly.

## Frame registration

`scripts/fit_scapulothoracic.py --zip <archive> register --write`.

| Item | Result |
|---|---|
| Clavicle, scapula, humerus meshes | identical to this package's, in identical body frames (203, 770, 309 vertices; 0.00 mm) |
| Sternoclavicular centre, glenohumeral centre, elbow offset | same values as MoBL-ARMS and this package |
| Sternoclavicular axes (`clav_prot`, `clav_elev`) | equal to `sternoclavicular_r2_r` and `sternoclavicular_r3_r` |
| Sternoclavicular centre in `chest_r` | this package 2.01 mm anterior of Seth's thorax frame |
| Thorax mesh | Seth's thorax points lie on this package's ribcage mesh (mean 0.06 mm) after a translation of (6.0, -5.0, 0.0) mm, 7.8 mm in all |

Seth's thorax-frame values (thoracic ellipsoid, thorax muscle points) are placed in `chest_r` by the sternoclavicular
shift, so the girdle keeps Seth's geometry relative to the clavicle and scapula. Relative to the ribcage mesh they are
then 7.8 mm off, mostly posterior and superior; the ellipsoid is a kinematic surface (below), so this offset affects only
the drawing and the thorax attachment points, which are checked against the bone surface when the muscles are added.

Seth's ellipsoid (radii 83.0 x 200.0 x 83.0 mm) carries a point of the scapula that lies about 9 mm in front of the
infraspinous fossa, not the bone itself. Along Seth's measured motions the inferior angle stays 26 to 48 mm outside the
ellipsoid. Seth's joint, checked against OpenSim 4.6 to machine precision, is
R = Ry(a) Rx(-e) Rz(u) Ry(w) with the point (rx sin a cos e, ry sin e, rz cos a cos e) on the ellipsoid, for
abduction a, elevation e, upward rotation u and winging w.

## Joint design: A against B

`scripts/fit_scapulothoracic.py --zip <archive> compare` replays all 18 inverse-kinematics trials of the release
(flexion, abduction and shrug, two subjects' sessions, 4312 frames at every third frame) in two candidate girdles built
in Seth's frames with a humerus and a lumped forearm. The IK coordinates belong to Seth's scaled subject model; they are
projected onto the generic model by the least change of the girdle coordinates that closes the AC constraint (as
OpenSim's assembler does; up to 13.8 deg, AC residual below 1e-9 mm).

- **A**: sternoclavicular hinges, clavicle, acromioclavicular ball joint at Seth's AC point, scapula. Scapula-thorax
  gliding is a frictionless contact between a 3 mm sphere at Seth's ellipsoid point and an ellipsoid geom with Seth's
  radii less 3 mm. The scapula can lift off the thorax but not sink into it.
- **B**: Seth's parameterisation as a chain on the thorax (two hinges at the ellipsoid centre, two slides tied to the
  elevation by quartic joint equalities, upward rotation and winging hinges), with the AC point closed by a `connect`
  equality to the clavicle.

Dynamic replay: joint springs pull every coordinate to the trajectory (500 N m/rad; clavicle 50 or 500 N m/rad) under
gravity, with a 20 N press of the scapula onto the thorax at the ellipsoid point; solref 0.008 for the contact (A) and
the connect (B); timestep 2 ms, Euler.

| | A | B |
|---|---|---|
| Kinematic replay, scapula pose error | 0 mm, 0 deg (exact) | 0.048 mm max (slide polynomials) |
| Kinematic replay, AC gap | 0 (tree joint) | 0.048 mm max |
| Dynamic, clavicle 50 N m/rad: AC gap p50 / p95 / max | 0 / 0 / 0 mm | 1.5 / 7.9 / 27.6 mm |
| Dynamic, clavicle 50: scapula position error p50 / p95 / max | 10.8 / 19.5 / 26.1 mm | 10.3 / 17.7 / 65.9 mm |
| Dynamic, clavicle 500: scapula position error p50 / p95 / max | 1.8 / 3.4 / 10.0 mm | unstable |
| Contact penetration p50 / p95 / max (clavicle 50) | 0.33 / 1.14 / 2.39 mm | - |
| Scapula lifted off by more than 0.5 mm (clavicle 50 / 500) | 9 % / 31 % of samples | - |
| Step time of the girdle prototype (CPU) | 5.4-6.3 us | 3.7-3.8 us |
| mujoco-warp (CPU device) against MuJoCo | contact distance equal to 1e-7 mm; 200 steps qpos within 2.9e-4 | 200 steps qpos within 1.6e-5 |
| MuJoCo 3.17 against 4 | identical results | identical results |

For scale, `myoarms` takes 182 us per step today. The position errors of the dynamic replay are mostly the sag of the
springs under the arm's weight and are similar in both designs; the AC gap is not. B is a closed loop whose `connect`
opens by millimetres to centimetres when the clavicle and the scapula are pulled differently, and with a stiff clavicle the
light clavicle in the loop made the 2 ms Euler step unstable. A keeps the AC joint exact; its only soft constraint is the
unilateral contact (penetration below 2.4 mm). A has one degree of freedom more than Seth's joint: the scapula can turn
about the line from the AC joint to the contact point (anterior and posterior tilt), which the scapular muscles control,
and it can lift off the thorax when nothing presses it on. Whether the scapula stays on the thorax at rest is decided by
the muscles' passive forces and is tested once they are in.

**Choice: A.** A support contact for the inferior angle on a larger ellipsoid fitted to its path (rms 2.0 mm) was
also tried; placed as the lower envelope of Seth's data it barely touched and changed nothing, and it is not used.

## Glenohumeral joint

The glenohumeral joint keeps three rotations and no translation, as in Seth's model; its coordinates become relative to
the scapula (the MoBL-ARMS "unrotate" bodies and the rhythm equalities go). The joint centre and frame stay defined on the
scapula, with a named site for the glenoid centre and its orientation and a named site for the humeral head centre, and
no scapular joint depends on a glenohumeral coordinate. Glenohumeral translation, with the capsule and ligaments and a
glenoid contact, is a later step and can be added without changing the scapula.

## The girdle in the model

`scripts/fit_scapulothoracic.py --zip <archive> girdle --write` (table section `girdle`).

- **Joints per side:** `sternoclavicular_r2_r` and `sternoclavicular_r3_r` (Seth's and MoBL-ARMS's axes and angles),
  the clavicle's axial rotation `sternoclavicular_r1_r` (below), the ball joint `acromioclavicular_r` at Seth's AC point, and the glenohumeral chain `elv_angle_r`, `shoulder_elv_r`
  (with the `shoulder1_r2_r` convention coupling) and `shoulder_rot_r`. Contact pair `scapulothoracic_r`: the 3 mm
  sphere `scapula_glide_r` at Seth's ellipsoid point against `scapulothoracic_ellipsoid_r` on `chest_r` (condim 1).
- **qpos0 is the rest posture** of Seth's trials (mean first frame of the 18 trials, assembled): clavicle protraction
  -17.8 deg and elevation -4.2 deg as the hinges' `ref`, the scapula's body frame its rest orientation relative to the
  clavicle (scapular abduction -16.4, elevation -4.3, upward rotation 15.5, winging 14.8 deg in Seth's coordinates).
  The contact point is on the ellipsoid there (0.003 mm).
- **Glenohumeral frame:** fixed on the scapula and equal to the thorax frame at rest, so the arm hangs at zero and the
  angles are relative to the scapula. The convention keeps its singularity: at `shoulder_elv_r` = 0 the plane of
  elevation is undefined. A ball joint would remove it (open item). Sites `glenoid_centre_r` (fossa centre and normal
  from the mesh faces facing the humeral head: 8.8 cm2, 25.4 mm from the joint centre) and `humeral_head_centre_r`.
- **Ranges:** protraction -60 to 0 deg, elevation -10 to 35 deg (Seth's data: -49 to -12 and -8 to 16 deg), axial
  rotation -15 to +50 deg, AC cone 80 deg about rest (Seth's data without axial clavicle rotation reach 68 deg).
- **Ligaments, residual springs and armature:** see "Girdle ligaments" below. Residual joint springs to the rest
  posture: 50 N m/rad at the sternoclavicular elevation hinge, 35 N m/rad at the axial hinge and at the AC ball, none
  at the protraction hinge. AC armature 0.005 kg m2; the axial hinge needs none.
- **Hand models** bake the arm with clavicle and scapula aligned to the thorax as before the girdle, so the hand stays
  where it was (within 1.4 mm: Seth's AC point against MoBL-ARMS's).
- **Postures:** the standing posture raises the shoulders to 0.25 rad (`postures.md`).

## Clavicle axial rotation

The clavicle turns about its long axis during arm elevation: Ludewig et al. (2009, bone pins, n = 11) measured about
31 deg of posterior rotation from rest to the end of elevation, Sahara et al. (2007, open MRI) 33 deg at maximal
abduction. Seth's model, like MoBL-ARMS, has none, and without it the scapula's rotation against the clavicle stretches
the conoid by 14 mm along Seth's motions, against 3 mm with abduction to 120 deg in vivo (Izadpanah 2012).

- **Joint:** `sternoclavicular_r1_r`, a third hinge of the clavicle after protraction and elevation, so the order is
  the ISB clavicle sequence (retraction, elevation, axial rotation). The axis is the line from the sternoclavicular
  centre to the AC joint centre (the ISB clavicle z axis); positive is posterior rotation, as in Ludewig 2009. The
  axis passes through the AC centre, so the axial rotation turns the clavicle under the scapula without moving it.
  Range -15 to +50 deg: 30-33 deg measured at the end of elevation (above), textbook totals of 20-50 deg, a few
  degrees of anterior rotation with protraction (Peeters 2023). No coupling to other joints: the coracoclavicular and
  AC ligaments, the clavicular muscles and a residual spring turn it. qpos0 is axial zero.
- **Replay of Seth's motions:** the release's IK has no axial angle, so the replay (ligament slack lengths, muscle
  maps) takes it from Ludewig 2009 Fig. 5A, digitised: posterior rotation against humerothoracic elevation, the
  abduction curve for the abduction and shrug trials and the flexion curve for the flexion trials, relative to the
  curve's value at the rest posture's elevation (17.6 deg) and held beyond the end of the measured range (31 and
  30 deg). The alternative, the axial angle that keeps the conoid at its rest length, needs 67 deg at the highest
  elevations, twice the measured rotation, so it is not used.
- **Conoid length along the replay:** with the measured rotation the conoid lengthens by 2.3 mm from elevation 15-45 to
  105-135 deg in abduction and by 3 mm in flexion (medians), to 5 mm at the end of range, against 3 mm with abduction
  to 120 deg in vivo (Izadpanah 2012); without the rotation it lengthened by 11 mm.
- **What turns it in the model** (`scripts/fit_ligaments.py --zip <archive> --axial`: at 8 frames per 30 deg of
  elevation in each plane, scapula, clavicle protraction and elevation and the glenohumeral angles held at Seth's
  values, the axial angle that minimises the passive energy of the ligaments, the girdle springs, the muscles' passive
  forces at zero activation and gravity):

| Passive layers in the energy | abduction, elevation 45-75 / 75-105 / 105-135 / 135-165 deg | flexion, same bins |
|---|---|---|
| Ludewig 2009 (the replay's curve, medians of the same frames) | 6.7 / 12.2 / 22.8 / 29.5 | 6.5 / 15.0 / 26.3 / 31.0 |
| ligaments and muscles only | 4.5 / -2.0 / 8.0 / 31.0 (flat, undetermined below 120) | -4.0 / 7.5 / 25.0 / 30.5 |
| plus the AC spring (35) | 8.5 / 21.0 / 37.0 / 50.0 (the limit) | 9.5 / 29.0 / 45.5 / 50.0 |
| plus the AC spring and an axial spring (35, 35): model | 4.5 / 10.5 / 18.5 / 32.0 | 5.0 / 15.0 / 24.0 / 30.5 |

  With the ligaments and muscles alone the axial angle is set only where the conoid becomes taut in this quasi-static
  equilibrium, above about 105 deg of elevation, and there it turns the clavicle posteriorly by about the measured amount: the conoid
  cranks the clavicle as described in vivo. Below, nothing in the model holds it. The isotropic AC spring, a stand-in
  for the AC joint surfaces and capsule, turns the clavicle with the scapula and gives about 1.7 times the measured
  rotation; an equal residual spring at the axial hinge, a stand-in for the sternoclavicular disc and capsule, splits
  the rotation between the two joints and brings the equilibrium within 4.5 deg of Ludewig's means (their SD is about
  7-10 deg). At rest the axial spring is needed as well (below). No torsional stiffness of either joint is published;
  the value is the AC spring's, not fitted further.

## Girdle ligaments

`scripts/fit_ligaments.py --zip <archive> --fit` (table `build/data/ligaments.json`), following the explicit-ligament
proposal (MyoHub/myo_sim#142): one straight passive spatial tendon per ligament between bone sites, tension only
(`springlength="0 L0"`), with the quadratic toe of the Blankevoort (1991) law, F = k e^2 / (2 e_t), e_t = 0.06, as the
polynomial stiffness `0 b 0`, b = k / (2 e_t L0^2). No damping (MuJoCo's tendon damper is not gated by the dead band).
The toe is exact up to e_t and stiffer beyond, so the slack length keeps each strand at or below e_t along Seth's
measured motions: L0 = max(L_rest / (1 + e_rest), L_p99 / (1 + e_t)). Sites are landmarked on this package's meshes
from published attachment descriptions; the thorax ends sit on `chest_r` (both sides), placed from the
sternoclavicular centre because the ribcage mesh has no manubrial facet.

| Strand (per side) | k (N per unit strain) | Source of k | Attachments | Rest strain | Slack set by |
|---|---|---|---|---|---|
| conoid | 800 | Costic 2003: 70 +- 23 N/mm x 11.2 mm | conoid tubercle (Boehm 2003, Rios 2007); coracoid base (Chahla 2018) | -0.18 | measured range |
| trapezoid | 800 | Costic 2003: 83 +- 40 N/mm x 9.6 mm | trapezoid line (Rios 2007); coracoid, 8 mm beyond the conoid (Chahla 2018) | -0.08 | measured range |
| AC capsule superior / posterior / anterior / inferior | 186 / 93 / 56 / 37 | Flores 2023 (33.8 N/mm x 11.0 mm, model fit) split by Klimkiewicz 1999 | 3.5 mm either side of the joint line (Stine 2009), 6 mm off the joint centre | 0 / -0.05 / -0.18 / 0 | rest / range / range / rest |
| SC anterior / posterior | 220 / 260 | 10 MPa x footprint-scaled area (Lee 2014) | manubrium below its anterior / posterior margin; medial clavicle (Lee 2014) | -0.04 / -0.05 | measured range |
| interclavicular (half) | 260 | 10 MPa x 7.2 x 3.6 mm (Tubbs 2007) | superomedial clavicle to the midline (Lee 2014) | -0.03 | rest (Tubbs 2007: lax at rest) |
| costoclavicular | 410 | 10 MPa x 12 x 3.4 mm (Tubbs 2009) | inferior clavicle 14 mm from the joint (Lee 2014); first rib below | -0.46 | measured range |

10 MPa is the apparent modulus of the coracoclavicular ligaments, k L / A from Costic 2003 (conoid 11.4, trapezoid
7.7 MPa); no tensile stiffness is published for the sternoclavicular, costoclavicular or interclavicular ligaments, and
the AC capsule value is a model fit, so these rows are `transferred`. In the linear region the strands give k / L0 =
800 / 24.5 mm = 33 N/mm (conoid) and 800 / 12.8 mm = 63 N/mm (trapezoid), 95 N/mm together, inside the measured range
of the coracoclavicular complex (Harris 2000 103 +- 30, Motamedi 2000 116 +- 36, Costic 2004 61 +- 12 N/mm).

**The conoid is slack at rest by 18 %.** With the clavicle's axial rotation in the replay its length changes by
8.8 mm over all of Seth's frames (14.3 mm without it), most of it between subjects and trials; the median rises by
2-3 mm to 120 deg of elevation and 5 mm to the end of range, against 3 mm (abduction to 120 deg) and 6 mm (counting
supine to sitting) in vivo (Izadpanah 2012). A strand 15-19 mm long that lengthens by 3-6 mm cannot be at the start of
its toe region at rest and stay in the toe region at the end of elevation, so the conoid takes up load from about
120 deg of elevation, which is where it turns the clavicle (above). Before the axial rotation it was slack at rest by
36 %. The costoclavicular strand now changes by 14.3 mm (9.4 mm before) and is slack at rest by 46 %: its clavicular
site lies 14 mm from the sternoclavicular centre but off the axial axis, so the single straight strand stretches as the
clavicle turns, whereas the broad ligament in vivo lies close to the axis of rotation (open check).

**Rest at zero activation** (`myoarm_r`, all controls 0, from qpos0; the arm settles within about 5 s, so the values
are at 6 s; before the axial rotation they were taken at 3 s): with the ligaments alone the clavicle settles
on its depression limit (-5.9 deg) and the scapula tilts 10 deg anteriorly. No ligament can hold either: the
sternoclavicular centre sits at the clavicle's medial end, so the superior strands have a lever of about 6 mm about the
elevation axis, against the 5.7 N m the arm and girdle put on it; and the anterior tilt is a rotation about the line from
the AC joint to the thoracic contact, on which the coracoclavicular strands have no lever. In vivo the girdle rests
similarly with and without muscle tone (sternoclavicular elevation 4 +- 5 deg in whole cadavers, Peeters 2023, against
5.9 deg in vivo, Ludewig 2009), so the passive support is real; it comes from the joint surfaces, disc and capsule, which
the fixed hinge cannot represent, and from contact of the whole costal surface of the scapula with the thorax, which the
single gliding contact does not. A residual joint spring therefore stays: the smallest round values that keep every girdle
angle within 5 deg of rest (about one between-subject SD of the resting posture: Peeters 2023 5-9 deg, Ludewig 2009) are
30 N m/rad (elevation) and 35 N m/rad (AC ball); protraction needs none (the ligaments hold it). With the axial hinge
free, the AC spring turns the clavicle anteriorly with the scapula's anterior tilt and the scapula moves 6.2 deg; the
axial spring of 35 N m/rad (chosen above) holds it at 4.5 deg. The springs did not shrink with the axial rotation: AC
and axial at 20 give 5.8 deg, and an elevation spring of 20 keeps the elevation at -4.8 deg, 1 deg from the depression
limit, so 30 stays. After the kept-path changes and the strength calibration ("Kept muscle paths",
"Shoulder strength") the settled arm hangs differently and an elevation spring of 30 lets the clavicle reach its
depression limit (-5.8 deg); with the same rule the elevation spring is now 50 N m/rad (40: -5.2, 45: -4.9, 50: -4.6
deg; scapula 4.3 deg).

| Zero activation | protraction | elevation | axial | scapula | contact |
|---|---|---|---|---|---|
| springs 40/40/40, no ligaments (3 s, before the axial hinge) | -1.1 deg | -3.4 deg | - | 3.9 deg | on |
| ligaments only (3 s, before the axial hinge) | +3.1 deg | -5.9 deg (limit) | - | 10.0 deg | on |
| ligaments + residual 0/30/35 (3 s, before the axial hinge) | +2.1 deg | -3.8 deg | - | 4.4 deg | on |
| ligaments + residual 0/30/35, axial hinge free (6 s) | +2.1 deg | -4.2 deg | -5.4 deg | 6.2 deg | on |
| ligaments + residual 0/30/35/35 with the axial hinge (6 s) | +1.4 deg | -4.2 deg | -1.9 deg | 4.5 deg | on |
| ligaments + residual 0/50/35/35, new paths and forces (6 s, model) | -0.8 deg | -4.6 deg | -1.2 deg | 4.3 deg | on |

The scapula stays on the thorax (contact distance within 0.1 mm) in every case.

**AC armature** (smallest value stable at the model's 2 ms Euler step with the ligaments in, by the criteria of
`myo_sim.build.stability` and `tests/test_numerical_stability.py`): every value from 0 to 0.008 kg m2 passes the dynamic
criteria (period-two reversals at most 2.2 % of steps against the 10 % bound at activation 0, 0.3 and 1 of all muscles
and of the shoulder muscles; random activation stays finite). The explicit force-velocity rate at full activation sets
it: 7.32 (0), 6.60 (0.001), 6.04 (0.002), 5.22 (0.004), 4.92 (0.005), 4.26 (0.008), against the bound of 5 (and 2 at
0.3 activation). 0.005 kg m2 is the smallest value below it; above 0.004 the mode is dominated by `elv_angle_r`.

## Scapulohumeral rhythm

`scripts/fit_scapulothoracic.py --zip <archive> rhythm --write` (table section `rhythm`, checked by
`tests/test_scapulothoracic_data.py`). The girdle replays Seth's measured abduction and flexion trials exactly; their
angles are computed in `myoarm_r` as ISB angles (Wu et al. 2005), positive in the named direction, and compared with
bone-pin data (Ludewig et al. 2009, n = 12; Braman et al. 2009, same cohort; McClure et al. 2001, n = 8) and with
Inman et al. 1944 (as reported in McClure 2001). For each trial the raising-phase frame nearest each humerothoracic
elevation (HT) and the frame of highest elevation are taken; values are medians over 6 trials per plane.

**Landmarks.** Seth's model has a marker for every ISB landmark. They are skin markers, so AA, TS, AI, EL and EM are the
mesh vertices nearest to them (2-15 mm). SC and GH are the joint centres, AC is the centre of the AC ball, and the
thorax uses Seth's IJ, C7, PX and T8. The clavicle's X axis is defined at the rest posture and moves with the bone, as a
bone-fixed sensor would.

| HT 30 / 60 / 90 / 120 / max (deg) | model, abduction | Ludewig, coronal abduction | model, flexion | Ludewig, flexion |
|---|---|---|---|---|
| scapular upward rotation | 13 / 27 / 40 / 52 / 71 | 16 / 27 / 36 / 46 / 52 | 6 / 22 / 38 / 53 / 74 | 16 / 26 / 34 / 44 / 48 |
| scapular internal rotation | 21 / 17 / 21 / 27 / 45 | 31 / 31 / 31 / 31 / 30 | 41 / 48 / 53 / 54 / 44 | 44 / 47 / 48 / 44 / 40 |
| scapular posterior tilt | -10 / -10 / -9 / -9 / -14 | -11 / -6 / -2 / 4 / 7 | -7 / -6 / -8 / -10 / -16 | -10 / -6 / -2 / 5 / 8 |
| clavicle elevation | 11 / 16 / 20 / 24 / 23 | 14 / 17 / 18 / 20 / 20 | 7 / 11 / 19 / 22 / 24 | 10 / 12 / 14 / 15 / 16 |
| clavicle retraction | 41 / 47 / 50 / 52 / 58 | 31 / 37 / 41 / 44 / 45 | 29 / 28 / 33 / 42 / 60 | 21 / 23 / 25 / 32 / 35 |
| clavicle posterior rotation (input) | 4 / 12 / 21 / 31 / 37 | 3 / 9 / 15 / 25 / 31 | 2 / 10 / 20 / 35 / 39 | 0 / 7 / 14 / 25 / 30 |
| glenohumeral elevation | 15 / 32 / 50 / 67 / 83 | 9 / 31 / 54 / 75 / 84 | 29 / 51 / 67 / 81 / 86 | 14 / 33 / 56 / 77 / 85 |

The model's maximum HT is 149 deg, against Ludewig's end range of 131-136 deg. At rest (qpos0) the scapula is within
1 SD of Ludewig's Table I: upward rotation 4.5 against 5.4 +- 3.5, internal rotation 37 against 41 +- 7, anterior tilt
8 against 14 +- 7 deg. The clavicle is elevated 8.5 against 5.9 +- 3.3 and retracted 31 against 19 +- 7 deg (+1.7 SD),
which comes from the generic clavicle's geometry.

Up to HT 60 the scapular upward rotation matches the bone-pin data. Above that, Seth's motions put more of the elevation
into the scapula: 71-74 deg at the end against 48.6 +- 4.0 (Braman, +5.6 SD), and the glenohumeral elevation stops near
85 deg. The GH:ST ratio is therefore about 1.2 (abduction 30-120: 1.24; flexion: 1.11), against 2.1-2.4 (Ludewig),
1.7-2.0 (McClure) and 2.4-2.7 +- 0.8 per interval (Braman). The scapula also barely tilts posteriorly (-9 to -15 deg
against +21 to +30 deg over the arc). Both come from the measured motions the model replays: Seth's inverse kinematics,
from skin markers, has the same upward rotation and no tilt, and skin markers are least accurate for these two angles
at high elevation (Karduna et al. 2001). They are not limits of the joint: the AC ball leaves tilt free for the muscles.
Seth's abduction trials also drift from the coronal plane to a plane 65 deg forward at the top, which lowers the
scapular internal rotation in abduction.

The clavicle's posterior rotation is taken from Ludewig in the replay, so its row checks consistency only. In ISB terms
it comes out 1-10 deg above Ludewig, because Seth's oblique protraction and elevation axes add 2-8 deg of ISB axial
rotation on their own. Clavicle elevation rises 14-16 deg over the arc: more than the bone-pin 6 deg, less than Inman's
radiographic 25-30 deg. In the shrug trials the clavicle elevates by 19 deg (8 to 27 deg). Ludewig's per-angle values
are digitised means (about +-1 deg) with SE only; the choice of scapular landmarks (AA here, the AC joint in Braman)
changes scapular angles by up to 23 deg (Kolz et al. 2020).

## Muscles

`scripts/generate_scapular_muscles.py --zip <archive> --fit` writes `build/data/scapular_muscles.json` and the labelled
generated blocks in the arm, torso and head files.

- **Added per side:** TRAP1 (clavicular, nuchal) and TRAPocc (clavicular, occipital), TRAP2-4 (scapular superior,
  middle, inferior), SERR1-3, RHOM1-2, LEVS1-2, PECMIN.
- **Replaced:** SUPSP, INFSP, SUBSC by Seth's sub-region lines SUPSP1-2 (anterior, posterior), INFSP1-2 (superior,
  inferior), SUBSC1-3 (superior, middle, inferior). The kept glenohumeral muscles are decided one by one below
  ("Kept muscle paths").
- **Paths:** Seth's two points per line, straight, except where a wrap changes the length along Seth's motions:
  SUPSP1-2 over Seth's humeral-head sphere, INFSP1-2 and SUBSC1-3 over Seth's scapular cylinders (his `-y` quadrant as
  the side site), SERR2-3 over spheres on `chest_r` fitted to the lengths of Seth's ellipsoidal thorax wrap (rms 0.11
  and 0.01 mm; MuJoCo wraps spheres and cylinders only). Along the 4312 assembled IK frames the lengths match Seth's
  OpenSim paths within 4.5 mm (SERR1), 3.1 mm (INFSP2) and below 3 mm elsewhere, most exactly.
- **Cervical origins:** LEVS1 and TRAP1 at C1, TRAP2 and RHOM1 at C7 (Seth's points lie within 5 mm of these
  vertebrae), LEVS2 at C3 and TRAPocc on the skull: on `myohead_cerv1`, `_cerv3`, `_cerv7` and the skull of the
  muscular neck in `myofullbody`, and at the same point on the rigid `neck` body in the other models, which carry no
  neck joints.
- **Neck lines.** Seth's levator scapulae is one line (DAS3's upper element). It is split into two slips with DAS3's
  shares (Chadwick et al. 2014, after Klein Breteler et al. 1999): LEVS1 (C1-C2, 48 %) on Seth's line and LEVS2 (C3-C4,
  52 %) from the tip of C3's transverse process (the most lateral vertex of the C3 mesh) with DAS3's lower-element
  lengths (l0 0.130 m, slack 0.018 m); both insert at Seth's point, so the twist of the slips along the medial border is
  not modelled. The total PCSA is Klein Breteler's 2.00 cm2, within Kamibayashi and Richmond 1998's 2.18 +- 0.80 cm2
  (as reported in Vasavada et al. 2010). The clavicular trapezius has no occipital fibres in Seth's or DAS3's model;
  TRAPocc takes 20 % of its PCSA from the superior nuchal line (medial third: the most posterior skull vertex 15-25 mm
  from the midline) to Seth's clavicular insertion, fibre length 0.084 m (Kamibayashi and Richmond 1998), slack such
  that the fibre is at optimal length at rest. The 20 % share has low confidence: Johnson et al. 1994 find the occipital
  and upper nuchal fibres the smallest of the trapezius, but no share has been measured; the clavicular part as a whole
  is 9.5 % of Klein Breteler's trapezius and 11-12 % in Stiver et al. 2021 (one specimen). `compose.attach_neck` re-creates the tendons that name
  rigid-neck sites when it swaps the neck.
- **Forces:** PCSA x the shoulder's specific tension (111.1 N/cm2, "Shoulder strength" below). Seth's forces are Klein
  Breteler 1999's PCSA (one specimen) x 100 N/cm2 as in DAS3, x 1.4, that is PCSA x 140 N/cm2; the scapular muscles take
  that PCSA. The cuff
  takes Ward et al. 2006's PCSA (supraspinatus 6.65, infraspinatus 10.71, subscapularis 15.53 cm2), split by Seth's
  shares. Klein Breteler's PCSA are 1.5 to 2.7 times Langenderfer 2004's, which underlie the kept MoBL-ARMS forces.
- **Lengths:** Ward 2006 fibre lengths for the cuff, with the tendon slack keeping Seth's optimal muscle-tendon length;
  Seth's otherwise (TRAP1 repeats TRAP2's in Seth's model; Klein Breteler's 0.155-0.168 m would leave this path's fibres
  below 0.5). Rigid-tendon maps fitted along Seth's motions and the rest posture, fibres within 0.5-1.5 optimal length
  (slack moved for SUPSP1 and SUBSC2). Passive scale: the fitted value of the replaced muscle for the cuff, 0.65 (the
  median of the kept shoulder muscles) for the scapular muscles.

## Kept muscle paths

The glenohumeral muscles kept from MoBL-ARMS have their scapular and clavicular ends on bones that now move, and the
glenohumeral zero changed. Their moment arms were compared, with the scapula fixed at rest, about the coronal,
scaption and flexion axes from 15 to 120 deg and in rotation at the side, against Kuechle et al. 1997 and 2000,
Ackland et al. 2008, Liu et al. 1997 and Otis et al. 1994, and against Seth's paths run in OpenSim at the same
humerus-in-scapula orientations (Seth's points in this model reproduce OpenSim to 0.1 mm).

| Muscle | Decision | Why |
|---|---|---|
| DELT1 | Seth's three points (humerus, acromion, clavicle) over the humeral-head sphere `SUPSP_hhead_r`; Seth's l0 and slack | The MoBL path adducted at 15 deg abduction and gave 47-50 mm of flexion at 15-45 deg (measured 20-28); rms against the data 20.3 -> 6.5 mm |
| DELT2 | Seth's three points over `SUPSP_hhead_r`; Seth's l0 and slack; passive scale 0.65, the median of the shoulder muscles (MoBL-ARMS's 1.73 on this path held the relaxed arm 23 deg abducted) | The MoBL path gave 4 mm of abduction arm at 15 deg (measured 15-20 mm), which left the arm without abduction strength at the side; now 25 mm at 15 deg and 35 mm at 60-75 deg (measured 15-20 and 26-30 mm; glenohumeral angles, scapula fixed at rest as in the cadaver studies) |
| CORB, BICshort | Scapular via point dropped, path over `SUPSP_hhead_r` | The via point sits 35 mm below the joint centre and made both muscles extensors (26-35 mm) above 60 deg flexion; now flexors that change sign near 95 deg, as Seth's |
| TRIlong | The humeral proxy origin and its 20 mm sphere dropped, path over `SUPSP_hhead_r` | The proxy point sat 2.6 mm from the origin, so the moment arm flipped between +40 and -40 mm about an arbitrary posture; at the new rest it made TRIlong an abductor and internal rotator at the side. Now an extensor and adductor (flexion -3.5, coronal -6 mm at the side), within 8 mm of Seth |
| DELT3, PECM1-3, LAT1-3, TMAJ, TMIN, BIClong | Kept | Within the spread between the measured studies; Seth's paths are not closer (PECM's thorax ellipsoid cannot be built in MuJoCo) |

The new paths take their rigid-tendon maps from the reference l0 and slack (`fit_arm_rigid_maps.py`, status adopted
in `build/references.py`), with every other adopted arm map refitted for the girdle. On the humeral-head sphere none of
the paths has a side site, so their length is continuous under both wrap rules.

**BIClong.** Lamplot et al. 2020 measured the long-head excursion on forequarters in sagittal glenohumeral positions
(-30, 45 and 90 deg, elbow 90 or extended) with an unloaded tendon and a neutral not defined against the scapula:
-9.4 +- 3.9, 6.9 +- 2.5 and 8.5 +- 3.6 mm. The model gives -9.6, 14.6 and 23.1 mm, Seth's path -10.9, 12.7 and 15.8 mm.
Extension, where the tendon is pulled taut, agrees; in flexion the path shortens and an unloaded tendon need not
follow it, so the measured values are lower bounds. The path is unchanged from MoBL-ARMS; the difference against the
earlier model comes from the glenohumeral zero, which now has the humerus 15-20 deg more adducted against the scapula.
The path is kept.

## Shoulder strength

`scripts/fit_shoulder_strength.py --zip <archive> --fit` (`build/data/shoulder_strength.json`). One specific tension
for every shoulder muscle: the glenohumeral muscles kept from MoBL-ARMS (PCSA of Holzbaur et al. 2005 Table 1, from
Langenderfer et al. 2004), the rotator cuff (Ward et al. 2006) and the scapular muscles (Klein Breteler et al. 1999).
The elbow muscles keep their forces, and passive force stays a fixed fraction of peak force. The maximal moment of a
posture is the sum of the positive active moments of the muscles crossing the glenohumeral joint about the target axis
plus all passive moments (no co-contraction, gravity corrected as on a dynamometer), with the humerus at the target's
humerothoracic angles and the girdle at the nearest frame of Seth's measured motions. Targets are young men, as for the
hand.

| Target (men) | Posture | Measured, N m | Model, N m | z |
|---|---|---|---|---|
| Abduction (Holzbaur 2007) | 60 deg abduction, elbow extended | 74.4 +- 10.8 | 65.4 | -0.8 |
| Adduction (Holzbaur 2007) | same | 93.7 +- 11.3 | 103.1 | +0.8 |
| Internal rotation (Smith 2001) | 45 deg scaption, 45 deg external rotation, elbow 90 | 52.8 +- 9.9 | 71.0 | +1.8 |
| External rotation (Smith 2001) | same | 36.2 +- 7.6 | 29.6 | -0.9 |
| Flexion (Aronis 2025) | 90 deg flexion, elbow extended | 77.2 +- 13.8 | 61.9 | -1.1 |
| Abduction (Harbo 2012) | arm at the side, elbow 90 assumed | 61.1 +- 14.1 | 42.0 | -1.4 |
| Internal rotation (Roy 2009, held out) | 45 deg scaption, elbow 90 | 46.8 +- 16.5 | 60.6 | +0.8 |
| External rotation (Roy 2009, held out) | same | 33.6 +- 9.0 | 40.5 | +0.8 |

Result: **111.1 N/cm2** (rms z 1.20). Against the digitised curves without spread (as reported in Holzbaur et al.
2005 Fig. 4), the model is low in flexion (55 against Otis's 93 N m at the side, 42 against 75 at 90 deg with the elbow
at 90 deg, where the triceps' passive force opposes it) and high in extension and adduction at 90 deg (134 against 90,
116 against 78). The value lies inside Holzbaur et al. 2005's 140 N/cm2 for the shoulder and below the hand's 166 N/cm2
(`hand-function-validation.md`), which reflects PCSA measured too small in elderly cadavers rather than a tissue
property; Klein Breteler's PCSA for the scapular muscles come from a larger specimen, so their forces are probably
generous. Held-out girdle forces at this value: a shrug of 582 N at the acromion, between Hannah et al. 2017's seated
0.31 body weight (about 230 N) and Garner and Shim 2008's 1101 +- 370 N with extended arms; a protraction of 840 N at
90 deg flexion, between IJspeert et al. 2019's 384 +- 78 N (handheld) and Garner and Shim's 1117 +- 471 N.

## Step time and mujoco-warp

Step time on MuJoCo 4 (CPU, zero activation, best of three runs of 1000 steps): `myoarm_r` 83.3 -> 78.3 us,
`myoarms` 170.9 -> 155.3 us, `myofullbody` 449.2 -> 455.7 us against the model before the girdle. mujoco-warp (CPU
device) against MuJoCo at 22 of Seth's frames: ligament lengths within 2e-4 mm, girdle passive forces within 3e-5 N m;
over 200 steps from rest, the girdle hinges agree within 0.002 deg and the AC ball within 0.02 deg, with the same
contacts.

## Open checks limited by the passive layers

The girdle's passive behaviour comes from three layers: the muscles' passive force, the ligament strands and the
residual joint springs. Each is fitted on its own data; some girdle properties depend on all three together and can
only be judged once each layer is complete and fitted, so they are listed here instead of being tuned through one
layer.

- **Rest posture at zero activation.** The residual springs (elevation 50, axial 35, AC 35 N m/rad; placeholders, to be revisited once every passive layer is fitted) hold it within
  5 deg; they stand in for the joint surfaces, disc and capsule and for the full scapula-thorax contact. Their values
  should fall once the scapular muscles' passive forces are fitted to data at the girdle (there is none yet: the
  scapular muscles take the median passive scale of the shoulder muscles) and the scapula has a fuller thoracic
  contact.
- **Split of the scapula's rotation between the AC joint and the clavicle's axial rotation.** In the model it is set by
  the ratio of the AC and axial springs below about 105 deg of elevation and by the conoid above (in the quasi-static
  equilibrium). No torsional stiffness
  of either joint is published; the isotropic AC spring cannot be stiffer about one axis than another, while the AC
  capsule restrains posterior rotation of the clavicle more than other motions (Fukuda 1986).
- **Conoid tension (accepted as is).** Slack by 18 % at rest. In the quasi-static passive equilibrium of the axial
  rotation it becomes taut from about 105-120 deg of elevation; along Seth's replayed motions its median length reaches
  the slack length only near the end of range (median strain -0.11 at 120-140 deg and +0.01 at 140-160 deg of
  abduction, -0.02 and +0.02 in flexion; taut in 12 % of all frames). This agrees
  with lax coracoclavicular ligaments in low elevation: the acromioclavicular capsule restrains small displacements and
  the conoid large ones (Fukuda et al. 1986), and the conoid lengthens mainly above 60 deg (Seo et al. 2012). In vivo it
  may also carry part of the arm's weight at rest (it lengthens from supine to sitting, Izadpanah et al. 2012); the
  quadratic toe without a linear region does not allow both a taut rest and the measured lengthening, so this is
  revisited with the other passive layers.
- **Costoclavicular strand.** A single strand off the axial axis, slack by 46 % at rest; the broad ligament lies near
  the axis in vivo.
- **Triceps passive force in elevation.** At 90 deg flexion with the elbow at 90 deg, the passive moment of the
  glenohumeral muscles is about 30 N m, mostly the long head of the triceps; Landin and Thompson 2011 measured 12.6-14.3
  N m in total at 80 deg scaption (another plane, so indicative). The triceps' passive scale has no shoulder data.
- **Shrug.** No ligament or spring was fitted to the shrug trials; Peeters 2023 report about 5 deg of posterior
  clavicle rotation in a passive shrug, the replay uses about 1 deg.

## Open items

Passive girdle behaviour (above); moment arms of the new scapular muscles (no scapulothoracic moment-arm data were
found); glenohumeral translation with the capsule and a glenoid contact; an EMG comparison.
