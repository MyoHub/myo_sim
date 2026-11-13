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
  the ball joint `acromioclavicular_r` at Seth's AC point, and the glenohumeral chain `elv_angle_r`, `shoulder_elv_r`
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
- **Ranges:** protraction -60 to 0 deg, elevation -10 to 35 deg (Seth's data: -49 to -12 and -8 to 16 deg), AC cone
  80 deg about rest (Seth's data reach 68 deg).
- **Springs and armature:** 40 N m/rad to the rest posture at the sternoclavicular hinges and the AC ball, a stand-in for
  the sternoclavicular, acromioclavicular and coracoclavicular ligaments and resting tone, none of which is modelled
  (no measured value; chosen as the smallest round value that keeps the girdle within a few degrees of rest under the
  arm's weight at zero activation: without springs the clavicle drops to its elevation limit and the scapula turns by
  up to 29 deg). AC armature 0.008 kg m2 keeps the explicit force-velocity rate of the scapula at full activation below
  5 (7.2 without, `myo_sim.build.stability`), as the thumb CMC armature does.
- **Hand models** bake the arm with clavicle and scapula aligned to the thorax as before the girdle, so the hand stays
  where it was (within 1.4 mm: Seth's AC point against MoBL-ARMS's).
- **Postures:** the standing posture raises the shoulders to 0.25 rad (`postures.md`).

## Muscles

`scripts/generate_scapular_muscles.py --zip <archive> --fit` writes `build/data/scapular_muscles.json` and the labelled
generated blocks in the arm, torso and head files.

- **Added per side:** TRAP1 (clavicular), TRAP2-4 (scapular superior, middle, inferior), SERR1-3, RHOM1-2, LEVS, PECMIN.
- **Replaced:** SUPSP, INFSP, SUBSC by Seth's sub-region lines SUPSP1-2 (anterior, posterior), INFSP1-2 (superior,
  inferior), SUBSC1-3 (superior, middle, inferior). DELT1-3, PECM1-3, LAT1-3, TMAJ, TMIN, CORB, BIC and TRIlong keep their
  paths for now; the per-muscle comparison of moment arms against the literature is part of the validation.
- **Paths:** Seth's two points per line, straight, except where a wrap changes the length along Seth's motions:
  SUPSP1-2 over Seth's humeral-head sphere, INFSP1-2 and SUBSC1-3 over Seth's scapular cylinders (his `-y` quadrant as
  the side site), SERR2-3 over spheres on `chest_r` fitted to the lengths of Seth's ellipsoidal thorax wrap (rms 0.11
  and 0.01 mm; MuJoCo wraps spheres and cylinders only). Along the 4312 assembled IK frames the lengths match Seth's
  OpenSim paths within 4.5 mm (SERR1), 3.1 mm (INFSP2) and below 3 mm elsewhere, most exactly.
- **Cervical origins:** LEVS and TRAP1 at C1, TRAP2 and RHOM1 at C7 (Seth's points lie within 5 mm of these vertebrae):
  on `myohead_cerv1` and `myohead_cerv7` of the muscular neck in `myofullbody`, and at the same point on the rigid
  `neck` body in the other models, which carry no neck joints. `compose.attach_neck` re-creates the tendons that name
  rigid-neck sites when it swaps the neck.
- **Forces:** PCSA x 140 N/cm2, the value Holzbaur 2005 used for the shoulder. Seth's forces are Klein Breteler 1999's
  PCSA (one specimen) x 100 N/cm2 as in DAS3, x 1.4, that is PCSA x 140 N/cm2; the scapular muscles keep them. The cuff
  takes Ward et al. 2006's PCSA (supraspinatus 6.65, infraspinatus 10.71, subscapularis 15.53 cm2), split by Seth's
  shares. Klein Breteler's PCSA are 1.5 to 2.7 times Langenderfer 2004's, which underlie the kept MoBL-ARMS forces.
- **Lengths:** Ward 2006 fibre lengths for the cuff, with the tendon slack keeping Seth's optimal muscle-tendon length;
  Seth's otherwise (TRAP1 repeats TRAP2's in Seth's model; Klein Breteler's 0.155-0.168 m would leave this path's fibres
  below 0.5). Rigid-tendon maps fitted along Seth's motions and the rest posture, fibres within 0.5-1.5 optimal length
  (slack moved for SUPSP1 and SUBSC2). Passive scale: the fitted value of the replaced muscle for the cuff, 0.65 (the
  median of the kept shoulder muscles) for the scapular muscles.

## Open items

Validation (scapular kinematics against Seth's data and the published rhythms, moment arms of the kept and new muscles
against Kuechle, Otis, Liu and Ackland, strength against Holzbaur 2007, passive girdle behaviour, mujoco-warp parity and
step time); the BIClong excursion over shoulder flexion is now 14.6 / 23.1 mm against Lamplot 2020's 6.9 / 8.5 mm
(glenohumeral angles, scapula at rest); glenohumeral translation with the capsule; an EMG comparison.
