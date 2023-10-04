# MyoLeg

## General:

The myoLeg mujoco musculoskeletal (MSK) models are generated with taking [Rajagopal's full body gait model](https://github.com/opensim-org/opensim-models/tree/master/Models/RajagopalModel) as close
reference.

These generated mujoco MSK models have almost identical kinematics, and very similar muscle kinematics (moment arms) and kinetic (forces) properties.


## Conversion process:

The myoLeg models were generated using our developed automatic conversion pipeline (will release at June 2023).

Three Conversion steps were taken to generate the myoLeg models from the reference Osim model:

1. Basic element conversion [bone meshes, joint definitions, muscle paths, wrapping objects]
2. Moment arm optimization [matching the moment arm of each muscle by optimizing how muscles wrap over wrapping objects]
3. Muscle force optimizaiton [matching the muscle force-length relationship by optimizing muscle parameters]

After the conversion, a manual adjusting process is done to correct the abnormal results.

## Maunal adjustment:

1. Removed wrapping objects for glmax1_l, glmax2_l, glmax1_r, glmax2_r, psoas_l, and psoas_r muscles to avoid the wrapping path jumping. These wrapping objects are:
	- Gmax1_at_pelvis_l_wrap
	- Gmax2_at_pelvis_l_wrap
	- Gmax1_at_pelvis_r_wrap
	- Gmax2_at_pelvis_r_wrap
	- PS_at_brim_l_wrap
	- PS_at_brim_r_wrap

2. Changed the wrapping objects from 'cylinder' to 'sphere' for iliacus_l and iliacus_r muscles to avoid the wrapping path jumping. These wrapping objects are:
	- IL_at_brim_l_wrap
	- IL_at_brim_r_wrap

3. Adjusted the 'lmin' of gaslat_l, gaslat_r, semimem_l, and semimem_r muscles to avoid negative muscle forces.
	- changed from 0.1 to 0.05

4. Adjustments post conversion to optimize for kitnematic and dynamic behaviors
5. Inertial properties
6. Dynamics properties of the joints


## Contact Geometries:

The contact geometries take the well defined [Yeadon measurement method](https://yeadon.readthedocs.io/en/latest/measurements.html#measurements) as reference. Geometries are very close to the reference but slightly adjusted to fit our MKS mode. Contact properties were optimized for contact rich behaviors.


## Issues:

1. Endpoints (markers) below the knee joints have around 1 cm differences between the converted Mujoco and Osim model.
	- This may due to the constraints defined at the knee translation DoF. In Mujoco model, a polynomial function is used to approximate the Osim lookup table. Differences may generated.
	- Will look into this approximation to improve the accuracy.

2. Muscle moment arms inside the reference Osim model contains sudden changes (muscle wrapping path jumping), which need to be corrected.
	- This is why the manual adjustment is needed

3. Vastus muscle moment arms at the knee joint has relatively large differences (same sign, a few cm).
	- Need to check whether this is caused by the dependend joint constraints at knee.
	- This moment arm difference also affected the knee extenser muscle force, will look into in future.

4. Muscle forces are not identical between the converted MuJoCo and Osim models, due to the difference in muscle model definitions (stiff vs elastic tendons).
	- Will investage to implement elastic tendon inside mujoco.

## ChangeLog

**myoleg_v0.51(mj120).mjb**
- new keyposes added to mark convenient poses.

**myoleg_v0.52(mj120).mjb**
- Removing the extra body that made the torso twice as heavy. The body now is ~80 kgs.

**myoleg_v0.53(mj120).mjb**
- Improved collisions. Adding a height field.

**myoleg_v0.56(mj237).mjb**
- height field adjusted
- moved to mujoco 2.3.7