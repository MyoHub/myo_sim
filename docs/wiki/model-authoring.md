# Model Authoring Guide

This guide explains how to add or edit body-part models in `myo_sim`. It is written for both human contributors and autonomous agents.

## Overview of the four file roles

Every body part under `myo_sim/models/<part>/assets/` is defined by four file types that separate concerns cleanly. Never collapse these roles into fewer files.

### `*_chain.xml` — skeleton

Contains `<body>`, `<joint>`, `<geom>` (collision and visual), and structural `<site>` elements. Sites here serve as attachment frames, anatomical landmarks, or tendon via-points — they are anchored to specific bodies and must live in the file that owns those bodies. This file defines what bodies exist and how they connect kinematically. Wrapping geoms (`class="wrap"`) also belong here on the body they geometrically attach to.

### `*_muscle.xml` — actuators

Contains only `<general>` or `<muscle>` elements with `gainprm`, `biasprm`, and `lengthrange` attributes. Each actuator references a tendon by name. No sites, no bodies, no tendons are declared here.

### `*_tendon.xml` — spatial tendon routing

Contains `<spatial>` elements that list sites and wrapping geoms in anatomical order. Sites are referenced by name — they must already exist in `*_chain.xml`. Each wrapping step uses `<geom geom="name" sidesite="sidesite_name"/>` where both the geom and the sidesite are defined in `*_chain.xml`.

### `*_assets.xml` — mesh/material/texture/default declarations

Declares `<mesh>`, `<material>`, `<texture>`, and `<default>` class hierarchies. These are referenced by the chain file. Also contains `<compiler>`, `<option>`, `<size>`, `<equality>`, and `<sensor>` elements that apply to the whole part. The chain file includes this file via MuJoCo's `<include>` mechanism.

---

## Adding a new muscle to an existing part

Follow these steps in order. The leg model (`myo_sim/models/leg/assets/`) is used as the running example.

**Step 1.** Add the muscle via-point sites to the relevant body in `*_chain.xml`.

```xml
<!-- in myolegs_r_chain.xml, inside the body the site anatomically belongs to -->
<site name="newmuscle-P1_r" pos="0.01 -0.05 0.02"/>
<site name="newmuscle-P2_r" pos="0.02 -0.15 0.01"/>
```

Sites that serve as wrapping sidesiotes follow the naming pattern `<WrapObjectName>_site_<musclename>_r`.

**Step 2.** Add the tendon definition in `*_tendon.xml`.

```xml
<spatial name="newmuscle_r_tendon" springlength="0.05" class="myoleg_muscle">
    <site site="newmuscle-P1_r"/>
    <!-- optional wrapping step: -->
    <geom geom="somewrap_r" sidesite="somewrap_site_newmuscle_r"/>
    <site site="newmuscle-P2_r"/>
</spatial>
```

**Step 3.** Add the actuator in `*_muscle.xml`.

```xml
<general class="myoleg_muscle"
         name="newmuscle_r"
         tendon="newmuscle_r_tendon"
         gainprm="0.5 1.4 800 1 0.05 3.0 10 1.5 1.4 0"
         biasprm="0.5 1.4 800 1 0.05 3.0 10 1.5 1.4 0"
         lengthrange="0.10 0.25"/>
```

**Step 4.** Verify the model loads and the actuator count increased by one.

```bash
uv run python -c "
import myo_sim
m, _ = myo_sim.load('myolegs')
print('nu:', m.nu)
"
```

**Step 5.** Run the relevant symmetry test to confirm the mirrored left side is consistent.

```bash
uv run pytest tests/test_leg_muscle_symmetry.py
```

---

## Adding a wrapping object

Wrapping geoms live in `*_chain.xml` on the body they geometrically belong to. Use `class="wrap"` (or the part-specific wrap default class, e.g. `class="myoleg_wrap"`).

```xml
<!-- in *_chain.xml, inside the appropriate body -->
<geom class="wrap" name="NewMuscle_wrap_r" type="cylinder"
      pos="0.01 -0.10 0.0" quat="0.707 0.707 0 0" size="0.015 0.03"/>
<site name="NewMuscle_wrap_sidesite_r" pos="0.01 -0.10 0.015"/>
```

Then reference them in the tendon:

```xml
<geom geom="NewMuscle_wrap_r" sidesite="NewMuscle_wrap_sidesite_r"/>
```

Both the geom and sidesite must be on the same body (or a nearby body if the muscle spans bodies). If the sidesite is missing or on a different body from the geom, MuJoCo will error at load time.

---

## Adding a new body part entirely

1. Create `myo_sim/models/<part>/assets/` directory with the four asset files following naming convention `myo<part>_assets.xml`, `myo<part>_r_chain.xml`, `myo<part>_tendon.xml`, `myo<part>_muscle.xml`.

2. Follow the same file-role separation described above.

3. If the part attaches to an existing part (e.g., a new arm segment attaches to the torso), add an attachment site in the parent's chain file — for example, in `myotorso_r_chain.xml` — named `<part>_attach_r`. Then add a new `BuildStrategy` entry and corresponding builder function in `myo_sim/build/compose.py`.

4. Add contact pairs between this part and any parts it interacts with in `myo_sim/models/contacts/myo<part>_contacts.xml`. Do not embed cross-part contact pairs inside the part's own asset files.

5. Register public assembled-model entry points in `myo_sim/build/compose.py` inside `MODEL_REGISTRY`.

6. Add the model path to the smoke test list in `test_sims.py` so the CI will verify it loads.

7. Create a `README.md` in `myo_sim/models/<part>/` using the template in `docs/wiki/model-readme-template.md`.

---

## Bilateral symmetry rules

The codebase enforces a strict "right side is the source of truth" policy.

- Never hand-edit a left-side XML. The left side is derived automatically by mirroring the right-side chain file in `myo_sim/build/compose.py` using `build_mirrored_child_xml()`.
- Muscle names, site names, geom names, joint names, and body names all carry `_r` (right) or `_l` (left) suffixes. Always use `_r` in your source files.
- The mirror arithmetic — z-axis position reflection, x/y axis negation, quaternion conjugate — is implemented by `MirrorRules` in `myo_sim/build/utils.py`. If a body has an unusual orientation that requires a different mirroring axis, add it to `MirrorRules.body_pos_x_mirror_names`.
- Verify that left/right symmetry holds by running the symmetry analysis scripts:

```bash
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
cd tests && uv run python debug_muscle_bimanual.py
```


---

For naming conventions, see `docs/wiki/engineering-standards.md`.
