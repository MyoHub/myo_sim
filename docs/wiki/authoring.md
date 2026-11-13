# Anatomy export and selected model edits

`myo_sim.authoring` exports compiled anatomy, applies selected edits to a copied
`MjSpec`, and saves validated native model archives. MuJoCo supplies the resolved
anatomy and reference measurements. Editor integrations can consume this data
contract independently of the package.

The [existing MuSkeMo XML importer](https://github.com/PashavanBijlert/MuSkeMo/blob/main/scripts/import_mujoco_model.py)
is one possible upstream integration point. An external adapter can also consume
the anatomy JSON and emit selected-edit patches using the contracts below.

## Export and apply

```bash
uv run python -m myo_sim.authoring export --model myoelbow --joint-sweep r_elbow_flex --output elbow.json
uv run python -m myo_sim.authoring apply --model myoelbow --patch edits.json --output edited-elbow.zip
```

Both commands also accept `--xml source.xml` or a native model zip instead of
`--model`. The CLI protects existing outputs unless `--overwrite` is given.
Create `edits.json` using the patch contract below and the fingerprint/version
from `elbow.json`.

The same workflow is available in Python:

```python
import json
from pathlib import Path

import myo_sim
from myo_sim.authoring import apply_model_patch, export_model, joint_sweep, save_model_archive

source = myo_sim.load_spec("myoelbow")
model = source.copy().compile()
export_model(model, "elbow.json", qpos=joint_sweep(model, "r_elbow_flex", samples=21), source_model="myoelbow")
patch = json.loads(Path("edits.json").read_text(encoding="utf-8"))
edited = apply_model_patch(source, patch)
save_model_archive(edited, "edited-elbow.zip")
```

Export evaluates optional `(frames, nq)` finite poses after normalizing free/ball
quaternions and projecting active joint couplings. `joint_sweep` spans a limited,
independent hinge or slide joint. The source model and caller's pose array stay
unchanged.

## Version 2 anatomy document

The root contains `schema: 2`, `patch_schema: 1`, a SHA-256 `fingerprint`, `mujoco_version`,
`units: "metres-radians-kilograms"`, `qpos0`, and the arrays below. IDs refer to the
compiled MuJoCo model. Body/site `source_name` retains the actual source name;
generated `name` labels for unnamed objects are display labels.

| Array | Contents |
|---|---|
| `bodies` | ID, name, source name, origin, parent body, mass, body-local COM and full inertia about the COM in body axes. |
| `joints` | ID, name, body, free/ball/hinge/slide type, local position/axis, qpos address, limits and range. |
| `sites` | ID, name, source name, origin, body and body-local position. |
| `tendons` | ID, name and ordered elements with native wrap type, object ID and parameter, including sites, wraps and pulleys. Geom wraps include an explicit `sidesite` ID or null. |
| `muscles` | Muscle actuator ID/name, tendon ID, peak force, gear, gain/bias/dynamics parameters, length range and `has_sidesite_constraints`. Rows follow source actuator order. |
| `geoms` | ID, name, body, type, size and effective RGBA; meshes embed their compiled vertices and faces. Material colors apply unless geom RGBA overrides the native default. |
| `equalities` | Native equality type, object type, activity, object IDs and parameter data. |
| `frames` | Evaluated qpos, world body/geom positions and wxyz quaternions, world sites, muscle lengths, physical tendon lengths, native moment arms and `wrap_flags`. |

Positions use metres, angles radians, masses kilograms and inertia kg m².
`muscle_length` includes actuator gear; `muscle_tendon_length` is physical tendon
length. Their rows and `moment_arms` rows follow `muscles`. Moment-arm columns
span all `nv` velocity DOFs and contain native `dL/dq`, including gear. These are
partial derivatives; an independent-coordinate sweep through a coupling also
has a chain-rule contribution from dependent coordinates.

Equality `object_type` preserves MuJoCo's object namespace, distinguishing body
and site IDs for `CONNECT` and `WELD`. It can be `UNKNOWN` when the equality type
itself identifies the namespace, as with `JOINT` and `TENDON`. A secondary object
ID of `-1` denotes an omitted object.

`provenance` records `scope: "instance"`, `status: "verified"` or `"unknown"`,
`source_model`, `reason` and human-readable `guidance`. The optional
`source_model` argument is a registry hint: canonical ownership is reported only
when its freshly compiled model exactly matches the source fingerprint.
Custom or mismatched models remain explicitly unknown, with null `source_model`
and an explanatory `reason`; a verified model has a null reason.

Each body/site `origin` contains `status: "canonical"` or `"unknown"`,
`chain_fragment` relative to `MODELS_DIR`, `canonical_name` and `generated_left`.
Unknown or ambiguous ownership uses null for the latter three fields. These
records identify source ownership; they do not convert instance edits into XML.

Each frame's `wrap_flags` contains `tendon`, `element`, `geom`, `sidesite` and
`sidesite_inside`. The flag tests radial distance in the geom frame for a cylinder
or Euclidean distance for a sphere. Top-level `warnings` aggregates affected
zero-based `frames` indices using `code: "inside_sidesite"` plus the same object references and
a message. This is a preview-compatibility flag, not a rejection of native wrapping.
Muscle `has_sidesite_constraints` is true for any native geom wrap with a sidesite,
including sidesites outside the wrap; inside/outside state is reported per frame.

Native frame measurements remain the reference when an external preview uses
different wrapping. The document supports inspection and selected edits;
preserve the original `MjSpec` for simulation and recompilation. Flexes, skins,
heightfields and SDF geometry are rejected by anatomy export.

## Version 1 patch

A patch has exactly these five root fields. Both edit lists are required, even
when empty. Use the anatomy document's `patch_schema` for `schema`, and copy its
fingerprint and MuJoCo version:

```json
{
  "schema": 1,
  "fingerprint": "<the source export's SHA-256>",
  "mujoco_version": "<the source export's version>",
  "sites": [{"name": "BRA-P2", "pos": [-0.002, -0.024, 0.001]}],
  "inertias": [{"name": "r_ulna_radius_hand", "mass": 1.6,
                "com": [0.0, -0.18, 0.0],
                "inertia": [[0.02, 0.001, 0.0], [0.001, 0.03, 0.0], [0.0, 0.0, 0.04]]}]
}
```

Site `pos` and inertial `com` use the named object's body-local axes. Inertia is
a symmetric positive-definite 3×3 tensor about the COM, in body axes; its
principal moments must satisfy the triangle inequality. Mass must be positive.
Names must identify existing named source sites/bodies. World inertia,
duplicates, unknown fields and unsupported or compiler-ignored edits are refused.
Framed sites retain their compiled orientation when moved into body-local axes.

The fingerprint binds the patch to the complete compiled source and exact
MuJoCo version, including source mesh paths and settings. A changed source or
version requires a new export and patch. `apply_model_patch` validates a copied
spec and leaves the caller's spec unchanged on success or failure. It preserves
muscle calibration; after changing a path or inertia, recheck native lengths,
moment arms and task/checkpoint compatibility.

For a homogeneous solid, `myo-sim[mesh]` enables `mesh_inertia(path, density)`
through trimesh. Supply a watertight, consistently oriented mesh in body-local
metres and a positive density in kg/m³. The caller must justify density and
exclude overlapping solids; the function does not infer tissue composition.

## Native archives

`save_model_archive` bundles referenced mesh, texture, heightfield and skin files,
writes a native zip and recompiles it with `MjSpec.from_zip` before publishing.
Asset lookup follows `strippath` and exposed compiler asset directories. An
attached child's relative files can use its exposed absolute mesh/texture
directory, but MuJoCo does not expose the child's owning `modelfiledir`.
MuJoCo 3.4 also lacks per-element compiler-directory access: use absolute asset
filenames or explicit bytes in the parent `spec.assets` across supported versions.
Unresolved child-relative paths raise `ValueError`; directories are never guessed
from the parent's working directory. Ambiguous basenames are refused.
Archive validation compares the reloaded native model and
native reference measurements with the supplied spec before replacing output.
Existing files require `overwrite=True`; failures preserve the destination and
source, and successful publication uses an atomic replacement.

Native XML has finite precision. Acceptance uses these explicit tolerances:

Explicit principal moments use the shared generated-XML precision helper, so
edited planar inertias remain on their physical triangle without enabling
`balanceinertia` or adjusting the source mass distribution.

| Quantity | Comparison |
|---|---|
| Other compiled floating-point arrays, option values and model statistics | Relative `1e-5`, absolute `1e-12`. |
| Geometric/pose arrays, including positions, rotations, sizes and `stat.center` | Relative `1e-5`, absolute `1e-6`. |
| Native physical tendon lengths | Relative `1e-5`, absolute `1e-6` m. |
| Native tendon Jacobians | Relative `1e-5`, absolute `1e-5`. |
| Integer fields, object names and custom text bytes | Exact. |

Reference evaluation uses the coupled default pose, up to two stored keyframes,
and two offsets of independent scalar coordinates: ±10% of limited ranges, or
±0.1 for unlimited coordinates. It checks
tendon lengths and all tendon Jacobian columns at those poses. Sampled lengths
and Jacobians must be finite before comparison. Evaluate
task-specific stability and other postures separately. Derived frame-optimization
flags and external asset-path spellings are treated as representation details.
Derived `mesh_poly*` polygon caches are exempt only when every mesh geom has
`contype = conaffinity = 0` and no explicit contact pair references a mesh geom.
Raw mesh vertices/faces, collision flags, poses and inferred inertias remain in
the equivalence check.
Custom text retains tabs, newlines and carriage returns through XML serialization.

Sources whose compilation actually fuses static bodies are refused for patches
and archives because native serialization can lose their transforms. Disable
`fusestatic` and export that source again. Enabling the option without body
fusion remains supported.

## Model instances and canonical source

An instance patch customizes the copied model used by one experiment. It can
change a generated left-side site without changing the canonical right-side
source, and it does not write component XML or redefine the mirroring policy.

For a canonical contribution, transfer accepted site/body edits into the owning
`*_chain.xml`. For a right-only fragment mirrored at composition time, edit the
right source and rebuild the generated left side through the existing pipeline;
keep parts authored bilaterally in their existing source fragment. Use the
[origin record](#version-2-anatomy-document) to locate the owning source, and express a
generated-left edit in the canonical right source's local frame before transfer.
Then follow the
[model authoring guide](model-authoring.md) and
[engineering standards](engineering-standards.md), rebuild composed models and
run the relevant symmetry tests and fast gate. Generated full-model XML remains
a viewing/compatibility snapshot.
