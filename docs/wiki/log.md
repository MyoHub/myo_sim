# Wiki Log

Append-only maintenance log, most recent first.

## 2026-11-18 — Use native integer type codes in anatomy and archive checks
Changed: myo_sim/authoring/interchange.py, myo_sim/authoring/archive.py, tests/test_authoring_archive.py, tests/test_authoring_sdf.py
Why: some MuJoCo and NumPy wheel combinations skipped scalar joints, quaternion blocks and tendon wraps when comparing model array entries to Python enums.

## 2026-11-15 — Add passive joint mechanics transforms
Changed: myo_sim/physics/joint_mechanics.py, myo_sim/physics/__init__.py, tests/test_physics_joint_mechanics.py, docs/wiki/index.md, docs/wiki/model-parameters.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, CHANGELOG.md
Why: passive joint damping, stiffness and `qpos0`-anchored range of motion can be scaled on independent 1-DOF joints without fighting `mjEQ_JOINT` couplings or changing automatic muscle peak forces.

## 2026-11-15 — Add muscle activation time-constant transforms
Changed: myo_sim/physics/activation_dynamics.py, myo_sim/physics/__init__.py, tests/test_physics_activation_dynamics.py, docs/wiki/index.md, docs/wiki/model-parameters.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, CHANGELOG.md
Why: `mjDYN_MUSCLE` excitation-contraction rise and relaxation time constants (`dynprm[:, :2]`) can be scaled orthogonally on compiled models or editable specs without changing steady-state peak force, stateless muscles or geometry.

## 2026-11-15 — Muscle operating-range and moment-arm CI gates
Changed: tests/test_muscle_operating_range.py, docs/wiki/testing-guide.md, docs/wiki/log.md
Why: tendon, wrap or `lmax` edits can move muscles outside `[lmin, lmax]` with zero active contractile force while still compiling cleanly; fast dataset-free checks now gate active force capacity and moment-arm coupling across all composed models.

## 2026-11-15 — Add isometric MVC torque probing and muscle strength calibration
Changed: myo_sim/physics/muscle_strength.py, myo_sim/physics/__init__.py, tests/test_physics_muscle_strength.py, docs/wiki/index.md, docs/wiki/model-parameters.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, CHANGELOG.md
Why: move isometric MVC torque probing (`max_torque_terms`) and per-muscle bounded least-squares strength calibration (`calibrate_muscle_strength`, pure NumPy BVLS) from MyoHub/myosuite#649 into `myo_sim` so models can be personalized to measured joint torques without MyoSuite or SciPy.

## 2026-11-13 — Keep anatomy geometry stable at finite numeric extremes
Changed: myo_sim/authoring/interchange.py, tests/test_authoring_interchange_numeric.py, docs/wiki/authoring.md
Why: real-only pose validation prevents silent complex-value truncation, scaled quaternion normalization avoids overflow and underflow, and stable radial norms preserve inside-wrap flags across geometric scales without mutating source or caller arrays.

## 2026-11-13 — Publish authoring outputs without clobbering inputs
Changed: myo_sim/authoring/_files.py, myo_sim/authoring/cli.py, myo_sim/authoring/archive.py, tests/test_authoring_files.py, tests/test_authoring_cli.py, docs/wiki/authoring.md
Why: shared atomic publication closes the concurrent-writer overwrite gap, CLI output guards preserve XML and patch inputs, and content detection supports archive reuse and explicit in-place ZIP edits regardless of filename extension.

## 2026-11-13 — Preserve active native assets in model archives
Changed: myo_sim/authoring/archive.py, tests/test_authoring_archive_assets.py, docs/wiki/authoring.md
Why: archives now follow native texture-source and VFS key precedence and skin directories, preserve inline heightfield rows and precision and attached texture defaults, and protect referenced disk assets from replacement.

## 2026-11-12 — Add selected-body mass transforms
Changed: myo_sim/physics/body_mass.py, tests/test_physics_body_mass.py, docs/wiki/model-parameters.md, docs/wiki/index.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, CHANGELOG.md
Why: represented model mass and an explicit participant/reference ratio are different inputs; model-only copy transforms preserve geometry and force capacity while refreshing native constants.

## 2026-11-12 — Add model-only muscle strength transforms
Changed: myo_sim/physics/muscle_strength.py, tests/test_physics_muscle_strength.py, docs/wiki/model-parameters.md, docs/wiki/index.md
Why: native active/passive force scaling and automatic-force resolution can be reused without importing subject fitting, environments or MyoSuite.

## 2026-11-12 — Decode only exported muscle moment arms
Changed: myo_sim/authoring/interchange.py, tests/test_authoring.py
Why: export now reuses one muscle-only dense Jacobian buffer, avoiding unused motor rows and repeated full-actuator allocations while preserving native geared derivatives and muscle order.

## 2026-11-12 — Avoid recompiling empty validated patches
Changed: myo_sim/authoring/edits.py, tests/test_authoring_edits.py
Why: an empty patch can return its already-validated copied spec without a second native compilation, while retaining source, version, list-type and static-fusion checks.

## 2026-11-12 — Keep anatomy references and geometry explicit
Changed: myo_sim/authoring/interchange.py, tests/test_authoring_equalities.py, tests/test_authoring_sdf.py, docs/wiki/authoring.md
Why: equality IDs need native body/site namespaces for unambiguous resolution, and SDF plugin geometry must be rejected rather than represented only by its bounding size.

## 2026-11-12 — Preserve custom text in native archives
Changed: myo_sim/authoring/archive.py, tests/test_authoring_archive.py, docs/wiki/authoring.md
Why: native XML attribute whitespace normalization could silently replace custom-text tabs and line endings with spaces; archives now preserve and compare the exact compiled text bytes.

## 2026-11-12 — Keep authoring independent of editor integrations
Changed: myo_sim/authoring/, tests/test_authoring*.py, tests/authoring_support.py, pyproject.toml, uv.lock, .github/workflows/ci.yml, README.md, CHANGELOG.md, docs/wiki/authoring.md, docs/wiki/index.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md
Why: compiled anatomy, selected instance edits and native archive validation belong to a generic model-authoring contract; editor adapters and their documentation were removed while canonical chain ownership and mirroring remain unchanged.

## 2026-11-12 — Preserve material colors in anatomy exports
Changed: myo_sim/muskemo/interchange.py, tests/test_muskemo.py, docs/wiki/muskemo.md
Why: anatomy JSON used raw geom RGBA and lost native material colors and alpha; exports now match MuJoCo's material/default override behavior without changing the source model.

## 2026-11-11 — Linear XML composition copies
Changed: myo_sim/build/utils.py, tests/test_build_utils.py, docs/wiki/build-and-composition.md, docs/wiki/log.md
Why: recursive subtree copies and child-list searches repeated work during model loading; copy each node once while preserving include expansion and mirror output.

## 2026-11-11 — Optional source-model authoring with MuSkeMo
Changed: myo_sim/muskemo/, tests/test_muskemo*.py, tests/muskemo_support.py, pyproject.toml, uv.lock, .github/workflows/ci.yml, README.md, CHANGELOG.md, docs/wiki/muskemo*.md, docs/wiki/index.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md
Why: anatomy editing belongs with the source model library; export, Blender import, validation and selected edits now run without MyoSuite or a training dependency.

## 2026-11-11 — Preserve native sensor and contact settings
Changed: myo_sim/build/utils.py, tests/test_native_includes.py, docs/wiki/build-and-composition.md, docs/wiki/log.md
Why: include imports discarded sensor reference frames and common options, lost pair class defaults and rejected unnamed sensors on MuJoCo 3.4.

## 2026-11-12 — Skip compilation for empty posture selections
Changed: myo_sim/build/keyframes.py, tests/test_postures.py, docs/wiki/postures.md
Why: requesting no keyframes still compiled the spec, adding model-build overhead and rejecting unfinished assemblies despite making no edits.

## 2026-11-12 — Copy uncoupled functional projections
Changed: myo_sim/physics/joint_coupling.py, tests/test_physics_joint_coupling.py
Why: the functional projection returned its inputs when no joint equalities were active, so editing an output could mutate the original state; empty projections now return independent arrays while preserving Torch gradients.

## 2026-11-14 — Reject unsupported sensor tags in add_sensors
Changed: myo_sim/build/utils.py, tests/test_sensor_includes.py, docs/wiki/build-and-composition.md, docs/wiki/testing-guide.md, CHANGELOG.md
Why: `add_sensors` skipped every sensor tag outside its table, so a `<gyro>` or `<framepos>` in an include vanished from the model without a message.

## 2026-11-11 — Keep interpreter caches out of distributions
Changed: pyproject.toml, MANIFEST.in
Why: recursive build-package data globs bundled local Python bytecode; explicitly include build documentation and posture tables, and exclude cached bytecode from both distribution formats while retaining Python sources and model assets.

## 2026-11-11 — Preserve the MuJoCo CI matrix overrides
Changed: .github/workflows/ci.yml, docs/wiki/testing-guide.md
Why: ordinary uv run restored locked MuJoCo 3.9 after the matrix installed its requested version; reporting and tests now keep the override, and latest stays below MuJoCo 4.

## 2026-11-11 — Preserve empty torch fatigue selections
Changed: myo_sim/physics/fatigue.py, tests/test_physics_fatigue.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: the torch model factory used every actuator as a fallback for an empty muscle selection, so motor-only models and explicit empty selections disagreed with the CPU factory and could not reset with an empty fatigue vector.

## 2026-11-11 — Preserve complete imported keyframe state
Changed: myo_sim/build/utils.py, tests/test_keyframe_import.py, CHANGELOG.md, docs/wiki/build-and-composition.md, docs/wiki/testing-guide.md
Why: spec-level keyframe import discarded activation, control and mocap state and rejected unnamed keys even though native MJCF keyframes support them.

## 2026-11-12 — Own Torch fatigue reset values before state writes
Changed: myo_sim/physics/fatigue.py, tests/test_torch_fatigue_reset_ownership.py, tests/test_physics_fatigue.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: a reset vector viewing live MA was overwritten before MF received it, selected-row resets could reject a valid MF view due to overlapping storage, and float32 reversed NumPy vectors retained unsupported negative strides during conversion.

## 2026-11-11 — Reset Torch fatigue directly from tensors
Changed: myo_sim/physics/fatigue.py, tests/test_physics_fatigue.py
Why: converting reset vectors through NumPy rejected device tensors and tensors requiring gradients; direct Torch conversion preserves the state device, float32 dtype and vector shape checks.

## 2026-11-11 — Snapshot differentiable Torch fatigue state
Changed: myo_sim/physics/fatigue.py, tests/test_physics_fatigue.py
Why: serializing a state after a gradient-tracked step raised instead of producing a checkpoint; snapshots now detach only the exported values and preserve live gradients.

## 2026-11-11 — Validate NumPy fatigue state dimensions before updates
Changed: myo_sim/physics/fatigue.py, tests/test_physics_fatigue.py
Why: multidimensional reset vectors and incompatible checkpoints could replace fixed-size muscle compartments or partially overwrite state before failing; validate all inputs before mutation.

## 2026-11-11 — Bound compiled fatigue timestep caching
Changed: myo_sim/physics/fatigue.py, tests/test_physics_fatigue_cache.py
Why: varying timesteps retained a scalar Torch tensor for every distinct value for the process lifetime; a bounded LRU keeps recent immutable scalars while preserving graph reuse.

## 2026-11-11 — Keep zero-rate fatigue gradients finite
Changed: myo_sim/physics/fatigue.py, tests/test_physics_fatigue_gradients.py
Why: unused slow-mode switching calculations overflowed float32 backward passes when fatigue and recovery were disabled, even for a zero-length step; inactive divisions now use benign operands.

## 2026-11-12 — Validate Torch fatigue restores before mutation
Changed: myo_sim/physics/fatigue.py, tests/test_torch_fatigue_restore.py, docs/wiki/testing-guide.md, CHANGELOG.md
Why: a missing, nonnumeric or incompatible later compartment in a checkpoint previously left earlier live compartments overwritten after the restore failed.

## 2026-11-12 — Preserve inertia precision in generated XML
Changed: myo_sim/build/compose.py, tests/test_xml_inertia_precision.py, docs/wiki/build-and-composition.md
Why: six-digit native formatting can move an edited planar body's principal moments past the inertia triangle; generated XML now preserves the compiled moments without balancing or changing them.

## 2026-11-05 — Stay below MuJoCo 4
Changed: pyproject.toml, uv.lock, CHANGELOG.md
Why: MuJoCo 4 changes side-site semantics and moves some muscle paths, and the upcoming release would otherwise install it. The cap goes once the models are checked against the final 4.0.

## 2026-11-05 — Tag and GitHub release after PyPI publication
Changed: .github/workflows/publish.yml, docs/wiki/repository-map.md, docs/wiki/log.md
Why: one manual main-branch run now publishes the package and releases the same commit and artifacts on GitHub; failed release jobs can resume without another PyPI upload.

## 2026-11-04 — mujoco-warp version warning
Changed: myo_sim/build/warp_version.py, myo_sim/__init__.py, myo_sim/models/arm/README.md, tests/test_warp_version_warning.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: the arm README lists a mujoco-warp limitation that only a parity test on a GPU would reveal; loading a model now says so when the installed version is affected.

## 2026-11-04 — Posture API in place of the standing keyframe helpers
Changed: myo_sim/build/keyframes.py, myo_sim/__init__.py, myo_sim/build/compose.py, scripts/generate_postures.py, myo_sim/build/data/postures.json, tests/test_postures.py, docs/wiki/postures.md, CHANGELOG.md
Why: review of the release asked for keyframes to cover postures in general, so 0.2.4 ships the general API from the start, with `standing` as its only posture. No release ever had the old names, so nothing needs to keep working.

## 2026-11-03 — Standing keyframe, 0.2.4
Changed: scripts/generate_standing_keyframes.py, myo_sim/build/keyframes.py, myo_sim/build/data/standing_keyframes.json, myo_sim/build/compose.py, myo_sim/__init__.py, tests/test_standing_keyframe.py, docs/wiki/standing-keyframe.md, docs/wiki/index.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, docs/wiki/build-and-composition.md, CHANGELOG.md, pyproject.toml, uv.lock
Why: qpos0 of the free-standing models violates the knee couplings and overlaps the arms with the thorax, so they had no clean start pose; the keyframe adds one without changing any model parameter.

## 2026-11-03 — Composition fixes from #150
Changed: myo_sim/build/compose.py, myo_sim/build/utils.py, tests/test_build_registry.py, tests/test_contact_paths.py, tests/test_legs_abdomen_build.py, tests/test_mirror_symmetry.py, tests/test_equivalence.py, scripts/render_composed_models.py, README.md, CHANGELOG.md, docs/wiki/build-and-composition.md, docs/wiki/engineering-standards.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md
Why: the build_kwargs leak, the pairs kept by disable_finger_collisions, the abdomen variants' contacts and heading, the mirrored cameras, and docs and a test that still used the removed build_model().

## 2026-10-28 — BIClong no longer wraps its elbow sphere
Changed: myo_sim/models/arm/assets/myoarm_r_tendon.xml, tests/test_model_stability.py, tests/test_tendon_path_continuity.py, tests/data/myofullbody_biclong_elbow_graze.json, docs/wiki/testing-guide.md, myo_sim/models/arm/README.md, CHANGELOG.md
Why: The wrap only acted past the elbow extension limit, where it gave NaN BIClong lengths (MuJoCo < 3.16) and 15-20 mm length jumps in random-activation rollouts of myofullbody.

## 2026-10-28 — CPU vs mujoco-warp tendon length parity test
Changed: tests/test_warp_parity.py, docs/wiki/testing-guide.md, myo_sim/models/arm/README.md
Why: EDC2 and the other tendons with a sidesite inside their wrap geom differed between CPU MuJoCo and mujoco-warp < 3.16 by up to about 3 mm (float32 inside-wrap fallback, google-deepmind/mujoco_warp#1713), so every hand and arm tendon is now checked on both backends.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.
