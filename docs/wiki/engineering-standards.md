# Engineering Standards

These rules apply to every change in this repository.

## XML / MJCF

- `*_chain.xml` — skeleton for one body part: bodies, joints, geoms, structural sites.
- `*_muscle.xml` — actuators only (`<general>` or `<muscle>`). No sites, bodies, or tendons.
- `*_tendon.xml` — spatial tendon routing. Sites referenced here must be defined in `*_chain.xml`.
- `*_assets.xml` — mesh, material, texture, and default declarations.

**Site ownership.** Sites belong to the file that owns their parent body. Never declare a site in a different file from its parent body.

**No duplication.** One canonical `*_chain.xml` per body part. Do not redefine bodies or joints across files.

**Bilateral symmetry.** Derive the left side via MjSpec mirroring in `build/compose.py`. Never maintain a hand-edited left XML.

**Cross-part contacts.** Contact pairs spanning two parts go in `myo_sim/models/contacts/`, injected by `add_contact_pairs()` in `build/compose.py`. Never embed them in a part's assets.

**Numeric formatting.** Preserve existing numeric formatting unless the value itself is part of an intentional edit.

**Multi-part composition (MuJoCo ≥ 3.8).** MuJoCo 3.8 rejects a compiled model where two included files declare a default class with the same name. The name `"main"` is required for the top-level default and cannot be renamed.

**Naming rule for `*_assets.xml` files:** all nested default class names must be scoped to the body part — never use generic names like `wrap` or `marker`. Use `myotorso_wrap`, `myoarm_wrap`, `myotorso_marker`, `myoarm_marker`, etc. The leg assets already follow this (`myoleg_wrap`, `myo_leg_marker`).

**Composition rule:** never write a static top-level XML that includes two or more `*_assets.xml` files that each declare `<default class="main">`. Multi-part assemblies must use `build_model()` in `myo_sim/build/compose.py`, which uses MjSpec attachment with independent per-child namespaces and is not affected by class name collisions. Asset files with bare `<default>` (no `class="main"`) are safe to combine: `leg/assets/myolegs_assets.xml`, `torso/assets/myotorso_abdomen_assets.xml`, `head/assets/myohead_simple_assets.xml`.

## Python

- Always use `uv run`, never bare `python`.
- Line length 128. `uv run ruff check .` and `uv run ruff format --check .` must pass before committing.
- Type hints on all new function signatures.
- No module-level code that loads MuJoCo models — it runs at pytest collection time.
- Search before writing: grep for existing helpers, check `BuildStrategy` enum, check `contacts/` before adding new files.

## PRs and commits

- Push to your fork; open PRs against `mm_refactor_mjspec`, not `main`.
- PR body: plain prose, no headers, no emojis. Describe the problem, the change, and non-obvious tradeoffs.
- One logical change per commit. Amend before reviewers arrive; use new commits after.
