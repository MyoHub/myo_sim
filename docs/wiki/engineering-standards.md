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

**Naming rule for `*_assets.xml` files:** all nested default class names must be scoped to the body part — never use generic names like `wrap`, `marker`, `muscle`, `coll`, `elbow`, `wrist`, or `fingers`. Use `myotorso_wrap`, `myoarm_wrap`, `myotorso_marker`, `myoarm_marker`, `myoarm_muscle`, `myoarm_elbow`, `myolegs_wrap`, `myolegs_coll`, etc. — one consistent `<part>_<role>` pattern per body part, applied to every nested class in the file, not just some of them.

**Composition rule:** multi-part assemblies must use `build_model()` in `myo_sim/build/compose.py`, which uses MjSpec attachment with independent per-child namespaces and sidesteps class name collisions entirely. Sharing `class="main"` across two included files is accepted by MuJoCo 3.8 — it is the nested sub-class names (e.g. `wrap`, `marker`) that must be unique. The naming rule above ensures this.

## Python

- Always use `uv run`, never bare `python`.
- Line length 128. `uv run ruff check .` and `uv run ruff format --check .` must pass before committing.
- Type hints on all new function signatures.
- No module-level code that loads MuJoCo models — it runs at pytest collection time.
- Search before writing: grep for existing helpers, check `BuildStrategy` enum, check `contacts/` before adding new files.

## PRs and commits

- Push to your fork; open PRs against `dev`, not `main`.
- PR body: plain prose, no headers, no emojis. Describe the problem, the change, and non-obvious tradeoffs.
- One logical change per commit. Amend before reviewers arrive; use new commits after.
