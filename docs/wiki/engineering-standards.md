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

**Multi-part composition (MuJoCo ≥ 3.8).** MuJoCo 3.8 rejects a compiled model that contains two default class names with the same identifier, whether from the top-level `class="main"` or from any nested `<default class="...">`. Two issues arise when `myotorso_assets.xml` and `myoarm_r_assets.xml` are combined via static `<include>`:

1. Both declare `<default class="main">` — the top-level required name, which cannot be renamed.
2. Both declare `<default class="wrap">` and `<default class="marker">` — duplicate nested class names.

Either collision alone is enough for MuJoCo 3.8 to raise `"repeated default class name"`.

**Rule:** never write a static top-level XML that combines `myotorso_assets.xml` with `myoarm_r_assets.xml`. More generally, when adding a new `*_assets.xml`, ensure its class names do not collide with those of any other asset file it could be included alongside. Asset files with bare `<default>` and no `class="main"` (`leg/assets/myolegs_assets.xml`, `torso/assets/myotorso_abdomen_assets.xml`, `head/assets/myohead_simple_assets.xml`) are safe to combine. Multi-part assemblies must use `build_model()` in `myo_sim/build/compose.py`, which uses MjSpec attachment with independent per-child namespaces. All current standalone XMLs comply.

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
