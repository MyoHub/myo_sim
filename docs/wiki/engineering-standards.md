# Engineering Standards

These rules apply to every change in this repository. Agents and contributors must follow them.

## XML / MJCF standards

**File suffix roles:**
- `*_chain.xml` — owns the skeleton for one body part: bodies, joints, geoms, and structural sites. This is the canonical kinematic definition; nothing else should redeclare these elements.
- `*_muscle.xml` — owns actuators (muscles) for one body part.
- `*_tendon.xml` — owns spatial tendon routing for one body part.
- `*_assets.xml` — owns mesh, material, texture, and default declarations.

**Site ownership:** Sites belong to the file that owns their parent body. Never declare a site in a different file from its parent body.

**No duplication:** Do not redefine a body, joint, or geom in more than one file. One canonical `*_chain.xml` per body part.

**Bilateral symmetry:** Derive the left side via MjSpec mirroring in `myo_sim/build/compose.py`. Do not maintain a hand-edited left XML alongside a right XML — the left is generated programmatically.

**Cross-part contacts:** Contact pairs that span two body parts go in `myo_sim/models/contacts/` and are injected by `add_contact_pairs()` in `build/compose.py`. Never embed cross-part contact pairs inside a part's own asset files.

**Numeric formatting:** Preserve existing numeric formatting unless the value itself is part of an intentional edit.

## Python standards

- Always invoke Python via `uv run`, never bare `python` or `python3`.
- Line length limit is 128 characters (enforced by Ruff).
- `uv run ruff check .` and `uv run ruff format --check .` must both pass before committing.
- Add type hints on all new function signatures.
- Do not write module-level code that loads or compiles MuJoCo models. Such code runs at pytest collection time and slows every `pytest` invocation.
- Before adding a new Python helper, grep for an existing implementation. Before adding a new `BuildStrategy`, check the `BuildStrategy` enum in `myo_sim/build/compose.py`. Before adding a new contact file, check `myo_sim/models/contacts/`.

## Testing standards

- Run `uv run pytest tests/ -x -n auto --ignore=tests/test_equivalence.py` before opening a PR.
- `tests/test_equivalence.py` requires the `musclemimic_models` package, which is not publicly available. Skip it in CI and in automated agent runs; run it locally only when working on equivalence-sensitive changes.
- Regression tests for path, registry, or naming changes must fail loudly if something moves unexpectedly.
- Prefer targeted tests over exhaustive edge-case coverage.

## PR and commit standards

- Push to your own fork (`Vittorio-Caggiano/myo_sim`), not directly to `MyoHub/myo_sim`.
- Open PRs against the `mm_refactor_mjspec` branch, not `main`, unless the change is genuinely branch-agnostic infrastructure.
- PR body: plain prose, no section headers, no emojis. Describe the problem, what the change does, and any non-obvious tradeoffs. Bullet points are fine.
- Do not hard-wrap PR or commit message text at 88 columns — GitHub renders these as flowing prose.
- One logical change per commit. Amend freely before reviewers arrive; use new commits after.
- When responding to PR review comments: reply to each comment individually, resolve addressed threads, and add a summary comment covering what was applied and what was intentionally skipped.
