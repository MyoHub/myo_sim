# AGENTS.md

`myo_sim` is a packaged library of MuJoCo musculoskeletal XML model definitions used by [MyoSuite](https://github.com/facebookresearch/myoSuite).

Read `docs/wiki/index.md` before making any substantial change.

## Code Review Rules

### Repository ownership and reuse checks

- Apply the ownership and minimum-change review below to the actual diff, not just the PR description. For each new model transform or abstraction, search the existing primitives and cite the canonical symbol/path when flagging duplication or the wrong owner. Model scaling, mass/inertia and muscle/tendon transforms belong in `myo_sim`; source processing, subject-fitting workflows, task wiring and provenance may remain in MyoSuite. A thin compatibility facade is allowed; a copied algorithm behind a fallback is not a second canonical implementation.
- Check whether configuration, composition or a small extension can replace new code, and whether superseded code/callers are removed within a compatible migration. Flag unnecessary parallel solvers, wrappers or frameworks only with concrete equivalent behavior/constraints; do not demand consolidation of different numerical objectives or use line count alone as a defect.
- Check all applicable contracts and validation rules in this file and the linked engineering guides: dependency/version gates, public imports/serialization/checkpoint compatibility, scientific provenance and diagnostics, supported-backend parity, and method-specific training/artifact requirements. Inspect the implementation and evidence; passing lint or absent CI does not prove compliance. If another repository or experiment is unavailable, say that it is unverified rather than assuming it passed.
- Keep defect findings actionable with the affected code, existing primitive or violated contract, consequence and safe correction. Separate optional simplification from defects; use severity based on impact, never inflate it to force visibility. In a review summary, when supported, state ownership/reuse, compatibility and checks run/not run; do not treat a bot's clean review as human design or implementation approval.

## Quickstart

```bash
# Install
uv sync

# Run tests
uv run pytest tests/ -x -n auto --ignore=tests/test_equivalence.py

# Compose a bilateral model
uv run python -m myo_sim.build.compose --model myoarms

# Run a muscle-symmetry analysis script (standalone, not pytest)
cd tests && uv run python debug_muscle_leg.py
cd tests && uv run python debug_muscle_torso.py
cd tests && uv run python debug_muscle_bimanual.py
```

## Ownership and minimum-change review

- Decide the owner before adding code. `myo_sim` owns reusable model construction and anatomical/physical transforms: geometry and segment scaling, mass/inertia, muscle/tendon parameters and model-only calibration. MyoSuite owns measurement/source adapters, scientific analysis and experiment workflows, task configuration and CPU/mjlab wiring. A workflow estimating subject parameters may stay in MyoSuite; reusable anatomical/physical model transformations must use the canonical `myo_sim` API. Task settings, sensor calibration and experimental control parameters remain with their workflow.
- Search both repositories and the relevant dependency APIs before adding a helper, class, wrapper or solver. Record the existing symbols considered and why configuration, composition or a small extension does not suffice. Review behavior and constraints, not a guessed similarity percentage.
- Prefer the smallest compatible change. Reuse the current composition/physics primitives and public APIs; remove superseded code and update callers within the compatible migration; document any staged compatibility period. Keep only deliberate compatibility facades, not a second algorithm hidden behind a fallback. Preserve distinct numerical objectives when equivalence has not been demonstrated.
- If the required library API is not released, link the prerequisite `myo_sim` PR and gate the consumer on a version that provides it. Do not copy the implementation into MyoSuite to bypass that dependency.
- Every code PR and updated review summary must state the owner, reused primitives, necessary new logic and code removed/simplified (or why none can be removed), with compatibility and validation evidence. Missing evidence needs revision before approval. Added/deleted line counts inform review; they are not a correctness quota.
- Reviewers must check the actual diff and imports against that evidence. Existing lint/tests do not establish absence of duplication or correct ownership; do not describe them as enforcing this semantic review gate.

## Commits

- **Never add an AI assistant (e.g. Claude, Anthropic) as a commit co-author** — via `Co-Authored-By` trailers or otherwise. This repo's CLA check requires every commit author/co-author to have signed the CLA, and AI tools cannot sign it, so AI co-author trailers break the check.

## More detail

Everything else — model architecture, file-naming conventions, XML standards, Python style, PR rules, naming conventions, build/composition pipeline, and muscle analysis utilities — is documented in `docs/wiki/`. Start with `docs/wiki/index.md`.
