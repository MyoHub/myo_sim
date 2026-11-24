# Review rules

These rules apply to every reviewer of a `myo_sim` pull request: people, Claude Code Review, PR Steward, Codex and the local
`scripts/claude_review.sh`. `AGENTS.md` / `CLAUDE.md` point here. MyoSuite's `REVIEW.md` states the same split from the
consumer side; keep the two in step.

## Scope

- Automated review covers defects in the proposed source, tests and documentation. **Do not emit findings about commit Author,
  Committer, AI attribution or CLA identity.** The CLA rule in `AGENTS.md` (no AI co-author trailers) applies when creating
  commits.

## Ownership: what belongs in myo_sim

- `myo_sim` owns reusable model construction and anatomical/physical transforms: MJCF/XML model definitions and their
  composition, geometry and segment scaling, mass and inertia, muscle and tendon parameters and model-only calibration, and
  the physics/plugin primitives that act on the model.
- MyoSuite owns measurement and source adapters, scientific analysis, experiment and subject-fitting workflows, task
  configuration, rewards, observations, training and CPU/mjlab wiring. A pull request here must not bring that logic into the
  library, even as a convenience helper.
- Before adding a helper, class or solver, search this repository and MyoSuite for an existing one. When MyoSuite already
  carries a model transform that belongs here, move it (with a MyoSuite PR that switches to the new API behind a version gate)
  instead of writing a second implementation.
- Prefer the smallest compatible change: reuse composition and physics primitives and public APIs, remove superseded code,
  and keep public names, XML file names and serialized assets compatible or document the migration.

## Authorship of pull requests

- A pull request is opened by a person, under their own GitHub account, who owns its design. Agents may write code locally for
  that person, and PR Steward may push follow-up fixes, but an agent or bot account must not open the pull request.
- A bot's clean review is not a human approval of design or implementation.

## What a review summary states

- Why the change belongs in `myo_sim`, the existing primitives reused (with paths, in both repositories), the new logic that
  was necessary, the code removed or simplified, compatibility consequences, and the checks run versus not run.
