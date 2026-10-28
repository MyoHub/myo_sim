# Wiki Index

Read this file before making any substantial change to this repository.

## Source-of-truth priority

Code > CLAUDE.md > wiki. If a wiki page contradicts the code, the code wins. When you discover a discrepancy, update the wiki in the same changeset as the code.

When a wiki page is stale, update it in the same PR as the code change and append an entry to `docs/wiki/log.md`.

## Pages

- `docs/wiki/repository-map.md` — where things live and where to make each kind of change
- `docs/wiki/engineering-standards.md` — XML/MJCF rules, Python style, PR rules
- `docs/wiki/agent-workflow.md` — required step-by-step workflow for any task
- `docs/wiki/model-authoring.md` — how to add or edit a model part
- `docs/wiki/model-readme-template.md` — agentic workflow for writing model READMEs
- `docs/wiki/build-and-composition.md` — MjSpec compose.py reference
- `docs/wiki/passive-moments-and-thenar.md` — passive joint moments and thenar muscles of the arm (generated XML blocks, residual fit): row statuses, sources, assumptions
- `docs/wiki/passive-muscles-tenodesis.md` — passive tension of the extrinsic finger, thumb and wrist muscles (tenodesis): method, fitted slack lengths and fpmax in `myoarm_r_muscle.xml`, active force-length constraint, validation
- `docs/wiki/hand-function-validation.md` — functional hand tests after McFarland et al. 2023 (tenodesis, maximum grip, lateral pinch): method, results per model version, known issues
- `docs/wiki/testing-guide.md` — test suite map and fast gate command
- `docs/wiki/log.md` — append-only wiki maintenance log

Open issues: https://github.com/MyoHub/myo_sim/issues
