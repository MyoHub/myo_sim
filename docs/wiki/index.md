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
- `docs/wiki/hand-cmc-joints.md` — fourth and fifth carpometacarpal joints: centres, axes, ranges, the coupling and its caveats, moved sites, end-range springs
- `docs/wiki/hand-extrinsic-muscles.md` — extrinsic hand muscles (S7): juncturae tendinum (nodes, links, fit), FCU over the pisiform and hamate hook, FDS5's force, the wrist wrap side sites, APL at the thumb CMC (S2c); the ECU and FDP-link trials
- `docs/wiki/extensor-mechanism.md` — finger extensor mechanism (option a): junction sites, pulley branches and their fractions, the oblique retinacular ligament, fits and limitations
- `docs/wiki/passive-moments-and-thenar.md` — passive joint moments and the hand intrinsic muscles defined from data (generated XML blocks, path and residual fits): row statuses, sources, assumptions
- `docs/wiki/passive-muscles-tenodesis.md` — passive tension of the extrinsic finger, thumb and wrist muscles (tenodesis): method, fitted slack lengths and fpmax in `myoarm_r_muscle.xml`, active force-length constraint, validation
- `docs/wiki/hand-function-validation.md` — functional hand tests after McFarland et al. 2023 (tenodesis, maximum grip, lateral pinch): method, results per model version, known issues
- `docs/wiki/hand-contacts-and-pads.md` — fingertip, palm, thenar and hypothenar pads (fitted to Dallard 2018 and to the fist) and the hand self-contact pairs (posture scan, selection, parameters, checks)
- `docs/wiki/hand-literature-comparison.md` — the hand against published data after each step of the hand rebuild (generated tables: model value, literature value ± spread, reference, protocol)
- `docs/wiki/testing-guide.md` — test suite map and fast gate command
- `docs/wiki/log.md` — append-only wiki maintenance log

Open issues: https://github.com/MyoHub/myo_sim/issues
