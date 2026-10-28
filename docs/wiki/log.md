# Wiki Log

Append-only maintenance log, most recent first.

## 2026-10-28 — Model policy: review round
Changed: docs/wiki/model-authoring.md
Why: load_spec takes no user build_kwargs (custom assemblies register their own recipe); generic post-processing functions instead of per-model simplification variants; fit scripts run only at development time; parameter updates within a release through MjSpec functions; passive joint properties only where data and a validation support them; downstream packages state compatible versions; the musclemimic equivalence comparison retires once synchronised; target structure without hand pruning.

## 2026-10-27 — Model policy and maintenance rules
Changed: docs/wiki/model-authoring.md, engineering-standards.md, index.md, repository-map.md, agent-workflow.md, CLAUDE.md, CHANGELOG.md
Why: state one reference model per part, no feature flags, variants as registered models, provenance and validation rules.
