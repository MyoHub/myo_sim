# Wiki Index

Read this file before making any substantial change to this repository.

## Source-of-truth priority

Code > CLAUDE.md > wiki. If a wiki page contradicts the code, the code wins. When you discover a discrepancy, update the wiki in the same changeset as the code.

## When a wiki page is stale

Update it in the same PR as the code change. Append an entry to `docs/wiki/log.md` describing what changed and why.

## Pages

- `docs/wiki/index.md` — this file, the entry point for all agents and contributors
- `docs/wiki/repository-map.md` — where things live in the repo: directories, key files, package layout
- `docs/wiki/engineering-standards.md` — XML/MJCF conventions, Python style, testing rules, PR and commit rules
- `docs/wiki/model-authoring.md` — how to add or edit a model part (skeleton, muscles, contacts)
- `docs/wiki/model-readme-template.md` — agentic workflow for writing per-model README files
- `docs/wiki/build-and-composition.md` — how MjSpec compose.py works, BuildStrategy enum, contact injection
- `docs/wiki/testing-guide.md` — test suite map, when to run which tests, equivalence test caveat
- `docs/wiki/issue-tracker.md` — open GitHub issues, their status, and recommended sequencing
- `docs/wiki/agent-workflow.md` — the required step-by-step workflow for any agent taking on a task
- `docs/wiki/log.md` — append-only log of wiki maintenance events
