# Model README Authoring Workflow

This document describes when to update a model README, provides the canonical README template, and gives step-by-step instructions for agents generating or refreshing a model README.

---

## When to update a model README

Update (or create) a model README when any of the following occur:

- A new body-part model is added to the repository.
- A manual adjustment is made to the model (wrapping change, parameter re-fit, joint range edit, inertia correction).
- An open issue linked in the README is resolved.
- A paper describing or validating this model is published or posted as a preprint.

---

## README template

Every `myo_sim/models/<part>/README.md` must follow this template exactly. Do not add section headers not listed here; do not omit any section.

```markdown
# <ModelName>

<One sentence describing the body region and the reference model this was derived from.>

## Anatomical scope

| Property | Value |
|---|---|
| Degrees of freedom | <count — from m.njnt> |
| Actuators (muscles) | <count — from m.nu> |
| Body segments | <comma-separated list> |
| Primary joints | <comma-separated list> |

## Reference model

- **Source:** [<OpenSim model name>](<SimTK or GitHub URL>)
- **Paper:** <Author et al., Year> ([DOI](<link>))

## Fidelity

What is preserved exactly, what is approximated, and what is deliberately different.
Quantitative where possible: "moment arms match reference within 2 mm across full ROM".

## Known limitations

- [ ] #<issue number> — <description>

(Write "None currently tracked." if none.)

## Manual adjustments

Bulleted list. Each entry: what changed, which element, why.
Do not copy the conversion pipeline boilerplate — see CONTRIBUTING.md.

## Changelog

Most recent first. Format: `**YYYY-MM-DD** — <what changed and why>`.

## Citation

BibTeX for the paper introducing or validating this model.
If covered by the main repo citation, write "See repository README."
```

---

## Step-by-step agentic authoring instructions

### Step 1 — Read structured data from XML

Use programmatic queries to populate the Anatomical scope table. Never guess counts from reading XML manually.

```bash
# DoF count and actuator count (authoritative)
uv run python -c "
import mujoco, myo_sim
m = mujoco.MjModel.from_xml_path(str(myo_sim.MODELS_DIR / '<part>/<model>.xml'))
print('njnt:', m.njnt, 'nu:', m.nu)
"

# Body segment names
grep '<body ' myo_sim/models/<part>/assets/*_chain.xml | grep -oP 'name="[^"]+"' | sed 's/name=//;s/"//g' | sort -u

# Primary joint names (first 20)
grep '<joint ' myo_sim/models/<part>/assets/*_chain.xml | grep -oP 'name="[^"]+"' | sed 's/name=//;s/"//g' | sort -u | head -20
```

### Step 2 — Read existing prose

Extract these items from the current README if one exists; preserve them verbatim:

- The SimTK or OpenSim URL in the Reference model section.
- The Manual adjustments section in its entirety. This section records human decisions and must not be paraphrased or summarised.
- Any existing BibTeX entries in the Citation section.

### Step 3 — Query open issues

```bash
gh issue list --repo MyoHub/myo_sim --state open --search "<ModelName>" --json number,title
```

List each result as `- [ ] #<number> — <title>` in the Known limitations section. If the query returns no results, write "None currently tracked."

### Step 4 — Build changelog from git history

```bash
git log --follow --format="%ad %s" --date=short -- myo_sim/models/<part>/ | head -20
```

Format each relevant commit as `**YYYY-MM-DD** — <summary>`. Omit CI-only, lint, and dependency-bump commits. If the git log predates structured messages, write a single entry summarising the initial conversion date.

### Step 5 — Write, validate, and open PR

Before writing the final README:

- Verify that `njnt` and `nu` counts in the table match the programmatic output from Step 1.
- Verify that all URLs (SimTK, GitHub, DOI) resolve with a `curl -sI <url> | head -1` check. If a URL returns 4xx/5xx, flag it with `<!-- TODO: verify URL -->`.
- Remove future-tense promises ("will release", "will investigate") — convert them to GitHub issues or delete them.
- Fix known typos: "Maunal" → "Manual", "kitnematic" → "kinematic", "optimizaiton" → "optimization", "investage" → "investigate".

Once the README is written:

```bash
# Branch and PR
git checkout -b docs/readme-<part>
git add myo_sim/models/<part>/README.md
git commit -m "docs: refresh README for <ModelName>"
git push -u origin docs/readme-<part>
gh pr create --title "docs: refresh README for <ModelName>" \
  --body "Refresh README for <ModelName> using the standard template." \
  --base mm_refactor_mjspec
```

---

## Fields requiring human review

The following fields must not be filled by an agent without a human confirmation step. Leave a `<!-- TODO: review -->` comment in the placeholder rather than guessing.

- **Fidelity** — quantitative claims about moment arm accuracy, force-length match, or joint angle fidelity require the author to run validation benchmarks. An agent may quote figures from an existing paper but must cite the source inline.
- **Citation completeness** — if there are multiple papers that could be cited (e.g., the original OpenSim model paper and a subsequent myo_sim validation paper), a human must decide which to include.
- **Undocumented biomechanical issues** — if the agent identifies a discrepancy (e.g., a moment arm sign flip in the analysis plots) that is not yet tracked as a GitHub issue, it must open an issue and link it rather than silently adding it to the README.
