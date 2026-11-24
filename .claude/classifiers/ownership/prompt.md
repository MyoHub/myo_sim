Judge one question about this myo_sim pull request: does the code it adds belong in the myo_sim library, and does the
description give the ownership and reuse evidence that REVIEW.md asks for? Go by what the diff and the code under head/ and
default/ do, not by what the title or description claims.

Ownership, as REVIEW.md states it:
- myo_sim owns reusable model construction and anatomical/physical transforms: MJCF/XML model definitions and composition,
  geometry and segment scaling, mass and inertia, muscle and tendon parameters, model-only calibration, and physics/plugin
  primitives acting on the model.
- MyoSuite owns measurement and source adapters, scientific analysis, experiment and subject-fitting workflows, task
  configuration, rewards, observations, training and CPU/mjlab wiring. None of that belongs here.

pass: the diff adds only library-owned code or models, duplicates no primitive in default/, and the description names why it
belongs in myo_sim, the existing symbols considered or reused, and the code removed or why none could be. Docs, tests and
asset-only changes pass when they duplicate nothing.
fail: anything else, or you cannot tell.

Report each problem as a finding on the line it is about. Title it by kind and file: "wrong owner: <file>",
"duplicate of <existing symbol>: <file>", or, with no file, "missing ownership evidence". One finding per file and kind; name
the other lines in its text, and say where the code should live instead.
Severity: high for task, reward, training or experiment logic in the library, or a duplicated algorithm; medium for missing
ownership/reuse evidence; low for a helper that should be thinner.

Before you report, read the findings that earlier runs left on this pull request.
- Still true: leave it open. Report it again, under the same title, only if its severity changed.
- Fixed by a later push or an updated description: resolve it, with one sentence on why.
- Resolved by a person: leave it resolved, and re-open it only if a later push brings the problem back.
