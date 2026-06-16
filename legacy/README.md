# Legacy Models

This directory contains models that are **not part of the active `myo_sim` package** and are provided here on a temporary basis only. Do not build new workflows that depend on these paths.

## Why legacy?

`myo_sim` is scoped to biological musculoskeletal models. Models in this directory either:

- Need to be properly integrated into the pip package (with correct mesh paths and package-relative includes), or
- Belong in a separate package (`myoassist`) that handles assistive/exoskeletal devices.

## Contents and migration plan

| Directory | Description | Tracking issue | Migration target |
|-----------|-------------|---------------|-----------------|
| `elbow/`  | 1-DOF and 2-DOF elbow with 6 muscles; exo/softexo variants | [#97](https://github.com/MyoHub/myo_sim/issues/97) | Biological variants → `myo_sim/models/elbow/`; softexo variants → `myoassist` |
| `finger/` | Passive, motor-actuated, and musculotendon finger models | [#99](https://github.com/MyoHub/myo_sim/issues/99) | `myo_sim/models/finger/` |
| `osl/`    | OSL prosthetic leg; uses shared leg bones with a prosthetic socket | [#100](https://github.com/MyoHub/myo_sim/issues/100) | `myoassist` |

## Using these models now

Paths have changed. If you previously referenced `myo_sim/elbow/`, `myo_sim/finger/`, or `myo_sim/osl/`, update to `myo_sim/legacy/<model>/`.

Once the migration issues above are resolved, these directories will be removed and the paths above will break. Pin to a tagged release if you need stability.
