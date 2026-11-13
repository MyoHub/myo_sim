# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Export anatomy or apply selected instance edits through versioned files."""

from __future__ import annotations

import argparse
import json
import zipfile
from collections.abc import Sequence
from pathlib import Path

import mujoco

import myo_sim
from myo_sim import authoring

from ._files import _check_destination, _output


def _source(args: argparse.Namespace) -> mujoco.MjSpec:
    if args.model:
        return myo_sim.load_spec(args.model)
    path = args.xml.resolve()
    return mujoco.MjSpec.from_zip(path) if zipfile.is_zipfile(path) else mujoco.MjSpec.from_file(str(path))


def main(argv: Sequence[str] | None = None) -> int:
    """Run the anatomy/edit CLI; all outputs require explicit overwrite permission."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    export = commands.add_parser("export", help="Export resolved anatomy and optional joint-sweep poses")
    apply = commands.add_parser("apply", help="Apply selected JSON edits and save a complete model zip")
    for command in (export, apply):
        source = command.add_mutually_exclusive_group(required=True)
        source.add_argument("--model", help="myo_sim model name")
        source.add_argument("--xml", type=Path, help="Source XML, or a prior native model zip")
    export.add_argument("--joint-sweep", help="Independent limited joint to sweep")
    export.add_argument("--samples", type=int, default=21)
    apply.add_argument("--patch", type=Path, required=True)
    for command in (export, apply):
        command.add_argument("--output", type=Path, required=True)
        command.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    try:
        destination = args.output.resolve()
        _check_destination(destination, args.overwrite)
        if args.command == "apply" and destination == args.patch.resolve():
            raise ValueError("Output must differ from the patch input")
        if args.xml is not None and destination == args.xml.resolve():
            if args.command != "apply" or not zipfile.is_zipfile(args.xml):
                raise ValueError("Output must differ from the source input, except for in-place archive apply")
        if args.command == "export":
            with _output(destination, args.overwrite) as staged:
                model = _source(args).compile()
                poses = None if args.joint_sweep is None else authoring.joint_sweep(model, args.joint_sweep, args.samples)
                authoring.export_model(model, staged, qpos=poses, source_model=args.model)
        else:
            patch = json.loads(args.patch.read_text(encoding="utf-8"))
            spec = authoring.apply_model_patch(_source(args), patch)
            authoring.save_model_archive(spec, destination, overwrite=args.overwrite)
    except (KeyError, ValueError, OSError, RuntimeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
