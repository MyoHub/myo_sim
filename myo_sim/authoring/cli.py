# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Export anatomy or apply selected instance edits through versioned files."""

from __future__ import annotations

import argparse
import contextlib
import json
import tempfile
from collections.abc import Iterator, Sequence
from pathlib import Path

import mujoco

import myo_sim
from myo_sim import authoring


def _source(args: argparse.Namespace) -> mujoco.MjSpec:
    if args.model:
        return myo_sim.load_spec(args.model)
    path = args.xml.resolve()
    return mujoco.MjSpec.from_zip(path) if path.suffix.lower() == ".zip" else mujoco.MjSpec.from_file(str(path))


@contextlib.contextmanager
def _output(path: Path, overwrite: bool) -> Iterator[Path]:
    destination = path.resolve()

    def check() -> None:
        if destination.exists() and (not overwrite or not destination.is_file()):
            raise FileExistsError(f"Output exists: {destination}; use --overwrite to replace a file")

    check()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as folder:
        staged = Path(folder) / destination.name
        yield staged
        check()
        if not staged.is_file():
            raise RuntimeError("The operation did not produce its requested output")
        staged.replace(destination)


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
        with _output(args.output, args.overwrite) as staged:
            if args.command == "export":
                model = _source(args).compile()
                poses = None if args.joint_sweep is None else authoring.joint_sweep(model, args.joint_sweep, args.samples)
                authoring.export_model(model, staged, qpos=poses, source_model=args.model)
            else:
                patch = json.loads(args.patch.read_text(encoding="utf-8"))
                spec = authoring.apply_model_patch(_source(args), patch)
                authoring.save_model_archive(spec, staged)
    except (KeyError, ValueError, OSError, RuntimeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
