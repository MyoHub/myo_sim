# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Stage complete outputs and publish without clobbering concurrent writers."""

from __future__ import annotations

import contextlib
import os
import tempfile
from collections.abc import Iterator
from pathlib import Path


def _check_destination(destination: Path, overwrite: bool) -> None:
    if destination.exists() and (not overwrite or not destination.is_file()):
        raise FileExistsError(f"Output exists: {destination}; use overwrite permission to replace a file")


@contextlib.contextmanager
def _output(path: str | Path, overwrite: bool = False) -> Iterator[Path]:
    """Publish a staged file atomically, with no-clobber semantics unless authorized."""
    destination = Path(path).resolve()
    _check_destination(destination, overwrite)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as folder:
        staged = Path(folder) / destination.name
        yield staged
        _check_destination(destination, overwrite)
        if not staged.is_file():
            raise RuntimeError("The operation did not produce its requested output")
        if overwrite:
            staged.replace(destination)
        elif os.name == "nt":
            # Windows rename fails atomically if the destination already exists.
            staged.rename(destination)
        else:
            # Staging under the destination parent guarantees the same filesystem.
            os.link(staged, destination)
