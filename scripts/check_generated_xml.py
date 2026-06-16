"""Pre-commit hook: reject manual edits to generated XML artefacts.

Triggered when myohand_r.xml is staged. Re-runs --generate into a temp file
and compares; fails if the staged content differs from the generator output,
which means the file was edited by hand rather than through the pipeline.

Usage (wired automatically via .pre-commit-config.yaml):
    uv run python scripts/check_generated_xml.py
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

GENERATED = Path("myo_sim/models/hand/myohand_r.xml")


def staged_files() -> set[str]:
    result = subprocess.run(
        ["git", "diff", "--cached", "--name-only"],
        capture_output=True,
        text=True,
        check=True,
    )
    return set(result.stdout.splitlines())


def main() -> int:
    if str(GENERATED) not in staged_files():
        return 0

    # Get the staged content of the file.
    staged = subprocess.run(
        ["git", "show", f":{GENERATED}"],
        capture_output=True,
        check=True,
    ).stdout

    # Re-generate into a temp file, then read it back.
    with tempfile.TemporaryDirectory():
        subprocess.run(
            [sys.executable, "-m", "myo_sim.build.compose", "--generate"],
            check=True,
            capture_output=True,
        )
        generated = GENERATED.read_bytes()

    if staged == generated:
        return 0

    print(
        f"ERROR: {GENERATED} appears to have been edited manually.\n"
        "This file is generated — do not edit it directly.\n"
        "To update it, run:\n"
        "    uv run python -m myo_sim.build.compose --generate\n"
        "then stage the result.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
