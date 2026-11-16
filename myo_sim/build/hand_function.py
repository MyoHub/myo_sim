"""Table of the functional hand validation (McFarland et al. 2023 protocols): ``data/hand_function.json``.

The protocols themselves are in ``scripts/validate_hand_function.py``; the tests are
``tests/test_hand_grip.py``, ``tests/test_hand_pinch.py`` and ``tests/test_hand_tenodesis.py``.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

HAND_FUNCTION_DATA_PATH = Path(__file__).with_name("data") / "hand_function.json"


def load_hand_function_table(path: Path | str | None = None) -> dict[str, Any]:
    """Read the hand-function validation table: protocols, targets, tolerances.

    Args:
        path: Table to read; the packaged one by default.

    Returns:
        The parsed table.
    """
    return json.loads(Path(path or HAND_FUNCTION_DATA_PATH).read_text())
