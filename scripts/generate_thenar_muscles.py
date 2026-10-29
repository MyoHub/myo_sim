"""Write the thenar muscles of ``build/data/thenar_muscles.json`` into the right-arm XML.

The attachment sites live in ``myoarm_r_chain.xml``. This script rewrites its
generated blocks in ``myoarm_r_tendon.xml`` (straight paths through those sites)
and ``myoarm_r_muscle.xml`` (peak force and length range from the table's
architecture), leaving the rest of both files untouched. The length range is a
rigid tendon of slack length ``(L_MT(q = 0) - L_fo cos(alpha)) / 1.033`` (Barry
et al. 2018, Eq. 1), so the muscle class's normalised fibre-length range spans
the same range of L_fo.

Run: ``uv run python scripts/generate_thenar_muscles.py``.
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from myo_sim.build.compose import RIGHT_ARM_MUSCLES_XML, RIGHT_ARM_TENDONS_XML, build_spec
from myo_sim.build.hand_data import load_thenar_table, write_generated_block

SCRIPT = "generate_thenar_muscles.py"
MUSCLE_CLASS = "myoarm_forearm_muscle"
CLASS_REFERENCE = "OP_r"  # existing muscle of MUSCLE_CLASS: source of the normalised fibre-length range


def _cite(table: dict[str, Any], key: str) -> str:
    citation = table["sources"][key]["citation"]
    return f"{citation.split()[0]} et al. {''.join(c for c in key if c.isdigit())}"


def tendon_block(table: dict[str, Any]) -> str:
    """Generated block of ``myoarm_r_tendon.xml``: one spatial tendon per thenar muscle."""
    lines = []
    for muscle in table["muscles"]:
        lines.append(
            f"        <!-- {muscle['name']} ({muscle['anatomy']}): path from {_cite(table, muscle['source'])} (MIT) -->"
        )
        lines.append(f'        <spatial name="{muscle["name"]}_r_tendon" class="{MUSCLE_CLASS}">')
        for i in range(1, len(muscle["sites"]) + 1):
            lines.append(f'            <site site="{muscle["name"]}-P{i}_r"/>')
        lines.append("        </spatial>")
    return "".join(f"{line}\n" for line in lines)


def architecture_lengthranges(table: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Length range per muscle from its straight site path at q = 0 (sites from ``myoarm_r_chain.xml``)."""
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_kinematics(model, data)
    lo, hi = model.actuator_gainprm[model.actuator(CLASS_REFERENCE).id, :2]
    ranges = {}
    for muscle in table["muscles"]:
        points = [data.site_xpos[model.site(f"{muscle['name']}-P{i}_r").id] for i in range(1, len(muscle["sites"]) + 1)]
        l_mt0 = float(sum(np.linalg.norm(b - a) for a, b in zip(points[:-1], points[1:])))
        l_fo = muscle["optimal_fiber_length_m"]["value"]
        l_ts = (l_mt0 - l_fo * np.cos(np.radians(muscle["pennation_deg"]["value"]))) / 1.033
        ranges[muscle["name"]] = (float(l_ts + lo * l_fo), float(l_ts + hi * l_fo))
    return ranges


def muscle_block(table: dict[str, Any], lengthranges: dict[str, tuple[float, float]]) -> str:
    """Generated block of ``myoarm_r_muscle.xml``: one muscle per table row."""
    lines = []
    for muscle in table["muscles"]:
        arch = _cite(table, muscle["architecture_source"])
        lines.append(
            f"        <!-- {muscle['name']}: force {muscle['force_N']['derivation']}, L_fo"
            f" {muscle['optimal_fiber_length_m']['value']} m, pennation {muscle['pennation_deg']['value']} deg ({arch});"
            " lengthrange from the rigid-tendon slack length of Barry et al. 2018 Eq. 1 -->"
        )
        lo, hi = lengthranges[muscle["name"]]
        lines.append(
            f'        <muscle name="{muscle["name"]}_r" force="{muscle["force_N"]["value"]:g}" class="{MUSCLE_CLASS}"'
            f' tendon="{muscle["name"]}_r_tendon" lengthrange="{lo!r} {hi!r}"/>'
        )
    return "".join(f"{line}\n" for line in lines)


def main() -> None:
    table = load_thenar_table()
    ranges = architecture_lengthranges(table)
    write_generated_block(RIGHT_ARM_TENDONS_XML, SCRIPT, tendon_block(table), "</tendon>")
    write_generated_block(RIGHT_ARM_MUSCLES_XML, SCRIPT, muscle_block(table, ranges), "</actuator>")
    for name, (lo, hi) in ranges.items():
        print(f"{name}: lengthrange {lo!r} {hi!r}")


if __name__ == "__main__":
    main()
