"""Write the deep hip muscles (pectineus, quadratus femoris, gemelli) into the leg XML from their table.

The table is ``build/data/deep_hip_muscles.json`` (points, forces and lengths from OpenSim's Gait2392, with provenance
and the comparison against it). The generated blocks hold both sides, as the leg files do; the right side is the source
and the left mirrors z::

    python scripts/generate_deep_hip_muscles.py        # write the XML from the table
    python scripts/generate_deep_hip_muscles.py --fit  # refit the length maps over the full hip range first

**Fit.** MuJoCo's linear length map is fitted to the rigid-tendon fibre length of Gait2392's L_fo and L_ts over the full
hip range (``myo_sim.build.rigid_conversion``, the same map and lengthrange rule as the other leg muscles).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco

from myo_sim.build.hand_data import write_generated_block
from myo_sim.build.rigid_conversion import (
    actuator_lengths,
    fit_rigid_muscle_map,
    joint_ranges,
    joint_sweep_postures,
    sample_postures,
)

ROOT = Path(__file__).resolve().parents[1]
DATA_PATH = ROOT / "myo_sim" / "build" / "data" / "deep_hip_muscles.json"
ASSETS = ROOT / "myo_sim" / "models" / "leg" / "assets"
CHAIN_XML, TENDON_XML, MUSCLE_XML = (ASSETS / f"myolegs_{role}.xml" for role in ("chain", "tendon", "muscle"))
SCRIPT = Path(__file__).name
SIDES = (("r", 1.0), ("l", -1.0))
HIP_JOINTS = ("hip_flexion_r", "hip_adduction_r", "hip_rotation_r")


def load_table(path: Path = DATA_PATH) -> dict[str, Any]:
    """Read the deep hip muscle table."""
    return json.loads(path.read_text())


def _pos(pos: list[float], sign: float) -> str:
    return f"{pos[0]:.6g} {pos[1]:.6g} {sign * pos[2]:.6g}"


def chain_block(table: dict[str, Any], body: str, side: str | None = None) -> str:
    """Sites of the block in body ``pelvis`` (both sides) or ``femur`` (one side)."""
    indent, lines = " " * (8 if body == "pelvis" else 12), []
    point = 1 if body == "pelvis" else 2
    for s, sign in SIDES:
        if side is None or s == side:
            lines += [
                f'{indent}<site name="{m["name"]}-P{point}_{s}" pos="{_pos(m[f"{body}_pos"], sign)}"/>\n'
                for m in table["muscles"]
            ]
    return "".join(lines)


def tendon_block(table: dict[str, Any]) -> str:
    """Spatial tendons, right then left."""
    lines = []
    for s, _ in SIDES:
        for m in table["muscles"]:
            lines += [
                f'        <spatial name="{m["name"]}_{s}_tendon" springlength="{m["tendon_slack_m"]:.6g}" class="myolegs_muscle">\n',
                f'            <site site="{m["name"]}-P1_{s}"/>\n',
                f'            <site site="{m["name"]}-P2_{s}"/>\n',
                "        </spatial>\n",
            ]
    return "".join(lines)


def muscle_block(table: dict[str, Any]) -> str:
    """Actuators, right then left; the curve parameters are the same as the leg's piriformis."""
    curve, lines = table["curve"], []
    for s, _ in SIDES:
        for m in table["muscles"]:
            r0, r1 = m["range"]
            prm = (
                f"{r0:.6f} {r1:.6f} {m['force_N']:g} 1 {curve['lmin']:g} {curve['lmax']:g} "
                f"{curve['vmax']:g} {curve['fpmax']:g} {curve['fvmax']:g} 0"
            )
            lr = m["lengthrange_m"]
            lines.append(
                f'        <general biasprm="{prm}" class="myolegs_muscle" gainprm="{prm}" '
                f'lengthrange="{lr[0]:.6f} {lr[1]:.6f}" name="{m["name"]}_{s}" tendon="{m["name"]}_{s}_tendon"/>\n'
            )
    return "".join(lines)


def write_xml(table: dict[str, Any]) -> None:
    """Write the generated blocks."""
    write_generated_block(CHAIN_XML, SCRIPT, chain_block(table, "pelvis"), "", "pelvis")
    for s, _ in SIDES:
        write_generated_block(CHAIN_XML, SCRIPT, chain_block(table, "femur", s), "", f"femur_{s}")
    write_generated_block(TENDON_XML, SCRIPT, tendon_block(table), "</tendon>")
    write_generated_block(MUSCLE_XML, SCRIPT, muscle_block(table), "</actuator>")


def fit_maps(model: mujoco.MjModel, table: dict[str, Any]) -> None:
    """Fit range and lengthrange per muscle over the full hip range; *model* holds the sites and tendons."""
    ranges = joint_ranges(model, HIP_JOINTS)
    postures = sample_postures(model, ranges, table["fit"]["postures"], table["fit"]["seed"])
    sweeps = joint_sweep_postures(model, list(HIP_JOINTS))
    curve = table["curve"]
    for m in table["muscles"]:
        act = [model.actuator(f"{m['name']}_r").id]
        lengths = actuator_lengths(model, postures, act)[:, 0]
        span = actuator_lengths(model, sweeps, act)[:, 0]
        r0, r1, l0, l1 = fit_rigid_muscle_map(
            lengths, m["optimal_fibre_m"], m["tendon_slack_m"], 0.0, curve["lmin"], curve["lmax"], span_length=span
        )
        m["range"] = [round(r0, 6), round(r1, 6)]
        m["lengthrange_m"] = [round(l0, 6), round(l1, 6)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="refit the length maps over the full hip range first")
    args = parser.parse_args()
    table = load_table()
    write_xml(table)
    if args.fit:
        import myo_sim

        fit_maps(myo_sim.load_spec("myolegs").compile(), table)
        DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
        write_xml(table)


if __name__ == "__main__":
    main()
