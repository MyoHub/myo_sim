"""Fit one specific tension for the forearm and hand muscles and write the peak forces it gives.

``build/data/specific_tension.json`` holds the PCSA of every extrinsic muscle (the intrinsic muscles' PCSA are in
``hand_intrinsic_muscles.json``), the strength targets of the reference subject and the fit. ``--fit`` measures every target at
the tabled grid of specific tensions on the importable model, with each muscle's passive force kept (its ``fpmax`` scaled
against its peak force), and fits one value by weighted least squares, with leave-one-out and held-out checks. Without it the
tabled value is written: the peak forces of the extrinsic muscles in ``myoarm_r_muscle.xml``, of the intrinsic muscles in their
table. Then run ``generate_hand_intrinsic_muscles.py`` and the refit order (``fit_muscle_passive.py``, ``fit_passive_moments.py``,
``generate_extrinsic_muscles.py --fit``, ``fit_passive_moments.py``).

Run: ``uv run python scripts/fit_specific_tension.py [--fit]``.
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import re
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from myo_sim.build.compose import RIGHT_ARM_MUSCLES_XML
from myo_sim.build.hand_data import DATA_DIR, INTRINSIC_DATA_PATH

DATA_PATH = DATA_DIR / "specific_tension.json"
EXTRINSIC_DATA_PATH = DATA_DIR / "extrinsic_muscles.json"


def load_table(path: Path = DATA_PATH) -> dict[str, Any]:
    return json.loads(path.read_text())


def _validation() -> Any:
    name = "validate_hand_function"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(f"{name}.py"))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def pcsa(table: dict[str, Any]) -> dict[str, float]:
    """PCSA (cm2) of every muscle the value applies to."""
    intrinsic = json.loads(INTRINSIC_DATA_PATH.read_text())["muscles"]
    return {k: v["cm2"] for k, v in table["extrinsic_pcsa"].items()} | {r["name"]: r["pcsa_cm2"]["value"] for r in intrinsic}


def forces(table: dict[str, Any], sigma: float) -> dict[str, float]:
    return {name: round(area * sigma, 1) for name, area in pcsa(table).items()}


def hand_at(table: dict[str, Any], sigma: float) -> Any:
    """The validation's hand with every peak force at *sigma* and each muscle's passive force as it is."""
    import myo_sim

    v = _validation()
    target = forces(table, sigma)

    def load() -> mujoco.MjSpec:
        spec = myo_sim.load_spec("myoarm_r")
        for actuator in spec.actuators:
            name = actuator.name.removesuffix("_r")
            if name in target:
                gain, bias = np.array(actuator.gainprm), np.array(actuator.biasprm)
                ratio = gain[2] / target[name] if gain[2] > 0 else 1.0
                gain[2] = bias[2] = target[name]
                gain[7] = bias[7] = bias[7] * ratio
                actuator.gainprm, actuator.biasprm = gain, bias
        return spec

    return dataclasses.replace(v.myohand(), load=load)


def _moment(model: mujoco.MjModel, data: mujoco.MjData, joint: str, sign: float, muscles: tuple[str, ...]) -> float:
    """Moment (N m) about *joint* of those of *muscles* that act in direction *sign*, at full activation."""
    from myo_sim.build.hand_data import fold_node_forces

    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
    dof = model.joint(joint).dofadr[0]
    total = 0.0
    for name in muscles:
        a = model.actuator(f"{name}_r").id
        gain = mujoco.mju_muscleGain(
            data.actuator_length[a], 0.0, model.actuator_lengthrange[a], model.actuator_acc0[a], model.actuator_gainprm[a][:9]
        )
        value = fold_node_forces(model, data, moment[a] * gain)[dof] * sign
        total += max(float(value), 0.0)
    return total


def _at(model: mujoco.MjModel, wrist_deg: float) -> mujoco.MjData:
    v = _validation()
    data = mujoco.MjData(model)
    data.qpos[model.joint("elbow_flexion_r").qposadr[0]] = np.pi / 2
    data.qpos[model.joint("flexion_r").qposadr[0]] = np.radians(wrist_deg)
    v.place_tendon_nodes(model, data)
    mujoco.mj_forward(model, data)
    return data


def measure(table: dict[str, Any], sigma: float, grip: bool = True) -> dict[str, Any]:
    """Every target of the table on the model at *sigma*."""
    v = _validation()
    hand = hand_at(table, sigma)
    vtable = v.load_table()
    model = hand.load().compile()
    rest = _at(model, 0.0)
    out: dict[str, Any] = {
        "index_abduction": _moment(model, rest, "mcp2_abduction_r", 1.0, ("DI1d", "DI1s")),
        "little_abduction": _moment(model, rest, "mcp5_abduction_r", -1.0, ("ADM", "FDM")),
        "index_mcp_flexion": _moment(model, rest, "mcp2_flexion_r", 1.0, ("DI1d", "DI1s", "PI1", "LUM1")),
    }
    wrist = table["wrist_muscles"]
    angles = range(-60, 61, 10)
    out["wrist_flexion_all"] = max(_moment(model, _at(model, a), "flexion_r", 1.0, tuple(wrist["flexion_all"])) for a in angles)
    out["wrist_flexion_prime"] = max(
        _moment(model, _at(model, a), "flexion_r", 1.0, tuple(wrist["flexion_prime"])) for a in angles
    )
    out["wrist_extension_all"] = max(
        _moment(model, _at(model, a), "flexion_r", -1.0, tuple(wrist["extension_all"])) for a in angles
    )
    out["wrist_extension_prime"] = max(
        _moment(model, _at(model, a), "flexion_r", -1.0, tuple(wrist["extension_prime"])) for a in angles
    )
    p = v.pinch(hand, vtable)
    out["key_pinch"] = p["force_N"] if p.get("feasible") else 0.0
    if grip:
        r = v.optimise_grip(v.grip_scene(hand, vtable, vtable["grip"]["wrist_postures_deg"]["self_selected"]), vtable)
        out["grip"] = r["grip"]
        out["grip_settled"] = r["settled"]["grip"]
        out["grip_levels"] = r["levels"]
    return out


def weighted_fit(
    curves: dict[str, list[float]], grid: list[float], targets: dict[str, dict[str, float]], keys: list[str]
) -> float:
    """The value minimising sum ((model(sigma) - mean) / SD)^2 over *keys*; each target linear in sigma through its curve."""
    fine = np.linspace(min(grid), max(grid), 2001)
    cost = sum(((np.interp(fine, grid, curves[k]) - targets[k]["mean"]) / targets[k]["sd"]) ** 2 for k in keys)
    return float(fine[int(np.argmin(cost))])


def fit(table: dict[str, Any]) -> None:
    grid = table["fit"]["grid_N_per_cm2"]
    runs = [measure(table, s) for s in grid]
    curves = {k: [float(r[k]) for r in runs] for k in runs[0] if k != "grip_levels"}
    targets = table["targets"]
    fitted = table["fit"]["fit_targets"]
    value = weighted_fit(curves, grid, targets, fitted)
    needed = {k: float(np.interp(t["mean"], [0.0, *curves[k]], [0.0, *grid])) for k, t in targets.items() if k in curves}
    held_out = {}
    for k in fitted:
        others = [o for o in fitted if o != k]
        s = weighted_fit(curves, grid, targets, others)
        held_out[k] = {
            "value_without": round(s, 1),
            "predicted": round(float(np.interp(s, grid, curves[k])), 2),
            "target": targets[k]["mean"],
            "z": round((float(np.interp(s, grid, curves[k])) - targets[k]["mean"]) / targets[k]["sd"], 2),
        }
    splits = {}
    for name, keys in table["fit"]["splits"].items():
        s = weighted_fit(curves, grid, targets, keys)
        rest = [k for k in targets if k not in keys and k in curves]
        splits[name] = {
            "fit_on": keys,
            "value": round(s, 1),
            "predicted": {k: round(float(np.interp(s, grid, curves[k])), 2) for k in rest},
        }
    table["value_N_per_cm2"] = {
        "value": round(value),
        "status": "fitted",
        "method": table["fit"]["method"],
    }
    table["fit"]["result"] = {
        "curves": {k: [round(x, 3) for x in c] for k, c in curves.items()},
        "needed_per_target_N_per_cm2": {k: round(x, 1) for k, x in needed.items()},
        "at_value": {k: round(float(np.interp(round(value), grid, c)), 2) for k, c in curves.items()},
        "leave_one_out": held_out,
        "splits": splits,
    }


def extrinsic_muscles_xml(text: str, table: dict[str, Any], sigma: float) -> str:
    for name, row in table["extrinsic_pcsa"].items():
        tags = re.findall(rf'<muscle name="{name}_r"[^>]*?/>', text)
        if len(tags) != 1:
            raise ValueError(f"expected one muscle {name}")
        text = text.replace(tags[0], re.sub(r'force\s*=\s*"[^"]*"', f'force="{row["cm2"] * sigma:.1f}"', tags[0]))
    return text


def write(table: dict[str, Any]) -> None:
    sigma = table["value_N_per_cm2"]["value"]
    RIGHT_ARM_MUSCLES_XML.write_text(extrinsic_muscles_xml(RIGHT_ARM_MUSCLES_XML.read_text(), table, sigma))
    intrinsic = json.loads(INTRINSIC_DATA_PATH.read_text())
    intrinsic["specific_tension"] = {
        "value_N_per_cm2": sigma,
        "derivation": "one value for the forearm and hand muscles: build/data/specific_tension.json",
    }
    for row in intrinsic["muscles"]:
        area = row["pcsa_cm2"]["value"]
        row["force_N"]["value"] = round(area * sigma, 1)
        row["force_N"]["derivation"] = f"{area} cm2 x {sigma} N/cm2"
    INTRINSIC_DATA_PATH.write_text(json.dumps(intrinsic, indent=2, ensure_ascii=False) + "\n")
    extrinsic = json.loads(EXTRINSIC_DATA_PATH.read_text())
    extrinsic["fds5"]["force_N"] = {
        "value": round(table["extrinsic_pcsa"]["FDS5"]["cm2"] * sigma, 1),
        "derivation": f"{table['extrinsic_pcsa']['FDS5']['cm2']} cm2 x {sigma} N/cm2",
    }
    EXTRINSIC_DATA_PATH.write_text(json.dumps(extrinsic, indent=2, ensure_ascii=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="measure the targets over the grid and fit the value first")
    args = parser.parse_args()
    table = load_table()
    if args.fit:
        fit(table)
        DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    write(table)
    print(json.dumps(table["value_N_per_cm2"]), json.dumps(table["fit"].get("result", {}).get("needed_per_target_N_per_cm2")))


if __name__ == "__main__":
    main()
