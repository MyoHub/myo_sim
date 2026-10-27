"""Fit tendon-slack (length-range) shifts of the index extrinsic muscles to Knutson et al. (2000).

Binder-Markey & Murray (2017) optimised the tendon slack lengths of FDSI, FDPI,
EDCI and EIP so that their passive force reproduced the extrinsic component of
Knutson's MCP passive moment over wrist and MCP angles. MyoHand muscles have
rigid tendons: the normalised fibre length is ``range[0] + (L - lr[0]) / L0``
with ``L0 = (lr[1] - lr[0]) / (range[1] - range[0])``, so a slack-length change
is a shift of ``actuator_lengthrange`` at constant ``L0``. This script fits one
shift per muscle, starting from each row's ``reference_value_m``, with MuJoCo's own
passive force-length curve, MyoHand's muscle-tendon lengths and moment arms, and
Binder-Markey's penalty on relative change. It writes the absolute slack lengths
(and the reference values they change) to ``build/data/muscle_passive.json`` and
the shifted ``lengthrange`` of each muscle to ``myoarm_r_muscle.xml``, the model's
source of truth (the left arm is mirrored from it).

Run: ``uv run python scripts/fit_muscle_passive.py`` (refits, rewrites the JSON and
the four ``lengthrange`` attributes); ``--xml-only`` only writes the JSON slack
lengths into the XML.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

import myo_sim

DATA_PATH = Path(__file__).parents[1] / "myo_sim" / "build" / "data" / "muscle_passive.json"
MUSCLE_XML = myo_sim.MODELS_DIR / "arm" / "assets" / "myoarm_r_muscle.xml"

KNUTSON_EXTRINSIC = {"A3": 1.2, "k3": 4.2e-2, "B3": -0.8, "phi3": 10.9, "A4": 0.8, "k4": 5.4e-2, "B4": -0.3, "phi4": 40.9}
MUSCLES = ("FDS2", "FDP2", "EDC2", "EIP")
MCP_DEG = np.arange(0.0, 90.1, 5.0)  # MyoHand's MCP range (no hyperextension; Knutson measured from about -45 deg)
WRIST_DEG = np.arange(-60.0, 60.1, 10.0)
DEG = np.pi / 180.0


def knutson_extrinsic(theta: np.ndarray, wrist: np.ndarray, p: dict[str, float] = KNUTSON_EXTRINSIC) -> np.ndarray:
    """Knutson et al. 2000 Eqs. (3)-(4): extrinsic MCP moment (N cm, +flexion), angles in deg (+flexion)."""
    t3 = p["B3"] * wrist + p["phi3"]
    t4 = p["B4"] * wrist + p["phi4"]
    return p["A3"] * (np.exp(-p["k3"] * (theta - t3)) - 1.0) - p["A4"] * (np.exp(p["k4"] * (theta - t4)) - 1.0)


def passive_fl(norm_len: np.ndarray, lmax: float, fpmax: float) -> np.ndarray:
    """MuJoCo's muscle passive force-length curve (``mju_muscleBias``), as a fraction of F0."""
    b = 0.5 * (1.0 + lmax)
    x1 = (norm_len - 1.0) / (b - 1.0)
    x2 = (norm_len - b) / (b - 1.0)
    return np.where(norm_len <= 1.0, 0.0, np.where(norm_len <= b, fpmax * 0.5 * x1 * x1, fpmax * (0.5 + x2)))


def geometry(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """Muscle-tendon lengths and MCP moment arms (dL/dq) on the (wrist, MCP) grid; IP joints at 0."""
    data = mujoco.MjData(model)
    ids = [model.actuator(n).id for n in MUSCLES]
    mcp, wrist = model.joint("mcp2_flexion_r"), model.joint("flexion_r")
    lengths = np.zeros((WRIST_DEG.size, MCP_DEG.size, len(MUSCLES)))
    arms = np.zeros_like(lengths)
    for i, u in enumerate(WRIST_DEG):
        for j, t in enumerate(MCP_DEG):
            out = []
            for dq in (0.0, 1e-5, -1e-5):
                data.qpos[:] = model.qpos0
                data.qpos[wrist.qposadr[0]] = u * DEG
                data.qpos[mcp.qposadr[0]] = t * DEG + dq
                mujoco.mj_kinematics(model, data)
                mujoco.mj_comPos(model, data)
                mujoco.mj_tendon(model, data)
                mujoco.mj_transmission(model, data)
                out.append(data.actuator_length[ids].copy())
            lengths[i, j] = out[0]
            arms[i, j] = (out[1] - out[2]) / 2e-5
    return lengths, arms


def model_torque(model: mujoco.MjModel, lengths: np.ndarray, arms: np.ndarray, shift: np.ndarray) -> np.ndarray:
    """Passive MCP moment (N cm, +flexion) of the four muscles with length ranges shifted by *shift* (m)."""
    total = np.zeros(lengths.shape[:2])
    for k, name in enumerate(MUSCLES):
        a = model.actuator(name).id
        g = model.actuator_gainprm[a]
        lr = model.actuator_lengthrange[a] + shift[k]
        L0 = (lr[1] - lr[0]) / (g[1] - g[0])
        norm = g[0] + (lengths[..., k] - lr[0]) / L0
        force = -g[2] * passive_fl(norm, g[5], g[7])  # MuJoCo muscles pull: negative actuator force
        total += 100.0 * arms[..., k] * force
    return total


def implied_slack(model: mujoco.MjModel, name: str) -> float:
    """Rigid-tendon slack length implied by MuJoCo's length range: ``lr[0] - range[0] * L0``."""
    a = model.actuator(name).id
    g, lr = model.actuator_gainprm[a], model.actuator_lengthrange[a]
    return float(lr[0] - g[0] * (lr[1] - lr[0]) / (g[1] - g[0]))


def load_table() -> dict[str, Any]:
    """The passive-tension table (``build/data/muscle_passive.json``)."""
    return json.loads(DATA_PATH.read_text())


def fitted_lengthranges(model: mujoco.MjModel, table: dict[str, Any]) -> dict[str, list[float]]:
    """``lengthrange`` of each tabled muscle shifted so its implied slack length is the tabled one."""
    ranges = {}
    for row in table["muscles"]:
        name = row["names"]["r"]
        shift = row["tendon_slack_m"]["value"] - implied_slack(model, name)
        lr = model.actuator_lengthrange[model.actuator(name).id]
        ranges[name] = [float(v) for v in lr] if abs(shift) < 1e-12 else [float(v + shift) for v in lr]
    return ranges


def write_lengthranges(ranges: dict[str, list[float]], path: Path = MUSCLE_XML) -> None:
    """Replace the ``lengthrange`` attribute of each named muscle in *path*, leaving the rest of the file as is."""
    text = path.read_text()
    for name, (lo, hi) in ranges.items():
        pattern = re.compile(rf'(<muscle name="{re.escape(name)}"[^>]*?lengthrange=")[^"]*(")')
        text, count = pattern.subn(rf"\g<1>{lo!r} {hi!r}\g<2>", text)
        if count != 1:
            raise ValueError(f"{path.name}: expected one lengthrange for {name}, found {count}")
    path.write_text(text)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--xml-only", action="store_true", help="only write the JSON slack lengths into the XML")
    args = parser.parse_args()
    model = myo_sim.load_spec("myoarm_r").compile()
    table = load_table()
    if args.xml_only:
        write_lengthranges(fitted_lengthranges(model, table))
        return
    from scipy.optimize import minimize

    # Fit from the reference slack lengths, whatever the XML currently holds.
    reference = {r["names"]["r"]: r["tendon_slack_m"]["reference_value_m"] for r in table["muscles"]}
    for name in MUSCLES:
        model.actuator_lengthrange[model.actuator(name).id] += reference[name] - implied_slack(model, name)
    lengths, arms = geometry(model)
    target = knutson_extrinsic(MCP_DEG[None, :], WRIST_DEG[:, None])
    slack = np.array([implied_slack(model, n) for n in MUSCLES])

    def cost(shift: np.ndarray) -> float:
        err = target - model_torque(model, lengths, arms, shift)
        return float(np.sum(err**2) + np.sum(np.abs(shift / slack)) / 100.0)  # Binder-Markey Eqs. (2)-(3), N cm

    best = None
    for start in (np.zeros(4), np.full(4, -0.01), np.full(4, -0.02)):
        res = minimize(cost, start, method="Powell", options={"xtol": 1e-7, "ftol": 1e-10, "maxiter": 20000})
        if best is None or res.fun < best.fun:
            best = res
    shift = best.x
    before = target - model_torque(model, lengths, arms, np.zeros(4))
    after = target - model_torque(model, lengths, arms, shift)
    for k, name in enumerate(MUSCLES):
        row = next(r for r in table["muscles"] if r["names"]["r"] == name)
        row.pop("lengthrange_shift_m", None)
        row["tendon_slack_m"] = {
            "value": round(float(slack[k] + shift[k]), 6),
            "reference_value_m": round(float(slack[k]), 6),
            "change_percent": round(float(100.0 * shift[k] / slack[k]), 3),
            "scales_with": "length",
            "reference_segment": "forearm",
        }
    table["fit"] = {
        "grid": {"mcp_deg": [float(MCP_DEG[0]), float(MCP_DEG[-1]), 5.0], "wrist_deg": [-60.0, 60.0, 10.0]},
        "rmse_Ncm_before": float(np.sqrt(np.mean(before**2))),
        "rmse_Ncm_after": float(np.sqrt(np.mean(after**2))),
        "max_abs_err_Ncm_after": float(np.max(np.abs(after))),
        "target_peak_Ncm": float(np.max(np.abs(target))),
    }
    DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
    ranges = fitted_lengthranges(model, table)
    write_lengthranges(ranges)
    print(json.dumps(table["fit"], indent=1), {n: round(1000 * s, 2) for n, s in zip(MUSCLES, shift)}, "mm")
    for name, (lo, hi) in ranges.items():
        print(f"{name}: lengthrange {lo!r} {hi!r}")


if __name__ == "__main__":
    main()
