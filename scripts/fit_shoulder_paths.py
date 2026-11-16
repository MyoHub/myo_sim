#!/usr/bin/env python3
"""Shoulder muscle path sites fitted to measured glenohumeral moment arms (``build/data/shoulder_paths.json``).

Paths changed after the independent mechanics check (``validate_shoulder_mechanics.py``), each with one free site per
line and the path's other elements fixed:

- **DELT2:** Seth's acromial via point (``DELT2_DELT2-P3_r``) moves within 8 mm of where it was; the scaption peak is
  held at or above 29.5 mm (Hik and Ackland 2019's pooled 33.4 mm less 4 mm).
- **DELT3:** the scapular via ``DELT3_DELT3-P2_r`` replaces MoBL-ARMS's two wraps (``Delt3_torus_r``,
  ``delt3hum_ellipsoid_DELT3_r``); it stays 28-45 mm from the glenohumeral centre, outside the humeral head and cuff,
  behind the joint. The rotation curve of Kuechle 2000 is left out of this fit (its sign is in question, see the wiki).
- **INFSP1-2, SUBSC1-3:** the humeral-head sphere ``SUPSP_hhead_r`` (no side site) and a humeral via ``<name>-V1_r``
  25-40 mm from its centre replace Seth's scapular cylinders, so the lines wrap the head about the rotation axis.

The fit uses the cadaver convention and the literature curves of ``validate_shoulder_mechanics.py`` (scapula fixed at
rest, elbow 90 deg): the excess outside the studies' band, a pull to the band's middle (0.2) and a smoothness term on the
curve (0.3), least squares from the stored start and from four seeded random starts. Those curves are then no longer
held out for these muscles; the validation marks them, and Gatti et al. 2007's ranges, the pooled peaks of the other
muscles and the strength rows stay independent.

    uv run python scripts/fit_shoulder_paths.py            # print the current fit
    uv run python scripts/fit_shoulder_paths.py --fit      # refit, write the table and the XML
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).parent))
import generate_scapular_muscles as gsm  # noqa: E402
import validate_shoulder_mechanics as vsm  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
TABLE = ROOT / "myo_sim" / "build" / "data" / "shoulder_paths.json"
CHAIN = ROOT / "myo_sim" / "models" / "arm" / "assets" / "myoarm_r_chain.xml"
H = 1e-4  # rad, central difference of the path length
W_MID, W_SMOOTH, W_PEAK = 0.2, 0.3, 30.0
PI = np.pi
HHEAD = "SUPSP_hhead_r"

# Free sites: "box" = body-frame position within half_mm of the start; "polar" = (rho, polar angle from the body's +y,
# azimuth from +x towards +z) about a centre. Cuff vias are polar about the humeral-head sphere's centre.
DESIGNS: list[dict[str, Any]] = [
    {
        "muscle": "middle deltoid",
        "actuators": ["DELT2"],
        "sites": {"DELT2_DELT2-P3_r": {"kind": "box", "half_mm": 8.0, "start": [-0.005138, -0.006741, 0.031029]}},  # Seth's point
        "peak_min_mm": {"scaption": 29.5},
    },
    {
        "muscle": "posterior deltoid",
        "actuators": ["DELT3"],
        "sites": {"DELT3_DELT3-P2_r": {"kind": "polar", "centre": "glenohumeral", "rho_mm": [28.0, 45.0], "theta": [0.3, 2.8], "phi": [PI / 2, 1.5 * PI]}},
        "planes": ["coronal", "scaption", "flexion"],
    },
    {
        "muscle": "infraspinatus",
        "actuators": ["INFSP1", "INFSP2"],
        "sites": {f"INFSP{k}-V1_r": {"kind": "polar", "centre": "humeral_head", "rho_mm": [25.0, 40.0], "theta": [0.0, 3.0], "phi": [0.0, 2 * PI]} for k in (1, 2)},
    },
    {
        "muscle": "subscapularis",
        "actuators": ["SUBSC1", "SUBSC2", "SUBSC3"],
        "sites": {f"SUBSC{k}-V1_r": {"kind": "polar", "centre": "humeral_head", "rho_mm": [25.0, 40.0], "theta": [0.0, 3.0], "phi": [-PI, PI]} for k in (1, 2, 3)},
    },
]  # fmt: skip


class Grid:
    """The validation's postures (scapula at rest, elbow 90 deg) and the humerus rotations about thorax axes."""

    def __init__(self, arm: vsm.Arm) -> None:
        self.keys: list[tuple[str, float]] = []
        plus, minus = [], []
        for plane, p in vsm.PLANES.items():
            for e in vsm.ELEV:
                q = arm.qpos(p, e, 0.0, 90.0, {})
                u = arm.axis(lambda x, p=p, e=e: arm.humerothoracic(p, e + np.degrees(x), 0.0))
                self._add(arm, (plane, e), q, u, plus, minus)
        for r in vsm.ROT:
            q = arm.qpos(0.0, 0.0, -r, 90.0, {})
            u = arm.axis(lambda x, r=r: arm.humerothoracic(0.0, 0.0, -r - np.degrees(x)))
            self._add(arm, ("rotation", r), q, u, plus, minus)
        self.plus, self.minus = np.array(plus), np.array(minus)

    def _add(self, arm: vsm.Arm, key: tuple[str, float], q: np.ndarray, u: np.ndarray, plus: list, minus: list) -> None:
        m, d = arm.model, arm.data
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, jp, jr, arm.humerus)
        dq = np.linalg.lstsq(jr[:, arm.gh_dof], d.xmat[arm.chest].reshape(3, 3) @ u, rcond=None)[0]
        for s, out in ((1.0, plus), (-1.0, minus)):
            qq = q.copy()
            for dof, v in zip(arm.gh_dof, dq):
                qq[m.jnt_qposadr[m.dof_jntid[dof]]] += s * H * v
            out.append(qq)
        self.keys.append(key)

    def arms_mm(self, m: mujoco.MjModel, d: mujoco.MjData, tendons: list[int]) -> np.ndarray:
        out = np.empty((len(self.keys), len(tendons)))
        for i in range(len(self.keys)):
            lengths = []
            for q in (self.plus[i], self.minus[i]):
                d.qpos[:] = q
                mujoco.mj_kinematics(m, d)
                mujoco.mj_tendon(m, d)
                lengths.append(d.ten_length[tendons].copy())
            out[i] = -(lengths[0] - lengths[1]) / (2 * H) * 1e3
        return out


def bands(muscle: str) -> dict[tuple[str, float], tuple[float, float]]:
    """(plane, angle) -> (low, high) over the studies of ``validate_shoulder_mechanics.LIT_CURVES``."""
    out = {}
    for plane, studies in vsm.LIT_CURVES.items():
        grid = vsm.ROT if plane == "rotation" else vsm.ELEV
        for i, a in enumerate(grid):
            vals = [v[muscle][i] for v in studies.values() if muscle in v and i < len(v[muscle])]
            if vals:
                out[(plane, a)] = (min(vals), max(vals))
    return out


def centres(m: mujoco.MjModel) -> dict[str, np.ndarray]:
    return {"glenohumeral": m.body_pos[m.body("humphant_r").id].copy(), "humeral_head": m.geom_pos[m.geom(HHEAD).id].copy()}


def polar_to_pos(x: np.ndarray, centre: np.ndarray) -> np.ndarray:
    rho, th, ph = x
    return centre + rho * np.array([np.sin(th) * np.cos(ph), np.cos(th), np.sin(th) * np.sin(ph)])


def pos_to_polar(p: np.ndarray, centre: np.ndarray) -> np.ndarray:
    v = np.asarray(p) - centre
    rho = np.linalg.norm(v)
    return np.array([rho, np.arccos(v[1] / rho), np.arctan2(v[2], v[0])])


class Fit:
    """One design: free sites of a muscle group against its literature bands."""

    def __init__(self, design: dict[str, Any], m: mujoco.MjModel, grid: Grid) -> None:
        self.design, self.m, self.d, self.grid = design, m, mujoco.MjData(m), grid
        self.tendons = [m.actuator_trnid[m.actuator(f"{a}_r").id, 0] for a in design["actuators"]]
        self.weights = np.array([m.actuator(f"{a}_r").gainprm[2] for a in design["actuators"]])
        self.band = bands(design["muscle"])
        self.centre = centres(m)
        self.sites = list(design["sites"])
        self.lb, self.ub, self.x0 = [], [], []
        for name, spec in design["sites"].items():
            pos = m.site_pos[m.site(name).id]
            if spec["kind"] == "box":
                h = spec["half_mm"] / 1e3
                start = np.array(spec.get("start", pos))
                self.lb += list(start - h)
                self.ub += list(start + h)
                self.x0 += list(pos)
            else:
                c = self.centre[spec["centre"]]
                x = pos_to_polar(pos, c)
                if spec["phi"][0] > x[2]:
                    x[2] += 2 * PI
                elif spec["phi"][1] < x[2]:
                    x[2] -= 2 * PI
                self.lb += [spec["rho_mm"][0] / 1e3, spec["theta"][0], spec["phi"][0]]
                self.ub += [spec["rho_mm"][1] / 1e3, spec["theta"][1], spec["phi"][1]]
                self.x0 += list(x)
        self.lb, self.ub = np.array(self.lb), np.array(self.ub)
        self.x0 = np.clip(np.array(self.x0), self.lb + 1e-9, self.ub - 1e-9)

    def positions(self, x: np.ndarray) -> dict[str, np.ndarray]:
        out = {}
        for k, (name, spec) in enumerate(self.design["sites"].items()):
            xi = x[3 * k : 3 * k + 3]
            out[name] = xi.copy() if spec["kind"] == "box" else polar_to_pos(xi, self.centre[spec["centre"]])
        return out

    def curve(self, x: np.ndarray) -> np.ndarray:
        """Force-weighted moment arm (mm) of the group per grid key."""
        for name, p in self.positions(x).items():
            self.m.site_pos[self.m.site(name).id] = p
        arms = self.grid.arms_mm(self.m, self.d, self.tendons)
        return arms @ self.weights / self.weights.sum()

    def residual(self, x: np.ndarray) -> np.ndarray:
        arm = self.curve(x)
        planes = self.design.get("planes")
        r = []
        for i, key in enumerate(self.grid.keys):
            if key in self.band and (planes is None or key[0] in planes):
                lo, hi = self.band[key]
                r += [max(lo - arm[i], arm[i] - hi, 0.0), W_MID * (arm[i] - 0.5 * (lo + hi))]
        for plane in ("coronal", "scaption", "flexion", "rotation"):
            r += list(W_SMOOTH * np.diff(arm[[i for i, k in enumerate(self.grid.keys) if k[0] == plane]], 2))
        for plane, low in self.design.get("peak_min_mm", {}).items():
            peak = arm[[i for i, k in enumerate(self.grid.keys) if k[0] == plane]].max()
            r.append(W_PEAK * max(0.0, low - peak))
        return np.array(r)

    def run(self, starts: int = 4, seed: int = 0) -> np.ndarray:
        rng = np.random.default_rng(seed)
        best = None
        for k in range(starts + 1):
            x = self.x0 if k == 0 else rng.uniform(self.lb, self.ub)
            res = least_squares(self.residual, x, bounds=(self.lb, self.ub), diff_step=1e-3, max_nfev=400)
            if best is None or res.cost < best.cost - 1e-9:
                best = res
        return best.x

    def summary(self, x: np.ndarray) -> dict[str, Any]:
        arm = self.curve(x)
        rows = {}
        for plane in ("coronal", "scaption", "flexion", "rotation"):
            idx = [i for i, k in enumerate(self.grid.keys) if k[0] == plane]
            exc = [
                max(self.band[self.grid.keys[i]][0] - arm[i], arm[i] - self.band[self.grid.keys[i]][1], 0.0)
                for i in idx
                if self.grid.keys[i] in self.band
            ]
            rows[plane] = {
                "model_mm": [round(float(arm[i]), 2) for i in idx],
                "in_band": int(sum(e == 0 for e in exc)),
                "within_5mm": int(sum(e <= 5 for e in exc)),
                "n": len(exc),
            }
        return {"cost": round(float(0.5 * np.sum(self.residual(x) ** 2)), 3), "planes": rows}  # fmt: skip


def write_chain_sites(sites: dict[str, list[float]]) -> None:
    """Set the pos of hand-written arm sites (the kept MoBL-ARMS / Seth paths) in the chain file."""
    text = CHAIN.read_text()
    for name, pos in sites.items():
        pattern = re.compile(rf'(<site name="{re.escape(name)}" pos=")[^"]*(")')
        text, n = pattern.subn(lambda mm: mm.group(1) + " ".join(f"{v:.6g}" for v in pos) + mm.group(2), text)
        if n != 1:
            raise SystemExit(f"{name}: {n} sites in {CHAIN.name}")
    CHAIN.write_text(text)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="refit and write the table and the XML")
    args = parser.parse_args()
    arm = vsm.Arm()
    grid = Grid(arm)
    table = json.loads(TABLE.read_text())
    out = {"format": "myo_sim.shoulder_paths/1", "description": __doc__.split("\n\n")[0], "mujoco": mujoco.__version__}
    out["provenance"] = table.get("provenance", {})
    out["designs"], out["cuff_vias"], out["kept_sites"] = [], {}, {}
    for design in DESIGNS:
        f = Fit(design, arm.model, grid)
        x = f.run() if args.fit else f.x0
        pos = f.positions(x)
        s = f.summary(x)
        print(design["muscle"], s["cost"], {p: f"{v['within_5mm']}/{v['n']}" for p, v in s["planes"].items()})
        for name, p in pos.items():
            body = arm.model.body(arm.model.site_bodyid[arm.model.site(name).id]).name
            entry = {"body": body, "pos": [round(float(v), 6) for v in p]}
            (out["cuff_vias"] if name.endswith("-V1_r") else out["kept_sites"])[
                name.split("-V1_r")[0] if name.endswith("-V1_r") else name
            ] = entry
        entry = {"muscle": design["muscle"], "actuators": design["actuators"], "free_sites": list(pos)}
        entry["planes"] = design.get("planes", ["coronal", "scaption", "flexion", "rotation"])
        out["designs"].append({**entry, "fit": s})
    if args.fit:
        TABLE.write_text(json.dumps(out, indent=1) + "\n")
        write_chain_sites({k: v["pos"] for k, v in out["kept_sites"].items()})
        muscles = json.loads(gsm.TABLE.read_text())
        for row in muscles["muscles"]:
            if row["name"] in out["cuff_vias"]:
                row["via"] = out["cuff_vias"][row["name"]]
        gsm.TABLE.write_text(json.dumps(muscles, indent=2) + "\n")
        gsm.write(muscles)
        print("wrote", TABLE.relative_to(ROOT))


if __name__ == "__main__":
    main()
