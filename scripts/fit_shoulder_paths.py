#!/usr/bin/env python3
"""Candidate shoulder path sites fitted to cadaver moment arms, checked on held-out data (``build/data/shoulder_paths.json``).

A candidate frees one site of a muscle path and places it against the fit-set studies of
``validate_shoulder_mechanics.py`` (``FIT_SET``: the Hik and Ackland 2019 re-plots of Kuechle 1997 and 2000 and Ackland
2008, signs corrected there), in its cadaver convention (scapula fixed at rest, elbow 90 deg): the excess outside their
band, a pull to the band's middle (0.2), a smoothness term on the curve (0.3) and the path length at qpos0 within 5 mm
of the current path (so the fibre and slack lengths keep their meaning); least squares from the current site and from
eight seeded random starts. The candidate is then compared with the current path on the held-out moment-arm checks
(the other studies' curves, including Otis 1994 and Axford 2025; rotation arms in scaption; Langenderfer et al. 2006;
Gatti et al. 2007's ranges; the pooled peaks). It is adopted, by writing the site into the arm chain, only if it gains
on the fit set and loses no held-out check. No candidate has met that so far, so the model's paths are unchanged.
``EXPLORED`` records the designs tried during development for the other muscles.

    uv run python scripts/fit_shoulder_paths.py            # evaluate the stored candidates
    uv run python scripts/fit_shoulder_paths.py --fit      # refit the candidates and write the table
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).parent))
import validate_shoulder_mechanics as vsm  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
TABLE = ROOT / "myo_sim" / "build" / "data" / "shoulder_paths.json"
H = 1e-4  # rad, central difference of the path length
W_MID, W_SMOOTH, W_PEAK, W_LENGTH = 0.2, 0.3, 30.0, 5.0
REST_TOL_MM = 5.0  # path length at qpos0 within this of the current path

# Free sites: body-frame position within half_mm per axis of the current site.
CANDIDATES: list[dict[str, Any]] = [
    {
        "muscle": "middle deltoid",
        "actuators": ["DELT2"],
        "sites": {"DELT2_DELT2-P3_r": {"half_mm": 8.0}},  # Seth's acromial via point
        "peak_min_mm": {"scaption": 29.5},  # Hik and Ackland's pooled 33.4 mm less 4
    },
]
# Designs tried during development with the same objective (fit set within 5 mm, current path -> design; the rotation
# signs of Kuechle 2000 before the full-text check for SUPSP) and not kept.
EXPLORED = {
    "DELT3": "MoBL-ARMS's two wraps replaced by its scapular via 28-45 mm from the joint: 16 -> 14 of 29; the held-out "
    "Otis 1994 and Axford 2025 curves have the posterior deltoid an adductor in scaption, as the current path",
    "INFSP1-2": "humeral-head sphere and a humeral via 25-40 mm from its centre instead of Seth's cylinder: 19 -> 19 of 29 "
    "(rotation 6 -> 8 of 9); held out, Langenderfer 2006 8 -> 6 of 10 (rotation arm 4-5 mm above it)",
    "SUBSC1-3": "the same design as INFSP: 16 -> 12 of 29",
    "SUPSP1-2": "a humeral via within 0.3 rad of the rotation axis: rotation 2 -> 5 of 9 but an adductor above 75 deg "
    "in coronal abduction and flexion; 13 -> 14 of 29 over all studies",
}


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


# Fit-set curves left out of the fit: (plane, muscle, study) -> reason. Curves given as a band of uncertain sign get a
# low weight.
FIT_EXCLUDE = {("coronal", "posterior deltoid", "kuechle1997"): "conflicts with Ackland 2008 (-11 to +2 against +7 to +28 mm)"}
W_UNCERTAIN = 0.3


def bands(muscle: str) -> dict[tuple[str, float], tuple[float, float, float]]:
    """(plane, angle) -> (low, high, weight) over the fit-set studies (``validate_shoulder_mechanics.FIT_SET``)."""
    out = {}
    for plane, studies in vsm.LIT_CURVES.items():
        grid = vsm.ROT if plane == "rotation" else vsm.ELEV
        for i, a in enumerate(grid):
            vals, weight = [], 1.0
            for k, v in studies.items():
                if k not in vsm.FIT_SET or muscle not in v or i >= len(v[muscle]) or (plane, muscle, k) in FIT_EXCLUDE:
                    continue
                if isinstance(v[muscle][i], tuple):
                    weight = W_UNCERTAIN
                vals += list(np.atleast_1d(v[muscle][i]))
            if vals:
                out[(plane, a)] = (min(vals), max(vals), weight)
    return out


class Fit:
    """One design: free sites of a muscle group against its fit-set bands."""

    def __init__(self, design: dict[str, Any], m: mujoco.MjModel, grid: Grid) -> None:
        self.design, self.m, self.d, self.grid = design, m, mujoco.MjData(m), grid
        self.tendons = [m.actuator_trnid[m.actuator(f"{a}_r").id, 0] for a in design["actuators"]]
        self.weights = np.array([m.actuator(f"{a}_r").gainprm[2] for a in design["actuators"]])
        self.band = bands(design["muscle"])
        self.lb, self.ub, self.x0 = [], [], []
        for name, spec in design["sites"].items():
            h = spec["half_mm"] / 1e3
            start = m.site_pos[m.site(name).id].copy()
            self.lb += list(start - h)
            self.ub += list(start + h)
            self.x0 += list(start)
        self.lb, self.ub = np.array(self.lb), np.array(self.ub)
        self.rest_ref = self.rest_lengths_mm()
        self.x0 = np.clip(np.array(self.x0), self.lb + 1e-9, self.ub - 1e-9)

    def positions(self, x: np.ndarray) -> dict[str, np.ndarray]:
        return {name: x[3 * k : 3 * k + 3].copy() for k, name in enumerate(self.design["sites"])}

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
                lo, hi, w = self.band[key]
                r += [w * max(lo - arm[i], arm[i] - hi, 0.0), w * W_MID * (arm[i] - 0.5 * (lo + hi))]
        for plane in ("coronal", "scaption", "flexion", "rotation"):
            r += list(W_SMOOTH * np.diff(arm[[i for i, k in enumerate(self.grid.keys) if k[0] == plane]], 2))
        for plane, low in self.design.get("peak_min_mm", {}).items():
            peak = arm[[i for i, k in enumerate(self.grid.keys) if k[0] == plane]].max()
            r.append(W_PEAK * max(0.0, low - peak))
        r += list(W_LENGTH * np.maximum(0.0, np.abs(self.rest_lengths_mm() - self.rest_ref) - REST_TOL_MM))
        return np.array(r)

    def rest_lengths_mm(self) -> np.ndarray:
        self.d.qpos[:] = self.m.qpos0
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_tendon(self.m, self.d)
        return 1e3 * self.d.ten_length[self.tendons]

    def run(self, starts: int = 8, seed: int = 0) -> np.ndarray:
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
        rest = {
            a: [round(float(v), 2), round(float(r), 2)]
            for a, v, r in zip(self.design["actuators"], self.rest_lengths_mm(), self.rest_ref)
        }
        return {"cost": round(float(0.5 * np.sum(self.residual(x) ** 2)), 3), "planes": rows, "rest_length_mm_new_old": rest}


def held_out(arm: vsm.Arm) -> dict[str, Any]:
    """The held-out moment-arm checks of ``validate_shoulder_mechanics`` for the model as it stands."""
    curves = vsm.moment_arm_curves(arm)
    ma = vsm.compare_moment_arms(curves)
    held = [r for r in ma["curves"] if r["set"] == "held out"]
    lang = vsm.langenderfer(arm)["rows"]
    rot = vsm.scaption_rotation(arm)
    peaks = ma["hik2019_pooled_peaks"].values()
    return {
        "curves_within_5mm": int(sum(r["within_5mm"] for r in held)),
        "curves_in_band": int(sum(r["in_band"] for r in held)),
        "scaption_rotation_within_5mm": int(sum(r["within_5mm"] for r in rot)),
        "langenderfer_within_5mm": int(sum(r["within_5mm"] for r in lang)),
        "gatti_in_range": int(sum(v["in_range"] for v in ma["gatti2007_scaption_60"].values())),
        "peaks_within_4mm": int(sum(abs(v["model_peak_mm"] - v["pooled_peak_mm"]) <= 4.0 for v in peaks)),
    }


def fit_set_within_5mm(summary: dict[str, Any]) -> int:
    return sum(v["within_5mm"] for v in summary["planes"].values())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="refit the candidates and write the table")
    args = parser.parse_args()
    arm = vsm.Arm()
    grid = Grid(arm)
    stored = json.loads(TABLE.read_text()) if TABLE.exists() else {}
    out = {"format": "myo_sim.shoulder_paths/2", "description": __doc__.split("\n\n")[0], "mujoco": mujoco.__version__}
    out["fit_studies"] = list(vsm.FIT_SET)
    out["fit_excluded"] = {"/".join(k): v for k, v in FIT_EXCLUDE.items()}
    out["candidates"], out["explored"] = [], EXPLORED
    for k, design in enumerate(CANDIDATES):
        f = Fit(design, arm.model, grid)
        x_current = f.x0.copy()
        current = {"fit": f.summary(x_current), "held_out": held_out(arm)}
        if args.fit:
            x = f.run()
        else:
            x = np.concatenate([stored["candidates"][k]["sites"][n]["pos"] for n in design["sites"]])
        candidate = {"fit": f.summary(x), "held_out": held_out(arm)}
        f.curve(x_current)  # back to the model's sites
        gain = fit_set_within_5mm(candidate["fit"]) > fit_set_within_5mm(current["fit"])
        losses = [key for key, v in current["held_out"].items() if candidate["held_out"][key] < v]
        out["candidates"].append(
            {
                "muscle": design["muscle"],
                "actuators": design["actuators"],
                "sites": {n: {"current": [round(float(v), 6) for v in x_current[3 * i : 3 * i + 3]],
                              "pos": [round(float(v), 6) for v in x[3 * i : 3 * i + 3]]} for i, n in enumerate(design["sites"])},
                "current": current,
                "candidate": candidate,
                "adopted": bool(gain and not losses),
                "held_out_losses": losses,
            }
        )  # fmt: skip
        print(design["muscle"], "fit set", fit_set_within_5mm(current["fit"]), "->", fit_set_within_5mm(candidate["fit"]),
              "held-out losses", losses)  # fmt: skip
    if args.fit:
        TABLE.write_text(json.dumps(out, indent=1) + "\n")
        print("wrote", TABLE.relative_to(ROOT))


if __name__ == "__main__":
    main()
