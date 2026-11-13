#!/usr/bin/env python3
"""Shoulder-girdle ligaments as passive spatial tendons (``build/data/ligaments.json``).

    uv run python scripts/fit_ligaments.py --zip ThoracoscapularShoulderPaperMaterials.zip --fit
    uv run python scripts/fit_ligaments.py --write

Follows the explicit-ligament proposal (MyoHub/myo_sim#142): one straight strand per ligament between bone sites, a
tension-only spring (``springlength="0 L0"``) with the quadratic toe of the Blankevoort law,
F = k e^2 / (2 e_t) with e = (L - L0) / L0, i.e. ``stiffness="0 b 0"`` with b = k / (2 e_t L0^2); no damping. k (N per
unit strain) and e_t come from the table's cited rows. The toe is exact up to e_t and stiffer beyond it, so the slack
length keeps every strand at or below e_t along Seth et al.'s measured motions (the release's IK frames, assembled):
L0 = max(L_rest / (1 + e_rest), L_p99 / (1 + e_t)), the rest strain e_rest being the cited one where it exists.

**--fit** landmarks the sites on this package's meshes at qpos0 (rest posture) from the published attachment
descriptions in the table, computes the lengths and writes table and XML. **--write** renders the XML from the table.
"""

from __future__ import annotations

import argparse
import json
import sys
import zipfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from generate_scapular_muscles import seth_frames  # noqa: E402

import myo_sim  # noqa: E402
from myo_sim.build.hand_data import write_generated_block  # noqa: E402

DATA = ROOT / "myo_sim" / "build" / "data"
TABLE = DATA / "ligaments.json"
MODELS = ROOT / "myo_sim" / "models"
CHAIN = MODELS / "arm" / "assets" / "myoarm_r_chain.xml"
TENDON = MODELS / "arm" / "assets" / "myoarm_r_tendon.xml"
TORSO = MODELS / "torso" / "assets" / "myotorso_chain.xml"
SCRIPT = "fit_ligaments.py"
PAD = {"arm": " " * 16, "torso": " " * 22}
UP, ANT = np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0])


class Bones:
    """Clavicle, scapula and thorax vertices in the chest_r frame at qpos0, with the joint centres."""

    def __init__(self, model: mujoco.MjModel) -> None:
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        chest = model.body("chest_r").id
        self.rc, self.pc = data.xmat[chest].reshape(3, 3), data.xpos[chest].copy()
        self.v: dict[str, np.ndarray] = {}
        for g in range(model.ngeom):
            body = model.body(model.geom_bodyid[g]).name
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and body in ("clavicle_r", "scapula_r"):
                mesh = model.geom_dataid[g]
                a, n = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                w = model.mesh_vert[a : a + n] @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
                self.v[body] = self.local(w)
        self.frame = {
            b: (self.local(data.xpos[model.body(b).id]), self.rc.T @ data.xmat[model.body(b).id].reshape(3, 3))
            for b in ("clavicle_r", "scapula_r")
        }
        self.sc = self.frame["clavicle_r"][0]
        self.ac = self.local(data.xanchor[model.joint("acromioclavicular_r").id])
        self.lat = (self.ac - self.sc) / np.linalg.norm(self.ac - self.sc)

    def local(self, w: np.ndarray) -> np.ndarray:
        return (w - self.pc) @ self.rc

    def to_body(self, body: str, p: np.ndarray) -> np.ndarray:
        if body == "chest_r":
            return p
        pos, rot = self.frame[body]
        return rot.T @ (p - pos)

    def extreme(self, body: str, near: np.ndarray, direction: np.ndarray, radius: float = 0.005) -> np.ndarray:
        """The vertex furthest along *direction* within *radius* of *near* (the nearest vertex if none is that close)."""
        v = self.v[body]
        d = np.linalg.norm(v - near, axis=1)
        if not (d < radius).any():
            return v[np.argmin(d)]
        c = v[d < radius]
        return c[np.argmax(c @ direction)]

    def clavicle_slice(self, from_lateral: float | None, from_medial: float | None, direction: np.ndarray) -> np.ndarray:
        """The clavicle vertex furthest along *direction* in a 8 mm slice at a distance from either end (SC-AC line)."""
        v = self.v["clavicle_r"]
        s = (v - self.sc) @ self.lat
        centre = s.max() - from_lateral if from_lateral is not None else s.min() + from_medial
        c = v[np.abs(s - centre) < 0.004]
        return c[np.argmax(c @ direction)]


def landmark(bones: Bones, rule: dict[str, Any]) -> np.ndarray:
    """A site in the chest_r frame from a row's landmark rule."""
    kind = rule["rule"]
    if kind == "clavicle_slice":
        return bones.clavicle_slice(rule.get("from_lateral_m"), rule.get("from_medial_m"), np.array(rule["direction"]))
    if kind == "surface":
        near = np.array(rule["near"], float)
        return bones.extreme(rule["bone"], near, np.array(rule["direction"]))
    if kind == "coracoid":
        base, tip = np.array(rule["base"]), np.array(rule["tip"])
        near = base + rule["along_m"] * (tip - base) / np.linalg.norm(tip - base)
        return bones.extreme("scapula_r", near, UP)
    if kind == "ac_capsule":
        side = -1.0 if rule["bone"] == "clavicle_r" else 1.0
        d = np.array(rule["direction"], float)
        near = bones.ac + side * rule["across_m"] * bones.lat + rule["radius_m"] * d
        return bones.extreme(rule["bone"], near, d)
    if kind == "midline":
        return np.array([bones.sc[0], bones.sc[1] - rule["below_sc_m"], 0.0])
    if kind == "from_sc":
        return bones.sc + np.array(rule["offset_m"])
    if kind == "below":
        return np.array(rule["of"]) + np.array(rule["offset_m"])
    raise ValueError(kind)


def place_all(bones: Bones, rows: list[dict[str, Any]]) -> None:
    for row in rows:
        ends = {}
        order = ("insertion", "origin") if row["origin"]["landmark"]["rule"] == "below" else ("origin", "insertion")
        for end in order:
            rule = dict(row[end]["landmark"])
            if rule["rule"] == "below":
                rule["of"] = ends["insertion"]
            ends[end] = landmark(bones, rule)
            row[end]["pos"] = [round(float(x), 6) for x in bones.to_body(row[end]["body"], ends[end])]


def site(row: dict[str, Any], k: int, side: str = "r") -> str:
    return f"{row['name']}_ligament-P{k}_{side}"


def blocks(table: dict[str, Any]) -> dict[tuple[Path, str], str]:
    out: dict[tuple[Path, str], list[str]] = {}
    for row in table["ligaments"]:
        for k, end in ((1, row["origin"]), (2, row["insertion"])):
            pos = np.array(end["pos"])
            if end["body"] == "chest_r":
                for side, p in (("r", pos), ("l", pos * [1, 1, -1])):
                    out.setdefault((TORSO, "chest_r"), []).append(
                        PAD["torso"] + f'<site name="{site(row, k, side)}" pos="{_f(p)}"/>'
                    )
            else:
                out.setdefault((CHAIN, end["body"]), []).append(PAD["arm"] + f'<site name="{site(row, k)}" pos="{_f(pos)}"/>')
    tendon = []
    for row in table["ligaments"]:
        tendon += [
            f'        <spatial name="{row["name"]}_ligament_r" class="myoarm_ligament" '
            f'springlength="0 {row["l0_m"]:.6g}" stiffness="0 {row["b_N_per_m2"]:.6g} 0">',
            f'            <site site="{site(row, 1)}"/>',
            f'            <site site="{site(row, 2)}"/>',
            "        </spatial>",
        ]
    out[(TENDON, "girdle")] = tendon
    return {key: "\n".join(lines) + "\n" for key, lines in out.items()}


def _f(v: Any) -> str:
    return " ".join(f"{x:.6g}" for x in np.ravel(v))


def write(table: dict[str, Any]) -> None:
    for (path, label), block in blocks(table).items():
        write_generated_block(path, SCRIPT, block, "", label=label)


def strand_lengths(model: mujoco.MjModel, qpos: np.ndarray, rows: list[dict[str, Any]]) -> np.ndarray:
    """Straight-line strand lengths for each qpos (sites from the table, computed without the tendons)."""
    data = mujoco.MjData(model)
    out = np.zeros((len(qpos), len(rows)))
    for i, q in enumerate(qpos):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        for j, row in enumerate(rows):
            p = [
                data.xpos[model.body(e["body"]).id] + data.xmat[model.body(e["body"]).id].reshape(3, 3) @ e["pos"]
                for e in (row["origin"], row["insertion"])
            ]
            out[i, j] = np.linalg.norm(p[0] - p[1])
    return out


def fit(table: dict[str, Any], zf: zipfile.ZipFile) -> None:
    model = myo_sim.load_model("myoarm_r")
    place_all(Bones(model), table["ligaments"])
    rows = table["ligaments"]
    frames = seth_frames(zf, model)
    lengths = strand_lengths(model, np.vstack([model.qpos0[None], frames]), rows)
    for j, row in enumerate(rows):
        rest, p99 = lengths[0, j], np.percentile(lengths[1:, j], 99)
        by_rest = rest / (1 + row["eps_rest"])
        by_range = p99 / (1 + row["eps_t"])
        l0 = max(by_rest, by_range)
        row["slack_rule"] = "rest strain" if by_rest >= by_range else "end of measured range"
        row["l0_m"] = round(float(l0), 6)
        row["b_N_per_m2"] = round(row["k_N_per_strain"] / (2 * row["eps_t"] * l0**2), 1)
        strain = lengths[1:, j] / l0 - 1
        row["length_rest_mm"] = round(1e3 * float(rest), 2)
        row["strain_rest"] = round(float(rest / l0 - 1), 4)
        row["strain_seth_motions_p50_max"] = [round(float(np.median(strain)), 4), round(float(strain.max()), 4)]
        row["length_change_seth_motions_mm"] = round(1e3 * float(lengths[1:, j].max() - lengths[1:, j].min()), 2)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--zip", type=Path, help="ThoracoscapularShoulderPaperMaterials.zip from SimTK (for --fit)")
    parser.add_argument("--fit", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    table = json.loads(TABLE.read_text())
    if args.fit:
        if args.zip is None:
            parser.error("--fit needs --zip")
        fit(table, zipfile.ZipFile(args.zip))
        TABLE.write_text(json.dumps(table, indent=2) + "\n")
    if args.fit or args.write:
        write(table)


if __name__ == "__main__":
    main()
