"""Fit and write the fourth and fifth carpometacarpal joints of ``build/data/cmc_joints.json``.

``--fit`` computes, on ``myoarm_r`` at q = 0:

- each joint centre: a least-squares sphere through the vertices of the proximal 15 % of the metacarpal
  mesh along its principal axis (Buffi et al. 2013), in the metacarpal body frame;
- each joint axis: the tabled lunate-frame direction expressed in the body frame, signed so that +q flexes
  (moves the metacarpal head palmarly);
- the position of each moved site in its new body, at the same world position as before.

Without ``--fit`` the tabled values are written: the ``pos``, ``axis``, ``range`` and ``damping`` of
``<joint name="cmc4_flexion_r"/>`` and ``cmc5_flexion_r`` in ``myoarm_r_chain.xml``, the ``pos`` of the
moved sites, and the coupling equality in this script's generated block of ``myoarm_r_assets.xml``.

``--fit-cupping`` refits the flexion side of the end-range springs (``passive_joint_moments.json``) to the measured active
cupping (``active_cupping`` in the table).

Run: ``uv run python scripts/fit_cmc_joints.py [--fit] [--fit-cupping]``.
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import json
import re
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from myo_sim.build.compose import RIGHT_ARM_CHAIN_XML, build_spec
from myo_sim.build.hand_data import DATA_DIR, write_generated_block

SCRIPT = "fit_cmc_joints.py"
DATA_PATH = DATA_DIR / "cmc_joints.json"
PASSIVE_PATH = DATA_DIR / "passive_joint_moments.json"
ASSETS_XML = RIGHT_ARM_CHAIN_XML.with_name("myoarm_r_assets.xml")
BASE_FRACTION = 0.15  # proximal share of the metacarpal length used for the base sphere


def load_table(path: Path = DATA_PATH) -> dict[str, Any]:
    return json.loads(path.read_text())


def _number(value: float) -> str:
    return f"{float(value) + 0.0:.6g}"


# --- fit --------------------------------------------------------------------------------------------------


def _frames(model: mujoco.MjModel) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_kinematics(model, data)
    return data


def base_sphere(model: mujoco.MjModel, data: mujoco.MjData, geom: str) -> tuple[np.ndarray, float]:
    """Centre (world) and radius (m) of the sphere through the proximal base of mesh *geom*."""
    g = model.geom(geom).id
    mesh = model.geom_dataid[g]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    vertices = data.geom_xpos[g] + model.mesh_vert[start : start + count] @ data.geom_xmat[g].reshape(3, 3).T
    lunate = data.xmat[model.body("lunate_r").id].reshape(3, 3)
    centred = vertices - vertices.mean(axis=0)
    axis = np.linalg.svd(centred, full_matrices=False)[2][0]
    axis *= np.sign(axis @ lunate[:, 1])  # proximal (lunate +y)
    along = centred @ axis
    base = vertices[along > along.max() - BASE_FRACTION * (along.max() - along.min())]
    a = np.c_[2.0 * base, np.ones(len(base))]
    solution = np.linalg.lstsq(a, (base**2).sum(axis=1), rcond=None)[0]
    centre = solution[:3]
    return centre, float(np.sqrt(solution[3] + centre @ centre))


def fit(table: dict[str, Any]) -> None:
    """Joint centres, axes and moved-site positions on ``myoarm_r`` at q = 0 (recorded in *table*)."""
    model = build_spec("myoarm_r").compile()
    data = _frames(model)
    lunate_body = model.body("lunate_r").id
    lunate, lunate_pos = data.xmat[lunate_body].reshape(3, 3), data.xpos[lunate_body]
    for joint in table["joints"]:
        body = model.body(joint["body"]).id
        rotation, origin = data.xmat[body].reshape(3, 3), data.xpos[body]
        centre, radius = base_sphere(model, data, joint["mesh_geom"])
        axis_world = lunate @ (np.asarray(joint["axis"]["lunate"]) / np.linalg.norm(joint["axis"]["lunate"]))
        # +q must move the metacarpal head (the distal end, far from the centre) palmarly (lunate -z).
        head = data.xpos[body] - centre
        if np.cross(axis_world, head) @ (-lunate[:, 2]) < 0:
            axis_world = -axis_world
        joint["fitted"] = {
            "pos": [round(float(v), 6) for v in rotation.T @ (centre - origin)],
            "axis": [round(float(v), 6) for v in rotation.T @ axis_world],
            "centre_lunate_mm": [round(float(v), 2) for v in 1e3 * lunate.T @ (centre - lunate_pos)],
            "radius_mm": round(radius * 1e3, 2),
        }
    for move in table["site_moves"]:
        world = data.site_xpos[model.site(move["site"]).id]
        body = model.body(move["to"]).id
        move["pos"] = [round(float(v), 6) for v in data.xmat[body].reshape(3, 3).T @ (world - data.xpos[body])]


# --- XML --------------------------------------------------------------------------------------------------


def _set(text: str, tag: str, name: str, attributes: dict[str, str]) -> str:
    """*text* with the attributes of the (uncommented) element *tag* named *name* set."""
    comments = [m.span() for m in re.finditer(r"<!--.*?-->", text, re.S)]
    found = [
        m
        for m in re.finditer(rf'<{tag}\b[^>]*\bname="{re.escape(name)}"[^>]*/>', text)
        if not any(a <= m.start() < b for a, b in comments)
    ]
    if len(found) != 1:
        raise ValueError(f"{RIGHT_ARM_CHAIN_XML.name}: expected one {tag} {name}, found {len(found)}")
    start, end = found[0].span()
    element = text[start:end]
    for key, value in attributes.items():
        if f' {key}="' in element:
            element = re.sub(rf'\s{key}="[^"]*"', f' {key}="{value}"', element)
        else:
            element = element[:-2].rstrip() + f' {key}="{value}"/>'
    return text[:start] + element + text[end:]


def chain_with_joints(text: str, table: dict[str, Any]) -> str:
    """*text* of ``myoarm_r_chain.xml`` with the tabled joints and moved sites, rest unchanged."""
    for joint in table["joints"]:
        f = joint["fitted"]
        attributes = {
            "pos": " ".join(_number(v) for v in f["pos"]),
            "axis": " ".join(_number(v) for v in f["axis"]),
            "range": " ".join(_number(round(np.radians(v), 6)) for v in joint["range_deg"]["value"]),
            "damping": _number(joint["damping"]["value"]),
        }
        text = _set(text, "joint", joint["name"], attributes)
    for move in table["site_moves"]:
        text = _set(text, "site", move["site"], {"pos": " ".join(_number(v) for v in move["pos"])})
    return text


def equality_block(table: dict[str, Any]) -> str:
    """Generated block of ``myoarm_r_assets.xml``: the CMC4/5 coupling."""
    c = table["coupling"]
    return (
        f"        <!-- {c['definition']} ({c['cite']}) -->\n"
        f'        <joint joint1="{c["joint1"]}" joint2="{c["joint2"]}" name="{c["name"]}"'
        f' polycoef="0 {_number(c["ratio"])} 0 0 0" solimp="{" ".join(_number(v) for v in c["solimp"])}"/>\n'
    )


def write_xml(table: dict[str, Any]) -> None:
    RIGHT_ARM_CHAIN_XML.write_text(chain_with_joints(RIGHT_ARM_CHAIN_XML.read_text(), table))
    write_generated_block(ASSETS_XML, SCRIPT, equality_block(table), "</equality>")


def _script(name: str) -> Any:
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(f"{name}.py"))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def active_cupping(limit_moment: float, table: dict[str, Any]) -> list[float]:
    """CMC4 / CMC5 flexion in the literature page's cupping protocol with *limit_moment* (N m) at both flexion limits."""
    rows = json.loads(PASSIVE_PATH.read_text())
    for row in rows["joints"]:
        if row["joint"] in ("digit4_cmc_flexion", "digit5_cmc_flexion"):
            limit = next(j for j in table["joints"] if j["name"] == row["names"]["r"])["range_deg"]["value"][1]
            row["source_curve"]["params"]["k_positive"] = round(limit_moment / np.radians(limit), 4)
    PASSIVE_PATH.write_text(json.dumps(rows, indent=2) + "\n")
    with contextlib.redirect_stdout(io.StringIO()):
        sys.argv = ["fit_passive_moments.py"]
        _script("fit_passive_moments").main()
    return _script("hand_literature_comparison").hypothenar(build_spec("myoarm_r").compile())["hypothenar_cupping"]


def fit_cupping(table: dict[str, Any]) -> None:
    """The end-range moment at which active cupping reaches the measured CMC5 maximum (bisection; the coupling sets CMC4)."""
    fit = table["active_cupping"]
    lo, hi = fit["bounds_Nm"]
    for _ in range(14):
        mid = np.sqrt(lo * hi)
        lo, hi = (mid, hi) if active_cupping(mid, table)[1] > fit["target_deg"][1] else (lo, mid)
    cups = active_cupping(hi, table)
    fit["result"] = {"limit_moment_Nm": round(float(hi), 3), "cupping_deg": [round(c, 2) for c in cups]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="recompute centres, axes and moved sites first")
    parser.add_argument("--fit-cupping", action="store_true", help="refit the end-range springs to the measured active cupping")
    args = parser.parse_args()
    table = load_table()
    if args.fit_cupping:
        fit_cupping(table)
        DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
        print(table["active_cupping"]["result"])
        return
    if args.fit:
        fit(table)
        DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    write_xml(table)
    for joint in table["joints"]:
        print(joint["name"], json.dumps(joint["fitted"]))


if __name__ == "__main__":
    main()
