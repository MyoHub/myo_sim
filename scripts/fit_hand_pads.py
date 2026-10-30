"""Fit the fingertip and palm pads of the right hand and write them into ``myoarm_r_chain.xml``.

Inputs and results are in ``build/data/hand_pads.json``::

    python scripts/fit_hand_pads.py             # fit, update the JSON and the chain XML
    python scripts/fit_hand_pads.py --xml-only  # write the tabled values into the chain XML

**Fingertip pads.** The palmar soft tissue of each fingertip, measured like Dallard et al. 2018's D_N0
(``fingertip_soft_tissue_mm``), is set to the index value (6.4 mm) scaled by the digit's distal-phalanx
length. The pad is an ellipsoid on the palmar side of the distal phalanx, its axes along the phalanx
(palmar, long axis, lateral); its palmar position is solved so that the outermost skin at the measuring
station is at the target. A distal capsule that alone already exceeds the target is moved dorsally by the
excess.

**Palm pads.** No palmar soft-tissue thickness is published (ultrasound gives muscle thickness only) and
the model has no skin mesh, so each palm pad's thickness is fitted to the fist: with MCP, PIP and DIP of
the digits it serves at their flexion limits, the nearest fingertip just touches the pad. A pad no
fingertip reaches keeps its tabled lower bound.

The model is the XML; the script runs only during development, and a test checks that the XML holds the
tabled values.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA_PATH = ROOT / "myo_sim" / "build" / "data" / "hand_pads.json"
CHAIN_XML = ROOT / "myo_sim" / "models" / "arm" / "assets" / "myoarm_r_chain.xml"
MESH = mujoco.mjtGeom.mjGEOM_MESH


def load_table(path: Path = DATA_PATH) -> dict[str, Any]:
    return json.loads(path.read_text())


# --- geometry helpers (shared with scripts/hand_literature_comparison.py) --------------------------------


def body_geoms(model: mujoco.MjModel, body: str) -> list[int]:
    b = model.body(body).id
    return [g for g in range(model.ngeom) if model.geom_bodyid[g] == b]


def is_collision(model: mujoco.MjModel, g: int) -> bool:
    return model.geom_group[g] == 4 and model.geom_type[g] != MESH


def bone_geom(model: mujoco.MjModel, body: str) -> int:
    return next(g for g in body_geoms(model, body) if model.geom_type[g] == MESH)


def mesh_vertices(model: mujoco.MjModel, data: mujoco.MjData, g: int) -> np.ndarray:
    mesh = model.geom_dataid[g]
    local = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    return local @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]


def ray_geom(model: mujoco.MjModel, data: mujoco.MjData, g: int, pnt: np.ndarray, vec: np.ndarray) -> float:
    if model.geom_type[g] == MESH:
        return mujoco.mj_rayMesh(model, data, g, pnt, vec)
    return mujoco.mju_rayGeom(data.geom_xpos[g], data.geom_xmat[g], model.geom_size[g], pnt, vec, model.geom_type[g])


def palmar_direction(model: mujoco.MjModel, data: mujoco.MjData, body: str, flexor: str) -> tuple[np.ndarray, np.ndarray]:
    """Unit long axis and palmar direction of a phalanx at the current posture.

    The long axis runs from the joint centre to the farthest bone vertex; palmar is the way that vertex
    moves when the joint turns in the direction that shortens *flexor*, made perpendicular to the axis.
    *data* is left at its original posture.
    """
    jid = model.body_jntadr[model.body(body).id]
    bone = bone_geom(model, body)
    adr = model.jnt_qposadr[jid]
    q0 = data.qpos[adr]
    verts = mesh_vertices(model, data, bone)
    anchor = data.xanchor[jid].copy()
    tip_index = int(np.argmax(np.linalg.norm(verts - anchor, axis=1)))
    tip, length = verts[tip_index], data.actuator_length[model.actuator(flexor).id]
    data.qpos[adr] = q0 + 1e-3
    mujoco.mj_forward(model, data)
    sign = 1.0 if data.actuator_length[model.actuator(flexor).id] < length else -1.0
    data.qpos[adr] = q0 + sign * 1e-3
    mujoco.mj_forward(model, data)
    moved = mesh_vertices(model, data, bone)[tip_index]
    data.qpos[adr] = q0
    mujoco.mj_forward(model, data)
    axis = (tip - anchor) / np.linalg.norm(tip - anchor)
    palmar = (moved - tip) - np.dot(moved - tip, axis) * axis
    return axis, palmar / np.linalg.norm(palmar)


def fingertip_frame(model: mujoco.MjModel, data: mujoco.MjData, body: str, flexor: str) -> dict[str, np.ndarray]:
    """Joint centre, long axis, palmar and lateral directions, mid-line offset and bone length (world)."""
    axis, palmar = palmar_direction(model, data, body, flexor)
    origin = data.xanchor[model.body_jntadr[model.body(body).id]].copy()
    verts = mesh_vertices(model, data, bone_geom(model, body))
    lateral = np.cross(axis, palmar)
    return {
        "origin": origin,
        "axis": axis,
        "palmar": palmar,
        "lateral": lateral,
        "midline": float(np.mean((verts - origin) @ lateral)),
        "length": float(np.max((verts - origin) @ axis)),
    }


def soft_tissue_at(
    model: mujoco.MjModel, data: mujoco.MjData, frame: dict, station: float, bone: int, skin: list[int]
) -> float:
    """Distance (m) along the palmar ray at *station* from the outermost of *skin* to *bone*; nan if missed."""
    start = frame["origin"] + frame["midline"] * frame["lateral"] + station * frame["axis"] + 0.05 * frame["palmar"]
    bone_hit = ray_geom(model, data, bone, start, -frame["palmar"])
    skin_hits = [x for x in (ray_geom(model, data, g, start, -frame["palmar"]) for g in skin) if x >= 0]
    return bone_hit - min(skin_hits) if bone_hit >= 0 and skin_hits else float("nan")


def fingertip_soft_tissue_mm(model: mujoco.MjModel, data: mujoco.MjData, body: str, flexor: str, corner_m: float) -> float:
    """Palmar soft tissue of a fingertip (mm), measured like Dallard 2018's D_N0.

    D_N0 runs from the distal palmar corner of the phalanx, perpendicular to its long axis, to the skin
    (Dallard Fig. 2). The corner is taken *corner_m* proximal of the bone tip (half the phalanx height);
    the skin is the outermost collision geom of the phalanx on the palmar ray through the bone's mid-line.
    """
    frame = fingertip_frame(model, data, body, flexor)
    skin = [g for g in body_geoms(model, body) if is_collision(model, g)]
    return 1000.0 * soft_tissue_at(model, data, frame, frame["length"] - corner_m, bone_geom(model, body), skin)


def quat_from_axes(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.column_stack([x, y, z]).ravel())
    return quat


def to_body(
    data: mujoco.MjData, body_id: int, point: np.ndarray | None = None, rotation: np.ndarray | None = None
) -> np.ndarray:
    """A world point (or a world rotation's quaternion) in the frame of *body_id*."""
    rot = data.xmat[body_id].reshape(3, 3)
    if point is not None:
        return rot.T @ (point - data.xpos[body_id])
    quat, body_inv = np.zeros(4), np.zeros(4)
    mujoco.mju_negQuat(body_inv, data.xquat[body_id])
    mujoco.mju_mulQuat(quat, body_inv, rotation)
    return quat


# --- fits ---------------------------------------------------------------------------------------------------


def _arm_model() -> mujoco.MjModel:
    import myo_sim

    return myo_sim.load_spec("myoarm_r").compile()


def fit_fingertips(table: dict[str, Any]) -> None:
    spec = table["fingertip"]
    model = _arm_model()
    for row in spec["digits"]:  # measure from the tabled capsule placement, so a rerun gives the same fit
        original = row["capsule_original"]
        ends = np.array(original["fromto"]).reshape(2, 3) if "fromto" in original else None
        model.geom_pos[model.geom(row["capsule"]).id] = ends.mean(axis=0) if ends is not None else original["pos"]
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    corner = spec["corner_m"]
    frames = {row["body"]: fingertip_frame(model, data, row["body"], row["flexor"]) for row in spec["digits"]}
    reference = frames[spec["digits"][0]["body"]]["length"]
    for row in spec["digits"]:
        frame, body = frames[row["body"]], model.body(row["body"]).id
        scale = frame["length"] / reference
        target = spec["target_m"] * scale
        bone = bone_geom(model, row["body"])
        capsule = model.geom(row["capsule"]).id
        station = frame["length"] - corner
        # The capsule alone must not exceed the target: move it dorsally by the excess.
        alone = soft_tissue_at(model, data, frame, station, bone, [capsule])
        shift = max(0.0, alone - target)
        dorsal = -shift * (data.xmat[body].reshape(3, 3).T @ frame["palmar"])  # body frame
        # Pad: semi-axes (lateral, long axis, palmar) scaled with the digit; its distal end at the bone tip.
        size = np.array(spec["pad_semi_axes_m"]) * scale
        centre_station = frame["length"] - size[1]
        start = frame["origin"] + frame["midline"] * frame["lateral"] + station * frame["axis"] + 0.05 * frame["palmar"]
        bone_surface = start - ray_geom(model, data, bone, start, -frame["palmar"]) * frame["palmar"]
        u = (station - centre_station) / size[1]
        bulge = size[2] * np.sqrt(max(0.0, 1.0 - u * u))
        centre = bone_surface + (target - bulge) * frame["palmar"] + (centre_station - station) * frame["axis"]
        quat = quat_from_axes(frame["lateral"], frame["axis"], frame["palmar"])
        row["target_m"] = round(float(target), 6)
        row["scale"] = round(float(scale), 4)
        row["fitted"] = {
            "pad_pos": [round(float(v), 6) for v in to_body(data, body, point=centre)],
            "pad_quat": [round(float(v), 6) for v in to_body(data, body, rotation=quat)],
            "pad_size": [round(float(v), 6) for v in size],
            "capsule_dorsal_shift_m": round(float(shift), 6),
            "capsule_shift_local_m": [round(float(v), 6) for v in dorsal],
        }


def full_fist(model: mujoco.MjModel, data: mujoco.MjData, digits: list[int]) -> None:
    """MCP, PIP and DIP of *digits* at their flexion limits, everything else at qpos0."""
    data.qpos[:] = model.qpos0
    for d in digits:
        for joint in (f"mcp{d}_flexion_r", f"pm{d}_flexion_r", f"md{d}_flexion_r"):
            j = model.joint(joint).id
            data.qpos[model.jnt_qposadr[j]] = model.jnt_range[j][1]
    mujoco.mj_forward(model, data)


def fingertip_to_pad_m(model: mujoco.MjModel, data: mujoco.MjData, pad: int, digits: list[int]) -> float:
    """Smallest signed distance from the distal collision geoms of *digits* to geom *pad*."""
    fromto = np.zeros(6)
    tips = [g for d in digits for g in body_geoms(model, f"distph{d}_r") if is_collision(model, g)]
    return min(mujoco.mj_geomDistance(model, data, g, pad, 0.2, fromto) for g in tips)


def place_palm_pad(model: mujoco.MjModel, data: mujoco.MjData, row: dict[str, Any], thickness: float) -> tuple:
    """Pad centre (body frame), quaternion and size for a palm pad of *thickness* (q = 0 posture)."""
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    body = model.body(row["body"]).id
    rot = data.xmat[body].reshape(3, 3)
    _, palmar = palmar_direction(model, data, row["palmar_from"]["body"], row["palmar_from"]["flexor"])
    long_axis = rot @ np.array(row["long_axis_local"])
    palmar = palmar - np.dot(palmar, long_axis) * long_axis
    palmar /= np.linalg.norm(palmar)
    lateral = np.cross(long_axis, palmar)
    anchor = data.xpos[body] + rot @ np.array(row["anchor_local_m"])
    # Palmar bone surface under the anchor: the most palmar bone vertex within 4 mm of the palmar line.
    verts = np.vstack([mesh_vertices(model, data, bone_geom(model, b)) for b in row["bones"]])
    rel = verts - anchor
    along = rel @ palmar
    near = np.linalg.norm(rel - np.outer(along, palmar), axis=1) < 0.004
    surface = anchor + float(np.max(along[near])) * palmar
    centre = surface + row["centre_depth_fraction"] * thickness * palmar
    size = np.array([row["semi_axes_m"][0], row["semi_axes_m"][1], thickness])
    quat = quat_from_axes(lateral, long_axis, palmar)
    return to_body(data, body, point=centre), to_body(data, body, rotation=quat), size


def palm_soft_tissue_mm(model: mujoco.MjModel, data: mujoco.MjData, pad: str, bones: list[str]) -> float:
    """Soft tissue under a palm pad (mm): from the pad's palmar surface to the most palmar bone vertex within
    4 mm of the pad's palmar axis (the pad's local z) through its centre, at the current posture."""
    g = model.geom(pad).id
    palmar = data.geom_xmat[g].reshape(3, 3)[:, 2]
    centre = data.geom_xpos[g]
    verts = np.vstack([mesh_vertices(model, data, bone_geom(model, b)) for b in bones])
    rel = verts - centre
    along = rel @ palmar
    near = np.linalg.norm(rel - np.outer(along, palmar), axis=1) < 0.004
    return 1000.0 * (model.geom_size[g][2] - float(np.max(along[near])))


def fit_palm(table: dict[str, Any]) -> None:
    import myo_sim

    for row in table["palm"]:
        spec = myo_sim.load_spec("myoarm_r")
        model = spec.compile()
        data = mujoco.MjData(model)

        def gap(thickness: float, _row: dict = row) -> float:
            pos, quat, size = place_palm_pad(model, data, _row, thickness)
            g = model.geom(_row["name"]).id
            model.geom_pos[g], model.geom_quat[g], model.geom_size[g] = pos, quat, size
            model.geom_rbound[g] = float(np.max(size))  # stale bounds would cull the distance query
            model.geom_aabb[g] = [0.0, 0.0, 0.0, *size]
            full_fist(model, data, _row["digits"] or [2, 3, 4, 5])
            return fingertip_to_pad_m(model, data, g, _row["digits"]) if _row["digits"] else 1.0

        lo, hi = row["thickness_bounds_m"]
        if row["digits"] and gap(lo) > 0 > gap(hi):
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if gap(mid) > 0 else (lo, mid)
            thickness, method = hi, "fist"
        else:
            thickness, method = row["lower_bound_m"], "lower bound (no fingertip reaches the pad in the fist)"
        pos, quat, size = place_palm_pad(model, data, row, thickness)
        row["fitted"] = {
            "thickness_m": round(float(thickness), 6),
            "method": method,
            "pos": [round(float(v), 6) for v in pos],
            "quat": [round(float(v), 6) for v in quat],
            "size": [round(float(v), 6) for v in size],
        }


# --- XML ------------------------------------------------------------------------------------------------------


def _number(value: float) -> str:
    return f"{float(value) + 0.0:.6g}"


def set_geom_attributes(text: str, name: str, attributes: dict[str, list[float]]) -> str:
    """*text* with geom *name*'s attributes set (``euler`` dropped when ``quat`` is set), rest unchanged."""
    tags = re.findall(rf'<geom\b[^>]*\bname="{re.escape(name)}"[^>]*/>', text)
    if len(tags) != 1:
        raise ValueError(f"{CHAIN_XML.name}: expected one geom {name}, found {len(tags)}")
    tag = tags[0]
    if "quat" in attributes:
        tag = re.sub(r'\s+euler="[^"]*"', "", tag)
    for key, values in attributes.items():
        value = " ".join(_number(v) for v in values)
        tag = (
            re.sub(rf'\s{key}="[^"]*"', f' {key}="{value}"', tag)
            if f' {key}="' in tag
            else tag[:-2].rstrip() + f' {key}="{value}"/>'
        )
    return text.replace(tags[0], tag)


def shift_geom(text: str, name: str, original: dict[str, list[float]], shift: list[float]) -> str:
    """*text* with geom *name* at its tabled original ``pos`` (or ``fromto``) plus *shift*."""
    if "fromto" in original:
        moved = [v + shift[i % 3] for i, v in enumerate(original["fromto"])]
        return set_geom_attributes(text, name, {"fromto": moved})
    return set_geom_attributes(text, name, {"pos": [v + s for v, s in zip(original["pos"], shift)]})


def chain_with_pads(text: str, table: dict[str, Any]) -> str:
    for row in table["fingertip"]["digits"]:
        f = row["fitted"]
        text = set_geom_attributes(text, row["pad"], {"pos": f["pad_pos"], "quat": f["pad_quat"], "size": f["pad_size"]})
        if f["capsule_dorsal_shift_m"] > 0:
            text = shift_geom(text, row["capsule"], row["capsule_original"], f["capsule_shift_local_m"])
    for row in table["palm"]:
        if "fitted" not in row:
            continue
        f = row["fitted"]
        text = set_geom_attributes(text, row["name"], {"pos": f["pos"], "quat": f["quat"], "size": f["size"]})
    return text


def write_xml(table: dict[str, Any]) -> None:
    CHAIN_XML.write_text(chain_with_pads(CHAIN_XML.read_text(), table))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--xml-only", action="store_true", help="only write the tabled values into the chain XML")
    args = parser.parse_args()
    table = load_table()
    if not args.xml_only:
        fit_fingertips(table)
        write_xml(table)  # the palm fit runs on the model with the fitted fingertips
        fit_palm(table)
        DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    write_xml(table)
    for row in table["fingertip"]["digits"]:
        print(
            f"{row['pad']}: target {1000 * row['target_m']:.2f} mm, capsule moved {1000 * row['fitted']['capsule_dorsal_shift_m']:.2f} mm"
        )
    for row in table["palm"]:
        print(f"{row['name']}: {1000 * row['fitted']['thickness_m']:.2f} mm ({row['fitted']['method']})")


if __name__ == "__main__":
    main()
