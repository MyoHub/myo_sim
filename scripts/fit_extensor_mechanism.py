"""Write the finger extensor mechanism of ``build/data/extensor_mechanism.json`` into the right-arm XML.

Option (a) of the extensor-mechanism proposal: every extensor (EDC2-5, EIP, EDM) and every intrinsic that
inserts into a lateral band (PI1-3, DI2-4, LUM1-4; their tendons are written by
``generate_hand_intrinsic_muscles.py`` from this table's fractions) keeps its actuator and its tendon. The tendon's main path
ends at a junction site on the proximal phalanx (hood ``EM{d}_H_r``, radial band ``EM{d}_RB_r``, ulnar band
``EM{d}_UB_r``) and continues in MuJoCo pulley branches, each carrying the force fraction ``1 / divisor``:

- central slip: junction -> ``EM{d}_CSP_r`` -> PIP wrap -> ``EM{d}_CS_r`` (base of the middle phalanx);
- lateral band: junction -> band site -> band wrap (coaxial with the PIP) -> band site over the PIP on the
  middle phalanx -> ``EM{d}_T_r`` (convergence on the middle phalanx) -> DIP wrap -> ``EM{d}_TI_r`` (base of
  the distal phalanx).

What the branches leave of the force stays on the proximal phalanx at the junction. The oblique retinacular
ligament (``lig_ORL{d}_r``) is a tension-only fixed tendon over PIP and DIP.

This script writes

- the junction sites and band wraps into labelled blocks of their bodies in ``myoarm_r_chain.xml``;
- each branched extensor tendon into a labelled block where it stood in ``myoarm_r_tendon.xml`` (tendon order
  kept) and the ligaments into its main block (the extrinsic slack lengths belong to ``fit_muscle_passive.py``).

``--fit`` first fits the lumbrical fractions and the extensor share ``c`` to the index moment arms and the
ORL stiffness to Ueba et al. 2011, and records them in the table.

Run: ``uv run python scripts/fit_extensor_mechanism.py [--fit]``.
"""

from __future__ import annotations

import argparse
import json
import re
from typing import Any

import mujoco
import numpy as np

from myo_sim.build.compose import RIGHT_ARM_CHAIN_XML, RIGHT_ARM_TENDONS_XML, build_spec
from myo_sim.build.hand_data import DATA_DIR, SPRING_FAR, write_generated_block

SCRIPT = "fit_extensor_mechanism.py"
DATA_PATH = DATA_DIR / "extensor_mechanism.json"
MUSCLE_CLASS = "myoarm_forearm_muscle"
DEG = np.pi / 180.0
LEE_POSTURES_DEG = [(m, p, d) for m in (0.0, 30.0, 60.0) for p, d in ((0.0, 0.0), (30.0, 20.0), (45.0, 30.0))]


def load_table() -> dict[str, Any]:
    return json.loads(DATA_PATH.read_text())


def _number(value: float) -> str:
    return f"{float(value) + 0.0:.6g}"


def _unit(v: np.ndarray) -> np.ndarray:
    return np.asarray(v, float) / np.linalg.norm(v)


# --- geometry -------------------------------------------------------------------------------------------


def digit_frame(model: mujoco.MjModel, digit: dict[str, Any]) -> dict[str, np.ndarray]:
    """PIP centre, axis (radial), distal and dorsal unit vectors in the proximal phalanx's frame."""
    pp, mp = model.body(digit["bodies"]["proximal"]), model.body(digit["bodies"]["middle"])
    if not np.allclose(model.body_quat[mp.id], [1, 0, 0, 0]):
        raise ValueError(f"{mp.name}: the middle phalanx frame must be aligned with the proximal one")
    centre = model.body_pos[mp.id].copy()
    axis = _unit(model.jnt_axis[model.joint(digit["joints"]["pip"]).id])
    distal = _unit(centre - (centre @ axis) * axis)
    dorsal = _unit(np.cross(distal, axis))
    if dorsal[2] < 0:  # +z of the phalanx frames is dorsal
        dorsal = -dorsal
    mesh = model.geom(pp.name).dataid[0]
    verts = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    g = model.geom(pp.name)
    verts = model.geom_pos[g.id] + verts @ _quat_mat(model.geom_quat[g.id]).T
    head = verts[(verts @ distal) > centre @ distal - 0.008]
    half_width = 0.5 * float(np.ptp(head @ axis))
    return {"centre": centre, "axis": axis, "distal": distal, "dorsal": dorsal, "half_width": half_width}


def _quat_mat(q: np.ndarray) -> np.ndarray:
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, np.asarray(q, float))
    return mat.reshape(3, 3)


def band_sites(table: dict[str, Any], model: mujoco.MjModel, key: str) -> dict[str, np.ndarray]:
    """Band sites, band wrap centre and side site of digit *key* (proximal-phalanx frame)."""
    geo, f = table["band_geometry"], digit_frame(model, table["digits"][key])
    base = f["centre"] - geo["proximal_of_pip_m"]["value"] * f["distal"] + geo["dorsal_of_pip_axis_m"]["value"] * f["dorsal"]
    md = model.body(table["digits"][key]["bodies"]["distal"])
    mid_distal = _unit(model.body_pos[md.id] - (model.body_pos[md.id] @ f["axis"]) * f["axis"])
    mid_dorsal = _unit(np.cross(mid_distal, f["axis"]))
    mid_dorsal = mid_dorsal if mid_dorsal[2] > 0 else -mid_dorsal
    via = geo["distal_band_site"]
    angle = via["angle_deg"]["value"] * DEG
    over = via["radius_m"]["value"] * (np.cos(angle) * mid_distal + np.sin(angle) * mid_dorsal)
    return {
        "RBD": over + f["half_width"] * f["axis"],
        "UBD": over - f["half_width"] * f["axis"],
        "RB": base + f["half_width"] * f["axis"],
        "UB": base - f["half_width"] * f["axis"],
        "wrap": f["centre"],
        "side": f["centre"] + geo["band_side_m"]["value"] * f["dorsal"],
        "axis": f["axis"],
        "half_width": f["half_width"],
    }


def _z_to(axis: np.ndarray) -> np.ndarray:
    """Quaternion turning +z onto *axis* (a quat, not ``zaxis``, so the mirroring of the left arm applies)."""
    quat = np.zeros(4)
    mujoco.mju_quatZ2Vec(quat, np.asarray(axis, float))
    return quat


def _site(name: str, pos: np.ndarray) -> str:
    return f'<site name="{name}" pos="{" ".join(_number(v) for v in pos)}"/>'


def chain_blocks(table: dict[str, Any], model: mujoco.MjModel) -> dict[str, str]:
    """Generated block text per body (label = body name)."""
    blocks = {}
    radius = table["band_geometry"]["dorsal_of_pip_axis_m"]["value"]
    for key, digit in table["digits"].items():
        s, b = digit["sites"], band_sites(table, model, key)
        pp, mp, dp = (digit["bodies"][k] for k in ("proximal", "middle", "distal"))
        lines = [
            f"<!-- extensor mechanism, digit {key}: junction sites (scripts/{SCRIPT}, build/data/extensor_mechanism.json) -->",
            _site(f"EM{key}_H_r", s["H"]["pos"]),
            _site(f"EM{key}_CSP_r", s["CSP"]["pos"]),
            _site(f"EM{key}_RB_r", b["RB"]),
            _site(f"EM{key}_UB_r", b["UB"]),
            f'<geom name="EM{key}_band_wrap_r" type="cylinder" size="{_number(radius)} {_number(b["half_width"])}"'
            f' pos="{" ".join(_number(v) for v in b["wrap"])}" quat="{" ".join(_number(v) for v in _z_to(b["axis"]))}"'
            ' class="myoarm_wrap"/>',
            _site(f"EM{key}_band_side_r", b["side"]),
        ]
        blocks[pp] = lines
        blocks[mp] = [
            _site(f"EM{key}_CS_r", s["CS"]["pos"]),
            _site(f"EM{key}_RBD_r", b["RBD"]),
            _site(f"EM{key}_UBD_r", b["UBD"]),
            _site(f"EM{key}_T_r", s["T"]["pos"]),
        ]
        blocks[dp] = [_site(f"EM{key}_TI_r", s["TI"]["pos"])]
    return {body: "".join(f"        {line}\n" for line in lines) for body, lines in blocks.items()}


# --- tendons --------------------------------------------------------------------------------------------


def branch_fractions(table: dict[str, Any], row: dict[str, Any]) -> dict[str, float]:
    """Force fraction of each branch of extensor *row* (central, radial_band, ulnar_band)."""
    c, r = table["fractions"]["extensor"]["c"]["value"], table["fractions"]["extensor"]["r"]["value"]
    return {"central": c * (1.0 - r), "radial_band": 0.5 * c * r, "ulnar_band": 0.5 * c * r}


def intrinsic_fractions(table: dict[str, Any], kind: str, junction: str) -> dict[str, float]:
    """Fractions of the force reaching band junction *junction* (``RB`` / ``UB``) that go on to each branch.

    *kind* is ``lumbrical`` or ``interosseous``; what the branches leave stays on the proximal phalanx at the
    junction. ``generate_hand_intrinsic_muscles.py`` multiplies them by each muscle's hood fraction.
    """
    fr = table["fractions"][kind]
    band = "radial_band" if junction == "RB" else "ulnar_band"
    return {"central": fr["central"]["value"], band: fr["band"]["value"]}


def _element(item: str) -> str:
    kind, *rest = item.split(":")
    if kind == "site":
        return f'<site site="{rest[0]}"/>'
    return f'<geom geom="{rest[0]}" sidesite="{rest[1]}"/>'


def junction_branches(table: dict[str, Any], key: str, junction: str, fractions: dict[str, float]) -> list[str]:
    """Pulley branches from junction site *junction* of digit *key*, each with divisor 1 / its force fraction."""
    digit = table["digits"][key]
    pip = f'<geom geom="{digit["pip_wrap"]["geom"]}" sidesite="{digit["pip_wrap"]["sidesite"]}"/>'
    dip = f'<geom geom="{digit["dip_wrap"]["geom"]}" sidesite="{digit["dip_wrap"]["sidesite"]}"/>'
    wrap = f'<geom geom="EM{key}_band_wrap_r" sidesite="EM{key}_band_side_r"/>'
    branches = {
        "central": [junction, f"EM{key}_CSP_r", pip, f"EM{key}_CS_r"],
        "radial_band": [junction, f"EM{key}_RB_r", wrap, f"EM{key}_RBD_r", f"EM{key}_T_r", dip, f"EM{key}_TI_r"],
        "ulnar_band": [junction, f"EM{key}_UB_r", wrap, f"EM{key}_UBD_r", f"EM{key}_T_r", dip, f"EM{key}_TI_r"],
    }
    lines = []
    for branch, fraction in fractions.items():
        lines.append(f'<pulley divisor="{_number(1.0 / fraction)}"/>')
        items = branches[branch]
        if items[1] == junction:  # a band junction starts its band branch at itself
            items = items[1:]
        lines += [item if item.startswith("<") else f'<site site="{item}"/>' for item in items]
    return lines


def tendon_block(table: dict[str, Any], row: dict[str, Any]) -> str:
    """Labelled block of extensor *row*'s branched tendon."""
    key = str(row["digit"])
    junction = f"EM{key}_H_r"
    fractions = branch_fractions(table, row)
    rest = 1.0 - sum(fractions.values())
    summary = ", ".join(f"{b} {fractions[b]:.3f}" for b in fractions) + f", proximal phalanx {rest:.3f}"
    lines = [
        f"<!-- {row['name']}: extensor mechanism option (a), force fractions {summary} (build/data/extensor_mechanism.json) -->",
        f'<spatial name="{row["name"]}_r_tendon" class="{MUSCLE_CLASS}">',
    ]
    lines += [f"    {_element(item)}" for item in row["main_path"]]
    lines.append(f'    <site site="{junction}"/>')
    lines += [f"    {line}" for line in junction_branches(table, key, junction, fractions)]
    lines.append("</spatial>")
    return "".join(f"        {line}\n" for line in lines)


def ligament_block(table: dict[str, Any]) -> str:
    """Main block: one tension-only ORL fixed tendon per finger."""
    orl = table["orl"]
    lines = [
        "<!-- Oblique retinacular ligaments: tension-only, slack until DIP - PIP exceeds "
        f"{orl['slack_difference_deg']['value']:g} deg (Harris & Rutledge 1972); stiffness from Ueba et al. 2011 -->"
    ]
    for key, digit in table["digits"].items():
        res = orl["result"][key]
        arm, slack, k = res["arm_m"], res["slack_m"], res["stiffness_N_per_m"]
        lines.append(
            f'<fixed name="lig_ORL{key}_r" springlength="{_number(-SPRING_FAR)} {_number(slack)}" stiffness="{_number(k)} 0 0">'
        )
        lines.append(f'    <joint joint="{digit["joints"]["dip"]}" coef="{_number(arm)}"/>')
        lines.append(f'    <joint joint="{digit["joints"]["pip"]}" coef="{_number(-arm)}"/>')
        lines.append("</fixed>")
    return "".join(f"        {line}\n" for line in lines)


# --- write ----------------------------------------------------------------------------------------------


def skeleton() -> mujoco.MjModel:
    """``myoarm_r`` without actuators and tendons: the geometry the junction sites are placed on."""
    spec = build_spec("myoarm_r")
    for element in list(spec.actuators) + list(spec.tendons):
        spec.delete(element)
    return spec.compile()


def write_xml(table: dict[str, Any]) -> None:
    """Write the chain blocks, the extensor tendon blocks and the ligaments."""
    model = skeleton()
    for body, block in chain_blocks(table, model).items():
        write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, block, "</body>", body)
    for row in table["muscles"]:
        write_generated_block(RIGHT_ARM_TENDONS_XML, SCRIPT, tendon_block(table, row), "</tendon>", row["name"])
    if "result" in table["orl"]:
        write_generated_block(RIGHT_ARM_TENDONS_XML, SCRIPT, ligament_block(table), "</tendon>")


# --- moment arms and fits -------------------------------------------------------------------------------


def moment_arm(model: mujoco.MjModel, data: mujoco.MjData, tendon: str, joint: str, posture: dict[str, float]) -> float:
    """Extension moment arm (mm, positive = extensor: dL/dq with flexion positive) of *tendon* about *joint* at *posture* (deg)."""
    lengths = []
    for h in (1e-5, -1e-5):
        data.qpos[:] = model.qpos0
        for name, deg in posture.items():
            data.qpos[model.joint(name).qposadr[0]] = deg * DEG
        data.qpos[model.joint(joint).qposadr[0]] += h
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_tendon(model, data)
        lengths.append(float(data.ten_length[model.tendon(tendon).id]))
    return (lengths[0] - lengths[1]) / 2e-5 * 1e3


def index_arms(
    model: mujoco.MjModel, table: dict[str, Any], name: str, tendon: str | None = None
) -> dict[str, dict[str, float]]:
    """DIP and PIP extension arms of index muscle *name* (or of *tendon*): An 1983 range average and Lee 2008b
    nine-posture mean."""
    data = mujoco.MjData(model)
    tendon = tendon or f"{name}_r_tendon"
    rng = table["ranges"]
    out: dict[str, dict[str, float]] = {}
    for key, joint, span in (("dip", "md2_flexion_r", rng["an1983_dip_deg"]), ("pip", "pm2_flexion_r", rng["an1983_pip_deg"])):
        an = np.mean([moment_arm(model, data, tendon, joint, {joint: q}) for q in np.linspace(*span, 10)])
        lee = np.mean(
            [
                moment_arm(model, data, tendon, joint, {"mcp2_flexion_r": m, "pm2_flexion_r": p, "md2_flexion_r": d})
                for m, p, d in LEE_POSTURES_DEG
            ]
        )
        out[key] = {"an1983": round(float(an), 3), "lee2008b": round(float(lee), 3)}
    return out


def fit_c(table: dict[str, Any]) -> None:
    """Least-squares c (the arms are linear in c) for EDC2 and EIP; recorded with the residual z-scores."""
    ext = table["fractions"]["extensor"]
    ext["c"]["value"] = 1.0
    write_xml(table)
    model = build_spec("myoarm_r").compile()
    num = den = 0.0
    unit = {}
    for name, targets in table["targets"].items():
        unit[name] = index_arms(model, table, name)
        for key, rows in targets.items():
            for t in rows:
                g, w = unit[name][key][t["source"]], 1.0 / t["sd_mm"] ** 2
                num, den = num + w * g * t["mean_mm"], den + w * g * g
    c = float(np.clip(num / den, 0.0, 1.0))
    ext["c"]["value"] = round(c, 3)
    z = {
        f"{name} {key} {t['source']}": round((ext["c"]["value"] * unit[name][key][t["source"]] - t["mean_mm"]) / t["sd_mm"], 2)
        for name, targets in table["targets"].items()
        for key, rows in targets.items()
        for t in rows
    }
    ext["c"]["result"] = {
        "unbounded_value": round(num / den, 3),
        "z": z,
        "rms_z": round(float(np.sqrt(np.mean(np.square(list(z.values()))))), 3),
    }


def fit_lumbrical(table: dict[str, Any]) -> None:
    """Central-slip and band fractions of the lumbricals from the index lumbrical's DIP and PIP arms.

    A lumbrical's IP arms are linear in its two fractions: f_c times the central branch's arm plus f_b times
    the band branch's arm (both branches start at the radial band junction ``EM2_RB_r``). Two targets (An et
    al. 1983, Table 3: DIP and PIP extension arms of LU, averaged over the range) fix both fractions; the
    effective arms of Lee et al. 2008b are reported as checks.
    """
    lum = table["fractions"]["lumbrical"]
    spec = build_spec("myoarm_r")
    pieces = {"central": [], "band": []}
    for line in junction_branches(table, "2", "EM2_RB_r", {"central": 1.0, "radial_band": 1.0}):
        if line.startswith("<pulley"):
            current = "central" if not pieces["central"] else "band"
            continue
        pieces[current].append(line)
    for branch, lines in pieces.items():
        tendon = spec.add_tendon(name=f"unit_{branch}")
        for line in lines:
            if line.startswith("<site"):
                tendon.wrap_site(re.search(r'site="([^"]+)"', line).group(1))
            else:
                geom, side = re.search(r'geom="([^"]+)" sidesite="([^"]+)"', line).groups()
                tendon.wrap_geom(geom, side)
    model = spec.compile()
    unit = {b: index_arms(model, table, "", f"unit_{b}") for b in pieces}
    targets = lum["targets"]
    a = np.array([[unit["central"][k]["an1983"], unit["band"][k]["an1983"]] for k in ("dip", "pip")])
    b = np.array([targets[k]["mean_mm"] for k in ("dip", "pip")])
    fc, fb = np.clip(np.linalg.solve(a, b), 0.0, 1.0)
    lum["central"]["value"], lum["band"]["value"] = round(float(fc), 4), round(float(fb), 4)
    model_mm = {
        k: {src: round(fc * unit["central"][k][src] + fb * unit["band"][k][src], 2) for src in ("an1983", "lee2008b")}
        for k in ("dip", "pip")
    }
    lum["result"] = {
        "unit_arms_mm": unit,
        "model_mm": model_mm,
        "z_an1983": {
            k: round((model_mm[k]["an1983"] - targets[k]["mean_mm"]) / targets[k]["sd_mm"], 2) for k in ("dip", "pip")
        },
        "bone": round(1.0 - fc - fb, 4),
    }


def fit_orl(table: dict[str, Any]) -> None:
    """Arm, slack and stiffness of each finger's ORL (method in the table)."""
    import importlib.util
    from pathlib import Path

    from myo_sim.build.hand_data import load_passive_moment_table

    path = Path(__file__).with_name("fit_passive_moments.py")
    spec = importlib.util.spec_from_file_location("fit_passive_moments", path)
    passive = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(passive)
    orl = table["orl"]
    tg = orl["targets"]
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    rows = {r["names"]["r"]: r for r in load_passive_moment_table()["joints"]}
    extensor = {"2": "EDC2", "3": "EDC3", "4": "EDC4", "5": "EDC5"}
    result = {}
    for key, digit in table["digits"].items():
        dip = digit["joints"]["dip"]
        arm = (
            moment_arm(model, data, f"{extensor[key]}_r_tendon", dip, {})
            * 1e-3
            / branch_fractions(table, next(r for r in table["muscles"] if r["name"] == extensor[key]))["radial_band"]
            / 2.0
        )
        slack = arm * orl["slack_difference_deg"]["value"] * DEG
        row = rows[dip]
        kamper = abs(
            float(
                passive.source_torque(row["source_curve"], model.jnt_range[model.joint(dip).id])(
                    np.array([tg["dip_deg"] * DEG])
                )[0]
            )
        )
        preload = tg["preload_N"] * abs(moment_arm(model, data, f"{extensor[key]}_r_tendon", dip, {dip: tg["dip_deg"]})) * 1e-3
        share = tg["share_percent"][tg["pip_deg"].index(0)] / 100.0
        stretch = arm * tg["dip_deg"] * DEG - slack
        k = share * (kamper + preload) / (stretch * arm)
        shares = []
        for pip in tg["pip_deg"]:
            s = max(0.0, arm * (tg["dip_deg"] - pip) * DEG - slack)
            shares.append(round(100.0 * k * s * arm / (kamper + preload), 1))
        result[key] = {
            "arm_m": round(arm, 6),
            "slack_m": round(slack, 7),
            "stiffness_N_per_m": round(float(k), 1),
            "total_moment_Nm_at_target": round(kamper + preload, 4),
            "share_percent": shares,
        }
    orl["result"] = result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="refit c and the ORL first")
    args = parser.parse_args()
    table = load_table()
    if args.fit:
        fit_lumbrical(table)
        fit_c(table)
        write_xml(table)
        fit_orl(table)
        DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    write_xml(table)
    print(json.dumps({"c": table["fractions"]["extensor"]["c"], "orl": table["orl"].get("result")}, indent=1))


if __name__ == "__main__":
    main()
