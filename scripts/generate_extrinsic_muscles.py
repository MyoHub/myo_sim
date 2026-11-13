"""Write the extrinsic-muscle revisions of ``build/data/extrinsic_muscles.json`` into the right-arm XML.

- **Juncturae tendinum:** a junction node on EDC3, EDC4 and EDC5 (labelled chain blocks ``node JT<d>``:
  a light body on a slide joint ``node_JT<d>_r`` in the metacarpal, on the EDC path), a tendon equality
  ``node_JT<d>_tie_r`` (assets block ``juncturae ties``) that ties the slide to the EDC's branched path
  from its via point ``EDC<d>-P4_r`` proximal of the node (``node_JT<d>_slide_r``, ``node_JT<d>_edc_r``;
  the same MCP wrap as the EDC, so the node moves with the tendon material), and the tension-only links
  ``lig_JT34_r`` and ``lig_JT45_r`` between the nodes (main tendon block). Same primitive and settings as
  the lumbrical nodes (``hand_intrinsic_muscles.json``, ``lumbrical_nodes``).
- **FCU:** the pisiform via point ``FCU-P4_r`` (position rewritten) and the hamate-hook via point
  ``FCU-P5_r`` (chain block ``FCU hamate``); FCU's ``lengthrange`` is shifted by its length change at q0,
  so its normalised fibre length at q0 is unchanged.
- **FDS5:** its force, PCSA times the specific tension of ``specific_tension.json``.
- **Wrap sides** (``wrap_sides``): the side site of PQ's ulnar wrap, moved from inside its geom (inside wrap) to outside (ordinary side wrap), and
  where tabled the geom's centre and radius.
- **MCP and PIP pulleys** (``mcp_pulleys``, ``pip_pulleys``): cylinders on the MCP and PIP flexion axes that hold
  the extrinsic finger tendons' arms past the via-site range (chain blocks ``MCP pulleys <d>``, ``PIP pulleys <d>``).
- **APL at the thumb CMC** (``thumb_extrinsics``): APL's MC1-base insertion from Barry et al. 2018 (site ``pos``
  in the chain), over a via site on the trapezium in the chain block ``thumb trapezium`` (``side_sites.json``).
- **ECU:** the groove via point ``ECU-P5_r`` on the ulna (chain block ``ECU groove``): the tendon's subsheath
  holds it in the groove of the ulnar head, which does not turn with the radius.

``--fit`` refits the FCU pisiform site to Horii 1993, the ECU groove site to Horii 1993 and Loren 1996, and
the junctura stiffness to von Schroeder & Botte 1993 (``juncturae.fit``) and records them in the table;
``--fit-ecu`` refits only the ECU groove site.

Run: ``uv run python scripts/generate_extrinsic_muscles.py [--fit | --fit-ecu]``.
"""

from __future__ import annotations

import argparse
import json
import re
from typing import Any

import mujoco
import numpy as np

from myo_sim.build.compose import (
    RIGHT_ARM_ASSETS_XML,
    RIGHT_ARM_CHAIN_XML,
    RIGHT_ARM_MUSCLES_XML,
    RIGHT_ARM_TENDONS_XML,
    build_spec,
)
from myo_sim.build.hand_data import DATA_DIR, load_intrinsic_table, place_tendon_nodes, write_generated_block

SCRIPT = "generate_extrinsic_muscles.py"
DATA_PATH = DATA_DIR / "extrinsic_muscles.json"
SIDE_SITES_JSON = DATA_DIR / "side_sites.json"
DEG = np.pi / 180.0
TRACTION_N = 14.7  # 1.5 kg (von Schroeder & Botte 1993)
K_MAX = 1.0e3  # N/m: above this the link force chatters at 2 ms when the neighbour flexes against it
REST_DEG = {"mcp": 56.0, "pm": 35.0, "md": 17.0}  # their resting posture (Fig. 2, Table I)


def load_table() -> dict[str, Any]:
    return json.loads(DATA_PATH.read_text())


def _number(value: float) -> str:
    return f"{float(value) + 0.0:.6g}"


# --- juncturae ------------------------------------------------------------------------------------------


def _path_after(tendon: str, start: str) -> list[str]:
    """Path elements (sites, wraps, pulleys) of *tendon* distal of its site *start*, as in the tendon file."""
    text = RIGHT_ARM_TENDONS_XML.read_text()
    body = re.search(rf'<spatial name="{re.escape(tendon)}_r_tendon"[^>]*>(.*?)</spatial>', text, re.S)
    if body is None:
        raise ValueError(f"{RIGHT_ARM_TENDONS_XML.name}: no tendon {tendon}_r_tendon")
    elements = re.findall(r"<(?:site|geom|pulley)\b[^>]*/>", re.sub(r"<!--.*?-->", "", body.group(1), flags=re.S))
    return elements[elements.index(f'<site site="{start}"/>') + 1 :]


def place_nodes(table: dict[str, Any], model: mujoco.MjModel | None = None) -> None:
    """Position and slide axis of each junctura node: on the EDC segment leaving ``segment_start``."""
    model = build_spec("myoarm_r").compile() if model is None else model
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for node in table["juncturae"]["nodes"].values():
        site = model.site(node["segment_start"])
        tendon = model.tendon(f"{node['tendon']}_r_tendon").id
        adr, num = data.ten_wrapadr[tendon], data.ten_wrapnum[tendon]
        points = data.wrap_xpos.reshape(-1, 3)[adr : adr + num]
        k = int(np.argmin(np.linalg.norm(points - data.site_xpos[site.id], axis=1)))
        a, b = points[k], points[k + 1]
        body = model.site_bodyid[site.id]
        rot, origin = data.xmat[body].reshape(3, 3), data.xpos[body]
        f = node["segment_fraction"]["value"]
        node["body"] = model.body(body).name
        node["pos"] = [round(float(v), 6) for v in rot.T @ (a + f * (b - a) - origin)]
        node["axis"] = [round(float(v), 6) for v in rot.T @ ((b - a) / np.linalg.norm(b - a))]


def _settings() -> dict[str, Any]:
    return load_intrinsic_table()["lumbrical_nodes"]


def chain_node_block(name: str, node: dict[str, Any]) -> str:
    """Generated chain block ``node <name>``: node body, slide and link site."""
    cfg = _settings()
    ind = " " * 52
    pos = " ".join(_number(v) for v in node["pos"])
    return (
        f"{ind}<!-- node_{name}_r: junctura node on {node['tendon']} (tendon equality node_{name}_tie_r) -->\n"
        f'{ind}<body name="node_{name}_r" pos="{pos}">\n'
        f'{ind}    <inertial pos="0 0 0" mass="{cfg["mass_kg"]["value"]:g}" diaginertia="1e-09 1e-09 1e-09"/>\n'
        f'{ind}    <joint name="node_{name}_r" type="slide" axis="{" ".join(_number(v) for v in node["axis"])}"'
        f' limited="false" stiffness="0" armature="{cfg["armature_kg"]["value"]:g}"'
        f' damping="{cfg["damping_Ns_per_m"]["value"]:g}"/>\n'
        f'{ind}    <site name="node_{name}_link_r" pos="0 0 0"/>\n'
        f"{ind}</body>\n"
    )


def tendon_block(table: dict[str, Any]) -> str:
    """Main generated block of the tendon file: node slides, EDC paths from P4 and the junctura links."""
    jt = table["juncturae"]
    lines = []
    for name, node in jt["nodes"].items():
        start = node["segment_start"]
        lines.append(f"        <!-- node_{name}_r: slide and {node['tendon']} from {start} (node_{name}_tie_r) -->")
        lines.append(f'        <fixed name="node_{name}_slide_r">')
        lines.append(f'            <joint joint="node_{name}_r" coef="1"/>')
        lines.append("        </fixed>")
        # From the EDC's own via point proximal of the node: the helper wraps the MCP as the tendon does.
        lines.append(f'        <spatial name="node_{name}_edc_r" rgba="0 0 0 0">')
        lines.append(f'            <site site="{start}"/>')
        lines += [f"            {e}" for e in _path_after(node["tendon"], node["segment_start"])]
        lines.append("        </spatial>")
    lines.append(
        "        <!-- Juncturae tendinum: tension-only, stiffness fitted to von Schroeder & Botte 1993 (extrinsic_muscles.json) -->"
    )
    for name, link in jt["links"].items():
        rest, k = link["rest_length_m"] + link["slack_m"]["value"], link["stiffness_N_per_m"]["value"]
        lines.append(f'        <spatial name="lig_{name}_r" springlength="0 {_number(rest)}" stiffness="{k:g}" width="0.001">')
        lines.append(f'            <site site="node_{link["from"]}_link_r"/>')
        lines.append(f'            <site site="node_{link["to"]}_link_r"/>')
        lines.append("        </spatial>")
    return "".join(f"{line}\n" for line in lines)


def equality_block(table: dict[str, Any]) -> str:
    solref = " ".join(_number(v) for v in _settings()["equality_solref"]["value"])
    return "".join(
        f'        <tendon name="node_{n}_tie_r" tendon1="node_{n}_slide_r" tendon2="node_{n}_edc_r"'
        f' polycoef="0 1 0 0 0" solref="{solref}"/>\n'
        for n in table["juncturae"]["nodes"]
    )


def link_rest_lengths(table: dict[str, Any]) -> None:
    """Rest length of each link: the longer of its lengths with straight fingers (q0) and in the relaxed resting
    posture of von Schroeder & Botte (nodes on their ties), so that neither posture loads it."""
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    lengths: dict[str, float] = {}
    for scale in (0.0, 1.0):
        data.qpos[:] = model.qpos0
        for d in (2, 3, 4, 5):
            for j, q in REST_DEG.items():
                data.qpos[model.joint(f"{j}{d}_flexion_r").qposadr[0]] = scale * q * DEG
        place_tendon_nodes(model, data)
        mujoco.mj_forward(model, data)
        for name in table["juncturae"]["links"]:
            lengths[name] = max(lengths.get(name, 0.0), float(data.ten_length[model.tendon(f"lig_{name}_r").id]))
    for name, link in table["juncturae"]["links"].items():
        link["rest_length_m"] = float(np.ceil(lengths[name] * 1e6) / 1e6)  # rounded up: never taut at either posture


# --- FCU and FDS5 ---------------------------------------------------------------------------------------


def chain_with_site(text: str, name: str, position: list[float]) -> str:
    """Rewrite the ``pos`` of the one chain site *name* (commented-out copies are left alone)."""
    comments = [m.span() for m in re.finditer(r"<!--.*?-->", text, re.S)]
    found = [
        m
        for m in re.finditer(rf'<site\b[^>]*\bname="{re.escape(name)}"[^>]*/>', text)
        if not any(a <= m.start() < b for a, b in comments)
    ]
    if len(found) != 1:
        raise ValueError(f"expected one site {name}")
    m = found[0]
    pos = " ".join(_number(v) for v in position)
    return text[: m.start()] + re.sub(r'\spos="[^"]*"', f' pos="{pos}"', m.group(0)) + text[m.end() :]


def _geom_tag(text: str, name: str) -> str:
    tags = re.findall(rf'<geom\b[^>]*\bname="{re.escape(name)}"[^>]*/>', text)
    if len(tags) != 1:
        raise ValueError(f"expected one geom {name}")
    return tags[0]


def wrap_sides(table: dict[str, Any]) -> None:
    """Side site of each tabled wrap: ``distance_radii`` radii from the geom's centre along ``direction_body``
    (the geom's tabled centre and radius where given, else the chain's)."""
    text = RIGHT_ARM_CHAIN_XML.read_text()
    for side in table["wrap_sides"]["muscles"].values():
        tag = _geom_tag(text, side["geom"])
        centre = side.get("geom_pos") or [float(v) for v in re.search(r'\spos="([^"]*)"', tag).group(1).split()]
        radius = side.get("radius_m") or float(re.search(r'\ssize="([^"]*)"', tag).group(1).split()[0])
        direction = np.asarray(side["direction_body"], float)
        pos = np.asarray(centre, float) + side["distance_radii"] * radius * direction / np.linalg.norm(direction)
        side["pos"] = [round(float(v), 6) for v in pos]


def chain_with_wrap_sides(text: str, table: dict[str, Any]) -> str:
    """Side-site positions and, where tabled, the geoms' centres and radii (other attributes kept)."""
    for side in table["wrap_sides"]["muscles"].values():
        text = chain_with_site(text, side["site"], side["pos"])
        if "geom_pos" in side:
            tag = _geom_tag(text, side["geom"])
            size = re.search(r'\ssize="([^"]*)"', tag).group(1).split()
            size[0] = _number(side["radius_m"])
            new = re.sub(r'\spos="[^"]*"', f' pos="{" ".join(_number(v) for v in side["geom_pos"])}"', tag)
            new = re.sub(r'\ssize="[^"]*"', f' size="{" ".join(size)}"', new)
            text = text.replace(tag, new)
    return text


def thumb_block(table: dict[str, Any]) -> str:
    """Generated chain block ``thumb trapezium``: APL's via site on the trapezium (fit in data/side_sites.json)."""
    sites = json.loads(SIDE_SITES_JSON.read_text())
    via = next(v for v in sites["vias"] if v["name"] == table["thumb_extrinsics"]["apl_via"]["site"])
    ind = " " * 52
    return (
        f"{ind}<!-- APL over the thumb CMC: via site on the trapezium, fitted to Smutz et al. 1998; side_sites.json -->\n"
        f'{ind}<site name="{via["name"]}" pos="{" ".join(_number(v) for v in via["pos"])}"/>\n'
    )


def fcu_block(table: dict[str, Any]) -> str:
    pos = " ".join(_number(v) for v in table["fcu"]["sites"]["FCU-P5_r"]["pos"])
    ind = " " * 52
    return (
        f"{ind}<!-- FCU over the hamate hook (pisohamate ligament) to the MC5 base; extrinsic_muscles.json -->\n"
        f'{ind}<site name="FCU-P5_r" pos="{pos}"/>\n'
    )


def muscles_with_revisions(text: str, table: dict[str, Any], fcu_length_q0: float) -> str:
    """FCU ``lengthrange`` shifted by its length change at q0; FDS5 ``force``."""
    fcu = table["fcu"]
    shift = fcu_length_q0 - fcu["length_q0_before_m"]
    lo, hi = (v + shift for v in fcu["lengthrange_before_m"])
    fcu["lengthrange_m"] = [round(lo, 9), round(hi, 9)]
    for name, attr, value in (
        ("FCU_r", "lengthrange", f"{lo!r} {hi!r}"),
        ("FDS5_r", "force", f"{table['fds5']['force_N']['value']:.4f}"),
    ):
        tags = re.findall(rf'<muscle name="{name}"[^>]*?/>', text)
        if len(tags) != 1:
            raise ValueError(f"expected one muscle {name}")
        tag = re.sub(rf'{attr}\s*=\s*"[^"]*"', f'{attr}="{value}"', tags[0])
        text = text.replace(tags[0], tag)
    return text


def fds5_force(table: dict[str, Any]) -> None:
    """FDS5's force: its PCSA times the specific tension of the forearm and hand (``specific_tension.json``)."""
    tension = json.loads((DATA_DIR / "specific_tension.json").read_text())
    area, sigma = tension["extrinsic_pcsa"]["FDS5"]["cm2"], tension["value_N_per_cm2"]["value"]
    table["fds5"]["force_N"] = {"value": round(area * sigma, 1), "derivation": f"{area} cm2 x {sigma} N/cm2"}


# --- MCP and PIP pulleys --------------------------------------------------------------------------------

# table key -> (name prefix of geoms and sites, block label, indentation of the block in the chain)
PULLEY_SETS = {"mcp_pulleys": ("mcp", "MCP", 52), "pip_pulleys": ("pip", "PIP", 56)}


def joint_frame(model: mujoco.MjModel, digit: dict[str, Any]) -> dict[str, np.ndarray]:
    """Flexion axis, centre and the distal / dorsal unit vectors normal to it, in the frame of the joint's parent body.

    The child body's frame is parallel to its parent's at q0, so the joint data carry over."""
    joint = model.joint(digit["joint"])
    child = model.body(digit["child"])
    if model.body(child.parentid[0]).name != digit["parent"] or not np.allclose(child.quat, [1, 0, 0, 0]):
        raise ValueError(f"{child.name}: expected a child of {digit['parent']} with its frame")
    axis = model.jnt_axis[joint.id] / np.linalg.norm(model.jnt_axis[joint.id])
    centre = child.pos + model.jnt_pos[joint.id]
    distal = np.array([0.0, -1.0, 0.0]) - axis * -axis[1]
    distal /= np.linalg.norm(distal)
    return {"centre": centre, "axis": axis, "distal": distal, "dorsal": np.cross(distal, axis)}


def _about_axis(frame: dict[str, np.ndarray], angle_deg: float, radius: float) -> np.ndarray:
    """Direction in the plane normal to the joint axis, *angle_deg* from distal toward dorsal, times *radius*."""
    a = angle_deg * DEG
    return radius * (np.cos(a) * frame["distal"] + np.sin(a) * frame["dorsal"])


def pulley_geometry(table: dict[str, Any], model: mujoco.MjModel | None = None) -> None:
    """Centre, axis and side site of each MCP and PIP pulley, and the moved extensor via sites (parent frame)."""
    model = build_spec("myoarm_r").compile() if model is None else model
    for key in PULLEY_SETS:
        cfg = table[key]
        for wrap in cfg["wraps"].values():
            frame = joint_frame(model, cfg["digits"][str(wrap["digit"])])
            radius = wrap["radius_m"]["value"]
            side = wrap["side_deg"]  # from distal toward dorsal about the joint axis
            wrap["geom_pos"] = [round(float(v), 6) for v in frame["centre"]]
            quat = np.zeros(4)
            mujoco.mju_quatZ2Vec(quat, frame["axis"])  # a quat, which the left-side mirroring handles (zaxis it does not)
            wrap["geom_quat"] = [round(float(v), 6) for v in quat]
            offset = _about_axis(frame, side, cfg["side_distance_radii"] * radius)
            wrap["side_pos"] = [round(float(v), 6) for v in frame["centre"] + offset]
        for via in cfg.get("vias", {}).values():
            frame = joint_frame(model, cfg["digits"][str(via["digit"])])
            rel = np.asarray(via["pos_before"], float) - frame["centre"]
            along = float(rel @ frame["axis"])
            pos = frame["centre"] + along * frame["axis"] + _about_axis(frame, via["angle_deg"], via["radius_m"])
            via["pos"] = [round(float(v), 6) for v in pos]


def pulley_block(table: dict[str, Any], key: str, digit: str) -> str:
    """Chain block ``MCP pulleys <digit>`` or ``PIP pulleys <digit>``: the cylinders on the joint axis and their side sites."""
    prefix, label, indent = PULLEY_SETS[key]
    cfg = table[key]
    ind = " " * indent
    lines = [f"{ind}<!-- {label} pulleys on the flexion axis (extrinsic_muscles.json, {key}) -->\n"]
    for muscle, wrap in cfg["wraps"].items():
        if str(wrap["digit"]) != digit:
            continue
        size = f"{_number(wrap['radius_m']['value'])} {_number(cfg['half_length_m'])}"
        lines.append(
            f'{ind}<geom name="{prefix}{digit}_{muscle}_wrap_r" type="cylinder" size="{size}"'
            f' pos="{" ".join(_number(v) for v in wrap["geom_pos"])}"'
            f' quat="{" ".join(_number(v) for v in wrap["geom_quat"])}" class="myoarm_wrap"/>\n'
        )
        side = " ".join(_number(v) for v in wrap["side_pos"])
        lines.append(f'{ind}<site name="{prefix}{digit}_{muscle}_side_r" pos="{side}"/>\n')
    return "".join(lines)


def tendons_with_pulleys(text: str, table: dict[str, Any]) -> str:
    """Flexor paths with each pulley after its ``after_site`` (the last site proximal of the joint); extensors are
    written by ``fit_extensor_mechanism.py`` from their ``main_path``."""
    for key, (prefix, _, _) in PULLEY_SETS.items():
        for muscle, wrap in table[key]["wraps"].items():
            if "after_site" not in wrap:
                continue
            found = re.search(rf'<spatial name="{re.escape(muscle)}_r_tendon"[^>]*>.*?</spatial>', text, re.S)
            if found is None:
                raise ValueError(f"no tendon {muscle}_r_tendon")
            name = f"{prefix}{wrap['digit']}_{muscle}"
            geom = f'<geom geom="{name}_wrap_r" sidesite="{name}_side_r"/>'
            body = re.sub(rf"\n\s*{re.escape(geom)}", "", found.group(0))
            site = f'<site site="{wrap["after_site"]}"/>'
            if body.count(site) != 1:
                raise ValueError(f"{muscle}: expected one {site}")
            indent = re.search(rf"\n(\s*){re.escape(site)}", body).group(1)
            body = body.replace(site, f"{site}\n{indent}{geom}")
            text = text[: found.start()] + body + text[found.end() :]
    return text


def write_xml(table: dict[str, Any]) -> None:
    for name, node in table["juncturae"]["nodes"].items():
        write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, chain_node_block(name, node), "", f"node {name}")
    write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, fcu_block(table), "", "FCU hamate")
    write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, ecu_block(table), "", "ECU groove")
    write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, thumb_block(table), "", "thumb trapezium")
    for key, (_, label, _) in PULLEY_SETS.items():
        for digit in table[key]["digits"]:
            block = pulley_block(table, key, digit)
            write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, block, "", f"{label} pulleys {digit}")
    chain = chain_with_site(RIGHT_ARM_CHAIN_XML.read_text(), "FCU-P4_r", table["fcu"]["sites"]["FCU-P4_r"]["pos"])
    chain = chain_with_wrap_sides(chain, table)
    for name, site in table["thumb_extrinsics"]["sites"].items():
        chain = chain_with_site(chain, name, site["pos"])
    for name, via in table["mcp_pulleys"]["vias"].items():
        chain = chain_with_site(chain, name, via["pos"])
    for wrap in table["pip_pulleys"]["wraps"].values():
        pts = wrap["pulley_points"]
        chain = chain_with_site(chain, pts["pp_site"], pts.get("pp_pos", pts["pp_before"]))
        chain = chain_with_site(chain, pts["mp_site"], pts.get("mp_pos", pts["mp_before"]))
    RIGHT_ARM_CHAIN_XML.write_text(chain)
    RIGHT_ARM_TENDONS_XML.write_text(tendons_with_pulleys(RIGHT_ARM_TENDONS_XML.read_text(), table))
    write_generated_block(RIGHT_ARM_ASSETS_XML, SCRIPT, equality_block(table), "", "juncturae ties")
    write_generated_block(RIGHT_ARM_TENDONS_XML, SCRIPT, tendon_block(table), "</tendon>")
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    length = float(data.ten_length[model.tendon("FCU_r_tendon").id])
    RIGHT_ARM_MUSCLES_XML.write_text(muscles_with_revisions(RIGHT_ARM_MUSCLES_XML.read_text(), table, length))


# --- fits -----------------------------------------------------------------------------------------------


def _arm(model: mujoco.MjModel, data: mujoco.MjData, tendon: int, joint: str, posture: dict[str, float]) -> float:
    lengths = []
    for h in (1e-5, -1e-5):
        data.qpos[:] = model.qpos0
        for name, q in posture.items():
            data.qpos[model.joint(name).qposadr[0]] = q
        data.qpos[model.joint(joint).qposadr[0]] += h
        mujoco.mj_fwdPosition(model, data)
        lengths.append(data.ten_length[tendon])
    return (lengths[0] - lengths[1]) / 2e-5 * 1e3


PIP_FIT_DEG = np.arange(-10.0, 120.1, 5.0)  # the PIP range and 5 deg past its flexion limit


def _pip_arms(model: mujoco.MjModel, data: mujoco.MjData, muscle: str, joint: str) -> np.ndarray:
    """PIP flexion moment arm (mm, + = flexion) of *muscle* over ``PIP_FIT_DEG``, other joints at 0."""
    tendon = model.tendon(f"{muscle}_r_tendon").id
    return np.array([-_arm(model, data, tendon, joint, {joint: q * DEG}) for q in PIP_FIT_DEG])


def fit_pip_pulleys(table: dict[str, Any]) -> None:
    """Place the pulley points next to each PIP cylinder so that the flexion arm stays continuous.

    A cylinder on the PIP axis can only hold a flexor's arm where the straight segment between its neighbouring
    sites cuts it: near extension. In flexion the middle-phalanx site swings toward the proximal-phalanx site,
    and past the angle where the two line up radially (180 deg minus their angles from the joint's long axis)
    the segment's line passes dorsal of the axis. Per tendon a grid search moves the proximal-phalanx site
    toward the previous one on its path (A2 pulley; fraction ``s``) and the middle-phalanx site dorsally and
    distally (A4 pulley), keeps the arm at least the cylinder's radius from ``floor_from_deg`` to 120 deg, and
    takes the smallest root-mean-square change of the arm over 0-90 deg (the old path matches the literature
    there)."""
    cfg = table["pip_pulleys"]
    grid = cfg["fit"]
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    mid = (PIP_FIT_DEG >= 0.0) & (PIP_FIT_DEG <= 90.0)
    deep = PIP_FIT_DEG >= grid["floor_from_deg"]
    for muscle, wrap in cfg["wraps"].items():
        digit = cfg["digits"][str(wrap["digit"])]
        frame = joint_frame(model, digit)
        pts = wrap["pulley_points"]
        ia, ib = model.site(pts["pp_site"]).id, model.site(pts["mp_site"]).id
        pa, pb = np.asarray(pts["pp_before"], float), np.asarray(pts["mp_before"], float)
        prev = pts.get("pp_previous")
        pp = np.asarray(model.site_pos[model.site(prev).id], float) if prev else pa
        geom = model.geom(f"pip{wrap['digit']}_{muscle}_wrap_r").id
        radius = float(model.geom_size[geom, 0])
        model.site_pos[ia], model.site_pos[ib] = pa, pb
        model.geom_size[geom, 0] = 0.0  # no wrap: the path before the change
        before = _pip_arms(model, data, muscle, digit["joint"])
        model.geom_size[geom, 0] = radius
        best = None
        for s in np.arange(0.0, grid["s_max"] + 1e-9, grid["s_step"]) if prev else [0.0]:
            for dv in np.arange(0.0, grid["dorsal_max_mm"] + 1e-9, grid["dorsal_step_mm"]):
                for dl in np.arange(0.0, grid["distal_max_mm"] + 1e-9, grid["distal_step_mm"]):
                    model.site_pos[ia] = pa + s * (pp - pa)
                    model.site_pos[ib] = pb + (dv * frame["dorsal"] + dl * frame["distal"]) / 1000.0
                    arms = _pip_arms(model, data, muscle, digit["joint"])
                    rms = float(np.sqrt(np.mean((arms[mid] - before[mid]) ** 2)))
                    # key: the floor held (or the closest miss), then the smallest change
                    key = (min(0.0, arms[deep].min() - radius * 1e3), -rms)
                    if best is None or key > best[0]:
                        best = (key, rms, s, dv, dl, arms, model.site_pos[ia].copy(), model.site_pos[ib].copy())
        model.site_pos[ia], model.site_pos[ib] = pa, pb
        _, rms, s, dv, dl, arms, new_a, new_b = best
        pts["pp_pos"] = [round(float(v), 6) for v in new_a]
        pts["mp_pos"] = [round(float(v), 6) for v in new_b]
        wrap["fit"] = {
            "pp_toward_previous": round(float(s), 3),
            "mp_dorsal_mm": round(float(dv), 2),
            "mp_distal_mm": round(float(dl), 2),
            "rms_change_0_90_mm": round(rms, 2),
            "angles_deg": [float(q) for q in PIP_FIT_DEG],
            "arm_before_mm": [round(float(v), 2) for v in before],
            "arm_after_mm": [round(float(v), 2) for v in arms],
        }
        print(f"{muscle}: s {s:.2f} dorsal {dv:.1f} distal {dl:.1f} mm, rms 0-90 {rms:.2f} mm, min {arms[deep].min():.2f}")


def fcu_arms(model: mujoco.MjModel, posture: dict[str, float] | None = None) -> tuple[float, float]:
    """FCU wrist flexion and ulnar-deviation moment arms (mm, positive = flexor / ulnar deviator)."""
    data = mujoco.MjData(model)
    t = model.tendon("FCU_r_tendon").id
    posture = posture or {}
    return -_arm(model, data, t, "flexion_r", posture), _arm(model, data, t, "deviation_r", posture)


def fit_fcu(table: dict[str, Any]) -> None:
    """Pisiform via point within the pisiform's bounding box, fitted to Horii 1993's neutral arms."""
    from scipy.optimize import least_squares

    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    g = model.geom("pisiform_r")
    mesh = g.dataid[0]
    vert = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    world = data.geom_xpos[g.id] + vert @ data.geom_xmat[g.id].reshape(3, 3).T
    body = model.body("pisiform_r").id
    local = (world - data.xpos[body]) @ data.xmat[body].reshape(3, 3)
    lo, hi = local.min(0), local.max(0)
    fcu = table["fcu"]
    site = model.site("FCU-P4_r").id
    start = np.asarray(fcu["sites"]["FCU-P4_r"]["start"])
    t = fcu["targets"]

    def res(x: np.ndarray) -> np.ndarray:
        model.site_pos[site] = x
        flex, uln = fcu_arms(model)
        return np.array(
            [(flex - t["flexion_mm"]["mean"]) / t["flexion_mm"]["sd"], (uln - t["ulnar_mm"]["mean"]) / t["ulnar_mm"]["sd"]]
            + list((x - start) / 0.005)
        )

    model.site_pos[site] = start
    before = fcu_arms(model)
    r = least_squares(res, np.clip(start, lo, hi), bounds=(lo, hi), diff_step=1e-3)
    fcu["sites"]["FCU-P4_r"]["pos"] = [round(float(v), 6) for v in r.x]
    after = fcu_arms(model)
    fcu["fit"] = {
        "bounds": "pisiform mesh bounding box in its body frame",
        "arms_before_mm": [round(v, 2) for v in before],
        "arms_after_mm": [round(v, 2) for v in after],
        "site_shift_mm": round(float(np.linalg.norm(r.x - start) * 1e3), 2),
    }


# --- ECU and the forearm rotation axis -----------------------------------------------------------------


def _mesh_in_body(model: mujoco.MjModel, data: mujoco.MjData, geom: str, body: str) -> np.ndarray:
    """Vertices of the mesh geom *geom* in the frame of *body* (current pose)."""
    g = model.geom(geom)
    mesh = g.dataid[0]
    vert = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    world = data.geom_xpos[g.id] + vert @ data.geom_xmat[g.id].reshape(3, 3).T
    b = model.body(body).id
    return (world - data.xpos[b]) @ data.xmat[b].reshape(3, 3)


def forearm_frame(model: mujoco.MjModel) -> dict[str, np.ndarray]:
    """``pro_sup_r`` at q0 in the ulna frame: a point, the axis (proximal) and the ulna's posterior and lateral
    directions perpendicular to it (the frame's -x and +z)."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    b = model.body("ulna_r").id
    rot, origin = data.xmat[b].reshape(3, 3), data.xpos[b]
    j = model.joint("pro_sup_r").id
    axis = rot.T @ data.xaxis[j]
    axis = axis if axis[1] > 0 else -axis
    lateral = np.array([0.0, 0.0, 1.0]) - axis[2] * axis
    lateral /= np.linalg.norm(lateral)
    return {"point": rot.T @ (data.xanchor[j] - origin), "axis": axis, "posterior": np.cross(lateral, axis), "lateral": lateral}


def ulnar_head(model: mujoco.MjModel, table: dict[str, Any]) -> tuple[np.ndarray, float, float]:
    """Sphere fitted (least squares) to the ulna mesh vertices of the ulnar head: centre (ulna frame), radius, rms."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    vert = _mesh_in_body(model, data, "ulna_r", "ulna_r")
    head = vert[vert[:, 1] < vert[:, 1].min() + table["ecu"]["ulnar_head"]["length_m"]]
    a = np.c_[2.0 * head, np.ones(len(head))]
    c = np.linalg.lstsq(a, (head**2).sum(1), rcond=None)[0]
    radius = float(np.sqrt(c[3] + c[:3] @ c[:3]))
    rms = float(np.sqrt(np.mean((np.linalg.norm(head - c[:3], axis=1) - radius) ** 2)))
    return c[:3], radius, rms


def radial_head_centre(model: mujoco.MjModel, table: dict[str, Any]) -> tuple[np.ndarray, float]:
    """Centre (ulna frame, at q0) of the circle fitted to the radial head rim: the radius mesh vertices within
    ``rim_m`` of its proximal end, projected on the plane normal to the rotation axis; and the circle's radius."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    frame = forearm_frame(model)
    vert = _mesh_in_body(model, data, "radius_r", "ulna_r")
    rim = vert[vert[:, 1] > vert[:, 1].max() - table["ecu"]["radial_head"]["rim_m"]]
    e1, e2 = frame["lateral"], frame["posterior"]
    q = rim - frame["point"]
    xy = np.c_[q @ e1, q @ e2]
    c = np.linalg.lstsq(np.c_[2.0 * xy, np.ones(len(xy))], (xy**2).sum(1), rcond=None)[0]
    centre = frame["point"] + c[0] * e1 + c[1] * e2 + float(np.mean(q @ frame["axis"])) * frame["axis"]
    return centre, float(np.sqrt(c[2] + c[0] ** 2 + c[1] ** 2))


def axis_offsets(model: mujoco.MjModel, table: dict[str, Any]) -> dict[str, float]:
    """Distance (mm) of the rotation axis from the radial head centre, and its offset from the ulnar head
    centre: posterior (dorsal) and lateral components and the distance."""
    frame = forearm_frame(model)

    def perp(p: np.ndarray) -> np.ndarray:
        q = frame["point"] - p
        return q - (q @ frame["axis"]) * frame["axis"]  # from p to the axis

    to_axis = perp(ulnar_head(model, table)[0])
    return {
        "radial_head_mm": float(np.linalg.norm(perp(radial_head_centre(model, table)[0])) * 1e3),
        "ulnar_head_posterior_mm": float(to_axis @ frame["posterior"] * 1e3),
        "ulnar_head_lateral_mm": float(to_axis @ frame["lateral"] * 1e3),
        "ulnar_head_mm": float(np.linalg.norm(to_axis) * 1e3),
    }


def ecu_arms(model: mujoco.MjModel, posture: dict[str, float] | None = None) -> tuple[float, float]:
    """ECU wrist extension and ulnar-deviation moment arms (mm, positive = extensor / ulnar deviator)."""
    data = mujoco.MjData(model)
    t = model.tendon("ECU_r_tendon").id
    posture = posture or {}
    return _arm(model, data, t, "flexion_r", posture), _arm(model, data, t, "deviation_r", posture)


def ecu_rotation_arms(model: mujoco.MjModel, table: dict[str, Any]) -> tuple[list[float], list[float]]:
    """ECU extension and ulnar arms at wrist neutral in each forearm rotation of the table (supination first)."""
    arms = [ecu_arms(model, {"pro_sup_r": np.radians(deg)}) for deg in table["ecu"]["rotations_deg"]]
    return [a[0] for a in arms], [a[1] for a in arms]


def ecu_residuals(model: mujoco.MjModel, table: dict[str, Any]) -> np.ndarray:
    """(model - target) / sd for every source and forearm rotation, extension then ulnar deviation."""
    targets = table["ecu"]["targets"]
    out = []
    for key, values in zip(("extension_mm", "ulnar_mm"), ecu_rotation_arms(model, table)):
        for source in targets[key]["values"].values():
            out += list((np.asarray(values) - source["mean"]) / targets[key]["sd"])
    return np.asarray(out)


def ecu_block(table: dict[str, Any]) -> str:
    pos = " ".join(_number(v) for v in table["ecu"]["sites"]["ECU-P5_r"]["pos"])
    ind = " " * 36
    return (
        f"{ind}<!-- ECU in the groove of the ulnar head (its subsheath); extrinsic_muscles.json -->\n"
        f'{ind}<site name="ECU-P5_r" pos="{pos}"/>\n'
    )


def fit_ecu(table: dict[str, Any]) -> None:
    """Groove via point on the ulnar head's posterior to lateral surface, just outside the sphere fitted to the
    head, fitted to Horii 1993's and Loren 1996's extension and ulnar arms in supination, neutral and pronation."""
    from scipy.optimize import least_squares

    ecu = table["ecu"]
    model = build_spec("myoarm_r").compile()
    frame = forearm_frame(model)
    centre, radius, rms = ulnar_head(model, table)
    ecu["ulnar_head"] |= {
        "centre_m": [round(float(v), 6) for v in centre],
        "radius_m": round(radius, 6),
        "rms_m": round(rms, 6),
    }
    bounds = ecu["fit"]["bounds"]
    site = model.site("ECU-P5_r").id

    def point(x: np.ndarray) -> np.ndarray:
        alpha, theta, depth = np.radians(x[0]), np.radians(x[1]), x[2]
        side = np.cos(alpha) * frame["posterior"] + np.sin(alpha) * frame["lateral"]
        return centre + (radius + depth) * (np.cos(theta) * frame["axis"] + np.sin(theta) * side)

    def res(x: np.ndarray) -> np.ndarray:
        model.site_pos[site] = point(x)
        return ecu_residuals(model, table)

    lo = [bounds["posterior_to_lateral_deg"][0], bounds["from_proximal_deg"][0], bounds["outside_head_m"][0]]
    hi = [bounds["posterior_to_lateral_deg"][1], bounds["from_proximal_deg"][1], bounds["outside_head_m"][1]]
    starts = ([20.0, 90.0, 0.001], [45.0, 120.0, 0.001], [30.0, 145.0, 0.002])
    best = min(
        (least_squares(res, x0, bounds=(lo, hi), x_scale=[10.0, 10.0, 0.001], diff_step=1e-3) for x0 in starts),
        key=lambda r: r.cost,
    )
    model.site_pos[site] = point(best.x)
    after = ecu_rotation_arms(model, table)
    ecu["sites"]["ECU-P5_r"]["pos"] = [round(float(v), 6) for v in point(best.x)]
    ecu["fit"]["result"] = {
        "posterior_to_lateral_deg": round(float(best.x[0]), 1),
        "from_proximal_deg": round(float(best.x[1]), 1),
        "outside_head_m": round(float(best.x[2]), 6),
        "extension_mm": [round(v, 2) for v in after[0]],
        "ulnar_mm": [round(v, 2) for v in after[1]],
        "rms_z": round(float(np.sqrt(np.mean(ecu_residuals(model, table) ** 2))), 2),
    }


def _hand_scene() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """myohand_r with wrist, forearm and thumb held and gravity toward the palm."""
    import myo_sim

    model = myo_sim.load_spec("myohand_r").compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    held = [j for j in range(model.njnt) if model.joint(j).name.split("_r")[0] in ("flexion", "deviation", "pro_sup")]
    held += [model.joint(n).id for n in ("cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r", "ip_flexion_r")]
    for j in held:
        model.dof_armature[model.jnt_dofadr[j]] = model.dof_damping[model.jnt_dofadr[j]] = 1e4
    mc = model.body("thirdmc_r").id
    dorsal = data.site_xpos[model.site("EDC3-P4_r").id] - data.xpos[mc]
    axis = data.xmat[mc].reshape(3, 3) @ np.array([0.0, -1.0, 0.0])  # along the metacarpal (distal)
    dorsal -= (dorsal @ axis) * axis
    model.opt.gravity[:] = -9.81 * dorsal / np.linalg.norm(dorsal)
    return model, data


def traction_transfer(stiffness: dict[str, float] | None, tendon: str, model_data: Any = None) -> dict[str, float]:
    """Net MP extension (deg) of every finger under 14.7 N on *tendon*, links at *stiffness* (None: off)."""
    model, data = model_data or _hand_scene()
    for name in ("JT34", "JT45"):
        t = model.tendon(f"lig_{name}_r").id
        model.tendon_stiffness[t] = 0.0 if stiffness is None else stiffness[name]
    mujoco.mj_resetData(model, data)
    for d in (2, 3, 4, 5):
        for j, q in REST_DEG.items():
            data.qpos[model.joint(f"{j}{d}_flexion_r").qposadr[0]] = q * DEG
    place_tendon_nodes(model, data)
    tid = model.tendon(f"{tendon}_r_tendon").id
    jac = np.zeros((model.ntendon, model.nv))

    def run(seconds: float, force: float) -> None:
        for _ in range(int(seconds / model.opt.timestep)):
            data.qfrc_applied[:] = 0.0
            if force:
                mujoco.mj_tendon(model, data)
                mujoco.mju_sparse2dense(jac, data.ten_J, model.ten_J_rownnz, model.ten_J_rowadr, model.ten_J_colind)
                data.qfrc_applied[:] = -force * jac[tid]
            mujoco.mj_step(model, data)

    run(2.0, 0.0)
    rest = {d: data.qpos[model.joint(f"mcp{d}_flexion_r").qposadr[0]] / DEG for d in (2, 3, 4, 5)}
    run(1.5, TRACTION_N)
    return {f"mcp{d}_flexion": float(rest[d] - data.qpos[model.joint(f"mcp{d}_flexion_r").qposadr[0]] / DEG) for d in rest}


def fit_juncturae(table: dict[str, Any]) -> None:
    """Link stiffness (log-scale least squares) against the juncturae share of the neighbour MP extension."""
    from scipy.optimize import least_squares

    jt = table["juncturae"]
    targets = jt["fit"]["targets"]
    scene = _hand_scene()
    base = {k: traction_transfer(None, k.split("_")[0], scene) for k in targets}

    def model_values(log_k: np.ndarray) -> dict[str, dict[str, float]]:
        k = {"JT34": float(np.exp(log_k[0])), "JT45": float(np.exp(log_k[1]))}
        return {key: traction_transfer(k, key.split("_")[0], scene) for key in targets}

    def res(log_k: np.ndarray) -> np.ndarray:
        values = model_values(log_k)
        return np.array(
            [values[key][j] - base[key][j] - target for key, tg in targets.items() for j, target in tg.items() if j != "where"]
        )

    r = least_squares(res, np.log([300.0, 300.0]), bounds=(np.log(10.0), np.log(K_MAX)), diff_step=0.05)
    values = model_values(r.x)
    for name, value in zip(("JT34", "JT45"), np.exp(r.x)):
        jt["links"][name]["stiffness_N_per_m"] = {"value": float(f"{value:.4g}"), "method": "fit (juncturae.fit)"}
    jt["fit"]["result"] = {
        "without_links_deg": {k: {j: round(v, 2) for j, v in b.items()} for k, b in base.items()},
        "with_links_deg": {k: {j: round(v, 2) for j, v in b.items()} for k, b in values.items()},
        "residual_deg": [round(float(v), 2) for v in res(r.x)],
        "rms_deg": round(float(np.sqrt(np.mean(res(r.x) ** 2))), 2),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="refit the FCU and ECU sites and the junctura stiffness first")
    parser.add_argument("--fit-ecu", action="store_true", help="refit only the ECU groove site first")
    parser.add_argument("--fit-pip", action="store_true", help="refit only the PIP pulley points first")
    args = parser.parse_args()
    table = load_table()
    fds5_force(table)
    wrap_sides(table)
    pulley_geometry(table)
    if args.fit:
        for link in table["juncturae"]["links"].values():
            link.setdefault("stiffness_N_per_m", {"value": 300.0, "method": "start"})
            link.setdefault("rest_length_m", 0.01)
        write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, fcu_block(table), "", "FCU hamate")
        place_nodes(table)  # on the EDC paths, which the nodes do not change
        write_xml(table)
        link_rest_lengths(table)
        write_xml(table)
        fit_fcu(table)
        write_xml(table)
        fit_ecu(table)
        write_xml(table)
        fit_juncturae(table)
    elif args.fit_ecu:
        fit_ecu(table)
    if args.fit_pip:
        write_xml(table)  # the cylinders must be in the model
        fit_pip_pulleys(table)
    DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    write_xml(table)
    print(json.dumps({k: table["juncturae"]["links"][k] for k in table["juncturae"]["links"]}, indent=1))
    print(json.dumps(table["fcu"].get("fit"), indent=1), table["fds5"]["force_N"])
    print(json.dumps(table["ecu"]["fit"].get("result"), indent=1))


if __name__ == "__main__":
    main()
