"""Write the hand intrinsic muscles of ``build/data/hand_intrinsic_muscles.json`` into the right-arm XML.

Each table row is one muscle (or muscle head): its sites, architecture and the moment arms it is fitted
to. This script writes

- the site positions (``pos`` of ``<name>-P<i>_r``) in ``myoarm_r_chain.xml``; the sites themselves are
  hand-written in their bodies;
- straight tendons through those sites in ``myoarm_r_tendon.xml`` and muscles in ``myoarm_r_muscle.xml``:
  rows without a ``block`` go to the script's main block, a row with a ``block`` label to its own block,
  which stands where the hand-written element it replaced stood (so actuator indices stay).

A row with an ``insertion`` (the interossei and lumbricals of digits 2-5) is a branched tendon: its first
``split`` sites are the main path; from the last of them an optional bony branch (fraction 1 - f) and the
hood branch (fraction f, the row's hood fraction) continue, the hood branch to a band junction of the
extensor mechanism (``EM{d}_RB_r`` / ``EM{d}_UB_r``, ``fit_extensor_mechanism.py``). From the junction the
central-slip and lateral-band branches of ``build/data/extensor_mechanism.json`` follow, so each divisor is
1 / (f x branch fraction) and this script stays the only writer of these tendons.

The length range is a rigid tendon of slack length ``(L_MT(q = 0) - L_fo cos(alpha)) / 1.033`` (Barry et
al. 2018, Eq. 1), so the muscle class's normalised fibre-length range spans the same range of ``L_fo``; for a
branched tendon L_MT is MuJoCo's tendon length (branches divided by their divisors).

``--fit`` first refits the sites to the rows' target moment arms (``path_fit`` in the table; ``fit.reference``
targets are only reported), then the
passive-force scale of the muscles that cross the thumb to its measured passive moments (``passive_fit``),
and records both in the table. Rows of one ``fit.group`` are fitted together: the first row's targets apply to
the force-weighted mean of the group's moment arms, the other rows' targets to that row alone.

The lumbricals arise from junction nodes on their FDP tendons (``lumbrical_nodes`` in the table): per
lumbrical a light body on a slide joint ``node_<name>_r`` in the metacarpal (labelled chain block
``node <name>``), the lumbrical's first site on it, and a tendon equality ``node_<name>_tie_r`` (assets
file) that ties the slide to the FDP path distal of the node: s = L_distal(q) - L_distal(q0), with
``node_<name>_slide_r`` (the slide) and ``node_<name>_fdp_r`` (the FDP's own path from the node's rest
point) in the main tendon block. ``--place-nodes`` recomputes the node positions and slide axes from the
compiled model (the lumbrical origin projected on the FDP path, which runs along the slide).

Run: ``uv run python scripts/generate_hand_intrinsic_muscles.py [--place-nodes] [--fit]``.
"""

from __future__ import annotations

import argparse
import json
import re
from functools import cache
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
from myo_sim.build.hand_data import (
    DATA_DIR,
    INTRINSIC_DATA_PATH,
    fold_node_forces,
    load_intrinsic_table,
    place_tendon_nodes,
    write_generated_block,
)

SCRIPT = "generate_hand_intrinsic_muscles.py"
MUSCLE_CLASS = "myoarm_forearm_muscle"
DEG = np.pi / 180.0
H = 1e-5  # rad, central differences of the tendon length
PENALTY = 1000.0  # residual per unit of normalised fibre length outside the window
# Anatomical offsets (deg; flexion, abduction positive) spanning the measurement box of Smutz et al. 1998.
THUMB_BOX_DEG = {"cmc_flexion_r": (20.0, 0.0, -25.0), "cmc_abduction_r": (20.0, 0.0, -20.0), "mp_flexion_r": (-10.0, 0.0, 40.0)}
INDEX_BOX_DEG = {"mcp2_flexion_r": (0.0, 45.0, 90.0), "mcp2_abduction_r": (-15.0, 0.0, 15.0)}
FINGER_BOX_DEG = {f"mcp{d}_{j}_r": INDEX_BOX_DEG[f"mcp2_{j}_r"] for d in (3, 4) for j in ("flexion", "abduction")} | {
    "cmc4_flexion_r": (-5.0, 0.0, 16.0)
}
LITTLE_BOX_DEG = {
    "mcp5_flexion_r": (0.0, 45.0, 90.0),
    "mcp5_abduction_r": (-15.0, 0.0, 15.0),
    "cmc5_flexion_r": (-10.3, 0.0, 33.0),
}
PROXIMAL_OF_MC5 = {"pisiform_r", "hamate_r", "capitate_r", "triquetrum_r", "lunate_r"}
PROXIMAL_OF_MC4 = PROXIMAL_OF_MC5 | {"thirdmc_r", "secondmc_r", "trapezoid_r"}
EM_DATA_PATH = DATA_DIR / "extensor_mechanism.json"


def _cite(table: dict[str, Any], key: str) -> str:
    citation = table["sources"][key]["citation"]
    year = "".join(c for c in key if c.isdigit())
    return f"{citation.split()[0]} et al. {year}"


def site_name(row: dict[str, Any], index: int) -> str:
    """Right-side name of site *index* (0-based) of *row*."""
    return f"{row['name']}-P{index + 1}_r"


def rows_in_block(table: dict[str, Any], label: str | None) -> list[dict[str, Any]]:
    return [row for row in table["muscles"] if row.get("block") == label]


def block_labels(table: dict[str, Any]) -> list[str | None]:
    """The main block (None) and every labelled block, in table order."""
    return [None] + list(dict.fromkeys(row["block"] for row in table["muscles"] if row.get("block")))


# --- branched rows (interossei and lumbricals) ---------------------------------------------------------


@cache
def _em_script() -> Any:
    import importlib.util
    from pathlib import Path

    path = Path(__file__).with_name("fit_extensor_mechanism.py")
    spec = importlib.util.spec_from_file_location("fit_extensor_mechanism", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def em_table() -> dict[str, Any]:
    return json.loads(EM_DATA_PATH.read_text())


def junction(row: dict[str, Any]) -> str | None:
    """Band junction site of a branched row (``None`` for a straight row)."""
    if "insertion" not in row:
        return None
    hood = row["insertion"]["hood"]
    return f"EM{hood['digit']}_{hood['junction']}_r"


def hood_fraction(row: dict[str, Any]) -> float:
    return float(row["insertion"]["hood"]["fraction"]["value"])


def band_fractions(row: dict[str, Any], em: dict[str, Any]) -> dict[str, float]:
    """Fractions of the muscle force on the central-slip and band branches from the junction."""
    kind = "lumbrical" if row["group"] == "lumbrical" else "interosseous"
    per = _em_script().intrinsic_fractions(em, kind, row["insertion"]["hood"]["junction"])
    return {branch: hood_fraction(row) * f for branch, f in per.items()}


def legs(row: dict[str, Any], points: list[Any], fixed: Any) -> Any:
    """Weighted straight legs of *row* from its site *points* and the junction point *fixed* (proximal of the
    junction only: the branches from the junction cross the interphalangeal joints, not MCP or CMC)."""
    ins = row["insertion"]
    split = ins["split"]
    main, last = points[:split], points[split - 1]
    out = [(1.0, main)]
    f = hood_fraction(row)
    if "bony" in ins:
        out.append((1.0 - f, [last] + [points[i] for i in ins["bony"]["sites"]]))
    out.append((f, [last] + [points[i] for i in ins["hood"]["sites"]] + [fixed]))
    return out


# --- XML ------------------------------------------------------------------------------------------------


def _number(value: float) -> str:
    return f"{float(value) + 0.0:.6g}"


def chain_with_sites(text: str, table: dict[str, Any]) -> str:
    """*text* of ``myoarm_r_chain.xml`` with each tabled site at its ``pos``, rest unchanged."""
    for row in table["muscles"]:
        for i, site in enumerate(row["sites"]):
            name = site_name(row, i)
            tags = re.findall(rf'<site\b[^>]*\bname="{re.escape(name)}"[^>]*/>', text)
            if len(tags) != 1:
                raise ValueError(f"{RIGHT_ARM_CHAIN_XML.name}: expected one site {name}, found {len(tags)}")
            pos = " ".join(_number(v) for v in site["pos"])
            text = text.replace(tags[0], re.sub(r'\spos="[^"]*"', f' pos="{pos}"', tags[0]))
    return text


def tendon_block(table: dict[str, Any], label: str | None = None) -> str:
    """Generated block *label* of ``myoarm_r_tendon.xml``: one straight spatial tendon per row."""
    lines = []
    for row in rows_in_block(table, label):
        starts = sorted({_cite(table, s["start"]["source"]) for s in row["sites"] if s["start"]["source"] in table["sources"]})
        group = [r for r in table["muscles"] if r["fit"]["group"] == row["fit"]["group"]]
        targets = sorted({_cite(table, t["source"]) for r in group for t in r["fit"].get("targets", {}).values()})
        fitted = f", fitted to {' and '.join(targets)}" if targets else ""
        lines.append(f"        <!-- {row['name']} ({row['anatomy']}): sites from {' and '.join(starts)}{fitted} -->")
        lines.append(f'        <spatial name="{row["name"]}_r_tendon" class="{MUSCLE_CLASS}">')
        if "insertion" in row:
            lines += [f"            {line}" for line in branched_path(row)]
        else:
            lines += [f'            <site site="{site_name(row, i)}"/>' for i in range(len(row["sites"]))]
        lines.append("        </spatial>")
    if label is None:
        lines += node_tendons(table)
    return "".join(f"{line}\n" for line in lines)


def branched_path(row: dict[str, Any]) -> list[str]:
    """Path elements of a branched row: main path, bony and hood branches, then the junction's branches."""
    ins, em = row["insertion"], em_table()
    site = [f'<site site="{site_name(row, i)}"/>' for i in range(len(row["sites"]))]
    split, f = ins["split"], hood_fraction(row)
    out = site[:split]
    if "bony" in ins:
        out += [f'<pulley divisor="{_number(1.0 / (1.0 - f))}"/>', site[split - 1]] + [site[i] for i in ins["bony"]["sites"]]
    if "bony" in ins or f < 1.0:
        out += [f'<pulley divisor="{_number(1.0 / f)}"/>', site[split - 1]]
    out += [site[i] for i in ins["hood"]["sites"]] + [f'<site site="{junction(row)}"/>']
    out += _em_script().junction_branches(em, ins["hood"]["digit"], junction(row), band_fractions(row, em))
    return out


# --- lumbrical junction nodes ----------------------------------------------------------------------------


def node_rows(table: dict[str, Any]) -> list[dict[str, Any]]:
    """Rows whose first site sits on a junction node (the lumbricals)."""
    return [r for r in table["muscles"] if r["name"] in table.get("lumbrical_nodes", {}).get("nodes", {})]


def node_name(row: dict[str, Any]) -> str:
    """Body and slide joint of the node of *row*."""
    return f"node_{row['name']}_r"


def _fdp_path_after(fdp: str, start: str) -> list[str]:
    """Path elements of tendon *fdp* distal of its site *start*, as written in ``myoarm_r_tendon.xml``."""
    text = RIGHT_ARM_TENDONS_XML.read_text()
    body = re.search(rf'<spatial name="{re.escape(fdp)}_r_tendon"[^>]*>(.*?)</spatial>', text, re.S)
    if body is None:
        raise ValueError(f"{RIGHT_ARM_TENDONS_XML.name}: no tendon {fdp}_r_tendon")
    elements = re.findall(r"<(?:site|geom)\b[^>]*/>", re.sub(r"<!--.*?-->", "", body.group(1), flags=re.S))
    index = elements.index(f'<site site="{start}"/>')
    return elements[index + 1 :]


def place_nodes(table: dict[str, Any]) -> None:
    """Node position and slide axis of every lumbrical, and its first site moved onto the node.

    On the compiled model at q0: the lumbrical origin is projected on the FDP path segment that leaves the
    node's ``segment_start`` site (to the next path point: a site or a wrap tangent point); the node sits
    at that point in the start site's body and slides along the segment (distal positive). The origin
    keeps its position: its site is re-expressed relative to the node (the node body is not rotated).
    """
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cfg = table["lumbrical_nodes"]
    for row in node_rows(table):
        node = cfg["nodes"][row["name"]]
        site = model.site(node["segment_start"])
        tendon = model.tendon(f"{node['fdp']}_r_tendon").id
        adr, num = data.ten_wrapadr[tendon], data.ten_wrapnum[tendon]
        points = data.wrap_xpos.reshape(-1, 3)[adr : adr + num]
        k = int(np.argmin(np.linalg.norm(points - data.site_xpos[site.id], axis=1)))
        a, b = points[k], points[k + 1]
        body = model.site_bodyid[site.id]
        rot, origin = data.xmat[body].reshape(3, 3), data.xpos[body]
        first = row["sites"][0]
        if first["body"] == node_name(row):  # already on the node: its old position in the parent frame
            local = np.add(node["pos"], first["pos"])
            start = np.add(node["pos"], first["start"]["pos"])
        else:
            local, start = np.asarray(first["pos"]), np.asarray(first["start"]["pos"])
        world = origin + rot @ local
        t = float(np.clip((world - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0))
        pos = rot.T @ (a + t * (b - a) - origin)
        axis = rot.T @ ((b - a) / np.linalg.norm(b - a))
        node.update(
            body=model.body(body).name,
            pos=[round(float(v), 6) for v in pos],
            axis=[round(float(v), 6) for v in axis],
            segment_fraction=round(t, 3),
            origin_offset_mm=round(float(np.linalg.norm(local - pos) * 1e3), 2),
        )
        first["body"] = node_name(row)
        first["pos"] = [round(float(v), 6) for v in local - pos]
        first["start"]["pos"] = [round(float(v), 6) for v in start - pos]


def chain_node_block(table: dict[str, Any], row: dict[str, Any]) -> str:
    """Generated chain block ``node <name>``: the node body with its slide and the lumbrical's first site."""
    cfg = table["lumbrical_nodes"]
    node = cfg["nodes"][row["name"]]
    ind = " " * 52
    pos = " ".join(_number(v) for v in node["pos"])
    first = " ".join(_number(v) for v in row["sites"][0]["pos"])
    return (
        f"{ind}<!-- {node_name(row)}: {row['name']} origin on {node['fdp']} (tendon equality {node_name(row)[:-2]}_tie_r) -->\n"
        f'{ind}<site name="{node_name(row)[:-2]}_ref_r" pos="{pos}"/>\n'
        f'{ind}<body name="{node_name(row)}" pos="{pos}">\n'
        f'{ind}    <inertial pos="0 0 0" mass="{cfg["mass_kg"]["value"]:g}" diaginertia="1e-09 1e-09 1e-09"/>\n'
        f'{ind}    <joint name="{node_name(row)}" type="slide" axis="{" ".join(_number(v) for v in node["axis"])}"'
        f' limited="false" stiffness="0" armature="{cfg["armature_kg"]["value"]:g}"'
        f' damping="{cfg["damping_Ns_per_m"]["value"]:g}"/>\n'
        f'{ind}    <site name="{site_name(row, 0)}" pos="{first}"/>\n'
        f"{ind}</body>\n"
    )


def node_tendons(table: dict[str, Any]) -> list[str]:
    """Per node the fixed tendon of its slide and the FDP path distal of the node's rest point."""
    lines = []
    for row in node_rows(table):
        node = table["lumbrical_nodes"]["nodes"][row["name"]]
        base = node_name(row)[:-2]
        lines.append(
            f"        <!-- {node_name(row)}: slide and {node['fdp']} distal of the node (tendon equality {base}_tie_r) -->"
        )
        lines.append(f'        <fixed name="{base}_slide_r">')
        lines.append(f'            <joint joint="{node_name(row)}" coef="1"/>')
        lines.append("        </fixed>")
        lines.append(f'        <spatial name="{base}_fdp_r" rgba="0 0 0 0">')
        lines.append(f'            <site site="{base}_ref_r"/>')
        lines += [f"            {e}" for e in _fdp_path_after(node["fdp"], node["segment_start"])]
        lines.append("        </spatial>")
    return lines


def equality_block(table: dict[str, Any]) -> str:
    """Generated block of ``myoarm_r_assets.xml``: the node ties."""
    cfg = table["lumbrical_nodes"]
    solref = " ".join(_number(v) for v in cfg["equality_solref"]["value"])
    lines = []
    for row in node_rows(table):
        base = node_name(row)[:-2]
        lines.append(
            f'        <tendon name="{base}_tie_r" tendon1="{base}_slide_r" tendon2="{base}_fdp_r"'
            f' polycoef="0 1 0 0 0" solref="{solref}"/>'
        )
    return "".join(f"{line}\n" for line in lines)


def architecture_lengthranges(table: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Length range per row from its straight site path at q = 0 (sites from ``myoarm_r_chain.xml``)."""
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    lo, hi = model.actuator_gainprm[model.actuator(f"{table['template_muscle']}_r").id, :2]
    ranges = {}
    for row in table["muscles"]:
        if "insertion" in row:  # MuJoCo's length of the branched tendon (mean over the slack postures)
            lengths = []
            for posture in slack_postures(table, row):
                data.qpos[:] = 0.0
                for joint, q in posture.items():
                    data.qpos[model.joint(joint).qposadr[0]] = q
                place_tendon_nodes(model, data)
                mujoco.mj_comPos(model, data)
                mujoco.mj_tendon(model, data)
                lengths.append(float(data.ten_length[model.tendon(f"{row['name']}_r_tendon").id]))
            l_mt0 = float(np.mean(lengths))
            data.qpos[:] = 0.0
            mujoco.mj_kinematics(model, data)
        else:
            points = [data.site_xpos[model.site(site_name(row, i)).id] for i in range(len(row["sites"]))]
            l_mt0 = float(sum(np.linalg.norm(b - a) for a, b in zip(points[:-1], points[1:])))
        l_fo = row["optimal_fiber_length_m"]["value"]
        l_ts = (l_mt0 - l_fo * np.cos(np.radians(row["pennation_deg"]["value"]))) / 1.033
        ranges[row["name"]] = (float(l_ts + lo * l_fo), float(l_ts + hi * l_fo))
    return ranges


def muscle_block(table: dict[str, Any], lengthranges: dict[str, tuple[float, float]], label: str | None = None) -> str:
    """Generated block *label* of ``myoarm_r_muscle.xml``: one muscle per row."""
    lines = []
    for row in rows_in_block(table, label):
        lines.append(
            f"        <!-- {row['name']}: force {row['force_N']['derivation']}; L_fo"
            f" {row['optimal_fiber_length_m']['value']} m, pennation {row['pennation_deg']['value']} deg"
            f" ({_cite(table, row['architecture_source'])}); lengthrange from the rigid-tendon slack length of"
            " Barry et al. 2018 Eq. 1 -->"
        )
        lo, hi = lengthranges[row["name"]]
        lines.append(
            f'        <muscle name="{row["name"]}_r" force="{row["force_N"]["value"]:g}" class="{MUSCLE_CLASS}"'
            f' tendon="{row["name"]}_r_tendon" lengthrange="{lo!r} {hi!r}"'
            + (f' fpmax="{row["fpmax"]["value"]:g}"' if "fpmax" in row else "")
            + "/>"
        )
    return "".join(f"{line}\n" for line in lines)


def write_xml(table: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Write the sites, then every tendon and muscle block (length ranges from the written sites)."""
    for row in node_rows(table):
        write_generated_block(RIGHT_ARM_CHAIN_XML, SCRIPT, chain_node_block(table, row), "", f"node {row['name']}")
    if node_rows(table):
        write_generated_block(RIGHT_ARM_ASSETS_XML, SCRIPT, equality_block(table), "</equality>")
    RIGHT_ARM_CHAIN_XML.write_text(chain_with_sites(RIGHT_ARM_CHAIN_XML.read_text(), table))
    for label in block_labels(table):
        write_generated_block(RIGHT_ARM_TENDONS_XML, SCRIPT, tendon_block(table, label), "</tendon>", label)
    ranges = architecture_lengthranges(table)  # branched tendons: from the tendons just written
    for label in block_labels(table):
        write_generated_block(RIGHT_ARM_MUSCLES_XML, SCRIPT, muscle_block(table, ranges, label), "</actuator>", label)
    return ranges


# --- path fit -------------------------------------------------------------------------------------------


@cache
def _model() -> mujoco.MjModel:
    return build_spec("myoarm_r").compile()


class Poses:
    """Body positions and orientations of ``myoarm_r`` at a list of postures (joint name -> q, others 0).

    The lumbrical nodes sit on their ties (``place_tendon_nodes``) at *node_postures* (default: the postures
    themselves); a sweep passes the centre posture there to read a muscle's own moment arm (node held).
    """

    def __init__(self, postures: list[dict[str, float]], node_postures: list[dict[str, float]] | None = None) -> None:
        model = _model()
        data = mujoco.MjData(model)
        slides = [j for j in range(model.njnt) if model.joint(j).name.startswith("node_")]
        pos, mat = [], []
        for posture, node_posture in zip(postures, node_postures or postures):
            nodes = {}
            if slides:
                data.qpos[:] = model.qpos0
                for joint, q in node_posture.items():
                    data.qpos[model.joint(joint).qposadr[0]] = q
                place_tendon_nodes(model, data)
                nodes = {j: data.qpos[model.jnt_qposadr[j]] for j in slides}
            data.qpos[:] = model.qpos0
            for joint, q in posture.items():
                data.qpos[model.joint(joint).qposadr[0]] = q
            for j, q in nodes.items():
                data.qpos[model.jnt_qposadr[j]] = q
            mujoco.mj_kinematics(model, data)
            pos.append(data.xpos.copy())
            mat.append(data.xmat.copy().reshape(-1, 3, 3))
        self.pos, self.mat = np.array(pos), np.array(mat)

    def length(self, sites: list[tuple[str, np.ndarray]]) -> np.ndarray:
        """Length (m) of the straight path through *sites* (body name, local position) in every posture.

        *sites* may also be weighted legs ``[(weight, sites), ...]`` (a branched tendon): their weighted sum.
        """
        if sites and isinstance(sites[0][0], float):
            return sum(w * self.length(leg) for w, leg in sites)
        model = _model()
        world = []
        for body, local in sites:
            b = model.body(body).id
            world.append(self.pos[:, b] + np.einsum("nij,j->ni", self.mat[:, b], np.asarray(local)))
        return sum(np.linalg.norm(q - p, axis=1) for p, q in zip(world[:-1], world[1:]))


class Sweep:
    """dL/dq (mm) about *joint* at posture *base* with *joint* at each of *values*.

    By default the lumbrical nodes are held where they sit at each value (the muscle's own moment arm, the
    quantity of An et al. 1983); ``systemic`` lets them follow the tendon (own arm minus the FDP's).
    """

    def __init__(self, base: dict[str, float], joint: str, values: np.ndarray, systemic: bool = False) -> None:
        centre = None if systemic else [base | {joint: float(v)} for v in values]
        self.plus = Poses([base | {joint: float(v) + H} for v in values], centre)
        self.minus = Poses([base | {joint: float(v) - H} for v in values], centre)

    def arms(self, sites: list[tuple[str, np.ndarray]]) -> np.ndarray:
        return (self.plus.length(sites) - self.minus.length(sites)) / (2.0 * H) * 1e3


def fist(row: dict[str, Any]) -> dict[str, float]:
    """The lumbrical's digit at its MCP, PIP and DIP flexion limits (the lumbrical's longest posture)."""
    model = _model()
    digit = table_digit(row)
    return {
        j: float(model.joint(j).range[1]) for j in (f"mcp{digit}_flexion_r", f"pm{digit}_flexion_r", f"md{digit}_flexion_r")
    }


def table_digit(row: dict[str, Any]) -> int:
    return int(row["insertion"]["hood"]["digit"])


def slack_postures(table: dict[str, Any], row: dict[str, Any]) -> list[dict[str, float]]:
    """Postures whose mean muscle-tendon length sets the rigid-tendon slack length of *row*.

    q = 0 (Barry et al. 2018, Eq. 1), except for a lumbrical on its node: its length changes by about 31 mm
    from the straight finger to the fist (Ranney & Wells 1988), so the mean of the two postures sits at the
    optimal fibre length and the whole excursion stays on MuJoCo's force-length curve.
    """
    return [{}, fist(row)] if row in node_rows(table) else [{}]


def thumb_neutral() -> dict[str, float]:
    """Smutz 1998 neutral: mid-point of the thumb CMC ranges, MP at 0."""
    model = _model()
    return {
        "cmc_flexion_r": float(np.mean(model.joint("cmc_flexion_r").range)),
        "cmc_abduction_r": float(np.mean(model.joint("cmc_abduction_r").range)),
        "mp_flexion_r": 0.0,
    }


def joint_q(joint: str, deg: float) -> float:
    """MyoHand angle of an anatomical angle *deg* (flexion, abduction positive); thumb: offset from neutral."""
    if joint not in THUMB_BOX_DEG:
        return deg * DEG
    sign = {"cmc_flexion_r": -1.0, "cmc_abduction_r": 1.0, "mp_flexion_r": -1.0}[joint]
    return thumb_neutral()[joint] + sign * deg * DEG


def crossed(row: dict[str, Any]) -> list[str]:
    """Joints of the box and range checks that *row* crosses (from the bodies of its sites)."""
    bodies = {s["body"] for s in row["sites"]}
    if junction(row):
        bodies.add(_model().body(_model().site_bodyid[_model().site(junction(row)).id]).name)
    joints = []
    if bodies & {"firstmc_r", "proximal_thumb_r"}:
        joints += ["cmc_flexion_r", "cmc_abduction_r"] + (["mp_flexion_r"] if "proximal_thumb_r" in bodies else [])
    if "2proxph_r" in bodies:
        joints += ["mcp2_flexion_r", "mcp2_abduction_r"]
    for d in (3, 4):
        if f"{d}proxph_r" in bodies:
            joints += [f"mcp{d}_flexion_r", f"mcp{d}_abduction_r"]
    if bodies & {"fourthmc_r", "4proxph_r"} and bodies & PROXIMAL_OF_MC4:
        joints += ["cmc4_flexion_r"]
    if "5proxph_r" in bodies:
        joints += ["mcp5_flexion_r", "mcp5_abduction_r"]
    if bodies & {"fifthmc_r", "5proxph_r"} and bodies & PROXIMAL_OF_MC5:
        joints += ["cmc5_flexion_r"]
    return joints


def _grid(ranges: dict[str, Any]) -> list[dict[str, float]]:
    postures: list[dict[str, float]] = [{}]
    for joint, values in ranges.items():
        postures = [p | {joint: float(v)} for p in postures for v in values]
    return postures


def boxes(row: dict[str, Any]) -> tuple[Poses, Poses]:
    """Measurement box and full range of motion of the joints *row* crosses (``path_fit.measurement_box``)."""
    model = _model()
    joints = crossed(row)
    box = {j: [joint_q(j, d) for d in (THUMB_BOX_DEG | INDEX_BOX_DEG | FINGER_BOX_DEG | LITTLE_BOX_DEG)[j]] for j in joints}
    full = {j: np.linspace(*model.joint(j).range, 5) for j in joints}
    return Poses(_grid(box)), Poses(_grid(full))


def target_sweep(joint: str, target: dict[str, Any]) -> tuple[Sweep, np.ndarray, np.ndarray, bool]:
    """Sweep, mean, SD and whether the model value is averaged over the sweep (``over``), for one target."""
    if "over" in target:  # index MCP2: averaged over the range (An et al. 1983)
        return (*average_sweep(joint), np.array([target["mean_mm"]]), np.array([target["sd_mm"]]), True)
    values = np.array([joint_q(joint, d) for d in target["posture_deg"]])
    return Sweep(thumb_neutral(), joint, values), np.atleast_1d(target["mean_mm"]), np.atleast_1d(target["sd_mm"]), False


def average_sweep(joint: str) -> tuple[Sweep]:
    """MCP2 flexion 0-90 deg or abduction -15 to 15 deg, the other joints at 0."""
    values = np.radians(np.linspace(0.0, 90.0, 10) if joint == "mcp2_flexion_r" else np.linspace(-15.0, 15.0, 7))
    return (Sweep({}, joint, values),)


def _junction_point(row: dict[str, Any]) -> tuple[str, np.ndarray]:
    model = _model()
    site = model.site(junction(row))
    return model.body(model.site_bodyid[site.id]).name, model.site_pos[site.id].copy()


def fit_group(table: dict[str, Any], group: str) -> None:
    """Fit the sites of the rows of *group* together and record positions and results in the table."""
    from scipy.optimize import least_squares

    cfg = table["path_fit"]
    floor = cfg["sd_floor_mm"]
    rows = [r for r in table["muscles"] if r["fit"]["group"] == group]
    forces = np.array([r["force_N"]["value"] for r in rows])
    weights = forces / forces.sum()
    sweeps = [(j, *target_sweep(j, t)) for j, t in rows[0]["fit"].get("targets", {}).items()]
    head_sweeps = [
        (k, j, *target_sweep(j, t)) for k, r in enumerate(rows[1:], 1) for j, t in r["fit"].get("targets", {}).items()
    ]
    names = [r["name"] for r in rows]
    order = [average_sweep(j)[0] for j in ("mcp2_flexion_r", "mcp2_abduction_r")] if {"DI1d", "DI1s"} <= set(names) else []
    windows = [boxes(r) for r in rows]
    lo_fl, hi_fl = cfg["fibre_length_window"]
    x0 = np.concatenate([np.ravel([s["start"]["pos"] for s in r["sites"]]) for r in rows])
    offsets = np.cumsum([0] + [3 * len(r["sites"]) for r in rows])
    rests = [Poses(slack_postures(table, r)) for r in rows]

    fixed = {r["name"]: _junction_point(r) for r in rows if junction(r)}

    def paths(x: np.ndarray) -> list[Any]:
        out = []
        for k, r in enumerate(rows):
            points = [(s["body"], x[offsets[k] + 3 * i : offsets[k] + 3 * i + 3]) for i, s in enumerate(r["sites"])]
            out.append(legs(r, points, fixed[r["name"]]) if junction(r) else points)
        return out

    def normalised(path: list, poses: Poses, row: dict[str, Any]) -> np.ndarray:
        l_fo = row["optimal_fiber_length_m"]["value"]
        l_mt = rests[rows.index(row)].length(path).mean()
        l_ts = (l_mt - l_fo * np.cos(np.radians(row["pennation_deg"]["value"]))) / 1.033
        return (poses.length(path) - l_ts) / l_fo

    def model_values(p: list) -> list[np.ndarray]:
        out = []
        for _, sweep, _, _, over in sweeps:
            arms = sum(w * sweep.arms(path) for w, path in zip(weights, p))
            out.append(np.atleast_1d(arms.mean()) if over else arms)
        return out

    def head_values(p: list) -> list[np.ndarray]:
        return [np.atleast_1d(s.arms(p[k]).mean()) if over else s.arms(p[k]) for k, _, s, _, _, over in head_sweeps]

    def residuals(x: np.ndarray) -> np.ndarray:
        p = paths(x)
        res = [(v - m) / np.maximum(sd, floor) for v, (_, _, m, sd, _) in zip(model_values(p), sweeps)]
        res += [(v - m) / np.maximum(sd, floor) for v, (_, _, _, m, sd, _) in zip(head_values(p), head_sweeps)]
        if len(rows) > 1:  # every head acts in the direction of the group's clear targets (|mean| >= 2 mm)
            for _, sweep, mean, _, over in sweeps:
                for path in p:
                    arms = np.atleast_1d(sweep.arms(path).mean()) if over else sweep.arms(path)
                    clear = np.abs(mean) >= 2.0
                    res.append(np.where(clear, np.maximum(0.0, -np.sign(mean) * arms), 0.0))
        if order:  # Masquelet 1986: the deep head is the larger MCP2 flexor, the superficial head the larger abductor
            deep, superficial = p[names.index("DI1d")], p[names.index("DI1s")]
            res.append(max(0.0, order[0].arms(deep).mean() - order[0].arms(superficial).mean()))
            res.append(max(0.0, order[1].arms(superficial).mean() - order[1].arms(deep).mean()))
        res.append((x - x0) / cfg["regularisation_m"])
        for path, row, (box, full) in zip(p, rows, windows):
            inside = normalised(path, box, row)
            res.append(PENALTY * np.maximum(0.0, lo_fl - inside))
            res.append(PENALTY * np.maximum(0.0, inside - hi_fl))
            if cfg["full_range_max"] is not None:  # bounds the passive force at the ends of the joint ranges
                res.append(PENALTY * np.maximum(0.0, normalised(path, full, row) - cfg["full_range_max"]))
        return np.concatenate([np.atleast_1d(np.asarray(r, dtype=float)) for r in res])

    bound = cfg["bound_m"]
    x = np.round(least_squares(residuals, x0, bounds=(x0 - bound, x0 + bound), x_scale=1e-3).x, 6)
    p, p0 = paths(x), paths(x0)
    values = model_values(p)
    z = np.concatenate([np.zeros(0)] + [(v - m) / np.maximum(sd, floor) for v, (_, _, m, sd, _) in zip(values, sweeps)])
    for k, (row, path, (box, full)) in enumerate(zip(rows, p, windows)):
        for i, site in enumerate(row["sites"]):
            site["pos"] = [float(v) for v in x[offsets[k] + 3 * i : offsets[k] + 3 * i + 3]]
        inside, everywhere = normalised(path, box, row), normalised(path, full, row)
        result: dict[str, Any] = {
            "site_shift_mm": [
                round(float(np.linalg.norm(np.subtract(s["pos"], s["start"]["pos"])) * 1e3), 2) for s in row["sites"]
            ],
            "fibre_length_measurement_box": [round(float(inside.min()), 3), round(float(inside.max()), 3)],
            "fibre_length_full_range": [round(float(everywhere.min()), 3), round(float(everywhere.max()), 3)],
        }
        if k == 0 and sweeps:
            result["model_mm"] = {j: [round(float(v), 2) for v in value] for (j, *_), value in zip(sweeps, values)}
            result["start_mm"] = {j: [round(float(v), 2) for v in value] for (j, *_), value in zip(sweeps, model_values(p0))}
            result["rms_z"] = round(float(np.sqrt(np.mean(z**2))), 3)
            if len(rows) > 1:
                result["force_weights"] = {n: round(float(w), 3) for n, w in zip(names, weights)}
        if row["fit"].get("reference"):  # reported against the row's reference targets, not fitted to them
            result["reference_mm"] = {
                j: [round(float(a), 2) for a in target_sweep(j, t)[0].arms(path)] for j, t in row["fit"]["reference"].items()
            }
        own = [(j, v) for (kk, j, *_), v in zip(head_sweeps, head_values(p)) if kk == k]
        if own:
            result["model_mm"] = {j: [round(float(a), 2) for a in v] for j, v in own}
        if len(rows) > 1:  # this head alone on the group's targets
            result["head_mm"] = {
                j: [round(float(a), 2) for a in (np.atleast_1d(s.arms(path).mean()) if over else s.arms(path))]
                for j, s, _, _, over in sweeps
            }
        if row in node_rows(table):  # with the node following its FDP (reported, not fitted)
            digit = table_digit(row)
            result["systemic_mm"] = {
                j: [round(float(Sweep({}, j, np.zeros(1), systemic=True).arms(path)[0]), 2)]
                for j in (f"mcp{digit}_flexion_r", f"mcp{digit}_abduction_r", f"pm{digit}_flexion_r", f"md{digit}_flexion_r")
            }
        row["fit"]["result"] = result


def fit(table: dict[str, Any]) -> None:
    """Refit the sites of every fit group."""
    for group in dict.fromkeys(r["fit"]["group"] for r in table["muscles"]):
        fit_group(table, group)


def _passive_script() -> Any:
    import importlib.util
    from pathlib import Path

    path = Path(__file__).with_name("fit_passive_moments.py")
    spec = importlib.util.spec_from_file_location("fit_passive_moments", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fit_passive_scale(table: dict[str, Any]) -> None:
    """Fit the passive-force scale (``fpmax``) of the rows that cross the joints of ``passive_fit.rows``.

    MuJoCo's passive force is linear in ``fpmax``, so each joint moment is the other muscles' moment plus
    ``sum fpmax_i m_i(q)`` (only the muscles a row's source includes, if it names them). Loss as for the extrinsic muscles (``fit_muscle_passive.py``): per row of
    ``passive_fit.rows`` (passive_joint_moments.json) the mean of ``excess_weight * excess^2 + deficit^2``
    (N cm) over the row's fit range, plus ``penalty * |ln(fpmax / reference)|``; the joint springs are
    fitted afterwards to what remains (``fit_passive_moments.py``).
    """
    from scipy.optimize import minimize

    from myo_sim.build.hand_data import load_passive_moment_table

    cfg = table["passive_fit"]
    passive = _passive_script()
    rows = [r for r in load_passive_moment_table()["joints"] if r["joint"] in cfg["rows"]]
    model = build_spec("myoarm_r").compile()
    data = mujoco.MjData(model)
    joints = {prow["names"]["r"] for prow in rows}
    fitted = [r for r in table["muscles"] if set(crossed(r)) & joints]
    ids = [model.actuator(f"{r['name']}_r").id for r in fitted]
    terms = []
    for prow in rows:
        joint = model.joint(prow["names"]["r"])
        span = prow["source_curve"].get("fit_range_deg")  # else the joint range
        lo, hi = np.radians(span) if span else model.jnt_range[joint.id]
        q = np.linspace(lo, hi, 25)
        target = passive.source_torque(prow["source_curve"], model.jnt_range[joint.id])(q) * 100.0
        unit, other = np.zeros((len(q), len(ids))), np.zeros(len(q))
        for n, value in enumerate(q):
            data.qpos[:] = model.qpos0
            data.qpos[joint.qposadr[0]] = value
            data.qvel[:] = data.act[:] = data.ctrl[:] = 0.0
            place_tendon_nodes(model, data)
            mujoco.mj_forward(model, data)
            moment = np.zeros((model.nu, model.nv))
            mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
            moment = fold_node_forces(model, data, moment)
            per = data.actuator_force * moment[:, joint.dofadr[0]] * 100.0
            unit[n] = per[ids] / model.actuator_gainprm[ids, 7]
            named = prow["source_curve"].get("subtract_model_muscles", {}).get("muscles")
            counted = [model.actuator(m).id for m in named] if named else range(model.nu)
            unit[n] *= np.array([float(i in counted) for i in ids])
            other[n] = sum(per[i] for i in counted if i not in ids)
        terms.append((prow["joint"], target, unit, other))
    reference, (f_lo, f_hi) = cfg["fpmax_reference"], cfg["fpmax_bounds"]

    def cost(log_f: np.ndarray) -> float:
        total = 0.0
        for _, target, unit, other in terms:
            moment = other + unit @ np.exp(log_f)
            inside = np.clip(moment, np.minimum(target, 0.0), np.maximum(target, 0.0))
            total += np.mean(cfg["excess_weight"] * (moment - inside) ** 2 + (target - inside) ** 2)
        return total + cfg["penalty"] * np.sum(np.abs(log_f - np.log(reference)))

    start = np.full(len(ids), np.log(reference))
    log_f = minimize(cost, start, method="L-BFGS-B", bounds=[(np.log(f_lo), np.log(f_hi))] * len(ids)).x
    for row, value in zip(fitted, np.exp(log_f)):
        row["fpmax"] = {"value": float(f"{value:.4g}"), "reference_value": reference, "method": "passive_fit"}
    cfg["result"] = {"cost_before": round(float(cost(start)), 3), "cost_after": round(float(cost(log_f)), 3)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fit", action="store_true", help="refit the sites to the tabled moment arms first")
    parser.add_argument("--place-nodes", action="store_true", help="recompute the lumbrical node positions first")
    args = parser.parse_args()
    table = load_intrinsic_table()
    if args.place_nodes:
        place_nodes(table)
        INTRINSIC_DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    if args.fit:
        fit(table)
        for row in table["muscles"]:
            row.pop("fpmax", None)
        write_xml(table)  # the passive scale is fitted on the model with the fitted paths
        fit_passive_scale(table)
        INTRINSIC_DATA_PATH.write_text(json.dumps(table, indent=2, ensure_ascii=False) + "\n")
    ranges = write_xml(table)
    for row in table["muscles"]:
        lo, hi = ranges[row["name"]]
        print(f"{row['name']}: lengthrange {lo:.5f} {hi:.5f}; {json.dumps(row['fit'].get('result', {}))}")


if __name__ == "__main__":
    main()
