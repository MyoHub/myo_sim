#!/usr/bin/env python3
"""Scapular muscles and rotator-cuff sub-regions after Seth et al. 2019 (``build/data/scapular_muscles.json``).

    uv run python scripts/generate_scapular_muscles.py --zip ThoracoscapularShoulderPaperMaterials.zip --fit
    uv run python scripts/generate_scapular_muscles.py --write

**--fit** reads Seth's muscle points from the published archive (CC BY 4.0, ``build/data/LICENSE-Seth2019.txt``),
places them in this package's frames (clavicle, scapula and humerus frames are Seth's; thorax points shifted by the
sternoclavicular offset, ``scapulothoracic.json``), moves the cervical origins onto the neck bodies, writes the XML,
then fits each muscle's rigid-tendon map along Seth's measured motions (the release's IK frames, assembled) and writes
the table again. **--write** renders the XML blocks from the table alone.

Paths: Seth's two points per line, straight, except where a wrap changes the length along the measured motions: the
supraspinatus over Seth's humeral-head sphere, infraspinatus and subscapularis over Seth's scapular cylinders (their
``-y`` quadrant as a side site), and serratus anterior M and I over thorax spheres fitted to the lengths of Seth's
ellipsoid wrap. Forces: PCSA x the table's specific tension. Fibre lengths: Ward 2006 for the cuff (Seth's optimal
muscle-tendon length kept, so the tendon takes the difference), Seth's otherwise.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from fit_scapulothoracic import (  # noqa: E402
    GENERIC,
    Seth,
    axis_rot,
    clavicle_axial_ik,
    euler_xyz,
    humerothoracic_elevation,
    ik_trials,
    read_osim,
)

import myo_sim  # noqa: E402
from myo_sim.build.hand_data import DATA_DIR, write_generated_block  # noqa: E402
from myo_sim.build.rigid_conversion import fit_rigid_muscle_map  # noqa: E402

DATA = ROOT / "myo_sim" / "build" / "data"
TABLE = DATA / "scapular_muscles.json"
ST = json.loads((DATA / "scapulothoracic.json").read_text())
MODELS = ROOT / "myo_sim" / "models"
ARM = MODELS / "arm" / "assets"
CHAIN, TENDON, MUSCLE = ARM / "myoarm_r_chain.xml", ARM / "myoarm_r_tendon.xml", ARM / "myoarm_r_muscle.xml"
TORSO = MODELS / "torso" / "assets" / "myotorso_chain.xml"
RIGID_HEAD = MODELS / "head" / "assets" / "myohead_rigid_chain.xml"
NECK = MODELS / "head" / "assets" / "myohead_chain.xml"
SCRIPT = "generate_scapular_muscles.py"
BODY = {"thorax": "chest_r", "clavicle": "clavicle_r", "scapula": "scapula_r", "humerus": "humerus_r"}
GAIN_TAIL = (0.0, 1.8, 10.0, 0.65, 1.4)  # lmin, lmax, vmax, fpmax, fvmax as the other upper-arm muscles
# Passive scale (fpmax): the cuff lines keep the fitted value of the muscle they replace (SUPSP, INFSP, SUBSC); the
# scapular muscles, without passive data, take the median of the shoulder muscles kept (DELT, PECM, LAT, TMAJ, TMIN,
# CORB, TRIlong): 0.65.
FPMAX = {"supraspinatus": 0.288421, "infraspinatus": 0.429583, "subscapularis": 0.653193, "klein_breteler": 0.65}

# name, Seth's muscle, PCSA source key, wrap key
ROWS = (
    ("SUPSP1", "Supraspinatus_A", "supraspinatus", "SUPSP_hhead"),
    ("SUPSP2", "Supraspinatus_P", "supraspinatus", "SUPSP_hhead"),
    ("INFSP1", "Infraspinatus_S", "infraspinatus", "INFSP_cylinder"),
    ("INFSP2", "Infraspinatus_I", "infraspinatus", "INFSP_cylinder"),
    ("SUBSC1", "Subscapularis_S", "subscapularis", "SUBSC_cylinder"),
    ("SUBSC2", "Subscapularis_M", "subscapularis", "SUBSC_cylinder"),
    ("SUBSC3", "Subscapularis_I", "subscapularis", "SUBSC_cylinder"),
    ("TRAP1", "TrapeziusClavicle_S", "klein_breteler", None),
    ("TRAP2", "TrapeziusScapula_S", "klein_breteler", None),
    ("TRAP3", "TrapeziusScapula_M", "klein_breteler", None),
    ("TRAP4", "TrapeziusScapula_I", "klein_breteler", None),
    ("SERR1", "SerratusAnterior_S", "klein_breteler", None),
    ("SERR2", "SerratusAnterior_M", "klein_breteler", "SERR2_thorax"),
    ("SERR3", "SerratusAnterior_I", "klein_breteler", "SERR3_thorax"),
    ("RHOM1", "Rhomboideus_S", "klein_breteler", None),
    ("RHOM2", "Rhomboideus_I", "klein_breteler", None),
    ("LEVS", "LevatorScapulae", "klein_breteler", None),
    ("PECMIN", "PectoralisMinor", "klein_breteler", None),
)
# Origins near the cervical spine (within 5 mm of a cervical vertebra mesh at the rest posture) go on that vertebra in the
# muscular neck and on the rigid neck body elsewhere: levator scapulae and the clavicular trapezius at C1, the superior
# scapular trapezius and the superior rhomboid at C7.
CERVICAL = {
    "TRAP1": "myohead_cerv1",
    "LEVS1": "myohead_cerv1",
    "TRAP2": "myohead_cerv7",
    "RHOM1": "myohead_cerv7",
    "LEVS2": "myohead_cerv3",
    "TRAPocc": "myohead_skull",
}
# Neck lines added to Seth's (scripts/fit: neck_lines). Levator scapulae in two slips with DAS3's split and lower element
# (Chadwick et al. 2014, after Klein Breteler et al. 1999): upper (C1-C2) 48 %, lower (C3-C4) 52 %, l0 0.1302 m, slack
# 0.018 m, the lower slip from the tip of C3's transverse process. The clavicular trapezius gives 20 % of its PCSA to an
# occipital line from the superior nuchal line (low confidence: Johnson et al. 1994 find the occipital fibres the
# smallest, without numbers), fibre length 0.084 m (Kamibayashi and Richmond 1998, as reported in Vasavada et al. 2010).
COMPOSED = {"myohead_skull": "head"}  # build/head.py renames the skull when the neck is composed
LEVATOR_SPLIT = (0.48, 0.52)
LEVATOR_LOWER = {"l0": 0.1302, "lts": 0.018}
OCCIPITAL_SHARE = 0.20
OCCIPITAL_L0 = 0.084
# Whole-muscle architecture (Ward et al. 2006, Clin Orthop Relat Res 448:157, Table 1): PCSA cm2, fibre length m, pennation
WARD2006 = {
    "supraspinatus": (6.65, 0.0450, 5.1),
    "infraspinatus": (10.71, 0.0657, 1.4),
    "subscapularis": (15.53, 0.0600, 0.0),
}


def _r(x: Any, n: int = 6) -> list[float]:
    return [round(float(v), n) for v in np.ravel(x)]


# ----------------------------------------------------------------------------------------------- fit
def seth_muscles(zf: zipfile.ZipFile) -> dict[str, dict[str, Any]]:
    root = ET.fromstring(zf.read(GENERIC))
    out = {}
    for mus in root.iter():
        if mus.tag.endswith("Muscle") and mus.get("name"):
            pts = [
                (p.findtext("socket_parent_frame").split("/")[-1], np.array(p.findtext("location").split(), float))
                for p in mus.iter()
                if p.tag == "PathPoint"
            ]
            out[mus.get("name")] = {
                "points": pts,
                "F": float(mus.findtext("max_isometric_force")),
                "l0": float(mus.findtext("optimal_fiber_length")),
                "lts": float(mus.findtext("tendon_slack_length")),
            }
    return out


def seth_wraps(zf: zipfile.ZipFile) -> dict[str, dict[str, Any]]:
    """Seth's sphere and cylinder wraps used here, as MuJoCo geoms in this package's frames."""
    root = ET.fromstring(zf.read(GENERIC))
    found = {w.get("name"): w for w in root.iter() if w.tag in ("WrapSphere", "WrapCylinder")}

    def geom(name: str, body: str, key: str) -> dict[str, Any]:
        w = found[name]
        rot = euler_xyz(np.array(w.findtext("xyz_body_rotation").split(), float))
        pos = np.array(w.findtext("translation").split(), float)
        q = np.roll(Rotation.from_matrix(rot).as_quat(), 1)
        out = {"body": body, "pos": _r(pos), "quat": _r(q), "source": f"Seth 2019 wrap {name}"}
        if w.tag == "WrapSphere":
            out.update(type="sphere", size=_r([float(w.findtext("radius"))]))
        else:
            r, length = float(w.findtext("radius")), float(w.findtext("length"))
            out.update(type="cylinder", size=_r([r, length / 2]))
            # OpenSim quadrant -y: wrap on the cylinder's -y side; the MuJoCo side site sits there, 2 radii from the axis
            out["side_site"] = _r(pos + rot @ np.array([0.0, -2 * r, 0.0]))
        return out

    return {
        "SUPSP_hhead": geom("hheadsupraTminor", "humerus_r", "SUPSP_hhead"),
        "INFSP_cylinder": geom("Infra", "scapula_r", "INFSP_cylinder"),
        "SUBSC_cylinder": geom("Subscapuralis", "scapula_r", "SUBSC_cylinder"),
    }


def fitted_thorax_spheres() -> dict[str, dict[str, Any]]:
    """Spheres on chest_r that reproduce the lengths of serratus anterior M and I over Seth's ellipsoid wrap "Thorax"
    (74 x 170 x 70 mm; MuJoCo wraps spheres and cylinders only): centre and radius least-squares fitted to the path
    lengths of Seth's OpenSim model (OpenSim 4.6) along the 4312 assembled IK frames."""
    return {
        "SERR2_thorax": {
            "body": "chest_r", "type": "sphere", "pos": [-0.0266, -0.0999, 0.0412], "quat": [1, 0, 0, 0], "size": [0.0864],
            "source": "fitted to Seth 2019 SerratusAnterior_M over wrap Thorax: rms 0.11 mm, max 1.02 mm (straight line: rms 8.41 mm)",
        },
        "SERR3_thorax": {
            "body": "chest_r", "type": "sphere", "pos": [-0.0113, -0.1568, 0.0494], "quat": [1, 0, 0, 0], "size": [0.094],
            "source": "fitted to Seth 2019 SerratusAnterior_I over wrap Thorax: rms 0.01 mm, max 0.08 mm (straight line: rms 1.42 mm)",
        },
    }  # fmt: skip


def body_transform(model: mujoco.MjModel, data: mujoco.MjData, body: str) -> tuple[np.ndarray, np.ndarray]:
    i = model.body(body).id
    return data.xmat[i].reshape(3, 3), data.xpos[i]


def neck_landmarks() -> dict[str, np.ndarray]:
    """Rest-posture points in chest_r (myofullbody): the tip of C3's right transverse process (its most lateral vertex) and
    the superior nuchal line, medial third (the most posterior skull vertex 15-25 mm right of the midline)."""
    model = myo_sim.load_model("myofullbody")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rc, pc = body_transform(model, data, "chest_r")

    def vertices(geom: str) -> np.ndarray:
        g = model.geom(geom).id
        mesh = model.geom_dataid[g]
        v = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        world = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
        return (world - pc) @ rc

    c3 = vertices("myohead_cerv3_visual")
    skull = vertices("hat_skull")
    band = skull[(skull[:, 2] > 0.015) & (skull[:, 2] < 0.025)]
    return {"c3_transverse": c3[np.argmax(c3[:, 2])], "superior_nuchal_line": band[np.argmin(band[:, 0])]}


def neck_lines(rows: list[dict[str, Any]], tension: float) -> None:
    """Split Seth's levator scapulae into two slips and add the occipital line of the clavicular trapezius."""
    marks = neck_landmarks()
    levs = next(r for r in rows if r["name"] == "LEVS")
    trap = next(r for r in rows if r["name"] == "TRAP1")
    total_levs, total_trap = levs["pcsa_cm2"], trap["pcsa_cm2"]
    levs["name"] = "LEVS1"
    lower = json.loads(json.dumps(levs))
    occipital = json.loads(json.dumps(trap))
    levs["pcsa_cm2"] = round(total_levs * LEVATOR_SPLIT[0], 3)
    levs["pcsa_source"] += f"; upper slip (C1-C2), {LEVATOR_SPLIT[0]:.0%} of the muscle as DAS3's lev_scap_1"
    lower.update(name="LEVS2", origin={"body": "chest_r", "pos": _r(marks["c3_transverse"])}, **LEVATOR_LOWER)
    lower["pcsa_cm2"] = round(total_levs * LEVATOR_SPLIT[1], 3)
    lower["pcsa_source"] += f"; lower slip (C3-C4), {LEVATOR_SPLIT[1]:.0%} of the muscle as DAS3's lev_scap_2"
    lower["length_source"] = "DAS3 lev_scap_2 (Chadwick et al. 2014)"
    lower["origin_note"] = "tip of C3's transverse process (most lateral vertex of the C3 mesh); Seth's insertion"
    trap["pcsa_cm2"] = round(total_trap * (1 - OCCIPITAL_SHARE), 3)
    trap["pcsa_source"] += f"; nuchal part, {1 - OCCIPITAL_SHARE:.0%} of the clavicular trapezius"
    occipital.update(name="TRAPocc", origin={"body": "chest_r", "pos": _r(marks["superior_nuchal_line"])})
    occipital["pcsa_cm2"] = round(total_trap * OCCIPITAL_SHARE, 3)
    occipital["pcsa_source"] += (
        f"; occipital part, {OCCIPITAL_SHARE:.0%} of the clavicular trapezius (low confidence: no measured share)"
    )
    occipital["l0"] = OCCIPITAL_L0
    occipital["lts"] = round(_rest_length(occipital) - OCCIPITAL_L0, 5)
    occipital["length_source"] = (
        "fibre length of Kamibayashi and Richmond 1998 (as reported in Vasavada et al. 2010); slack: optimal fibre at rest"
    )
    occipital["origin_note"] = "superior nuchal line, medial third (most posterior skull vertex 15-25 mm from the midline)"
    for row in (levs, lower, trap, occipital):
        row["force_N"] = round(row["pcsa_cm2"] * tension, 1)
    del lower["seth_parameters"], occipital["seth_parameters"]
    rows.insert(rows.index(levs) + 1, lower)
    rows.insert(rows.index(trap) + 1, occipital)


def _rest_length(row: dict[str, Any]) -> float:
    """Straight distance between the row's origin (chest_r) and insertion at the rest posture."""
    model = myo_sim.load_model("myoarm_r")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rc, pc = body_transform(model, data, "chest_r")
    rb, pb = body_transform(model, data, row["insertion"]["body"])
    return float(np.linalg.norm(pb + rb @ np.array(row["insertion"]["pos"]) - pc - rc @ np.array(row["origin"]["pos"])))


def cervical_sites(rows: list[dict[str, Any]]) -> None:
    """Origins on the neck: the same world point at the rest posture, in the HYOID vertebra and in the rigid neck."""
    for model_name, key in (("myofullbody", "hyoid"), ("myoarms", "rigid")):
        model = myo_sim.load_model(model_name)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        rc, pc = body_transform(model, data, "chest_r")
        for row in rows:
            if row["name"] not in CERVICAL:
                continue
            body = CERVICAL[row["name"]] if key == "hyoid" else "neck"
            rb, pb = body_transform(model, data, COMPOSED.get(body, body))
            p = np.array(row["origin"]["pos"])
            row["origin"][key] = {
                "body": body,
                "pos": _r(rb.T @ (pc + rc @ p - pb)),
                "pos_l": _r(rb.T @ (pc + rc @ (p * [1, 1, -1]) - pb)),
            }


TENSION_NOTE = (
    "the shoulder value calibrated to isometric shoulder moments (scripts/fit_shoulder_strength.py, shoulder_strength.json)"
)


def derive(zf: zipfile.ZipFile) -> dict[str, Any]:
    seth = seth_muscles(zf)
    shift = np.array(ST["registration"]["shift_seth_thorax_to_chest_r"])
    wraps = {**seth_wraps(zf), **fitted_thorax_spheres()}
    tension = json.loads((DATA_DIR / "shoulder_strength.json").read_text())["fit"]["result"]["value_N_per_cm2"]
    rows = []
    for name, sname, src, wrap in ROWS:
        s = seth[sname]
        (b1, p1), (b2, p2) = s["points"]
        p1 = p1 + (shift if b1 == "thorax" else 0)
        p2 = p2 + (shift if b2 == "thorax" else 0)
        row: dict[str, Any] = {
            "name": name,
            "seth": sname,
            "origin": {"body": BODY[b1], "pos": _r(p1)},
            "insertion": {"body": BODY[b2], "pos": _r(p2)},
            "wrap": wrap,
            "seth_parameters": {"F0": s["F"], "l0": s["l0"], "lts": s["lts"]},
        }
        if src == "klein_breteler":
            # Seth's F0 = Klein Breteler 1999 PCSA x 100 N/cm2 (DAS3) x 1.4, i.e. PCSA x 140 N/cm2
            row["pcsa_cm2"] = round(s["F"] / 140.0, 3)
            row["pcsa_source"] = "Klein Breteler et al. 1999 via DAS3 (Chadwick 2014): Seth's F0 / 140 N/cm2"
            row["l0"], row["lts"], row["alpha_deg"] = s["l0"], s["lts"], 0.0
            row["length_source"] = "Seth 2019"
            if name == "TRAP1":
                # Seth's row repeats TRAP2's l0 and slack; Klein Breteler's clavicular elements (DAS3: l0 0.155/0.168 m) belong
                # to longer, occipital paths and would leave this path's fibres below 0.5 l0, so Seth's values stay
                row["length_source"] = "Seth 2019 (same as TRAP2's; Klein Breteler's 0.155-0.168 m does not fit this path)"
        else:
            pcsa, lf, alpha = WARD2006[src]
            parts = [r for r in ROWS if r[2] == src]
            share = s["F"] / sum(seth[p[1]]["F"] for p in parts)
            row["pcsa_cm2"] = round(pcsa * share, 3)
            row["pcsa_source"] = f"Ward et al. 2006 {src} {pcsa} cm2 x Seth's share of the muscle ({share:.3f})"
            row["l0"], row["alpha_deg"] = lf, alpha
            row["lts"] = round(s["l0"] + s["lts"] - lf, 5)
            row["length_source"] = "Ward 2006 fibre length; tendon slack keeps Seth's optimal muscle-tendon length"
        row["force_N"] = round(row["pcsa_cm2"] * tension, 1)
        row["fpmax"] = FPMAX[src]
        rows.append(row)
    neck_lines(rows, tension)
    cervical_sites(rows)
    return {
        "format": "myo_sim.scapular_muscles/1",
        "description": "Scapular muscles and rotator-cuff sub-regions after Seth et al. 2019; scripts/generate_scapular_muscles.py",
        "specific_tension_N_per_cm2": tension,
        "specific_tension_note": TENSION_NOTE,
        "gain_tail_lmin_lmax_vmax_fpmax_fvmax": list(GAIN_TAIL),
        "wraps": wraps,
        "muscles": rows,
    }


# ----------------------------------------------------------------------------------------------- rendering
def _f(v: Any) -> str:
    return " ".join(f"{x:.6g}" for x in np.ravel(v))


def _site(name: str, pos: Any) -> str:
    return f'<site name="{name}" pos="{_f(pos)}"/>'


def _geom(name: str, w: dict[str, Any], flip: bool = False) -> str:
    pos, quat = np.array(w["pos"], float), np.array(w["quat"], float)
    if flip:
        pos, quat = pos * [1, 1, -1], quat * [1, -1, -1, 1]
    return f'<geom class="{{cls}}" name="{name}" type="{w["type"]}" size="{_f(w["size"])}" pos="{_f(pos)}" quat="{_f(quat)}"/>'


def site_name(row: dict[str, Any], k: int) -> str:
    return f"{row['name']}-P{k}_r"


def blocks(table: dict[str, Any]) -> dict[tuple[Path, str], str]:
    rows, wraps = table["muscles"], table["wraps"]
    pad = {"arm": " " * 16, "torso": " " * 22, "head": " " * 12}
    out: dict[tuple[Path, str], list[str]] = {}

    def add(path: Path, label: str, line: str, indent: str) -> None:
        out.setdefault((path, label), []).append(indent + line)

    for row in rows:
        for k, end in ((1, row["origin"]), (2, row["insertion"])):
            name = site_name(row, k)
            if end["body"] == "chest_r":
                if "hyoid" in end:
                    add(NECK, end["hyoid"]["body"], _site(name, end["hyoid"]["pos"]), pad["head"])
                    add(RIGID_HEAD, "neck", _site(name, end["rigid"]["pos"]), pad["head"])
                    add(RIGID_HEAD, "neck", _site(name[:-2] + "_l", end["rigid"]["pos_l"]), pad["head"])
                else:
                    add(TORSO, "chest_r", _site(name, end["pos"]), pad["torso"])
                    add(TORSO, "chest_r", _site(name[:-2] + "_l", np.array(end["pos"]) * [1, 1, -1]), pad["torso"])
            else:
                add(CHAIN, end["body"], _site(name, end["pos"]), pad["arm"])
    for key, w in wraps.items():
        if w["body"] == "chest_r":
            for side, flip in (("r", False), ("l", True)):
                add(TORSO, "chest_r", _geom(f"{key}_{side}", w, flip).format(cls="myotorso_wrap"), pad["torso"])
        else:
            add(CHAIN, w["body"], _geom(f"{key}_r", w).format(cls="myoarm_wrap"), pad["arm"])
            if "side_site" in w:
                add(CHAIN, w["body"], _site(f"{key}_sidesite_r", w["side_site"]), pad["arm"])
    tendon, muscle = [], []
    for row in rows:
        path = [f'<site site="{site_name(row, 1)}"/>']
        if row["wrap"]:
            side = f' sidesite="{row["wrap"]}_sidesite_r"' if "side_site" in wraps[row["wrap"]] else ""
            path.append(f'<geom geom="{row["wrap"]}_r"{side}/>')
        path.append(f'<site site="{site_name(row, 2)}"/>')
        tendon.append(f'        <spatial name="{row["name"]}_r_tendon" class="myoarm_upperarm_muscle">')
        tendon += ["            " + p for p in path] + ["        </spatial>"]
        m = row.get("map", {"range": [0.5, 1.5], "lengthrange": [0.05, 0.3]})
        tail = list(table["gain_tail_lmin_lmax_vmax_fpmax_fvmax"])
        tail[3] = row["fpmax"]
        gain = _f([*m["range"], row["force_N"], 1.0, *tail])
        muscle.append(
            f'        <general name="{row["name"]}_r" tendon="{row["name"]}_r_tendon" biasprm="{gain}" '
            f'class="myoarm_upperarm_muscle" gainprm="{gain}" lengthrange="{_f(m["lengthrange"])}"/>'
        )
    out[(TENDON, "shoulder")] = tendon
    out[(MUSCLE, "shoulder")] = muscle
    return {key: "\n".join(lines) + "\n" for key, lines in out.items()}


def write(table: dict[str, Any]) -> None:
    for (path, label), block in blocks(table).items():
        write_generated_block(path, SCRIPT, block, "", label=label)


# ----------------------------------------------------------------------------------------------- maps
def our_qpos(model: mujoco.MjModel, seth: Seth, q: np.ndarray, axial: float = 0.0) -> np.ndarray:
    """qpos of the arm for one of Seth's frames (girdle and glenohumeral; the muscles here cross neither elbow nor forearm).

    *axial* is the clavicle's axial rotation, which Seth's model lacks; it turns the clavicle about the line through the
    AC joint centre, so the scapula keeps Seth's pose and the AC ball takes the difference."""
    adr = {
        j: model.jnt_qposadr[model.joint(j).id]
        for j in ("sternoclavicular_r2_r", "sternoclavicular_r3_r", "sternoclavicular_r1_r", "acromioclavicular_r")
    }
    qp = model.qpos0.copy()
    qp[adr["sternoclavicular_r2_r"]], qp[adr["sternoclavicular_r3_r"]] = q[0], q[1]
    qp[adr["sternoclavicular_r1_r"]] = axial
    ra = axis_rot(model.jnt_axis[model.joint("sternoclavicular_r1_r").id], axial)
    rc, _ = seth.clavicle(q)
    rs, _ = seth.scapula(q)
    qs = Rotation.from_quat(np.roll(ST["girdle"]["scapula"]["quat"], -1)).as_matrix()
    a = adr["acromioclavicular_r"]
    qp[a : a + 4] = np.roll(Rotation.from_matrix(qs.T @ ra.T @ rc.T @ rs).as_quat(), 1)
    gh = seth.g["gh_axes"]
    rh_rel = axis_rot(gh[0], q[6]) @ axis_rot(gh[1], q[7]) @ axis_rot(gh[2], q[8])  # humerus in the scapula frame
    h0 = Rotation.from_quat(np.roll(ST["girdle"]["glenohumeral_frame"]["quat"], -1)).as_matrix()
    target = h0.T @ rh_rel
    ay = model.jnt_axis[model.joint("elv_angle_r").id]
    ax = model.jnt_axis[model.joint("shoulder_elv_r").id]
    ar = model.jnt_axis[model.joint("shoulder_rot_r").id]

    def res(x: np.ndarray) -> np.ndarray:
        r = axis_rot(ay, x[0]) @ axis_rot(ax, x[1]) @ axis_rot(ay, -x[0]) @ axis_rot(ar, x[2])
        return Rotation.from_matrix(target.T @ r).as_rotvec()

    best = min((least_squares(res, x0) for x0 in ([0.0, 0.3, 0.0], [1.0, 0.8, 0.0], [-0.8, 0.8, 0.5])), key=lambda r: r.cost)
    for j, v in zip(("elv_angle_r", "shoulder_elv_r", "shoulder_rot_r"), best.x):
        qp[model.jnt_qposadr[model.joint(j).id]] = v
    qp[model.jnt_qposadr[model.joint("shoulder1_r2_r").id]] = -best.x[0]
    return qp


def seth_frames(zf: zipfile.ZipFile, model: mujoco.MjModel, step: int = 3, info: bool = False) -> Any:
    """qpos for every assembled IK frame of the archive, the clavicle's axial angle from Ludewig 2009 (fit_scapulothoracic).
    With *info*, also each frame's trial name and humerothoracic elevation (deg)."""
    seth = Seth(read_osim(zf))
    trials = ik_trials(zf, seth, step)
    ht0 = ST["girdle"]["clavicle"]["rest_humerothoracic_elevation_deg"]
    out, names, elevation = [], [], []
    for name, tr in trials.items():
        plane = "flexion" if name.startswith("flx") else "abduction"
        for q in tr["q"]:
            ht = humerothoracic_elevation(seth, q)
            out.append(our_qpos(model, seth, q, clavicle_axial_ik(plane, ht, ht0)))
            names.append(name)
            elevation.append(ht)
    return (np.array(out), names, np.array(elevation)) if info else np.array(out)


def gh_sweeps(model: mujoco.MjModel, n: int = 41) -> np.ndarray:
    """Each glenohumeral coordinate over its range at the rest girdle, the others at qpos0 (shoulder_elv at 0.5 rad)."""
    out = []
    for j in ("elv_angle_r", "shoulder_elv_r", "shoulder_rot_r"):
        lo, hi = model.jnt_range[model.joint(j).id]
        for v in np.linspace(lo, hi, n):
            qp = model.qpos0.copy()
            qp[model.jnt_qposadr[model.joint("shoulder_elv_r").id]] = 0.5
            qp[model.jnt_qposadr[model.joint(j).id]] = v
            if j == "elv_angle_r":
                qp[model.jnt_qposadr[model.joint("shoulder1_r2_r").id]] = -v
            out.append(qp)
    return np.array(out)


def lengths(model: mujoco.MjModel, qpos: np.ndarray, names: list[str]) -> np.ndarray:
    data = mujoco.MjData(model)
    ids = [model.tendon(f"{n}_r_tendon").id for n in names]
    out = np.zeros((len(qpos), len(names)))
    for k, q in enumerate(qpos):
        data.qpos[:] = q
        mujoco.mj_fwdPosition(model, data)
        out[k] = data.ten_length[ids]
    return out


def fit_maps(table: dict[str, Any], zf: zipfile.ZipFile) -> None:
    """Rigid-tendon map per muscle along Seth's motions and the rest posture (qpos0, arm hanging); the slack length moves
    only if the fibre would leave 0.5-1.5 L0 there."""
    model = myo_sim.load_model("myoarm_r")
    names = [r["name"] for r in table["muscles"]]
    frames = lengths(model, np.vstack([seth_frames(zf, model), model.qpos0[None]]), names)
    sweeps = lengths(model, gh_sweeps(model), names)
    lmin, lmax = table["gain_tail_lmin_lmax_vmax_fpmax_fvmax"][:2]
    for j, row in enumerate(table["muscles"]):
        path, alpha = frames[:, j], np.deg2rad(row["alpha_deg"])
        lts = row["lts"]
        fib = (path - lts) / row["l0"]
        if fib.min() < 0.5 or fib.max() > 1.5:
            span = fib.max() - fib.min()
            shift = (fib.min() - 0.5) if fib.min() < 0.5 and span <= 1.0 else (fib.max() - 1.5) if span <= 1.0 else 0.0
            if span > 1.0:
                shift = (fib.min() + fib.max()) / 2 - 1.0
            lts = round(lts + shift * row["l0"], 5)
            row["lts_adjusted_from"] = row["lts"]
            row["lts"] = lts
        fib = np.sqrt(np.maximum(path - lts, 0) ** 2 + (row["l0"] * np.sin(alpha)) ** 2) / row["l0"]
        r0, r1, l0, l1 = fit_rigid_muscle_map(path, row["l0"], lts, alpha, lmin, lmax, span_length=sweeps[:, j])
        row["map"] = {"range": _r([r0, r1]), "lengthrange": _r([l0, l1])}
        row["fibre_length_seth_motions"] = _r([fib.min(), fib.max()], 3)
        row["path_length_seth_motions_mm"] = _r(1e3 * np.array([path.min(), path.max()]), 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--zip", type=Path, help="ThoracoscapularShoulderPaperMaterials.zip from SimTK (for --fit)")
    parser.add_argument("--fit", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    if args.fit:
        with zipfile.ZipFile(args.zip) as zf:
            table = derive(zf)
            write(table)
            fit_maps(table, zf)
        TABLE.write_text(json.dumps(table, indent=2) + "\n")
        write(table)
        print("wrote", TABLE)
    elif args.write:
        write(json.loads(TABLE.read_text()))
    for row in json.loads(TABLE.read_text())["muscles"]:
        print(
            f"{row['name']:7s} F {row['force_N']:7.1f}  l0 {row['l0']:.4f} lts {row['lts']:.4f}"
            f"  fibre {row.get('fibre_length_seth_motions')}  map {row.get('map')}"
        )


if __name__ == "__main__":
    main()
