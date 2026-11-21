#!/usr/bin/env python3
"""Scapulothoracic shoulder after Seth et al. 2019: frame registration and the joint-design comparison.

Reads the published SimTK "thoracoscapular" archive (release FrontiersPubMaterials, CC BY 4.0) directly; nothing from
it is committed except the derived values in ``myo_sim/build/data/scapulothoracic.json``
(licence: ``myo_sim/build/data/LICENSE-Seth2019.txt``)::

    uv run python scripts/fit_scapulothoracic.py --zip ThoracoscapularShoulderPaperMaterials.zip register [--write]
    uv run python scripts/fit_scapulothoracic.py --zip ThoracoscapularShoulderPaperMaterials.zip compare
    uv run python scripts/fit_scapulothoracic.py --zip ThoracoscapularShoulderPaperMaterials.zip rhythm [--write]

**register** checks that Seth's clavicle, scapula and humerus meshes and frames equal this package's, places Seth's
thorax-frame values in ``chest_r`` (shifted by the sternoclavicular offset between the two models) and writes the table.

**compare** replays every inverse-kinematics trial of the archive in two candidate girdles built in Seth's frames:
A (sternoclavicular hinges, acromioclavicular ball joint, scapula-thorax contact at Seth's ellipsoid point) and
B (Seth's ellipsoid parameterisation as a hinge/slide chain on the thorax, AC as a ``connect`` equality). The IK
coordinates belong to Seth's scaled subject model; they are first projected onto the generic model (least change of the
girdle coordinates that closes the AC constraint, as OpenSim's assembler does).

**rhythm** computes ISB clavicle, scapula and humerus angles (Wu et al. 2005) of the model along the measured motions
and compares them with bone-pin and radiographic studies (table section ``rhythm``).
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import re
import time
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
DATA_PATH = ROOT / "myo_sim" / "build" / "data" / "scapulothoracic.json"
MESHES = ROOT / "myo_sim" / "models" / "meshes"
PREFIX = "ThoracoscapularShoulderPaperMaterials/"
GENERIC = PREFIX + "Model/ThoracoscapularShoulderModel.osim"
GIRDLE = ("clav_prot", "clav_elev", "scapula_abduction", "scapula_elevation", "scapula_upward_rot", "scapula_winging")
GH = ("plane_elv", "shoulder_elv", "axial_rot")
RHO = 0.003  # radius of the scapular contact sphere (design A)


# ----------------------------------------------------------------------------------------------- archive readers
def vec(text: str | None) -> np.ndarray:
    return np.array((text or "").split(), float)


def read_osim(zf: zipfile.ZipFile, name: str = GENERIC) -> dict[str, Any]:
    """Girdle geometry of Seth's model: frames, ellipsoid, AC points, sternoclavicular axes."""
    root = ET.fromstring(zf.read(name))
    joints = {j.get("name"): j for j in root.iter() if j.tag.endswith("Joint") and j.get("name")}

    def frames(j: ET.Element) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        return {
            f.get("name"): (vec(f.findtext("translation")), vec(f.findtext("orientation")))
            for f in j.iter("PhysicalOffsetFrame")
        }

    stj, sc, gh = joints["scapulothoracic"], joints["sternoclavicular"], joints["GlenoHumeral"]
    ac = next(c for c in root.iter("PointConstraint") if c.get("name") == "AC")
    sc_axes = {
        t.findtext("coordinates").strip(): vec(t.findtext("axis"))
        for t in sc.find("SpatialTransform")
        if t.findtext("coordinates")
    }
    gh_axes = {
        t.findtext("coordinates").strip(): vec(t.findtext("axis"))
        for t in gh.find("SpatialTransform")
        if t.findtext("coordinates")
    }
    f_stj = frames(stj)
    return {
        "ellipsoid_radii": vec(stj.findtext("thoracic_ellipsoid_radii_x_y_z")),
        "ellipsoid_pos": f_stj["thorax_offset"][0],
        "ellipsoid_euler": f_stj["thorax_offset"][1],
        "scapula_joint_pos": f_stj["scapula_offset"][0],
        "scapula_joint_euler": f_stj["scapula_offset"][1],
        "winging_origin": vec(stj.findtext("scapula_winging_axis_origin")),
        "winging_direction": float(stj.findtext("scapula_winging_axis_direction")),
        "sc_pos": frames(sc)["thorax_offset"][0],
        "sc_axes": (sc_axes["clav_prot"], sc_axes["clav_elev"]),
        "gh_pos": frames(gh)["scapula_offset"][0],
        "gh_axes": tuple(gh_axes[c] for c in GH),
        "ac_clavicle": vec(ac.findtext("location_body_1")),
        "ac_scapula": vec(ac.findtext("location_body_2")),
    }


def read_vtp_points(zf: zipfile.ZipFile, name: str) -> np.ndarray:
    text = zf.read(name).decode()
    match = re.search(r"<Points>.*?<DataArray[^>]*>(.*?)</DataArray>", text, re.S)
    return np.array(match.group(1).split(), float).reshape(-1, 3)


def read_stl_points(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    n = int(np.frombuffer(raw[80:84], np.uint32)[0])
    rec = np.frombuffer(raw[84 : 84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return np.unique(np.round(rec["v"].reshape(-1, 3).astype(float), 7), axis=0)


def read_mot(zf: zipfile.ZipFile, name: str) -> tuple[list[str], np.ndarray]:
    lines = zf.read(name).decode().splitlines()
    i = next(k for k, line in enumerate(lines) if line.strip() == "endheader")
    head = [h.strip() for h in lines[i + 1].split("\t")]
    return head, np.loadtxt(io.StringIO("\n".join(lines[i + 2 :])))


# ----------------------------------------------------------------------------------------------- Seth's kinematics
def rx(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_xyz(e: np.ndarray) -> np.ndarray:
    """OpenSim body-fixed X-Y-Z orientation."""
    return rx(e[0]) @ ry(e[1]) @ rz(e[2])


def axis_rot(axis: np.ndarray, t: float) -> np.ndarray:
    return Rotation.from_rotvec(np.asarray(axis) / np.linalg.norm(axis) * t).as_matrix()


class Seth:
    """Forward kinematics of Seth's clavicle and scapula in the thorax frame (generic model)."""

    def __init__(self, g: dict[str, Any]):
        self.g = g
        self.r_ell = euler_xyz(g["ellipsoid_euler"])
        self.r_sj = euler_xyz(g["scapula_joint_euler"])
        # The winging axis is scapula-frame y through the joint point in the archive's model; the formula relies on it.
        assert np.allclose(g["winging_origin"], 0) and g["winging_direction"] == 0

    def clavicle(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return axis_rot(self.g["sc_axes"][0], q[0]) @ axis_rot(self.g["sc_axes"][1], q[1]), self.g["sc_pos"]

    def scapula(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """ScapulothoracicJoint: R = Ry(a) Rx(-e) Rz(u) Ry(w), joint point (rx sin a cos e, ry sin e, rz cos a cos e)."""
        a, e, u, w = q[2:6]
        r = self.g["ellipsoid_radii"]
        rm = ry(a) @ rx(-e) @ rz(u) @ ry(w)
        pm = np.array([r[0] * np.sin(a) * np.cos(e), r[1] * np.sin(e), r[2] * np.cos(a) * np.cos(e)])
        rs = self.r_ell @ rm @ self.r_sj.T
        return rs, self.g["ellipsoid_pos"] + self.r_ell @ pm - rs @ self.g["scapula_joint_pos"]

    def ac_gap(self, q: np.ndarray) -> np.ndarray:
        rc, pc = self.clavicle(q)
        rs, ps = self.scapula(q)
        return pc + rc @ self.g["ac_clavicle"] - ps - rs @ self.g["ac_scapula"]

    def assemble(self, q_ik: np.ndarray, q0: np.ndarray | None = None) -> np.ndarray:
        """Least change of the girdle coordinates that closes the AC constraint."""
        res = minimize(
            lambda q: np.sum((q - q_ik) ** 2),
            q_ik if q0 is None else q0,
            jac=lambda q: 2 * (q - q_ik),
            constraints={"type": "eq", "fun": self.ac_gap},
            method="SLSQP",
            options={"ftol": 1e-14, "maxiter": 200},
        )
        return res.x

    def ellipsoid_dist(self, p: np.ndarray) -> np.ndarray:
        """Signed distance of thorax-frame points to the ellipsoid surface (negative inside), Eberly bisection."""
        x = np.atleast_2d((np.asarray(p) - self.g["ellipsoid_pos"]) @ self.r_ell)
        a2 = self.g["ellipsoid_radii"] ** 2
        lo, hi = np.full(len(x), -a2.min() + 1e-12), np.full(len(x), 1.0)
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            f = np.sum(a2 * x**2 / (a2 + mid[:, None]) ** 2, axis=1) - 1
            lo, hi = np.where(f > 0, mid, lo), np.where(f > 0, hi, mid)
        t = 0.5 * (lo + hi)
        return np.sign(t) * np.linalg.norm(a2 * x / (a2 + t[:, None]) - x, axis=1)


def ik_trials(zf: zipfile.ZipFile, seth: Seth, step: int = 3) -> dict[str, dict[str, np.ndarray]]:
    """Every IK trial of the archive (every ``step``-th frame) projected onto the generic model."""
    out = {}
    for name in sorted(n for n in zf.namelist() if n.startswith(PREFIX + "Results/IK/") and n.endswith("_IK.mot")):
        head, data = read_mot(zf, name)
        rows = data[::step]
        q_ik = np.deg2rad(rows[:, [head.index(c) for c in GIRDLE + GH]])
        q, prev = [], None
        for row in q_ik:
            girdle = seth.assemble(row[:6], prev)
            q.append(np.r_[girdle, row[6:]])
            prev = girdle
        out[Path(name).name[:-7]] = {"t": rows[:, 0], "q_ik": q_ik, "q": np.array(q)}
    return out


# ----------------------------------------------------------------------------------------------- registration
def our_frames() -> dict[str, np.ndarray]:
    """Sternoclavicular centre and ribcage mesh origin in ``chest_r`` of the composed arm models."""
    import myo_sim

    model = myo_sim.load_model("myoarm_r")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    chest = model.body("chest_r").id
    rc, pc = data.xmat[chest].reshape(3, 3), data.xpos[chest]
    rib = model.geom("torso_geom_13").id
    mesh = model.geom_dataid[rib]
    vert = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    world = data.geom_xpos[rib] + vert @ data.geom_xmat[rib].reshape(3, 3).T
    return {"sc": rc.T @ (data.xpos[model.body("clavicle_r").id] - pc), "ribcage": (world - pc) @ rc}


def register(zf: zipfile.ZipFile) -> dict[str, Any]:
    g = read_osim(zf)
    seth = Seth(g)
    meshes = {}
    for name in ("thorax", "clavicle", "scapula", "humerus"):
        p = read_vtp_points(zf, PREFIX + f"Model/Geometry/{name}.vtp")
        q = read_stl_points(MESHES / f"{name}.stl")
        d_pq = cKDTree(q).query(p)[0]
        meshes[name] = {
            "seth_vertices": len(p),
            "ours_vertices": len(q),
            "seth_to_ours_mean_mm": 1e3 * d_pq.mean(),
            "seth_to_ours_max_mm": 1e3 * d_pq.max(),
        }
    ours = our_frames()
    shift = ours["sc"] - g["sc_pos"]
    # ribcage: translation that puts Seth's thorax mesh (placed by the sternoclavicular shift) onto this package's ribcage
    seth_rib, tree, offset = (
        read_vtp_points(zf, PREFIX + "Model/Geometry/thorax.vtp") + shift,
        cKDTree(ours["ribcage"]),
        np.zeros(3),
    )
    for _ in range(50):
        idx = tree.query(seth_rib + offset)[1]
        offset = np.mean(ours["ribcage"][idx] - seth_rib, axis=0)
    rib_resid = tree.query(seth_rib + offset)[0]

    def chest(p: np.ndarray) -> list[float]:
        return [round(float(v), 7) for v in p + shift]

    r_ell = seth.r_ell
    return {
        "meshes": meshes,
        "sc_ours_in_chest_r": [round(float(v), 7) for v in ours["sc"]],
        "sc_seth_in_thorax": [round(float(v), 7) for v in g["sc_pos"]],
        "shift_seth_thorax_to_chest_r": [round(float(v), 7) for v in shift],
        "ribcage_offset_from_seth_mm": [round(1e3 * float(v), 2) for v in offset],
        "ribcage_offset_norm_mm": round(1e3 * float(np.linalg.norm(offset)), 2),
        "ribcage_residual_after_offset_mm_mean_max": [
            round(1e3 * float(rib_resid.mean()), 2),
            round(1e3 * float(rib_resid.max()), 2),
        ],
        "thorax_ellipsoid": {
            "pos_chest_r": chest(g["ellipsoid_pos"]),
            "quat_chest_r": [round(float(v), 7) for v in np.roll(Rotation.from_matrix(r_ell).as_quat(), 1)],
            "radii": [round(float(v), 7) for v in g["ellipsoid_radii"]],
        },
        "scapula_joint_point": [round(float(v), 7) for v in g["scapula_joint_pos"]],
        "scapula_joint_quat": [round(float(v), 7) for v in np.roll(Rotation.from_matrix(seth.r_sj).as_quat(), 1)],
        "ac_clavicle": [round(float(v), 7) for v in g["ac_clavicle"]],
        "ac_scapula": [round(float(v), 7) for v in g["ac_scapula"]],
        "glenohumeral_centre_scapula": [round(float(v), 7) for v in g["gh_pos"]],
        "sternoclavicular_axes": [[round(float(v), 7) for v in a] for a in g["sc_axes"]],
    }


# ----------------------------------------------------------------------------------------------- prototypes A and B
def _f(v: Any) -> str:
    return " ".join(f"{x:.9g}" for x in np.ravel(v))


def _quat(r: np.ndarray) -> np.ndarray:
    return np.roll(Rotation.from_matrix(r).as_quat(), 1)


def slide_polys(seth: Seth, emax: float = 0.6) -> tuple[np.ndarray, np.ndarray, float]:
    """Quartic polynomials of B's two slides in the elevation angle; with rx = rz the joint point in the elevated frame
    is (0, (ry - rz) sin e cos e, rz + (ry - rz) sin^2 e)."""
    r = seth.g["ellipsoid_radii"]
    yz = r[1] - r[2]
    e = np.linspace(-emax, emax, 401)
    fy, fz = yz * np.sin(e) * np.cos(e), yz * np.sin(e) ** 2
    py, pz = np.polyfit(e, fy, 4)[::-1], np.polyfit(e, fz, 4)[::-1]
    err = max(np.abs(np.polyval(py[::-1], e) - fy).max(), np.abs(np.polyval(pz[::-1], e) - fz).max())
    return py, pz, float(err)


def prototype_xml(seth: Seth, design: str, solref: str = "0.008 1") -> str:
    """Thorax, clavicle, scapula, humerus and a lumped forearm in Seth's frames, girdle A or B."""
    g = seth.g
    gh = f"""
          <body name="humerus" pos="{_f(g["gh_pos"])}">
            <inertial mass="1.99757" pos="0.018064 -0.140141 -0.012746" diaginertia="0.0128576 0.0121576 0.00239265"/>
            <joint name="plane_elv" axis="{_f(g["gh_axes"][0])}" range="-1.6 2.3"/>
            <joint name="shoulder_elv" axis="{_f(g["gh_axes"][1])}" range="-0.8 3.1"/>
            <joint name="axial_rot" axis="{_f(g["gh_axes"][2])}" range="-1.6 1.6"/>
            <body name="forearm" pos="0.0061 -0.2904 -0.0123">
              <inertial mass="1.6" pos="0.005 -0.13 0.01" diaginertia="0.012 0.011 0.002"/>
            </body>
          </body>"""
    scap = '<inertial mass="0.70396" pos="-0.054694 -0.035032 -0.043734" diaginertia="0.00199576 0.00103993 0.001"/>'
    sites = f'<site name="ac_scap" pos="{_f(g["ac_scapula"])}"/><site name="stj" pos="{_f(g["scapula_joint_pos"])}"/>'
    imp = 'solimp="0.95 0.99 0.001 0.5 2"'
    child = chain = thorax = contact = eq = ""
    if design == "A":
        child = f"""
        <body name="scapula" pos="{_f(g["ac_clavicle"] - g["ac_scapula"])}">
          {scap}
          <joint name="ac" type="ball" pos="{_f(g["ac_scapula"])}" range="0 1.6"/>
          <geom name="stj_ball" type="sphere" size="{RHO}" pos="{_f(g["scapula_joint_pos"])}"/>
          {sites}{gh}
        </body>"""
        thorax = (
            f'<geom name="st_ell" type="ellipsoid" size="{_f(g["ellipsoid_radii"] - RHO)}" pos="{_f(g["ellipsoid_pos"])}"'
            f' quat="{_f(_quat(seth.r_ell))}"/>'
        )
        contact = f'<pair geom1="st_ell" geom2="stj_ball" condim="1" solref="{solref}" {imp}/>'
    else:
        py, pz, _ = slide_polys(seth)
        r = g["ellipsoid_radii"]
        chain = f"""
      <body name="st" pos="{_f(g["ellipsoid_pos"] + seth.r_ell @ np.array([0, 0, r[2]]))}" quat="{_f(_quat(seth.r_ell))}">
        <inertial mass="0.001" pos="0 0 0" diaginertia="1e-6 1e-6 1e-6"/>
        <joint name="st_abd" axis="0 1 0" pos="0 0 {-r[2]:.9g}" range="-1.2 0.6"/>
        <joint name="st_elev" axis="-1 0 0" pos="0 0 {-r[2]:.9g}" range="-0.6 0.6"/>
        <joint name="st_sy" type="slide" axis="0 1 0" limited="false"/>
        <joint name="st_sz" type="slide" axis="0 0 1" limited="false"/>
        <joint name="st_uprot" axis="0 0 1" range="-0.5 1.8"/>
        <joint name="st_wing" axis="0 1 0" range="-0.8 0.8"/>
        <body name="scapula" pos="{_f(-seth.r_sj.T @ g["scapula_joint_pos"])}" quat="{_f(_quat(seth.r_sj.T))}">
          {scap}{sites}{gh}
        </body>
      </body>"""
        eq = (
            f'<joint joint1="st_sy" joint2="st_elev" polycoef="{_f(py)}"/>'
            f'<joint joint1="st_sz" joint2="st_elev" polycoef="{_f(pz)}"/>'
            f'<connect site1="ac_clav" site2="ac_scap" solref="{solref}" {imp}/>'
        )
    return f"""<mujoco model="girdle_{design}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 -9.81 0" integrator="Euler"/>
  <default><joint damping="1.05" armature="1e-4"/><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="thorax">
      {thorax}
      <body name="clavicle" pos="{_f(g["sc_pos"])}">
        <inertial mass="0.156" pos="-0.011096 0.0063723 0.054168" diaginertia="0.001 0.001 0.001"/>
        <joint name="clav_prot" axis="{_f(g["sc_axes"][0])}" range="-1.2 0.4"/>
        <joint name="clav_elev" axis="{_f(g["sc_axes"][1])}" range="-0.6 0.8"/>
        <site name="ac_clav" pos="{_f(g["ac_clavicle"])}"/>{child}
      </body>{chain}
    </body>
  </worldbody>
  <contact>{contact}</contact>
  <equality>{eq}</equality>
</mujoco>"""


def prototype_qpos(model: mujoco.MjModel, seth: Seth, design: str, q: np.ndarray) -> np.ndarray:
    qpos = model.qpos0.copy()
    adr = {model.joint(j).name: model.jnt_qposadr[j] for j in range(model.njnt)}
    qpos[adr["clav_prot"]], qpos[adr["clav_elev"]] = q[0], q[1]
    for name, v in zip(GH, q[6:9]):
        qpos[adr[name]] = v
    if design == "A":
        qpos[adr["ac"] : adr["ac"] + 4] = _quat(seth.clavicle(q)[0].T @ seth.scapula(q)[0])
    else:
        py, pz, _ = slide_polys(seth)
        for name, v in zip(("st_abd", "st_elev", "st_uprot", "st_wing"), q[2:6]):
            qpos[adr[name]] = v
        qpos[adr["st_sy"]], qpos[adr["st_sz"]] = np.polyval(py[::-1], q[3]), np.polyval(pz[::-1], q[3])
    return qpos


def replay_kinematic(seth: Seth, trials: dict[str, dict[str, np.ndarray]], design: str) -> dict[str, float]:
    model = mujoco.MjModel.from_xml_string(prototype_xml(seth, design))
    data = mujoco.MjData(model)
    b, ac1, ac2 = model.body("scapula").id, model.site("ac_clav").id, model.site("ac_scap").id
    ep, er, gap, cd = [], [], [], []
    for tr in trials.values():
        for q in tr["q"]:
            data.qpos[:] = prototype_qpos(model, seth, design, q)
            mujoco.mj_forward(model, data)
            rs, ps = seth.scapula(q)
            ep.append(np.linalg.norm(data.xpos[b] - ps))
            er.append(np.linalg.norm(Rotation.from_matrix(data.xmat[b].reshape(3, 3).T @ rs).as_rotvec()))
            gap.append(np.linalg.norm(data.site_xpos[ac1] - data.site_xpos[ac2]))
            if design == "A":
                cd.append(data.contact.dist[0] if data.ncon else np.nan)
    out = {
        "frames": len(ep),
        "pos_max_mm": 1e3 * max(ep),
        "rot_max_deg": float(np.rad2deg(max(er))),
        "ac_gap_max_mm": 1e3 * max(gap),
    }
    if design == "A":
        out["contact_dist_range_mm"] = [1e3 * float(np.nanmin(cd)), 1e3 * float(np.nanmax(cd))]
    return out


def replay_dynamic(
    seth: Seth,
    trials: dict[str, dict[str, np.ndarray]],
    design: str,
    k: float = 500.0,
    k_clav: float = 50.0,
    press: float = 20.0,
) -> dict[str, float]:
    """Joint springs pull every coordinate to Seth's trajectory under gravity, with an inward press on the scapula at the
    ellipsoid point (a stand-in for serratus and rhomboid tone); the girdle's own constraint carries the rest."""
    model = mujoco.MjModel.from_xml_string(prototype_xml(seth, design))
    data = mujoco.MjData(model)
    held = ["clav_prot", "clav_elev", *GH] + (["ac"] if design == "A" else ["st_abd", "st_elev", "st_uprot", "st_wing"])
    for name in held:
        j = model.joint(name).id
        kj = k_clav if name.startswith("clav") else k
        model.jnt_stiffness[j] = kj
        model.dof_damping[model.jnt_dofadr[j] : model.jnt_dofadr[j] + (3 if name == "ac" else 1)] = 0.4 * kj**0.5
    b, stj = model.body("scapula").id, model.site("stj").id
    ac1, ac2 = model.site("ac_clav").id, model.site("ac_scap").id
    ep, er, gap, pen, lift, wall, nstep, unstable = [], [], [], [], [], 0.0, 0, False
    settle, dt = 0.5, model.opt.timestep
    for tr in trials.values():
        t, qs = tr["t"], tr["q"]
        targets = np.array([prototype_qpos(model, seth, design, q) for q in qs])
        mujoco.mj_resetData(model, data)
        data.qpos[:] = targets[0]
        mujoco.mj_forward(model, data)
        for i in range(int((t[-1] - t[0] + settle) / dt)):
            tt = t[0] + max(0.0, i * dt - settle)
            kk = min(int(np.searchsorted(t, tt)), len(t) - 1)
            a = 0.0 if kk == 0 else (tt - t[kk - 1]) / (t[kk] - t[kk - 1])
            spring = targets[kk] if kk == 0 else (1 - a) * targets[kk - 1] + a * targets[kk]
            if design == "A":
                ja = model.jnt_qposadr[model.joint("ac").id]
                spring[ja : ja + 4] /= np.linalg.norm(spring[ja : ja + 4])
            model.qpos_spring[:] = spring
            data.qfrc_applied[:] = 0
            x = (data.site_xpos[stj] - seth.g["ellipsoid_pos"]) @ seth.r_ell
            n = seth.r_ell @ (x / seth.g["ellipsoid_radii"] ** 2)
            mujoco.mj_applyFT(
                model, data, -press * n / np.linalg.norm(n), np.zeros(3), data.site_xpos[stj].copy(), b, data.qfrc_applied
            )
            w0 = time.perf_counter()
            mujoco.mj_step(model, data)
            wall, nstep = wall + time.perf_counter() - w0, nstep + 1
            if not np.all(np.isfinite(data.qpos)) or np.abs(data.qvel).max() > 1e3:
                unstable = True
                break
            if i * dt >= settle and i % 5 == 0:
                q = qs[kk if a > 0.5 or kk == 0 else kk - 1]
                rs, ps = seth.scapula(q)
                ep.append(np.linalg.norm(data.xpos[b] - ps))
                er.append(np.linalg.norm(Rotation.from_matrix(data.xmat[b].reshape(3, 3).T @ rs).as_rotvec()))
                gap.append(np.linalg.norm(data.site_xpos[ac1] - data.site_xpos[ac2]))
                if design == "A":
                    dist = data.contact.dist[0] if data.ncon else np.inf
                    pen.append(-min(dist, 0.0))
                    lift.append(dist > 0.5e-3)

    def pct(x: list[float], s: float) -> list[float]:
        return [round(float(v), 2) for v in s * np.percentile(x, [50, 95, 100])] if len(x) else []

    out = {
        "unstable": unstable,
        "pos_err_mm_p50_p95_max": pct(ep, 1e3),
        "rot_err_deg_p50_p95_max": pct(np.rad2deg(er), 1.0),
        "ac_gap_mm_p50_p95_max": pct(gap, 1e3),
        "us_per_step": round(1e6 * wall / max(nstep, 1), 1),
    }
    if design == "A":
        out["penetration_mm_p50_p95_max"] = pct(pen, 1e3)
        out["lift_off_percent"] = round(100 * float(np.mean(lift)), 1) if lift else None
    return out


def compare(zf: zipfile.ZipFile, step: int) -> dict[str, Any]:
    seth = Seth(read_osim(zf))
    trials = ik_trials(zf, seth, step)
    moved = np.rad2deg(max(np.abs(tr["q"][:, :6] - tr["q_ik"][:, :6]).max() for tr in trials.values()))
    gap = max(np.abs([seth.ac_gap(q) for q in tr["q"]]).max() for tr in trials.values())
    print(
        f"{len(trials)} trials, {sum(len(tr['q']) for tr in trials.values())} frames; projection moved girdle coordinates "
        f"by up to {moved:.1f} deg, AC residual {1e3 * gap:.2e} mm; B slide polynomial error {1e3 * slide_polys(seth)[2]:.3f} mm"
    )
    result: dict[str, Any] = {}
    for design in "AB":
        result[design] = {"kinematic": replay_kinematic(seth, trials, design)}
        for k_clav in (50.0, 500.0):
            result[design][f"dynamic_k500_kclav{int(k_clav)}"] = replay_dynamic(seth, trials, design, k_clav=k_clav)
        print(design, json.dumps(result[design], indent=1))
    return result


# ----------------------------------------------------------------------------------------------- girdle A in the model
CLAV_RANGES = ((-1.047, 0.0), (-0.175, 0.611))  # protraction, elevation (rad): Seth's data -49..-12 and -8..+16 deg, widened
AC_RANGE = 1.4  # rad, cone about the rest orientation; Seth's data reach 68.4 deg
# Residual joint springs to the rest posture (N m/rad) next to the ligaments (scripts/fit_ligaments.py): round values
# that keep every girdle angle within 5 deg of rest at zero activation; the ligaments hold protraction, none can hold
# elevation or the AC tilt (levers in docs/wiki/shoulder-complex.md). The axial clavicle hinge takes the AC value: an
# isotropic AC spring alone turns the clavicle with the scapula, and the passive equilibrium along Seth's motions then
# reaches about 1.7 times Ludewig 2009's posterior rotation; equal springs at both joints put it within 4.5 deg of it
# (scripts/fit_ligaments.py --axial).
GIRDLE_STIFFNESS = {"sternoclavicular": [0.0, 30.0], "axial": 35.0, "acromioclavicular": 35.0}
# Clavicle axial rotation: a third sternoclavicular hinge about the line from the sternoclavicular centre to the AC
# joint centre (the ISB clavicle z axis), applied after protraction and elevation, positive = posterior rotation
# (Ludewig 2009's sign). Seth's model has none. Range: -15 to +50 deg; Ludewig 2009 (bone pins) reach 30-32 deg
# posterior rotation at the end of elevation, Sahara 2007 (MRI) 33 deg at maximal abduction, textbook totals 20-50 deg;
# anterior rotation of a few degrees with protraction (Peeters 2023). No coupling: the coracoclavicular and AC
# ligaments, the clavicular muscles and a residual spring (below) set it.
CLAV_AXIAL_RANGE = (-0.262, 0.873)
# Seth's IK has no axial clavicle angle; the replay takes it from Ludewig et al. 2009 Fig. 5A (bone pins, n = 11,
# digitised, raising): posterior rotation against humerothoracic elevation (deg), coronal abduction for the abduction
# and shrug trials, flexion for the flexion trials; the last point is the end of range, the value is held beyond it.
# The angle is taken relative to the curve's value at the rest posture's elevation, the rest posture being axial zero.
CLAV_AXIAL_IK = {
    "abduction": ((15.0, 30.0, 60.0, 90.0, 120.0, 136.0), (1.2, 3.0, 8.8, 15.4, 24.5, 31.0)),
    "flexion": ((15.0, 30.0, 60.0, 90.0, 120.0, 131.0), (-1.2, 0.0, 6.8, 14.0, 24.5, 30.0)),
}


def humerothoracic_elevation(seth: Seth, q: np.ndarray) -> float:
    """Angle (deg) between the humeral shaft (humerus y) and the thorax vertical, Seth's coordinates *q*."""
    gh = seth.g["gh_axes"]
    rh = seth.scapula(q)[0] @ axis_rot(gh[0], q[6]) @ axis_rot(gh[1], q[7]) @ axis_rot(gh[2], q[8])
    return float(np.degrees(np.arccos(np.clip(rh[1, 1], -1.0, 1.0))))


def clavicle_axial_ik(plane: str, elevation: float, rest_elevation: float) -> float:
    """Clavicle posterior rotation (rad) for a humerothoracic elevation (deg), relative to the rest posture."""
    x, y = CLAV_AXIAL_IK[plane]
    return float(np.deg2rad(np.interp(elevation, x, y) - np.interp(rest_elevation, x, y)))


# AC armature (kg m2): the smallest value keeping the explicit force-velocity rate at full activation below 5
# (myo_sim.build.stability; 0 gives 7.3, 0.004 gives 5.2); the dynamic stability criteria pass without it
AC_ARMATURE = 0.005


def glenoid(path: Path = MESHES / "scapula.stl", radius: float = 0.035, cos_min: float = 0.85) -> dict[str, list[float]]:
    """Glenoid fossa of the scapula mesh: area-weighted centre and normal of the faces near the glenohumeral centre
    that face it. The site's z axis is the normal, its x axis the scapula's y axis made orthogonal to it."""
    raw = path.read_bytes()
    n = int(np.frombuffer(raw[80:84], np.uint32)[0])
    rec = np.frombuffer(raw[84 : 84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    v = rec["v"].astype(float)
    c = v.mean(1)
    nrm = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    area = np.linalg.norm(nrm, axis=1) / 2
    nrm /= np.linalg.norm(nrm, axis=1)[:, None]
    gh = np.array(json.loads(DATA_PATH.read_text())["registration"]["glenohumeral_centre_scapula"])
    r = gh - c
    d = np.linalg.norm(r, axis=1)
    sel = (d < radius) & (np.sum(nrm * r, axis=1) / d > cos_min)
    w = area[sel]
    centre = (c[sel] * w[:, None]).sum(0) / w.sum()
    z = (nrm[sel] * w[:, None]).sum(0)
    z /= np.linalg.norm(z)
    x = np.array([0.0, 1.0, 0.0]) - z[1] * z
    x /= np.linalg.norm(x)
    rot = np.column_stack([x, np.cross(z, x), z])
    return {
        "pos": [round(float(t), 6) for t in centre],
        "quat": [round(float(t), 6) for t in _quat(rot)],
        "area_cm2": round(1e4 * float(w.sum()), 2),
        "centre_to_glenohumeral_mm": round(1e3 * float(np.linalg.norm(gh - centre)), 1),
    }


def girdle(zf: zipfile.ZipFile, step: int = 3) -> dict[str, Any]:
    """Design A in the model's frames, with the rest posture of Seth's trials as ``qpos0``.

    Rest: the mean first frame of the 18 trials (arm at the side), assembled. The sternoclavicular hinges keep Seth's
    (and MoBL-ARMS's) axes and angles; their ``ref`` is the rest angle, so the clavicle body carries the rest rotation and
    the first axis is written in the rotated frame. The scapula's body frame is its rest orientation relative to the
    clavicle (identity of the AC ball joint), and the glenohumeral chain starts in a frame fixed on the scapula that equals
    the thorax frame at rest, so the glenohumeral angles are relative to the scapula and zero with the arm hanging.
    """
    g = read_osim(zf)
    seth = Seth(g)
    trials = ik_trials(zf, seth, step)
    rest = np.mean([tr["q"][0] for tr in trials.values()], axis=0)
    rest[:6] = seth.assemble(rest[:6])
    axial = g["ac_clavicle"] / np.linalg.norm(g["ac_clavicle"])  # the clavicle frame's origin is the SC centre
    a1, a2 = (np.asarray(a) / np.linalg.norm(a) for a in g["sc_axes"])
    r1, r2 = axis_rot(a1, rest[0]), axis_rot(a2, rest[1])
    rc, _ = seth.clavicle(rest)
    rs, ps = seth.scapula(rest)
    rel = rc.T @ rs
    dev = [
        np.linalg.norm(Rotation.from_matrix(rel.T @ seth.clavicle(q)[0].T @ seth.scapula(q)[0]).as_rotvec())
        for tr in trials.values()
        for q in tr["q"]
    ]
    jp = float(seth.ellipsoid_dist(ps + rs @ g["scapula_joint_pos"])[0])

    def r6(x: np.ndarray) -> list[float]:
        return [round(float(t), 6) for t in np.ravel(x)]

    return {
        "rest_posture_deg": dict(zip(GIRDLE, [round(float(t), 2) for t in np.rad2deg(rest[:6])])),
        "rest_joint_point_to_ellipsoid_mm": round(1e3 * jp, 4),
        "clavicle": {
            "quat": r6(_quat(r1 @ r2)),
            "axis_sternoclavicular_r2": r6(r2.T @ a1),
            "axis_sternoclavicular_r3": r6(a2),
            "ref": r6(rest[:2]),
            "stiffness": GIRDLE_STIFFNESS["sternoclavicular"],
            "range": [list(r) for r in CLAV_RANGES],
            "axis_sternoclavicular_r1": r6(axial),
            "axial_range": list(CLAV_AXIAL_RANGE),
            "axial_stiffness": GIRDLE_STIFFNESS["axial"],
            "axial_ik_ludewig2009_deg": {k: [list(v) for v in xy] for k, xy in CLAV_AXIAL_IK.items()},
            "rest_humerothoracic_elevation_deg": round(humerothoracic_elevation(seth, rest), 2),
        },
        "scapula": {
            "pos": r6(g["ac_clavicle"] - rel @ g["ac_scapula"]),
            "quat": r6(_quat(rel)),
            "ac_joint_pos": r6(g["ac_scapula"]),
            "ac_range": AC_RANGE,
            "ac_stiffness": GIRDLE_STIFFNESS["acromioclavicular"],
            "ac_armature": AC_ARMATURE,
            "ac_data_max_deg": round(float(np.rad2deg(max(dev))), 1),
            "contact_sphere_pos": r6(g["scapula_joint_pos"]),
            "contact_sphere_radius": RHO,
        },
        "glenohumeral_frame": {"pos": r6(g["gh_pos"]), "quat": r6(_quat(rs.T))},
        "glenoid": glenoid(),
        "thorax_ellipsoid_geom_radii": r6(g["ellipsoid_radii"] - RHO),
    }


# ----------------------------------------------------------------------------------------------- scapulohumeral rhythm
RHYTHM_JOINTS = (
    "sternoclavicular_r2_r", "sternoclavicular_r3_r", "sternoclavicular_r1_r", "acromioclavicular_r",
    "elv_angle_r", "shoulder_elv_r", "shoulder1_r2_r", "shoulder_rot_r",
)  # fmt: skip
RHYTHM_BINS = (30.0, 60.0, 90.0, 120.0)  # humerothoracic elevation (deg)
# Wu et al. 2005 (ISB) landmarks. Seth's generic model carries a marker for each (skin markers, 2-15 mm off the bone);
# the bony landmarks are the mesh vertex nearest to that marker, the joint centres are the model's.
ISB_LANDMARKS = {
    "IJ": ("thorax", "ij", "incisura jugularis: Seth's marker ij (skin), in chest_r by the sternoclavicular shift"),
    "C7": ("thorax", "c7", "spinous process of C7: Seth's marker c7 (skin), in chest_r by the sternoclavicular shift"),
    "PX": ("thorax", "px", "processus xiphoideus: Seth's marker px (skin), in chest_r by the sternoclavicular shift"),
    "T8": ("thorax", "t8", "spinous process of T8: Seth's marker t8 (skin), in chest_r by the sternoclavicular shift"),
    "SC": ("clavicle", None, "sternoclavicular joint centre (clavicle_r origin; Seth's marker sc)"),
    "AC": ("clavicle", None, "acromioclavicular joint centre (Seth's AC constraint point, centre of acromioclavicular_r)"),
    "AA": ("scapula", "aa", "angulus acromialis: scapula mesh vertex nearest to Seth's marker aa"),
    "TS": ("scapula", "ts", "trigonum spinae: scapula mesh vertex nearest to Seth's marker ts"),
    "AI": ("scapula", "ai", "angulus inferior: scapula mesh vertex nearest to Seth's marker ai"),
    "GH": ("humerus", None, "glenohumeral rotation centre (humerus_r origin; Seth's marker gu)"),
    "EL": ("humerus", "EpL", "lateral epicondyle: humerus mesh vertex nearest to Seth's marker EpL"),
    "EM": ("humerus", "EpM", "medial epicondyle: humerus mesh vertex nearest to Seth's marker EpM"),
}
ANGLES = (
    "humerothoracic_elevation", "humerothoracic_plane", "scapula_upward_rotation", "scapula_internal_rotation",
    "scapula_posterior_tilt", "clavicle_elevation", "clavicle_retraction", "clavicle_posterior_rotation",
    "glenohumeral_elevation",
)  # fmt: skip
LUDEWIG = (
    "Ludewig PM, Phadke V, Braman JP, Hassett DR, Cieminski CJ, LaPrade RF. Motion of the shoulder complex during "
    "multiplanar humeral elevation. J Bone Joint Surg Am 91(2):378-389, 2009 (doi:10.2106/JBJS.G.01483)"
)
MCCLURE = (
    "McClure PW, Michener LA, Sennett BJ, Karduna AR. Direct 3-dimensional measurement of scapular kinematics during "
    "dynamic movements in vivo. J Shoulder Elbow Surg 10(3):269-277, 2001 (doi:10.1067/mse.2001.112954)"
)
BRAMAN = (
    "Braman JP, Engel SC, LaPrade RF, Ludewig PM. In vivo assessment of scapulohumeral rhythm during unconstrained "
    "overhead reaching in asymptomatic subjects. J Shoulder Elbow Surg 18(6):960-967, 2009 (doi:10.1016/j.jse.2009.02.001)"
)
INMAN = (
    "Inman VT, Saunders JB, Abbott LC. Observations on the function of the shoulder joint. J Bone Joint Surg 26(1):1-30, "
    "1944, as reported in McClure et al. 2001 (discussion table)"
)
PROTOCOL = {
    "ludewig": "12 adults (SC 11), bone pins, electromagnetic tracking, ISB angles, standing, active, unloaded, raising",
    "mcclure": "8 adults, scapular bone pins, electromagnetic tracking, the authors' own scapular axes (same Euler order)",
    "braman": "12 adults (Ludewig 2009 cohort), bone pins, self-selected overhead reach (63 deg forward of coronal)",
    "inman": "radiographs, subjects and spread not stated",
}
# Literature rows: (source, location, plane, model plane, angle or ratio, HT (deg, "rest", "max", "change 30-120",
# "change 30-max" or "30-120" for a ratio), mean, SD or None). Ludewig Fig. 5 is digitised (+-1 deg) and shows SE only.
# Ludewig 2009 Fig. 5A-D, digitised (+-1 deg, GH +-1.5 deg), raising: HT -> (flexion, scapular plane, coronal abduction)
LUDEWIG_FIG5 = {
    "scapula_upward_rotation": {
        30: (15.5, 15.5, 15.5),
        60: (25.5, 26.0, 26.5),
        90: (33.5, 34.5, 36.0),
        120: (43.5, 43.5, 46.0),
    },
    "scapula_internal_rotation": {
        30: (44.3, 38.4, 31.2),
        60: (46.8, 38.7, 30.5),
        90: (47.8, 38.9, 30.7),
        120: (43.5, 37.2, 30.6),
    },
    "scapula_posterior_tilt": {30: (-10.4, -11.2, -10.5), 60: (-5.9, -6.8, -6.0), 90: (-1.6, -2.9, -1.7), 120: (5.4, 2.7, 3.9)},
    "clavicle_retraction": {30: (20.5, 24.8, 31.4), 60: (22.7, 29.0, 37.3), 90: (25.0, 32.2, 40.6), 120: (31.6, 36.5, 44.1)},
    "clavicle_elevation": {30: (10.1, 11.6, 14.3), 60: (12.1, 14.3, 16.8), 90: (13.5, 15.6, 18.1), 120: (14.5, 17.0, 19.5)},
    "clavicle_posterior_rotation": {30: (0.0, 1.8, 3.0), 60: (6.8, 8.0, 8.8), 90: (14.0, 14.7, 15.4), 120: (24.5, 24.5, 24.5)},
    "glenohumeral_elevation": {30: (14.0, 14.5, 9.0), 60: (33.0, 33.0, 30.5), 90: (56.0, 56.5, 54.0), 120: (77.0, 77.0, 74.5)},
}
FIG5 = "Fig. 5A-D, digitised"
SLOPE = "text (regression slope GH on ST upward rotation)"
INMAN_TABLE = "McClure 2001 discussion table"
BRAMAN_MIN, BRAMAN_MAX = (
    "self-selected reach, minimum elevation (about 15 deg)",
    "self-selected reach, maximum (132.9 +- 9.9 deg)",
)
# Literature rows: source, location, plane, Seth trials compared, angle or "gh_st", elevation, mean, SD (None: not published).
# Elevation: an HT bin (deg), "rest" (qpos0), "start" / "max" (first / highest frame of a trial), "change a-b" (difference
# between two bins) or an interval for GH:ST.
RHYTHM_LIT: tuple[tuple[str, str, str, str, str, Any, float, float | None], ...] = (
    *(
        ("ludewig", "Table I (mean +- SE; SD = SE x sqrt(n))", "standing, arm at the side", "rest", a, "rest", m, sd)
        for a, m, sd in (
            ("clavicle_retraction", 19.2, 6.6),
            ("clavicle_elevation", 5.9, 3.3),
            ("clavicle_posterior_rotation", 0.1, None),
            ("scapula_internal_rotation", 41.1, 6.9),
            ("scapula_upward_rotation", 5.4, 3.5),
            ("scapula_posterior_tilt", -13.5, 6.9),
            ("glenohumeral_elevation", 0.8, 3.5),
        )
    ),
    *(
        ("ludewig", FIG5, plane, trials, a, ht, values[col], None)
        for plane, trials, col in (("coronal abduction", "abduction", 2), ("flexion", "flexion", 0))
        for a, rows in LUDEWIG_FIG5.items()
        for ht, values in rows.items()
    ),
    ("ludewig", SLOPE, "coronal abduction", "abduction", "gh_st", "30-120", 2.1, None),
    ("ludewig", SLOPE, "flexion", "flexion", "gh_st", "30-120", 2.4, None),
    *(
        (
            "ludewig",
            "text (start to end of elevation, planes pooled)",
            "all three planes",
            trials,
            a,
            "change start-max",
            m,
            None,
        )
        for trials in ("abduction", "flexion")
        for a, m in (
            ("scapula_upward_rotation", 39.0),
            ("scapula_posterior_tilt", 21.0),
            ("clavicle_retraction", 16.0),
            ("clavicle_elevation", 6.0),
            ("clavicle_posterior_rotation", 31.0),
        )
    ),
    *(
        ("mcclure", "Figs 3-4, digitised (mean +- SEM)", "flexion", "flexion", a, "change 30-120", m, None)
        for a, m in (
            ("scapula_upward_rotation", 30.7),
            ("scapula_posterior_tilt", 9.9),
            ("clavicle_retraction", 9.7),
            ("clavicle_elevation", 7.4),
        )
    ),
    *(
        (
            "mcclure",
            "abstract (minimum to maximum elevation, mean [SD])",
            "scapular plane",
            "abduction",
            a,
            "change start-max",
            m,
            sd,
        )
        for a, m, sd in (("scapula_upward_rotation", 50.0, 4.8), ("scapula_posterior_tilt", 30.0, 13.0))
    ),
    ("mcclure", "results (GH:ST from minimum elevation)", "flexion", "flexion", "gh_st", "start-max", 2.0, None),
    ("mcclure", "results (GH:ST from minimum elevation)", "scapular plane", "abduction", "gh_st", "start-max", 1.7, None),
    *(
        ("braman", "Table I (raising, mean +- SD)", "self-selected reach", trials, "gh_st", interval, m, sd)
        for trials in ("abduction", "flexion")
        for interval, m, sd in (("30-60", 2.36, 0.75), ("60-90", 2.71, 0.85), ("90-120", 2.50, 0.90))
    ),
    *(
        ("braman", "text (mean +- SD)", plane, "abduction", a, ht, m, sd)
        for plane, ht, rows in (
            (BRAMAN_MIN, "start", (("scapula_upward_rotation", 11.4, 5.8), ("scapula_internal_rotation", 33.2, 5.2))),
            (BRAMAN_MIN, "start", (("scapula_posterior_tilt", -11.8, 4.9), ("glenohumeral_elevation", -7.4, 5.0))),
            (BRAMAN_MAX, "max", (("scapula_upward_rotation", 48.6, 4.0), ("scapula_internal_rotation", 42.8, 8.7))),
            (BRAMAN_MAX, "max", (("scapula_posterior_tilt", 9.8, 7.5), ("glenohumeral_elevation", 96.9, 5.5))),
        )
        for a, m, sd in rows
    ),
    *(
        ("inman", INMAN_TABLE, f"{plane} 30-150 deg", plane, a, "change 30-max", m, None)
        for plane, values in (("abduction", (40.0, 30.0, 30.0)), ("flexion", (50.0, 25.0, 30.0)))
        for a, m in zip(("scapula_upward_rotation", "clavicle_elevation", "clavicle_posterior_rotation"), values)
    ),
    ("inman", "Inman 1944 (classic statement)", "abduction beyond 30 deg", "abduction", "gh_st", "30-max", 2.0, None),
)


def isb_landmarks(zf: zipfile.ZipFile, model: mujoco.MjModel) -> dict[str, dict[str, Any]]:
    """The ISB landmarks in this package's body frames (see ``ISB_LANDMARKS``)."""
    root = ET.fromstring(zf.read(GENERIC))
    markers = {m.get("name"): vec(m.findtext("location")) for m in root.iter("Marker")}
    shift = np.array(json.loads(DATA_PATH.read_text())["registration"]["shift_seth_thorax_to_chest_r"])
    body = {"thorax": "chest_r", "clavicle": "clavicle_r", "scapula": "scapula_r", "humerus": "humerus_r"}
    out = {}
    for name, (bone, marker, definition) in ISB_LANDMARKS.items():
        if bone == "thorax":
            pos = markers[marker] + shift
        elif marker is None:
            pos = read_osim(zf)["ac_clavicle"] if name == "AC" else np.zeros(3)
        else:
            b = model.body(body[bone]).id
            g = next(
                i for i in range(model.ngeom) if model.geom_bodyid[i] == b and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
            )
            mesh = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
            v = v @ Rotation.from_quat(np.roll(model.geom_quat[g], -1)).as_matrix().T + model.geom_pos[g]
            d = np.linalg.norm(v - markers[marker], axis=1)
            pos = v[np.argmin(d)]
            definition += f" ({1e3 * d.min():.1f} mm)"
        out[name] = {"body": body[bone], "pos": [round(float(t), 6) for t in pos], "definition": definition}
    # SC and AC fix only the clavicle's long axis; as with a bone-fixed sensor, its X axis (perpendicular to the long axis
    # and the thorax Y axis) is defined at the rest posture (qpos0) and then moves with the clavicle
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    th = {k: np.array(out[k]["pos"]) for k in ("IJ", "C7", "PX", "T8")}
    yt = data.xmat[model.body("chest_r").id].reshape(3, 3) @ ((th["IJ"] + th["C7"] - th["PX"] - th["T8"]) / 2)
    rc = data.xmat[model.body("clavicle_r").id].reshape(3, 3)
    zc = rc @ np.array(out["AC"]["pos"])
    xc = rc.T @ np.cross(yt, zc)
    out["CX"] = {
        "body": "clavicle_r",
        "pos": [round(float(t), 6) for t in 0.01 * xc / np.linalg.norm(xc)],
        "definition": "point 10 mm from SC along the ISB clavicle X axis at the rest posture (qpos0), fixed on the clavicle",
    }
    return out


def isb_angles(model: mujoco.MjModel, data: mujoco.MjData, landmarks: dict[str, dict[str, Any]]) -> dict[str, float]:
    """ISB angles (Wu et al. 2005, deg) at the current ``data`` (after ``mj_kinematics``), positive = the named direction.

    Thorax, clavicle, scapula and humerus frames from the landmarks; clavicle and scapula relative to the thorax as
    Y-X-Z Cardan angles (retraction, elevation, posterior axial rotation; internal rotation, upward rotation, posterior
    tilt), humerus elevation as the angle between the long axes (humerus Y against thorax Y or scapula Y), plane of
    elevation of the humeral shaft from the coronal plane (0 = abduction, 90 = flexion)."""
    p = {}
    for name, lm in landmarks.items():
        b = model.body(lm["body"]).id
        p[name] = data.xpos[b] + data.xmat[b].reshape(3, 3) @ np.array(lm["pos"])

    def unit(v: np.ndarray) -> np.ndarray:
        return v / np.linalg.norm(v)

    yt = unit((p["IJ"] + p["C7"]) / 2 - (p["PX"] + p["T8"]) / 2)
    zt = unit(np.cross(p["C7"] - p["IJ"], (p["PX"] + p["T8"]) / 2 - p["IJ"]))
    t = np.column_stack([np.cross(yt, zt), yt, zt])
    zc = unit(p["AC"] - p["SC"])
    xc = unit(p["CX"] - p["SC"] - ((p["CX"] - p["SC"]) @ zc) * zc)
    c = np.column_stack([xc, np.cross(zc, xc), zc])
    zs = unit(p["AA"] - p["TS"])
    xs = unit(np.cross(p["TS"] - p["AI"], p["AA"] - p["AI"]))
    s = np.column_stack([xs, np.cross(zs, xs), zs])
    yh = unit(p["GH"] - (p["EL"] + p["EM"]) / 2)
    cy, cx, cz = Rotation.from_matrix(t.T @ c).as_euler("YXZ", degrees=True)
    sy, sx, sz = Rotation.from_matrix(t.T @ s).as_euler("YXZ", degrees=True)
    shaft = -(t.T @ yh)
    values = (
        np.degrees(np.arccos(np.clip(yh @ yt, -1, 1))), np.degrees(np.arctan2(shaft[0], shaft[2])), -sx, sy, sz,
        -cx, -cy, cz, np.degrees(np.arccos(np.clip(yh @ s[:, 1], -1, 1))),
    )  # fmt: skip
    return {k: float(v) for k, v in zip(ANGLES, values)}


def rhythm_frame_angles(model: mujoco.MjModel, landmarks: dict[str, Any], frames: dict[str, list[float]]) -> np.ndarray:
    """ISB angles (frames x ANGLES) of stored joint values (``RHYTHM_JOINTS``; the rest at qpos0)."""
    data = mujoco.MjData(model)
    out = []
    for k in range(len(frames[RHYTHM_JOINTS[0]])):
        data.qpos[:] = model.qpos0
        for j in RHYTHM_JOINTS:
            a = model.joint(j).qposadr[0]
            v = frames[j][k]
            data.qpos[a : a + np.size(v)] = v
        mujoco.mj_kinematics(model, data)
        out.append([isb_angles(model, data, landmarks)[a] for a in ANGLES])
    return np.array(out)


def rhythm_summary(model: mujoco.MjModel, landmarks: dict[str, Any], picks: dict[str, Any]) -> dict[str, Any]:
    """Model rows of the rhythm table from the picked frames: per plane and bin the median and range over trials,
    GH:ST ratios per trial (change of GH elevation over change of ST upward rotation), shrug clavicle elevation range."""

    def r2(x: Any) -> Any:
        return [round(float(v), 2) for v in x] if np.ndim(x) else round(float(x), 2)

    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    out: dict[str, Any] = {"rest": {a: r2(v) for a, v in isb_angles(model, data, landmarks).items()}}
    for plane in ("abduction", "flexion"):
        bins = {key: rhythm_frame_angles(model, landmarks, f) for key, f in picks[plane].items()}
        rows = {}
        for key, ang in bins.items():
            rows[key] = {"trials": len(ang)} | {
                a: {"median": r2(np.median(ang[:, i])), "range": r2([ang[:, i].min(), ang[:, i].max()])}
                for i, a in enumerate(ANGLES)
            }
        trials = {key: picks[plane][key]["trial"] for key in bins}
        ratio = {}
        ur, gh = ANGLES.index("scapula_upward_rotation"), ANGLES.index("glenohumeral_elevation")
        for lo, hi in (
            ("30", "60"),
            ("60", "90"),
            ("90", "120"),
            ("30", "120"),
            ("30", "max"),
            ("start", "max"),
        ):
            per = []
            for k, name in enumerate(trials[lo]):
                if name in trials[hi]:
                    d = bins[hi][trials[hi].index(name)] - bins[lo][k]
                    per.append(d[gh] / d[ur])
            ratio[f"{lo}-{hi}"] = {"median": r2(np.median(per)), "range": r2([min(per), max(per)]), "trials": len(per)}
        out[plane] = rows | {"gh_st": ratio}
    shrug = rhythm_frame_angles(model, landmarks, picks["shrug"])[:, ANGLES.index("clavicle_elevation")].reshape(-1, 2)
    span = np.ptp(shrug, axis=1)
    out["shrug"] = {
        "clavicle_elevation_min_max": {"median": r2(np.median(shrug, axis=0)), "range": r2([shrug.min(), shrug.max()])},
        "clavicle_elevation_range": {"median": r2(np.median(span)), "range": r2([span.min(), span.max()])},
        "trials": len(shrug),
    }
    return out


def rhythm_compare(model_rows: dict[str, Any]) -> list[dict[str, Any]]:
    """Literature rows with the model value next to them and the difference (deg or ratio, and in SD where published)."""
    cite = {"ludewig": LUDEWIG, "mcclure": MCCLURE, "braman": BRAMAN, "inman": INMAN}
    out = []
    for src, loc, plane, mp, quantity, ht, mean, sd in RHYTHM_LIT:
        if quantity == "gh_st":
            model = model_rows[mp]["gh_st"][ht]["median"]
        elif ht == "rest":
            model = model_rows["rest"][quantity]
        elif str(ht).startswith("change"):
            lo, hi = ht.split()[1].split("-")
            model = model_rows[mp][hi][quantity]["median"] - model_rows[mp][lo][quantity]["median"]
        else:
            model = model_rows[mp][ht if isinstance(ht, str) else str(int(ht))][quantity]["median"]
        row = {
            "citation": cite[src], "location": loc, "protocol": PROTOCOL[src], "plane": plane, "model_trials": mp,
            "quantity": quantity, "elevation_deg": ht, "mean": mean, "sd": sd, "model": round(model, 2),
            "difference": round(model - mean, 2),
        }  # fmt: skip
        if sd:
            row["difference_sd"] = round((model - mean) / sd, 2)
        out.append(row)
    return out


def rhythm(zf: zipfile.ZipFile, step: int = 3) -> dict[str, Any]:
    """Scapulohumeral rhythm of the model along Seth's measured motions against Ludewig 2009, McClure 2001, Braman 2009
    and Inman 1944 (ISB angles; table section ``rhythm``).

    The model's girdle reproduces every assembled IK frame exactly; the clavicle's axial angle follows Ludewig 2009
    (``clavicle_axial_ik``), so that row is a consistency check, not a prediction. Per trial (raising phase, up to the
    highest elevation) the frame nearest to each bin and the frame of highest elevation are picked; the shrug trials
    give their lowest and highest clavicle elevation. The picked joint values are stored, so the model rows can be
    recomputed from the table and the model alone."""
    from generate_scapular_muscles import seth_frames

    import myo_sim

    model = myo_sim.load_model("myoarm_r")
    landmarks = isb_landmarks(zf, model)
    qpos, names, _ = seth_frames(zf, model, step, info=True)
    data = mujoco.MjData(model)
    ang = []
    for q in qpos:
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        ang.append(isb_angles(model, data, landmarks))
    adr = {j: (model.joint(j).qposadr[0], 4 if j == "acromioclavicular_r" else 1) for j in RHYTHM_JOINTS}
    names = np.array(names)
    picks: dict[str, Any] = {"abduction": {}, "flexion": {}, "shrug": {"trial": []}}

    def add(store: dict[str, Any], name: str, k: int) -> None:
        store.setdefault("trial", []).append(name)
        for j, (a, n) in adr.items():
            v = qpos[k, a : a + n]
            store.setdefault(j, []).append(round(float(v[0]), 6) if n == 1 else [round(float(t), 6) for t in v])

    for name in sorted(set(names)):
        idx = np.flatnonzero(names == name)
        ht = np.array([ang[k]["humerothoracic_elevation"] for k in idx])
        if name.startswith("shrug"):
            el = [ang[k]["clavicle_elevation"] for k in idx]
            add(picks["shrug"], name, idx[int(np.argmin(el))])
            add(picks["shrug"], name, idx[int(np.argmax(el))])
            continue
        plane = "flexion" if name.startswith("flx") else "abduction"
        top = int(np.argmax(ht))
        add(picks[plane].setdefault("start", {}), name, idx[0])
        for b in RHYTHM_BINS:
            if ht[: top + 1].max() >= b:
                add(picks[plane].setdefault(str(int(b)), {}), name, idx[int(np.argmin(np.abs(ht[: top + 1] - b)))])
        add(picks[plane].setdefault("max", {}), name, idx[top])
    model_rows = rhythm_summary(model, landmarks, picks)
    return {
        "description": (
            "Scapulohumeral rhythm along Seth's 18 measured motions (assembled IK, every 3rd frame), ISB angles "
            "(Wu et al. 2005, J Biomech 38:981-992) in myoarm_r, against bone-pin and radiographic literature. "
            "Positive = the named direction. Per trial the raising-phase frame nearest to each humerothoracic "
            "elevation bin (deg) and the frame of highest elevation; median and range over trials. The clavicle's "
            "axial angle is taken from Ludewig 2009 in the replay, so its rows check consistency only."
        ),
        "landmarks": landmarks,
        "model": model_rows,
        "literature": rhythm_compare(model_rows),
        "frames": picks,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--zip", type=Path, required=True, help="ThoracoscapularShoulderPaperMaterials.zip from SimTK")
    sub = parser.add_subparsers(dest="cmd", required=True)
    reg = sub.add_parser("register", help="frame registration; --write updates the table")
    reg.add_argument("--write", action="store_true")
    cmp_ = sub.add_parser("compare", help="replay the IK trials in girdle designs A and B")
    cmp_.add_argument("--step", type=int, default=3, help="use every step-th IK frame")
    gir = sub.add_parser("girdle", help="design A in the model's frames, rest posture as qpos0; --write updates the table")
    gir.add_argument("--write", action="store_true")
    rhy = sub.add_parser("rhythm", help="scapulohumeral rhythm against the literature; --write updates the table")
    rhy.add_argument("--write", action="store_true")
    args = parser.parse_args()
    with zipfile.ZipFile(args.zip) as zf:
        if args.cmd == "register":
            values = register(zf)
            print(json.dumps(values, indent=1))
            if args.write:
                table = json.loads(DATA_PATH.read_text())
                table["registration"] = values
                table["source"]["archive_sha256"] = hashlib.sha256(args.zip.read_bytes()).hexdigest()
                DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
                print("wrote", DATA_PATH)
        elif args.cmd == "girdle":
            values = girdle(zf)
            print(json.dumps(values, indent=1))
            if args.write:
                table = json.loads(DATA_PATH.read_text())
                table["girdle"] = values
                DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
                print("wrote", DATA_PATH)
        elif args.cmd == "rhythm":
            values = rhythm(zf)
            print(json.dumps({k: values[k] for k in ("landmarks", "model", "literature")}, indent=1))
            if args.write:
                table = json.loads(DATA_PATH.read_text())
                table["rhythm"] = values
                DATA_PATH.write_text(json.dumps(table, indent=2) + "\n")
                print("wrote", DATA_PATH)
        else:
            compare(zf, args.step)


if __name__ == "__main__":
    main()
