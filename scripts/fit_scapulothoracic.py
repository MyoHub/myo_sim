#!/usr/bin/env python3
"""Scapulothoracic shoulder after Seth et al. 2019: frame registration and the joint-design comparison.

Reads the published SimTK "thoracoscapular" archive (release FrontiersPubMaterials, CC BY 4.0) directly; nothing from
it is committed except the derived values in ``myo_sim/build/data/scapulothoracic.json``
(licence: ``myo_sim/build/data/LICENSE-Seth2019.txt``)::

    uv run python scripts/fit_scapulothoracic.py --zip ThoracoscapularShoulderPaperMaterials.zip register [--write]
    uv run python scripts/fit_scapulothoracic.py --zip ThoracoscapularShoulderPaperMaterials.zip compare

**register** checks that Seth's clavicle, scapula and humerus meshes and frames equal this package's, places Seth's
thorax-frame values in ``chest_r`` (shifted by the sternoclavicular offset between the two models) and writes the table.

**compare** replays every inverse-kinematics trial of the archive in two candidate girdles built in Seth's frames:
A (sternoclavicular hinges, acromioclavicular ball joint, scapula-thorax contact at Seth's ellipsoid point) and
B (Seth's ellipsoid parameterisation as a hinge/slide chain on the thorax, AC as a ``connect`` equality). The IK
coordinates belong to Seth's scaled subject model; they are first projected onto the generic model (least change of the
girdle coordinates that closes the AC constraint, as OpenSim's assembler does).
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
# Joint springs to the rest posture (N m/rad), a stand-in for the sternoclavicular, acromioclavicular and coracoclavicular
# ligaments and resting muscle tone, none of which is modelled: the smallest round value that keeps the girdle within
# a few degrees of rest under the arm's weight at zero activation (no measured value; see docs/wiki/shoulder-complex.md)
GIRDLE_STIFFNESS = {"sternoclavicular": 40.0, "acromioclavicular": 40.0}
# AC armature (kg m2): keeps the explicit force-velocity rate of the scapula at full activation below 5
# (myo_sim.build.stability, as the thumb CMC armature); 0.0001 gives 7.2
AC_ARMATURE = 0.008


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
        else:
            compare(zf, args.step)


if __name__ == "__main__":
    main()
