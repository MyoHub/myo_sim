"""Functional validation of the hand: passive tenodesis, maximum grip and lateral pinch.

The protocols follow McFarland et al. 2023 (IEEE TBME 70(5):1424-1435, doi:10.1109/TBME.2022.3217722),
reimplemented in MuJoCo; postures, targets and tolerances are in ``build/data/hand_function.json``. The
scenes compose the right-arm model with locks, a dynamometer and contact pairs through MjSpec; no model
XML is edited.

The script uses whichever ``myo_sim`` is importable, so other model versions can be compared::

    PYTHONPATH=<myo_sim worktree> python scripts/validate_hand_function.py --label <name> --out results.json

The table is read next to this file, not from the imported package.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

TABLE_PATH = Path(__file__).resolve().parents[1] / "myo_sim" / "build" / "data" / "hand_function.json"
MUSCLE_PASSIVE_PATH = TABLE_PATH.with_name("muscle_passive.json")
DEG = np.pi / 180.0
DIGITS = (2, 3, 4, 5)
THUMB = ("cmc_flexion", "cmc_abduction", "mp_flexion", "ip_flexion")
ARM = ("elevation_plane", "shoulder_elevation", "shoulder_rotation", "elbow_flexion", "forearm_pronation")
WRIST = ("wrist_flexion", "wrist_radial_deviation")


def load_table(path: Path | str = TABLE_PATH) -> dict[str, Any]:
    """Read the protocol, target and tolerance table."""
    return json.loads(Path(path).read_text())


@dataclass(frozen=True)
class Hand:
    """Names and anatomical signs of a hand model.

    Attributes:
        load: Returns a fresh MjSpec of the model.
        joints: Anatomical key -> (joint name, sign); joint angle = sign x anatomical angle. Keys are ``ARM``,
            ``WRIST``, ``THUMB`` and ``mcp_flexion<d>``, ``mcp_abduction<d>``, ``pip_flexion<d>``, ``dip_flexion<d>``.
        segments: Digit -> (proximal, middle, distal) phalanx body names.
        metacarpal3: Body of the third metacarpal.
        thumb_distal: Body of the distal thumb phalanx.
        muscles: Group -> actuator names (wrist, thumb_extrinsic, EIP, EDM, FDS, FDP, EDC, finger_intrinsic, thumb_intrinsic).
    """

    load: Callable[[], mujoco.MjSpec]
    joints: dict[str, tuple[str, float]]
    segments: dict[int, tuple[str, str, str]]
    metacarpal3: str
    thumb_distal: str
    muscles: dict[str, tuple[str, ...]] = field(default_factory=dict)


def myohand() -> Hand:
    """The right hand of ``myoarm_r`` from the importable myo_sim (missing muscles are skipped)."""
    import myo_sim

    spec = myo_sim.load_spec("myoarm_r")
    names = {j.name for j in spec.joints}
    actuators = {a.name for a in spec.actuators}

    def joint(base: str) -> str:
        return next(c for c in (f"{base}_r", base) if c in names)

    def muscles(*bases: str) -> tuple[str, ...]:
        """Actuator names of the muscles *bases* present in the model (right side, ``_r``)."""
        return tuple(c for base in bases for c in (f"{base}_r", base) if c in actuators)

    signs = {"cmc_flexion": -1.0, "mp_flexion": -1.0, "ip_flexion": -1.0}  # MyoHand: +q extends these
    joints = {
        "elevation_plane": (joint("elv_angle"), 1.0),
        "shoulder_elevation": (joint("shoulder_elv"), 1.0),
        "shoulder_rotation": (joint("shoulder_rot"), 1.0),
        "elbow_flexion": (joint("elbow_flexion"), 1.0),
        "forearm_pronation": (joint("pro_sup"), 1.0),
        "wrist_flexion": (joint("flexion"), 1.0),
        "wrist_radial_deviation": (joint("deviation"), 1.0),
    }
    joints |= {key: (joint(key), signs.get(key, 1.0)) for key in THUMB}
    for d in DIGITS:
        for key, base in (("mcp_flexion", "mcp{}_flexion"), ("mcp_abduction", "mcp{}_abduction")):
            joints[f"{key}{d}"] = (joint(base.format(d)), 1.0)
        joints[f"pip_flexion{d}"] = (joint(f"pm{d}_flexion"), 1.0)
        joints[f"dip_flexion{d}"] = (joint(f"md{d}_flexion"), 1.0)
    groups = {
        "wrist": muscles("ECRL", "ECRB", "ECU", "FCR", "FCU", "PL"),
        "thumb_extrinsic": muscles("FPL", "EPL", "EPB", "APL"),
        "EIP": muscles("EIP"),
        "EDM": muscles("EDM"),
        "FDS": muscles(*(f"FDS{d}" for d in DIGITS)),
        "FDP": muscles(*(f"FDP{d}" for d in DIGITS)),
        "EDC": muscles(*(f"EDC{d}" for d in DIGITS)),
        "finger_intrinsic": muscles(*(f"{m}{d}" for d in DIGITS for m in ("RI", "LU_RB", "UI_UB"))),
        "thumb_intrinsic": muscles("OP", "AdP", "FPB", "APB"),
    }
    return Hand(
        load=lambda: myo_sim.load_spec("myoarm_r"),
        joints=joints,
        segments={2: ("2proxph_r", "midph2_r", "distph2_r")}
        | {d: (f"{d}proxph_r", f"midph{d}_r", f"distph{d}_r") for d in (3, 4, 5)},
        metacarpal3="thirdmc_r",
        thumb_distal="distal_thumb_r",
        muscles=groups,
    )


# --- scene helpers ------------------------------------------------------------------------------------


@dataclass
class Scene:
    """A compiled model with lock equalities (joint key -> equality id) and the hand it came from."""

    model: mujoco.MjModel
    hand: Hand
    locks: dict[str, int]
    colliders: frozenset[str]

    def collision_geoms(self, body: str) -> list[int]:
        """Geoms of *body* that collide in the original model (before the scene switched collisions off)."""
        b = self.model.body(body).id
        return [
            g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == b and self.model.geom(g).name in self.colliders
        ]

    def joint(self, key: str) -> tuple[int, float]:
        name, sign = self.hand.joints[key]
        return self.model.joint(name).id, sign

    def qadr(self, key: str) -> int:
        return int(self.model.jnt_qposadr[self.joint(key)[0]])

    def dof(self, key: str) -> int:
        return int(self.model.jnt_dofadr[self.joint(key)[0]])

    def angle(self, data: mujoco.MjData, key: str) -> float:
        """Anatomical angle (deg) of *key*."""
        return float(data.qpos[self.qadr(key)] / DEG * self.joint(key)[1])

    def set_lock(self, key: str, deg: float) -> None:
        """Move the lock of *key* to the anatomical angle *deg*."""
        jid, sign = self.joint(key)
        self.model.eq_data[self.locks[key], 0] = sign * deg * DEG - self.model.qpos0[self.model.jnt_qposadr[jid]]

    def pose(self, data: mujoco.MjData, angles: dict[str, float]) -> None:
        """Set anatomical angles (deg) in qpos."""
        for key, deg in angles.items():
            data.qpos[self.qadr(key)] = self.joint(key)[1] * deg * DEG

    def actuator(self, name: str) -> int:
        """Id of actuator *name*, given as the model name or as the muscle name without the side suffix."""
        try:
            return self.model.actuator(name).id
        except KeyError:
            return self.model.actuator(f"{name}_r").id


def build_scene(
    hand: Hand,
    locks: dict[str, float],
    table: dict[str, Any],
    edit_spec: Callable[[mujoco.MjSpec, Hand], None] | None = None,
) -> Scene:
    """Compile *hand* with every key of *locks* held by a joint equality at that anatomical angle (deg).

    Collisions are switched off (contype = conaffinity = 0); scenes that need contacts add explicit pairs
    in *edit_spec*.
    """
    spec = hand.load()
    lock = table["simulation"]["lock"]
    colliders = frozenset(g.name for g in spec.geoms if g.name and (g.contype or g.conaffinity))
    for geom in spec.geoms:
        geom.contype = geom.conaffinity = 0
    for key in locks:
        eq = spec.add_equality(
            type=mujoco.mjtEq.mjEQ_JOINT, objtype=mujoco.mjtObj.mjOBJ_JOINT, name1=hand.joints[key][0], name=f"lock_{key}"
        )
        eq.solref, eq.solimp = lock["solref"], lock["solimp"]
    if edit_spec is not None:
        edit_spec(spec, hand)
    model = spec.compile()
    scene = Scene(model, hand, {key: model.equality(f"lock_{key}").id for key in locks}, colliders)
    for key, deg in locks.items():
        scene.set_lock(key, deg)
    return scene


def start(scene: Scene, angles: dict[str, float]) -> mujoco.MjData:
    """Data at the anatomical *angles* (deg), every other joint at qpos0."""
    data = mujoco.MjData(scene.model)
    scene.pose(data, angles)
    mujoco.mj_forward(scene.model, data)
    return data


def set_activation(scene: Scene, data: mujoco.MjData, levels: dict[str, float]) -> None:
    """Set control and activation (steady state, no activation dynamics) of the named muscles; others to 0."""
    data.ctrl[:] = 0.0
    if scene.model.na:
        data.act[:] = 0.0
    for name, level in levels.items():
        a = scene.actuator(name)
        data.ctrl[a] = level
        if scene.model.actuator_actadr[a] >= 0:
            data.act[scene.model.actuator_actadr[a]] = level


def lock_force(scene: Scene, data: mujoco.MjData, key: str) -> float:
    """Generalised force (N m, anatomical sign) the lock of *key* applies to its joint."""
    rows = (data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (data.efc_id == scene.locks[key])
    return float(np.sum(data.efc_force[rows]) * scene.joint(key)[1])


def force_length(scene: Scene, data: mujoco.MjData, name: str) -> float:
    """MuJoCo's active force-length factor of muscle *name* at the current length."""
    m, a = scene.model, scene.actuator(name)
    prm = m.actuator_gainprm[a]
    peak = prm[2] if prm[2] > 0 else prm[3] / m.actuator_acc0[a]
    gain = mujoco.mju_muscleGain(data.actuator_length[a], 0.0, m.actuator_lengthrange[a], m.actuator_acc0[a], prm[:9])
    return float(-gain / peak)


# --- model mutations (negative controls) ----------------------------------------------------------------


def without_springs(model: mujoco.MjModel) -> None:
    """Remove the passive joint-moment springs (``passive_*`` fixed tendons)."""
    for t in range(model.ntendon):
        if model.tendon(t).name.startswith("passive_"):
            model.tendon_stiffness[t] = 0.0
            model.tendon_stiffnesspoly[t] = 0.0


def reference_slack(model: mujoco.MjModel) -> None:
    """Restore the extrinsic slack lengths and fpmax the passive fit started from (the #123 values)."""
    rows = json.loads(MUSCLE_PASSIVE_PATH.read_text())["muscles"]
    for row in rows:
        a = model.actuator(row["names"]["r"]).id
        lo, hi = model.actuator_lengthrange[a]
        r0, r1 = model.actuator_gainprm[a][:2]
        shift = row["tendon_slack_m"]["reference_value_m"] - (lo - r0 * (hi - lo) / (r1 - r0))
        model.actuator_lengthrange[a] += shift
        model.actuator_gainprm[a][7] = model.actuator_biasprm[a][7] = row["fpmax"]["reference_value"]


MUTATIONS: dict[str, Callable[[mujoco.MjModel], None]] = {
    "without_springs": without_springs,
    "reference_slack": reference_slack,
}


# --- 1. passive tenodesis -----------------------------------------------------------------------------


def tenodesis(
    hand: Hand, table: dict[str, Any], mutate: Callable[[mujoco.MjModel], None] | None = None
) -> dict[str, dict[str, np.ndarray]]:
    """Run trial A (flexion -> extension) and then trial B (extension -> flexion) with the wrist prescribed.

    Returns, per trial, time (s), wrist flexion (deg) and every finger flexion angle (deg), sampled at
    ``sample_dt`` over the hold and the sweep.
    """
    p = table["tenodesis"]
    free = {f"{k}{d}" for d in DIGITS for k in ("mcp_flexion", "pip_flexion", "dip_flexion")}
    held = [k for k in (*ARM, *WRIST, *THUMB) if k != "wrist_flexion"] + [f"mcp_abduction{d}" for d in DIGITS]
    angles = {k: p["posture_deg"].get(k, 0.0) for k in held} | {"wrist_flexion": p["wrist_deg"]}
    scene = build_scene(hand, angles, table)
    model = scene.model
    if mutate is not None:
        mutate(model)
    if not p["gravity"]:
        model.opt.gravity[:] = 0.0
    data = start(scene, angles)
    every = max(1, round(p["sample_dt"] / model.opt.timestep))
    trials = {}
    for trial, (a, b) in (("A", (p["wrist_deg"], -p["wrist_deg"])), ("B", (-p["wrist_deg"], p["wrist_deg"]))):
        sweep = abs(b - a) / p["speed_deg_s"]
        n = int(round((p["hold_s"] + sweep) / model.opt.timestep))
        rec: dict[str, list[float]] = {"time": [], "wrist_flexion": []} | {k: [] for k in sorted(free)}
        for i in range(n + 1):
            t = i * model.opt.timestep
            target = a + (b - a) * min(max(t - p["hold_s"], 0.0) / sweep, 1.0)
            scene.set_lock("wrist_flexion", target)
            if i % every == 0:
                rec["time"].append(t)
                rec["wrist_flexion"].append(scene.angle(data, "wrist_flexion"))
                for k in free:
                    rec[k].append(scene.angle(data, k))
            if i < n:
                mujoco.mj_step(model, data)
        trials[trial] = {k: np.asarray(v) for k, v in rec.items()}
    return trials


def tenodesis_metrics(trials: dict[str, dict[str, np.ndarray]], table: dict[str, Any]) -> dict[str, Any]:
    """ROM, direction and inter-digit spread of each trial over its sweep (hold excluded)."""
    hold = table["tenodesis"]["hold_s"]
    out: dict[str, Any] = {}
    for trial, rec in trials.items():
        s = rec["time"] >= hold
        # Trial A extends the wrist: the fingers should flex; trial B the reverse.
        sign = 1.0 if trial == "A" else -1.0
        res: dict[str, Any] = {"rom_deg": {}, "reversal_deg": {}, "net_deg": {}}
        for k in (k for k in rec if k not in ("time", "wrist_flexion")):
            q = rec[k][s]
            res["rom_deg"][k] = float(q.max() - q.min())
            res["net_deg"][k] = float(sign * (q[-1] - q[0]))
            flexing = sign * q
            res["reversal_deg"][k] = float(np.max(np.maximum.accumulate(flexing) - flexing))
        for joint in ("mcp_flexion", "pip_flexion", "dip_flexion"):
            q = np.stack([rec[f"{joint}{d}"][s] for d in DIGITS])
            res[f"mean_rom_{joint}"] = float(np.mean([res["rom_deg"][f"{joint}{d}"] for d in DIGITS]))
            res[f"spread_{joint}"] = float(np.max(q.max(axis=0) - q.min(axis=0)))
            index_gap = np.abs(q[0] - q[1:]).max(axis=0)
            res[f"index_gap_{joint}"] = float(index_gap.max())
            # Mean distance of each digit from the others: the digit that differs most.
            dist = [np.mean(np.abs(q[i] - np.delete(q, i, axis=0))) for i in range(len(DIGITS))]
            res[f"outlier_{joint}"] = DIGITS[int(np.argmax(dist))]
        out[trial] = res
    return out


def tenodesis_checks(metrics: dict[str, Any], table: dict[str, Any]) -> dict[str, bool]:
    """Pass/fail of each tenodesis tolerance in the table (trial A: extension, trial B: release)."""
    tol = table["tenodesis"]["tolerances"]
    a, b = metrics["A"], metrics["B"]
    lo, hi = tol["mcp_rom_deg"]
    out = {"direction": max(a["reversal_deg"].values()) <= tol["max_reversal_deg"]}
    out |= {f"mcp_rom{d}": lo <= a["rom_deg"][f"mcp_flexion{d}"] <= hi for d in DIGITS}
    lo, hi = tol["mean_pip_rom_deg"]
    out["mean_pip_rom"] = lo <= a["mean_rom_pip_flexion"] <= hi
    out["dip_constant"] = max(a["rom_deg"][f"dip_flexion{d}"] for d in DIGITS) <= tol["max_dip_rom_deg"]
    lo, hi = tol["mcp_spread_deg"]
    out["mcp_spread"] = lo <= a["spread_mcp_flexion"] <= hi
    release = [b["net_deg"][f"mcp_flexion{d}"] for d in DIGITS]
    out["release"] = (
        min(release) >= tol["release_min_mcp_extension_deg"] and np.mean(release) >= tol["release_mean_mcp_extension_deg"]
    )
    return out


# --- 2. maximum grip ------------------------------------------------------------------------------------


@dataclass
class GripScene:
    """Grip scene: the hand with a dynamometer on the third metacarpal and contact pairs to the phalanges."""

    scene: Scene
    dynamometer: int
    counted: dict[int, tuple[int, str]]  # geom id -> (digit, segment)
    start: mujoco.MjData


def _dynamometer_frame(scene: Scene, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """World palmar normal, cylinder axis (MCP 5 -> MCP 2, following the transverse arch) and MCP-3 position."""
    m, hand = scene.model, scene.hand
    prox, _, dist = (m.body(b).id for b in hand.segments[3])
    mcp3 = data.xpos[prox]
    jid, sign = scene.joint("mcp_flexion3")
    normal = np.cross(sign * data.xaxis[jid], data.xpos[dist] - mcp3)
    normal /= np.linalg.norm(normal)
    axis = data.xpos[m.body(hand.segments[2][0]).id] - data.xpos[m.body(hand.segments[5][0]).id]
    axis /= np.linalg.norm(axis)
    normal -= (normal @ axis) * axis
    return normal / np.linalg.norm(normal), axis, mcp3.copy()


def grip_scene(hand: Hand, table: dict[str, Any], wrist: dict[str, float]) -> GripScene:
    """Hand with the wrist held at *wrist* (anatomical deg), thumb and arm locked, and the dynamometer.

    The dynamometer is an elliptical prism (convex mesh) in the third-metacarpal body (welded, massless).
    Its axis runs through the MCP 5 and MCP 2 joint centres, its major axis along the palmar normal of the
    flat third digit. Along the digit it sits ``centre_distal_m`` from MCP 3 (its distal edge at the MCP
    head), across the digits midway between MCP 2 and 5, and it is moved palmarly until it touches the palm.
    """
    g = table["grip"]
    held = {k: g["posture_deg"].get(k, 0.0) for k in (*ARM, *THUMB)} | wrist
    probe = build_scene(hand, held, table)
    data = start(probe, held)
    normal, axis, mcp3 = _dynamometer_frame(probe, data)
    mc = probe.model.body(hand.metacarpal3).id
    rot, origin = data.xmat[mc].reshape(3, 3), data.xpos[mc]
    a, b, length = (g["dynamometer"][k] for k in ("semi_axis_palmar_m", "semi_axis_distal_m", "length_m"))
    frame = np.column_stack([normal, np.cross(axis, normal), axis])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, (rot.T @ frame).ravel())
    mid = 0.5 * (data.xpos[probe.model.body(hand.segments[2][0]).id] + data.xpos[probe.model.body(hand.segments[5][0]).id])
    distal = np.cross(normal, axis)
    distal *= np.sign(distal @ (data.xpos[probe.model.body(hand.segments[3][2]).id] - mcp3))
    centre = mcp3 + axis * ((mid - mcp3) @ axis) + distal * g["dynamometer"]["centre_distal_m"]
    theta = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)
    ring = np.column_stack([a * np.cos(theta), b * np.sin(theta)])
    verts = np.vstack([np.column_stack([ring, np.full(len(theta), z)]) for z in (-length / 2, length / 2)])
    contact = g["contact"]

    def edit(spec: mujoco.MjSpec, _: Hand) -> None:
        spec.add_mesh(name="dynamometer_mesh", uservert=verts.ravel().tolist())
        spec.body(hand.metacarpal3).add_geom(
            name="dynamometer",
            type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname="dynamometer_mesh",
            pos=rot.T @ (centre - origin),
            quat=quat,
            mass=0.0,
            group=3,
            contype=0,
            conaffinity=0,
        )
        for d in DIGITS:
            for body in hand.segments[d]:
                for geom in probe.collision_geoms(body):
                    spec.add_pair(
                        geomname1="dynamometer",
                        geomname2=probe.model.geom(geom).name,
                        condim=3,
                        friction=[contact["friction"]] * 2 + [0.005, 1e-4, 1e-4],
                        solref=contact["solref"],
                        solimp=contact["solimp"],
                    )

    scene = build_scene(hand, held, table, edit)
    m = scene.model
    dyn = m.geom("dynamometer").id
    data = start(scene, held)
    # Move the dynamometer palmarly until it just touches the palm or a flat proximal phalanx (the palm
    # has no pairs: it is rigid with the dynamometer).
    base = m.geom_pos[dyn].copy()
    shift_local = rot.T @ normal
    touch = scene.collision_geoms(hand.metacarpal3) + [
        geom for d in DIGITS for geom in scene.collision_geoms(hand.segments[d][0])
    ]

    def gap(offset: float) -> float:
        m.geom_pos[dyn] = base + shift_local * offset
        mujoco.mj_forward(m, data)
        return min(mujoco.mj_geomDistance(m, data, dyn, p, 0.2, None) for p in touch)

    lo, hi = 0.0, 0.1
    for _ in range(40):
        mid_off = 0.5 * (lo + hi)
        lo, hi = (mid_off, hi) if gap(mid_off) < 0.0 else (lo, mid_off)
    gap(hi + g["dynamometer"]["palm_gap_m"])
    counted: dict[int, tuple[int, str]] = {}
    for d in DIGITS:
        for seg, body in zip(("proximal", "middle", "distal"), hand.segments[d]):
            for geom in scene.collision_geoms(body):
                counted[geom] = (d, seg)
    data = _close_hand(scene, held, table)
    return GripScene(scene, dyn, counted, data)


def _close_hand(scene: Scene, held: dict[str, float], table: dict[str, Any]) -> mujoco.MjData:
    """Close the fingers onto the dynamometer with a moderate flexor drive; the start of every evaluation."""
    g = table["grip"]["closing"]
    data = start(scene, held | {f"{k}{d}": v for d in DIGITS for k, v in g["start_deg"].items()})
    hand = scene.hand
    levels = {n: g["level"] for grp in ("FDS", "FDP", "finger_intrinsic") for n in hand.muscles[grp]}
    set_activation(scene, data, levels)
    for _ in range(int(round(g["duration_s"] / scene.model.opt.timestep))):
        mujoco.mj_step(scene.model, data)
    return data


def grip_forces(gs: GripScene, data: mujoco.MjData) -> dict[str, Any]:
    """Contact forces on the dynamometer, perpendicular to its axis: per digit and segment, and the grip."""
    m = gs.scene.model
    axis = data.geom_xmat[gs.dynamometer].reshape(3, 3)[:, 2]
    out = {f"{d}_{s}": 0.0 for d in DIGITS for s in ("proximal", "middle", "distal")}
    depth = 0.0
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        other = c.geom2 if c.geom1 == gs.dynamometer else c.geom1 if c.geom2 == gs.dynamometer else -1
        if other not in gs.counted:
            continue
        mujoco.mj_contactForce(m, data, i, f6)
        f = c.frame.reshape(3, 3).T @ f6[:3]
        d, seg = gs.counted[other]
        out[f"{d}_{seg}"] += float(np.linalg.norm(f - (f @ axis) * axis))
        depth = max(depth, -c.dist)
    grip = {d: out[f"{d}_proximal"] + out[f"{d}_middle"] for d in DIGITS}
    return {"segments": out, "digits": grip, "grip": sum(grip.values()), "penetration_m": depth}


def grip_evaluate(
    gs: GripScene, levels: dict[str, float], table: dict[str, Any], duration_s: float | None = None
) -> dict[str, Any]:
    """Simulate ``eval_s`` (or *duration_s*) from the closed hand at *levels*; forces and lock moments averaged over
    ``average_s``."""
    p = table["grip"]
    scene, m = gs.scene, gs.scene.model
    data = mujoco.MjData(m)
    mujoco.mj_copyData(data, m, gs.start)
    set_activation(scene, data, levels)
    n = int(round((p["eval_s"] if duration_s is None else duration_s) / m.opt.timestep))
    k = int(round(p["average_s"] / m.opt.timestep))
    samples = []
    for i in range(n):
        mujoco.mj_step(m, data)
        if i >= n - k:
            mujoco.mj_forward(m, data)
            f = grip_forces(gs, data)
            f["wrist_lock_Nm"] = [lock_force(scene, data, key) for key in WRIST]
            samples.append(f)
    mean = {
        "grip": float(np.mean([s["grip"] for s in samples])),
        "digits": {d: float(np.mean([s["digits"][d] for s in samples])) for d in DIGITS},
        "segments": {key: float(np.mean([s["segments"][key] for s in samples])) for key in samples[0]["segments"]},
        "penetration_m": float(np.max([s["penetration_m"] for s in samples])),
        "wrist_lock_Nm": np.mean([s["wrist_lock_Nm"] for s in samples], axis=0).tolist(),
    }
    mean["share"] = {d: mean["digits"][d] / mean["grip"] if mean["grip"] > 0 else 0.0 for d in DIGITS}
    mean["fl"] = {n: force_length(scene, data, n) for grp in ("FDS", "FDP") for n in scene.hand.muscles[grp]}
    mean["angles_deg"] = {
        f"{k}{d}": scene.angle(data, f"{k}{d}") for d in DIGITS for k in ("mcp_flexion", "pip_flexion", "dip_flexion")
    }
    return mean


def grip_settled(gs: GripScene, levels: dict[str, float], table: dict[str, Any]) -> dict[str, Any]:
    """The grip at the same *levels* after ``settled.settle_s``, reported next to the ``eval_s`` value (not tested).

    Returns the force, digit forces and shares, wrist-lock moment and penetration at ``settle_s``, and the relative
    change of the grip from ``settled.check_s`` to ``settle_s`` (how far from settled it still is).
    """
    s = table["grip"]["settled"]
    check = grip_evaluate(gs, levels, table, s["check_s"])["grip"]
    late = grip_evaluate(gs, levels, table, s["settle_s"])
    keys = ("grip", "digits", "share", "wrist_lock_Nm", "penetration_m")
    return {k: late[k] for k in keys} | {"change_from_check": late["grip"] / check - 1.0 if check > 0 else 0.0}


def _wrist_columns(gs: GripScene, names: tuple[str, ...]) -> np.ndarray:
    """Wrist moment (anatomical sign) per unit activation of each muscle in *names* (2 x n).

    Read as the change of the wrist-lock moment, so wrists driven through coupled joints count too.
    """
    scene, m = gs.scene, gs.scene.model
    data = mujoco.MjData(m)
    mujoco.mj_copyData(data, m, gs.start)
    data.qvel[:] = 0.0

    def lock(levels: dict[str, float]) -> np.ndarray:
        set_activation(scene, data, levels)
        mujoco.mj_forward(m, data)
        return np.array([lock_force(scene, data, k) for k in WRIST])

    base = lock({})
    cols = [base - lock({n: 1.0}) for n in names]
    return np.array(cols).T if cols else np.zeros((2, 0))


def optimise_grip(gs: GripScene, table: dict[str, Any], balance: bool = True) -> dict[str, Any]:
    """Maximise the grip over the free activation levels (paper: 15 levels in [0, 1]); report it settled too.

    The wrist and thumb are held, so the wrist-only and extrinsic thumb muscles change nothing but the
    moment the wrist lock supplies. Their levels therefore come from a linear program that minimises that
    moment (L1), and the remaining levels (shared FDS, FDP, EDC; EIP; EDM) are optimised by bounded
    Powell (several deterministic starts) on grip - weight x residual lock moment, each evaluation a ``eval_s`` simulation. With
    *balance* False the lock may supply any moment (wrist rigidly held). The optimum's grip at the same levels
    after ``settled.settle_s`` is added under ``settled`` (``grip_settled``); the optimisation uses ``eval_s`` only.
    """
    from scipy.optimize import linprog, minimize

    p = table["grip"]
    hand = gs.scene.hand
    wrist_names = hand.muscles["wrist"] + hand.muscles["thumb_extrinsic"]
    cols = _wrist_columns(gs, wrist_names)
    outer = [grp for grp in ("FDS", "FDP", "EDC", "EIP", "EDM") if hand.muscles[grp]]
    fixed = {n: 1.0 for n in hand.muscles["finger_intrinsic"]}

    def levels_of(x: np.ndarray, wrist_levels: np.ndarray) -> dict[str, float]:
        lv = dict(fixed)
        for grp, v in zip(outer, x):
            lv |= {n: float(v) for n in hand.muscles[grp]}
        return lv | {n: float(v) for n, v in zip(wrist_names, wrist_levels)}

    def balance_lp(moment: np.ndarray) -> tuple[np.ndarray, float]:
        n = cols.shape[1]
        # min sum(s+ + s-) + 1e-4 sum(a)  s.t.  cols a - s+ + s- = moment
        c = np.concatenate([np.full(n, 1e-4), np.ones(4)])
        a_eq = np.hstack([cols, -np.eye(2), np.eye(2)])
        res = linprog(c, A_eq=a_eq, b_eq=moment, bounds=[(0, 1)] * n + [(0, None)] * 4, method="highs")
        return res.x[:n], float(np.sum(res.x[n:]))

    cache: dict[tuple[float, ...], tuple[float, dict[str, Any]]] = {}

    def objective(x: np.ndarray) -> float:
        key = tuple(np.round(np.clip(x, 0, 1), 4))
        if key not in cache:
            r = grip_evaluate(gs, levels_of(np.array(key), np.zeros(len(wrist_names))), table)
            wl, residual = balance_lp(np.array(r["wrist_lock_Nm"])) if balance else (np.zeros(len(wrist_names)), 0.0)
            r["wrist_levels"], r["residual_Nm"] = wl, residual
            cache[key] = (-(r["grip"] - p["balance_weight_N_per_Nm"] * residual), r)
        return cache[key][0]

    rng = np.random.default_rng(0)
    starts = [np.array([{"FDS": 1.0, "FDP": 1.0}.get(grp, 0.0) for grp in outer]), np.full(len(outer), 0.5)]
    starts += [rng.uniform(0.0, 1.0, len(outer)) for _ in range(p["optimiser_restarts"])]
    runs = [
        minimize(objective, x0, method="Powell", bounds=[(0.0, 1.0)] * len(outer), options={"xtol": 1e-3, "ftol": 1e-3})
        for x0 in starts
    ]
    key = tuple(np.round(np.clip(min(runs, key=lambda r: r.fun).x, 0, 1), 4))
    objective(np.array(key))
    r = cache[key][1]
    levels = levels_of(np.array(key), r["wrist_levels"])
    final = grip_evaluate(gs, levels, table)
    final["levels"] = {grp: float(v) for grp, v in zip(outer, key)} | {
        n.removesuffix("_r"): float(v) for n, v in zip(wrist_names, r["wrist_levels"])
    }
    final["residual_Nm"] = r["residual_Nm"]
    final["evaluations"] = len(cache)
    final["settled"] = grip_settled(gs, levels, table)
    return final


def grip_levels(hand: Hand, group_levels: dict[str, float]) -> dict[str, float]:
    """Per-muscle levels from group levels (shared FDS/FDP/EDC levels, single muscles by name)."""
    out = {n: 1.0 for n in hand.muscles["finger_intrinsic"]}
    for key, v in group_levels.items():
        out |= {n: v for n in hand.muscles.get(key, (key,))}
    return out


# --- 3. lateral pinch -----------------------------------------------------------------------------------


def _thumb_tip(scene: Scene, data: mujoco.MjData) -> np.ndarray:
    """World point of the distal thumb phalanx farthest from the IP joint (capsule end or mesh vertex)."""
    m = scene.model
    ip = data.xanchor[scene.joint("ip_flexion")[0]]
    points = []
    for g in scene.collision_geoms(scene.hand.thumb_distal):
        rot, pos = data.geom_xmat[g].reshape(3, 3), data.geom_xpos[g]
        if m.geom_type[g] in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
            points += [pos + rot[:, 2] * m.geom_size[g][1], pos - rot[:, 2] * m.geom_size[g][1]]
        elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mesh] : m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
            points += list(pos + v @ rot.T)
    points = np.array(points)
    return points[np.argmax(np.linalg.norm(points - ip, axis=1))]


def pinch(hand: Hand, table: dict[str, Any], restarts: int = 24) -> dict[str, Any]:
    """Maximum lateral pinch force: static optimum over the thumb muscles, checked in MuJoCo.

    The thumb tip is held at a point (the tip pressing on a force sensor). Statics on the four thumb DOFs,
    ``J^T F = tau(a)``, with J the 3 x 4 tip Jacobian: tau must have no component along J's null space (one
    condition), and F is maximised along the palmar normal of the distal phalanx with the off-axis part
    below ``off_axis_max`` of it. MuJoCo's forces with the sensor reaction applied at the tip confirm the equilibrium.
    """
    from scipy.optimize import minimize

    p = table["pinch"]
    held = [*ARM, *WRIST] + [f"{k}{d}" for d in DIGITS for k in ("mcp_flexion", "mcp_abduction", "pip_flexion", "dip_flexion")]
    angles = {k: p["posture_deg"].get(k, 0.0) for k in (*held, *THUMB)}
    scene = build_scene(hand, {k: angles[k] for k in held}, table)
    m = scene.model
    data = start(scene, angles)
    tip = _thumb_tip(scene, data)
    dofs = [scene.dof(k) for k in THUMB]
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jac(m, data, jacp, None, tip, m.body(hand.thumb_distal).id)
    jac = jacp[:, dofs]
    palmar = scene.joint("ip_flexion")[1] * jac[:, 3]
    palmar /= np.linalg.norm(palmar)
    names = hand.muscles["thumb_extrinsic"] + hand.muscles["thumb_intrinsic"]
    set_activation(scene, data, {})
    mujoco.mj_forward(m, data)
    tau0 = (data.qfrc_actuator + data.qfrc_passive - data.qfrc_bias)[dofs]
    cols = []
    for n in names:
        set_activation(scene, data, {n: 1.0})
        mujoco.mj_forward(m, data)
        cols.append((data.qfrc_actuator + data.qfrc_passive - data.qfrc_bias)[dofs] - tau0)
    tmat = np.array(cols).T
    null = np.linalg.svd(jac)[2][-1]
    pinv = np.linalg.pinv(jac.T)
    off = np.eye(3) - np.outer(palmar, palmar)

    def force(a: np.ndarray) -> np.ndarray:
        return pinv @ (tmat @ a + tau0)

    cons = [
        {"type": "eq", "fun": lambda a: null @ (tmat @ a + tau0)},
        {"type": "ineq", "fun": lambda a: p["off_axis_max"] * (palmar @ force(a)) - np.linalg.norm(off @ force(a))},
    ]

    def solve(constraints: list[dict[str, Any]]) -> Any:
        rng = np.random.default_rng(0)
        best = None
        for x0 in [np.full(len(names), 0.5)] + [rng.uniform(0, 1, len(names)) for _ in range(restarts)]:
            r = minimize(
                lambda a: -(palmar @ force(a)), x0, method="SLSQP", bounds=[(0, 1)] * len(names), constraints=constraints
            )
            ok = r.success and all(
                c["fun"](r.x) >= -1e-6 if c["type"] == "ineq" else abs(c["fun"](r.x)) < 1e-6 for c in constraints
            )
            if ok and (best is None or r.fun < best.fun):
                best = r
        return best

    best = solve(cons)
    if best is None:  # report the best palmar force with any off-axis part
        relaxed = solve(cons[:1])
        out: dict[str, Any] = {"force_N": 0.0, "feasible": False, "muscles": list(names)}
        if relaxed is not None:
            f = force(np.clip(relaxed.x, 0, 1))
            out |= {"relaxed_force_N": float(palmar @ f), "relaxed_off_axis": float(np.linalg.norm(off @ f) / abs(palmar @ f))}
        return out
    a = np.clip(best.x, 0, 1)
    f = force(a)
    result = {
        "feasible": True,
        "force_N": float(palmar @ f),
        "off_axis": float(np.linalg.norm(off @ f) / (palmar @ f)),
        "levels": {n.removesuffix("_r"): float(v) for n, v in zip(names, a)},
        "muscles": list(names),
    }
    result |= _pinch_simulation(hand, table, held, angles, tip, f, names, a)
    return result


def _pinch_simulation(
    hand: Hand,
    table: dict[str, Any],
    held: list[str],
    angles: dict[str, float],
    tip: np.ndarray,
    force: np.ndarray,
    names: tuple[str, ...],
    a: np.ndarray,
) -> dict[str, Any]:
    """Check the static optimum in MuJoCo: the sensor's reaction (-force at the tip) balances the thumb.

    A simulated weld cannot serve as the check: four thumb DOFs under a six-DOF weld leave the split
    between tip force and tip moment to the constraint softness. Returns the residual thumb moment
    (MuJoCo's actuator, passive and bias forces plus the reaction).
    """
    scene = build_scene(hand, {k: angles[k] for k in held}, table)
    m = scene.model
    data = start(scene, angles)
    set_activation(scene, data, dict(zip(names, a)))
    body = m.body(hand.thumb_distal).id
    data.xfrc_applied[body, :3] = -force
    data.xfrc_applied[body, 3:] = np.cross(tip - data.xipos[body], -force)
    mujoco.mj_forward(m, data)
    dofs = [scene.dof(k) for k in THUMB]
    reaction = np.zeros(m.nv)
    mujoco.mj_applyFT(m, data, -force, np.zeros(3), tip, body, reaction)
    residual = float(np.linalg.norm((data.qfrc_actuator + data.qfrc_passive - data.qfrc_bias + reaction)[dofs]))
    return {"sim_residual_Nm": residual}


# --- report -------------------------------------------------------------------------------------------------


def run_all(hand: Hand, table: dict[str, Any], label: str, protocols: tuple[str, ...]) -> dict[str, Any]:
    """Run the protocols on *hand*; a JSON-serialisable result dict."""
    out: dict[str, Any] = {"label": label}
    if "tenodesis" in protocols:
        out["tenodesis"] = tenodesis_metrics(tenodesis(hand, table), table)
    if "grip" in protocols:
        out["grip"] = {}
        for name, wrist in table["grip"]["wrist_postures_deg"].items():
            gs = grip_scene(hand, table, wrist)
            out["grip"][name] = {
                "balanced": optimise_grip(gs, table, balance=True),
                "held": optimise_grip(gs, table, balance=False),
            }
    if "pinch" in protocols:
        out["pinch"] = pinch(hand, table)
    return out


def _fmt(results: dict[str, Any], table: dict[str, Any]) -> str:
    lines = [f"== {results['label']}"]
    if "tenodesis" in results:
        t = table["tenodesis"]["targets"]
        for trial, r in results["tenodesis"].items():
            rom = " ".join(
                f"{d}:{r['rom_deg'][f'mcp_flexion{d}']:.1f}/{r['rom_deg'][f'pip_flexion{d}']:.1f}/{r['rom_deg'][f'dip_flexion{d}']:.1f}"
                for d in DIGITS
            )
            lines.append(
                f"tenodesis {trial}: ROM MCP/PIP/DIP per digit {rom} | mean MCP {r['mean_rom_mcp_flexion']:.1f} (paper {t['mean_rom_mcp_deg']}), "
                f"PIP {r['mean_rom_pip_flexion']:.1f} (paper {t['mean_rom_pip_deg']}), DIP {r['mean_rom_dip_flexion']:.1f} | "
                f"index gap MCP {r['index_gap_mcp_flexion']:.1f} (paper {t['index_gap_mcp_deg']}), PIP {r['index_gap_pip_flexion']:.1f} "
                f"(paper {t['index_gap_pip_deg']}) | outlier MCP {r['outlier_mcp_flexion']} PIP {r['outlier_pip_flexion']} | "
                f"max reversal {max(r['reversal_deg'].values()):.2f}"
            )
    if "tenodesis" in results:
        failed = [k for k, ok in tenodesis_checks(results["tenodesis"], table).items() if not ok]
        lines.append(f"tenodesis checks failed: {', '.join(failed) or 'none'}")
    if "grip" in results:
        base = results["grip"].get("self_selected", {}).get("balanced", {}).get("grip")
        if base:
            change = {k: v["balanced"]["grip"] / base - 1.0 for k, v in results["grip"].items() if k != "self_selected"}
            lines.append(
                "grip change vs self-selected (balanced): " + ", ".join(f"{k} {100 * v:+.0f} %" for k, v in change.items())
            )
            settled = {k: v["balanced"].get("settled", {}).get("grip") for k, v in results["grip"].items()}
            if all(settled.values()):
                lines.append(
                    "  settled: "
                    + ", ".join(
                        f"{k} {100 * (v / settled['self_selected'] - 1.0):+.0f} %"
                        for k, v in settled.items()
                        if k != "self_selected"
                    )
                )
        for posture, modes in results["grip"].items():
            for mode, r in modes.items():
                share = " ".join(f"{d}:{100 * r['share'][d]:.1f}%" for d in DIGITS)
                fl = " ".join(f"{n}:{v:.2f}" for n, v in r["fl"].items())
                lines.append(
                    f"grip {posture} {mode}: {r['grip']:.1f} N, shares {share}, residual {r.get('residual_Nm', 0):.3f} N m, "
                    f"pen {1000 * r['penetration_m']:.2f} mm, levels {json.dumps({k: round(v, 3) for k, v in r['levels'].items()})}, FL {fl}"
                )
                if "settled" in r:
                    st = r["settled"]
                    share = " ".join(f"{d}:{100 * st['share'][d]:.1f}%" for d in DIGITS)
                    lines.append(
                        f"  settled ({table['grip']['settled']['settle_s']:g} s, same levels): {st['grip']:.1f} N, shares {share}, "
                        f"change since {table['grip']['settled']['check_s']:g} s {100 * st['change_from_check']:+.1f} %, "
                        f"wrist lock {max(abs(v) for v in st['wrist_lock_Nm']):.3f} N m"
                    )
    if "pinch" in results:
        r = results["pinch"]
        if not r["feasible"]:
            lines.append(
                f"pinch: infeasible within the off-axis limit; best palmar force {r.get('relaxed_force_N', 0):.1f} N "
                f"at off-axis {r.get('relaxed_off_axis', 0):.2f}"
            )
            return "\n".join(lines)
        lines.append(
            f"pinch: {r['force_N']:.1f} N (off-axis {r['off_axis']:.2f}); MuJoCo residual {1000 * r['sim_residual_Nm']:.3f} N mm; levels "
            f"{json.dumps({k: round(v, 3) for k, v in r.get('levels', {}).items()})}"
        )
    return "\n".join(lines)


def _jsonable(x: Any) -> Any:
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, list | tuple):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, np.generic):
        return x.item()
    return x


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", default="myo_sim")
    parser.add_argument("--protocols", nargs="+", default=["tenodesis", "grip", "pinch"])
    parser.add_argument("--mutation", choices=sorted(MUTATIONS), help="apply a negative-control mutation (tenodesis only)")
    parser.add_argument("--out", type=Path, help="write the results as JSON")
    args = parser.parse_args()
    table = load_table()
    hand = myohand()
    if args.mutation:
        results = {"label": f"{args.label} ({args.mutation})"}
        results["tenodesis"] = tenodesis_metrics(tenodesis(hand, table, MUTATIONS[args.mutation]), table)
    else:
        results = run_all(hand, table, args.label, tuple(args.protocols))
    print(_fmt(results, table))
    if args.out:
        args.out.write_text(json.dumps(_jsonable(results), indent=1))


if __name__ == "__main__":
    main()
