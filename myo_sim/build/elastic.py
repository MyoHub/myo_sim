"""Optional series-elastic (SEE) tendon DOFs, opt-in per muscle.

myo_sim's tendons are rigid by default (MuJoCo's muscle model assumes an
inelastic tendon). This module adds compliant tendons for individual muscles
without touching the shipped rigid XML: it mutates a compiled `MjSpec` by
inserting a small virtual body + slide joint between the muscle's last tendon
waypoint and its insertion, with joint stiffness set to match the requested
tendon compliance. The spatial tendon is retargeted to the new virtual site,
and (optionally) the actuator's `gainprm`/`biasprm` normalized range is
retuned so the muscle's implied optimal fiber length matches the supplied
anatomical value.

Nothing here is wired into any default composed model — every muscle is
opt-in. Calling `apply_elastic_tendons(spec, ..., muscles=[])` (the default)
is a no-op; the model compiles identically to the rigid baseline.

Validated (see `sandbox/elastic_tendon/`) against the OpenSim/Millard
analytic reference and, for the Achilles group + patellar tendon + tibialis
anterior, against published in-vivo human tendon strain data. `DEFAULT_ACHILLES`
below is the only parameter set with that level of validation; other muscles
require the same `verify_see_kinematics` check (see below) before trusting
their SEE output -- a prior version of this fix used the wrong tendon
waypoint for two other muscles and produced silently wrong (negative, or
exactly zero) strain until that check was added.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

# Millard2012EquilibriumMuscle default toe-region strain-at-max-isometric-force.
DEFAULT_E0 = 0.049


@dataclass
class SEEParams:
    """Series-elastic tendon parameters for one muscle.

    `origin_site`/`insert_site` must already exist on the muscle's tendon path
    (`insert_site` must be the path's actual last site -- see
    `verify_see_kinematics`). `l0`/`pennation0` are only needed if
    `retune_anatomical=True`.
    """

    muscle: str
    origin_site: str
    insert_site: str
    fmax: float
    lt_slack: float
    l0: float | None = None
    pennation0: float | None = None
    e0: float = DEFAULT_E0
    armature: float | None = None
    damp_frac: float = 1.2  # fraction of critical damping
    max_strain: float = 0.12
    mass: float = 0.02
    retune_anatomical: bool = True

    def resolved_armature(self) -> float:
        if self.armature is not None:
            return float(self.armature)
        return float(np.clip(self.fmax / 1000.0, 2.0, 20.0))


def secant_stiffness(fmax: float, lt_slack: float, e0: float = DEFAULT_E0) -> float:
    """Linear spring reaching `fmax` at strain `e0` (secant match to the Millard tendon curve)."""
    return fmax / (e0 * lt_slack)


def _axis_and_pos(model: mujoco.MjModel, data: mujoco.MjData, origin: str, insert: str):
    sid_o = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, origin)
    sid_i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, insert)
    bid = model.site_bodyid[sid_i]
    xmat = data.xmat[bid].reshape(3, 3)
    world = data.site_xpos[sid_o] - data.site_xpos[sid_i]
    axis = xmat.T @ world
    n = np.linalg.norm(axis)
    axis = axis / n if n > 1e-9 else np.array([0.0, 1.0, 0.0])
    return axis, np.array(model.site_pos[sid_i], float), bid


def _retune_actuator_anatomical(spec: mujoco.MjSpec, p: SEEParams, model: mujoco.MjModel) -> None:
    """Map lengthrange to anatomical L0/LT while preserving the measured path lengthrange."""
    if p.l0 is None:
        return
    act = spec.actuator(p.muscle)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, p.muscle)
    lr = np.array(model.actuator_lengthrange[aid], float)
    fmax = p.fmax
    r0 = (lr[0] - p.lt_slack) / p.l0
    r1 = r0 + (lr[1] - lr[0]) / p.l0
    gp = np.array(model.actuator_gainprm[aid], float).copy()
    bp = np.array(model.actuator_biasprm[aid], float).copy()
    gp[0], gp[1], gp[2] = r0, r1, fmax
    bp[0], bp[1], bp[2] = r0, r1, fmax
    act.gainprm = gp
    act.biasprm = bp


def apply_elastic_tendons(
    spec: mujoco.MjSpec,
    geom_model: mujoco.MjModel,
    geom_data: mujoco.MjData,
    muscles: list[SEEParams] | None = None,
) -> dict[str, SEEParams]:
    """Mutate `spec` in place: add SEE slide bodies and retarget the listed tendons.

    `geom_model`/`geom_data` must come from compiling the *unmutated* spec
    first (to measure the reference-pose axis/position for each muscle) --
    see `build_elastic_spec` for the standard two-pass usage.

    `muscles=None` or `[]` is a no-op: `spec` compiles identically to the
    rigid baseline.
    """
    muscles = muscles or []
    applied: dict[str, SEEParams] = {}

    for p in muscles:
        axis, pos, bid = _axis_and_pos(geom_model, geom_data, p.origin_site, p.insert_site)
        parent_name = mujoco.mj_id2name(geom_model, mujoco.mjtObj.mjOBJ_BODY, bid)
        parent = spec.body(parent_name)

        k = secant_stiffness(p.fmax, p.lt_slack, p.e0)
        arm = p.resolved_armature()
        damp = float(2.0 * np.sqrt(k * arm) * p.damp_frac)
        qmax = p.max_strain * p.lt_slack

        child = parent.add_body()
        child.name = f"{p.muscle}_see"
        child.pos = pos
        child.mass = p.mass
        child.inertia = np.array([1e-6, 1e-6, 1e-6])

        j = child.add_joint()
        j.name = f"{p.muscle}_tendon_q"
        j.type = mujoco.mjtJoint.mjJNT_SLIDE
        j.axis = np.asarray(axis, float)
        j.stiffness = np.array([k, 0.0, 0.0])
        j.damping = np.array([damp, 0.0, 0.0])
        j.armature = float(arm)
        j.springref = 0.0
        j.limited = True
        j.range[0] = 0.0
        j.range[1] = float(qmax)

        site = child.add_site()
        site.name = f"{p.muscle}_P_elastic"
        site.pos = np.zeros(3)

        old = spec.tendon(f"{p.muscle}_tendon")
        wraps = []
        for i in range(len(old.path)):
            w = old.path[i]
            side = w.sidesite.name if w.sidesite is not None else ""
            wraps.append((w.type, w.target.name if w.target is not None else "", side))
        springlength = np.array(old.springlength, float)
        width = float(old.width)
        rgba = np.array(old.rgba, float)
        spec.delete(old)

        newt = spec.add_tendon()
        newt.name = f"{p.muscle}_tendon"
        newt.springlength = springlength
        newt.width = width
        newt.rgba = rgba
        for i, (wtype, target, side) in enumerate(wraps):
            is_last = i == len(wraps) - 1
            if is_last:
                newt.wrap_site(site.name)
            elif wtype == mujoco.mjtWrap.mjWRAP_SITE:
                newt.wrap_site(target)
            elif wtype in (mujoco.mjtWrap.mjWRAP_SPHERE, mujoco.mjtWrap.mjWRAP_CYLINDER):
                newt.wrap_geom(target, side)
            else:
                newt.wrap_site(target)

        act = spec.actuator(p.muscle)
        act.target = f"{p.muscle}_tendon"
        if p.retune_anatomical:
            _retune_actuator_anatomical(spec, p, geom_model)
        applied[p.muscle] = p

    return applied


def build_elastic_spec(base_spec_fn, muscles: list[SEEParams] | None) -> tuple[mujoco.MjSpec, dict]:
    """Standard two-pass build: compile `base_spec_fn()` once to measure
    reference geometry, then apply `apply_elastic_tendons` to a fresh spec.

    `base_spec_fn` is a zero-arg callable returning an uncompiled `MjSpec`
    (e.g. `lambda: myo_sim.load_spec("myolegs")`) -- called twice, since the
    first spec is consumed by compilation and the second is mutated.
    """
    spec0 = base_spec_fn()
    m0 = spec0.compile()
    d0 = mujoco.MjData(m0)
    mujoco.mj_forward(m0, d0)

    spec = base_spec_fn()
    applied = apply_elastic_tendons(spec, m0, d0, muscles=muscles)

    meta = {"applied": applied}
    return spec, meta


def verify_see_kinematics(model: mujoco.MjModel, muscle: str, tol: float = 0.03) -> float:
    """Sanity check every new SEE muscle must pass before its output is trusted.

    Perturbs the SEE joint directly and measures how much the actuator's
    tendon-path length changes (`dL/dq`). A correctly attached SEE DOF gives
    `dL/dq ~= -1` (stretching the joint shortens the muscle-side path by
    about the same amount) -- exactly -1 for a single-joint muscle with an
    unobstructed final tendon segment (e.g. soleus_r: -1.0000), and up to a
    few percent off for a biarticular muscle whose fixed local axis is only
    an approximation to the true path sensitivity (gasmed_r/gaslat_r cross
    both the knee and ankle: -0.986). The default tolerance (3%) is set to
    pass that known, already-validated approximation while still catching
    the failure mode this check exists for: a wrong attachment site (e.g.
    not the tendon's actual last waypoint) gives a wildly wrong dL/dq --
    +0.82 or +1.0 instead of about -1 -- which an earlier version of this
    fix shipped with for two non-Achilles muscles, caught only by running
    this check.

    Returns the measured dL/dq. Raises ValueError if it's not within `tol` of -1.
    """
    data = mujoco.MjData(model)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{muscle}_tendon_q")
    qadr = model.jnt_qposadr[jid]

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    l0 = float(data.actuator_length[aid])

    dq = 0.01
    data.qpos[qadr] = dq
    mujoco.mj_forward(model, data)
    l1 = float(data.actuator_length[aid])

    dl_dq = (l1 - l0) / dq
    if abs(dl_dq - (-1.0)) > tol:
        raise ValueError(
            f"{muscle}: dL/dq = {dl_dq:.4f}, expected -1.0 (tol {tol}). "
            "Check that insert_site is the tendon's actual last path waypoint."
        )
    return dl_dq


# Achilles group (soleus, gastrocnemius medialis/lateralis), myolegs, right side.
# Rajagopal2016.osim (Millard2012EquilibriumMuscle) anatomical parameters.
# Validated against the OpenSim analytic reference (<2% strain error) and
# against published in-vivo human strain data (RMSE 0.0295-0.0319 at the
# default e0 here; a per-muscle experimental e0 refit is available in
# sandbox/elastic_tendon/ but not adopted as the default -- see that
# directory's validation report for the tradeoffs before changing e0 here).
DEFAULT_ACHILLES: dict[str, SEEParams] = {
    "soleus_r": SEEParams(
        muscle="soleus_r",
        origin_site="soleus-P1_r",
        insert_site="soleus-P2_r",
        fmax=6194.84262295082,
        lt_slack=0.276755872375976,
        l0=0.044,
        pennation0=0.38142888,
    ),
    "gasmed_r": SEEParams(
        muscle="gasmed_r",
        origin_site="gasmed-P1_r",
        insert_site="gasmed-P2_r",
        fmax=3115.51475409836,
        lt_slack=0.398716332626429,
        l0=0.051,
        pennation0=0.16568155,
    ),
    "gaslat_r": SEEParams(
        muscle="gaslat_r",
        origin_site="gaslat-P1_r",
        insert_site="gaslat-P2_r",
        fmax=1575.05901639344,
        lt_slack=0.376070169933794,
        l0=0.0588,
        pennation0=0.21022682,
    ),
}
