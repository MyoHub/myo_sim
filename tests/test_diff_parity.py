"""myo_sim.diff against MuJoCo: tendon lengths, velocities, moment arms, forces and activation dynamics.

Random postures and velocities of the registered models and one composition (``myoarms``), plus the
feature test model ``tests/data/diff_features.xml`` (pulleys, fixed tendons, ball and free joints,
inside side sites, non-muscle actuators, limits). Values at wrap onsets, where MuJoCo's discrete wrap
decision depends on round-off, are checked to a looser tolerance and must be rare.
"""

import copy

import mujoco
import numpy as np
import pytest

torch = pytest.importorskip("torch")

from diff_utils import MODELS, actuator_onset, load, mujoco_reference, random_state  # noqa: E402

from myo_sim.diff import MuscleTendonModel, default_side_rule  # noqa: E402

N = 48


def _forward(mt: MuscleTendonModel, Q, V, A, U, dtype=torch.float64):
    t = [torch.tensor(x, dtype=dtype) for x in (Q, V, A, U)]
    out = mt.forward(*t)
    return out, mt.act_dot(t[3], t[2])


@pytest.mark.parametrize("name", MODELS + ("features",))
def test_matches_mujoco(name):
    m = load(name)
    mt = MuscleTendonModel(m)
    Q, V, A, U = random_state(m, N, seed=0)
    ref = mujoco_reference(m, Q, V, A, U)
    out, act_dot = _forward(mt, Q, V, A, U)
    onset = ref["onset"]
    ok = ~onset
    assert onset.mean() < 0.02, f"{onset.mean():.3f} of the tendons at a wrap onset"
    L, Ld = out.ten_length.numpy(), out.ten_velocity.numpy()
    np.testing.assert_allclose(L[ok], ref["ten_length"][ok], atol=1e-9, rtol=0)
    np.testing.assert_allclose(L[onset], ref["ten_length"][onset], atol=1e-5, rtol=0)
    np.testing.assert_allclose(Ld[ok], ref["ten_velocity"][ok], atol=1e-9, rtol=0)
    # moment arms: the tendon Jacobian and the actuator moments, in MuJoCo's sparse layout
    row = mt.tendons.row_of_slot.numpy()
    okJ = ok[:, row]
    np.testing.assert_allclose(out.ten_J.numpy()[okJ], ref["ten_J"][okJ], atol=1e-8, rtol=0)
    aok = ~actuator_onset(m, onset)
    okM = aok[:, mt.actuators.m_row.numpy()]
    np.testing.assert_allclose(out.actuator_moment.numpy()[okM], ref["actuator_moment"][okM], atol=1e-8, rtol=0)
    dense = mt.dense_moment(out.actuator_moment).numpy()
    for i in range(3):
        ref_dense = np.zeros((m.nu, m.nv))
        d = mujoco.MjData(m)
        d.qpos[:] = Q[i]
        mujoco.mj_forward(m, d)
        mujoco.mju_sparse2dense(ref_dense, d.actuator_moment, d.moment_rownnz, d.moment_rowadr, d.moment_colind)
        if aok[i].all():
            np.testing.assert_allclose(dense[i], ref_dense, atol=1e-8, rtol=0)
    # actuator length, velocity and force
    np.testing.assert_allclose(out.actuator_length.numpy()[aok], ref["actuator_length"][aok], atol=1e-9, rtol=0)
    np.testing.assert_allclose(out.actuator_velocity.numpy()[aok], ref["actuator_velocity"][aok], atol=1e-9, rtol=0)
    np.testing.assert_allclose(out.actuator_force.numpy()[aok], ref["actuator_force"][aok], atol=1e-9, rtol=1e-6)
    np.testing.assert_allclose(act_dot.numpy(), ref["act_dot"], atol=1e-10, rtol=1e-12)
    # generalised forces, at states without onsets
    s = aok.all(1)
    scale = np.abs(ref["qfrc_actuator"]).max()
    np.testing.assert_allclose(out.qfrc_actuator.numpy()[s], ref["qfrc_actuator"][s], atol=1e-6 * scale, rtol=0)
    # torque per actuator and DOF sums to the per-DOF torque
    per = mt.dense_qfrc_per_actuator(out.qfrc_per_actuator).sum(-2)
    torch.testing.assert_close(per, out.qfrc_actuator, atol=1e-9, rtol=0)
    # the postures exercise the wraps, wrapped and not
    if mt.tendons.ntriple:
        w = out.wrapped.numpy()
        assert w.any() and not w.all()


@pytest.mark.parametrize("name", ("myoarm_r", "features"))
def test_float32_round_off(name):
    m = load(name)
    Q, V, A, U = random_state(m, N, seed=1)
    o64, _ = _forward(MuscleTendonModel(m), Q, V, A, U)
    o32, _ = _forward(MuscleTendonModel(m, dtype=torch.float32), Q, V, A, U, torch.float32)
    assert o32.ten_length.dtype == torch.float32
    same = (o32.wrapped == o64.wrapped).all(-1)
    assert same.float().mean() > 0.9
    L32, L64 = o32.ten_length.double()[same], o64.ten_length[same]
    torch.testing.assert_close(L32, L64, atol=2e-6, rtol=0)
    torch.testing.assert_close(o32.ten_J.double()[same], o64.ten_J[same], atol=5e-5, rtol=0)
    F32, F64 = o32.actuator_force.double()[same], o64.actuator_force[same]
    torch.testing.assert_close(F32, F64, atol=1e-3 * F64.abs().max().item(), rtol=0)


def test_side_rule_matters():
    """Negative control: the other MuJoCo version's side-site rule moves some arm muscle paths."""
    m = load("myoarm_r")
    other = "mujoco4" if default_side_rule() == "mujoco3" else "mujoco3"
    Q, V, A, U = random_state(m, 64, seed=2)
    ref = mujoco_reference(m, Q, V, A, U, ndir=0)
    out, _ = _forward(MuscleTendonModel(m, side_rule=other), Q, V, A, U)
    assert np.abs(out.ten_length.numpy() - ref["ten_length"]).max() > 1e-3


@pytest.mark.parametrize("tausmooth", (0.0, 0.2))
def test_activation_steps_match_mujoco(tausmooth):
    m = copy.copy(load("myoarm_r"))
    m.actuator_dynprm[:, 2] = tausmooth
    mt = MuscleTendonModel(m)
    rng = np.random.default_rng(3)
    d = mujoco.MjData(m)
    d.act[:] = rng.uniform(0, 1, m.na)
    act = torch.tensor(d.act.copy())
    for _ in range(5):
        ctrl = rng.uniform(-0.2, 1.2, m.nu)
        d.ctrl[:] = ctrl
        mean_mj = np.zeros(m.na)
        for _ in range(4):
            mean_mj += d.act / 4
            mujoco.mj_step(m, d)
        act, mean = mt.step_act(act, torch.tensor(ctrl), nsteps=4)
        np.testing.assert_allclose(act.numpy(), d.act, atol=1e-12)
        np.testing.assert_allclose(mean.numpy(), mean_mj, atol=1e-12)


def test_activation_features_match_mujoco():
    """Filter, exact filter, integrator, actearly and activation limits of the feature model, over steps."""
    m = load("features")
    mt = MuscleTendonModel(m)
    rng = np.random.default_rng(4)
    d = mujoco.MjData(m)
    d.act[:] = rng.uniform(-0.2, 1, m.na)
    for _ in range(20):
        ctrl = rng.uniform(-1.5, 1.5, m.nu)
        act = torch.tensor(d.act.copy())
        d.ctrl[:] = ctrl
        mujoco.mj_forward(m, d)
        out = mt.forward(torch.tensor(d.qpos), torch.tensor(d.qvel), act, torch.tensor(ctrl))
        np.testing.assert_allclose(out.actuator_force.numpy(), d.actuator_force, atol=1e-9, rtol=1e-9)
        nxt, _ = mt.step_act(act, torch.tensor(ctrl))
        mujoco.mj_step(m, d)
        np.testing.assert_allclose(nxt.numpy(), d.act, atol=1e-12)


@pytest.mark.parametrize("name", ("myoarm_r", "myolegs", "features"))
def test_effective_moment_arms(name):
    """Moment arms through the joint couplings match MuJoCo's moments times the coupling Jacobian, and the
    derivative of the actuator lengths along the independent coordinates."""
    m = load(name)
    mt = MuscleTendonModel(m)
    cp = mt.couplings
    assert cp.nc > 0
    rng = np.random.default_rng(5)
    Q = random_state(m, 16, seed=5)[0]
    q_ind = torch.tensor(Q[:, cp.indep_qpos])
    qpos, _ = cp.expand(q_ind)
    # the expansion satisfies every joint equality
    d = mujoco.MjData(m)
    for i in range(len(Q)):
        for j1, (j2, c) in cp.coupled.items():
            a1 = m.jnt_qposadr[j1]
            x = 0.0 if j2 < 0 else qpos[i, m.jnt_qposadr[j2]].item() - m.qpos0[m.jnt_qposadr[j2]]
            want = m.qpos0[a1] + (c[0] if j2 < 0 else np.polyval(c[::-1], x))
            assert abs(qpos[i, a1].item() - want) < 1e-12
    out = mt.forward(qpos)
    eff = mt.effective_moment(out.actuator_moment, qpos).numpy()
    for i in range(len(Q)):
        d.qpos[:] = qpos[i].numpy()
        mujoco.mj_forward(m, d)
        dense = np.zeros((m.nu, m.nv))
        mujoco.mju_sparse2dense(dense, d.actuator_moment, d.moment_rownnz, d.moment_rowadr, d.moment_colind)
        C = np.zeros((m.nv, cp.nind))  # dqvel / dqvel_independent
        for k, dof in enumerate(cp.indep_dofs):
            C[dof, k] = 1.0
        for j1, (j2, c) in cp.coupled.items():
            if j2 >= 0:
                x = d.qpos[m.jnt_qposadr[j2]] - m.qpos0[m.jnt_qposadr[j2]]
                C[m.jnt_dofadr[j1], cp.indep_dofs.index(m.jnt_dofadr[j2])] = (
                    c[1] + 2 * c[2] * x + 3 * c[3] * x**2 + 4 * c[4] * x**3
                )
        np.testing.assert_allclose(eff[i], dense @ C, atol=1e-8, rtol=0)
    # autograd of the actuator lengths through the expansion, for hinge and slide coordinates
    scalar = [k for k, a in enumerate(cp.indep_qpos) if a in set(m.jnt_qposadr[m.jnt_type >= 2].tolist())]
    dof_of = {int(m.jnt_qposadr[j]): int(m.jnt_dofadr[j]) for j in range(m.njnt) if m.jnt_type[j] >= 2}
    cols = [cp.indep_dofs.index(dof_of[cp.indep_qpos[k]]) for k in scalar]
    w = torch.tensor(rng.normal(0, 1, (len(Q), m.nu)))
    qi = q_ind.clone().requires_grad_()
    L = mt.forward(cp.expand(qi)[0]).actuator_length
    g = torch.autograd.grad((L * w).sum(), qi)[0][:, scalar]
    want = torch.einsum("bu,buj->bj", w, torch.tensor(eff))[:, cols]
    # inside wraps: the derivative of the computed length differs by Newton's residual (see test_diff_grad)
    torch.testing.assert_close(g, want, atol=1e-8, rtol=0)
