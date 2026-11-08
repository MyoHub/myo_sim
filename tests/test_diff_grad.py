"""Gradients of myo_sim.diff: autograd against the analytic moment arms and against central finite
differences away from wrap onsets, and the behaviour at onsets."""

import mujoco
import numpy as np
import pytest

torch = pytest.importorskip("torch")

from diff_utils import load, random_state  # noqa: E402

from myo_sim.diff import MuscleTendonModel  # noqa: E402

_SCALAR = (int(mujoco.mjtJoint.mjJNT_SLIDE), int(mujoco.mjtJoint.mjJNT_HINGE))


def _scalar_columns(m: mujoco.MjModel) -> tuple[list[int], list[int]]:
    """qpos and DOF addresses of the hinge and slide joints, where dL/dqpos is the moment arm."""
    js = [j for j in range(m.njnt) if int(m.jnt_type[j]) in _SCALAR]
    return [int(m.jnt_qposadr[j]) for j in js], [int(m.jnt_dofadr[j]) for j in js]


@pytest.mark.parametrize("name", ("myoarm_r", "myolegs", "features"))
def test_autograd_matches_moment_arms(name):
    """dL/dq by autograd equals the tendon Jacobian (MuJoCo's moment arms) for every hinge and slide.

    MuJoCo's Jacobian holds the tangent points fixed, which is exact where the path length is stationary in
    them; for inside wraps (side site within the geom) the solved point meets the tangent condition only to
    Newton's tolerance, and the derivative of the computed length differs from the Jacobian by about 1e-9.
    """
    m = load(name)
    mt = MuscleTendonModel(m)
    Q = random_state(m, 8, seed=10)[0]
    qa, da = _scalar_columns(m)
    q = torch.tensor(Q, requires_grad=True)
    out = mt.forward(q)
    J = mt.dense_ten_J(out.ten_J)
    for t in range(m.ntendon):
        g = torch.autograd.grad(out.ten_length[:, t].sum(), q, retain_graph=True)[0]
        torch.testing.assert_close(g[:, qa], J[:, t, da], atol=1e-8, rtol=0)


@pytest.mark.parametrize("name", ("myoarm_r", "myofullbody", "features"))
def test_gradients_match_finite_differences(name):
    """Directional derivatives of tendon lengths and actuator forces in qpos, qvel and act, against central
    differences, at states whose wrap decisions are the same at q - h dq, q and q + h dq."""
    m = load(name)
    mt = MuscleTendonModel(m)
    Q, V, A, U = (torch.tensor(x) for x in random_state(m, 32, seed=11))
    qa, _ = _scalar_columns(m)
    rng = np.random.default_rng(11)
    h = 1e-6

    def f(q, v, a):
        o = mt.forward(q, v, a, U)
        return o.ten_length, o.actuator_force, o.wrapped

    wL = torch.tensor(rng.normal(0, 1, (len(Q), m.ntendon)))
    wF = torch.tensor(rng.normal(0, 1, (len(Q), m.nu)))
    q, v, a = (x.clone().requires_grad_() for x in (Q, V, A))
    L, F, _ = f(q, v, a)
    gq_L = torch.autograd.grad((L * wL).sum(), q, retain_graph=True)[0]
    gq, gv, ga = torch.autograd.grad((F * wF).sum(), (q, v, a))
    assert all(torch.isfinite(x).all() for x in (gq_L, gq, gv, ga))
    dq = torch.zeros_like(Q)
    dq[:, qa] = torch.tensor(rng.normal(0, 1, (len(Q), len(qa))))
    dv, da = torch.tensor(rng.normal(0, 1, V.shape)), torch.tensor(rng.normal(0, 1, A.shape))
    with torch.no_grad():
        Lp, Fp, wp = f(Q + h * dq, V + h * dv, A + h * da)
        Lm, Fm, wm = f(Q - h * dq, V - h * dv, A - h * da)
        _, _, w0 = f(Q, V, A)
    same = ((wp == w0) & (wm == w0)).all(-1)
    assert same.float().mean() > 0.8
    fd_L = ((Lp - Lm) * wL).sum(-1) / (2 * h)
    fd_F = ((Fp - Fm) * wF).sum(-1) / (2 * h)
    jvp_L = (gq_L * dq).sum(-1)
    jvp_F = (gq * dq).sum(-1) + (gv * dv).sum(-1) + (ga * da).sum(-1)
    torch.testing.assert_close(jvp_L[same], fd_L[same], atol=1e-7, rtol=1e-6)
    torch.testing.assert_close(jvp_F[same], fd_F[same], atol=1e-5, rtol=1e-5)


ONSET_XML = """
<mujoco>
  <worldbody>
    <geom name="ball" type="sphere" size="0.05"/>
    <site name="side" pos="-0.2 0 -0.03"/>
    <site name="a" pos="-0.2 0 0.07"/>
    <body name="arm">
      <joint name="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <site name="b" pos="0.2 0 0.07"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="plain"><site site="a"/><geom geom="ball"/><site site="b"/></spatial>
    <spatial name="sided"><site site="a"/><geom geom="ball" sidesite="side"/><site site="b"/></spatial>
  </tendon>
</mujoco>
"""


def test_wrap_onset_behaviour():
    """What happens where a wrap switches on or off, here as one site circles a sphere.

    * Grazing onset (the straight segment touches the geom): length and moment arm are continuous, only the
      slope of the moment arm changes, so gradients are well defined on both sides.
    * Side-site switch (the side-site rule sends the path round the other side of the geom): the length jumps
      by centimetres and the moment arm changes sign. Autograd returns the derivative of the branch the state
      is on and never sees the jump; finite differences across it do. Both MuJoCo rules have such switches,
      at different postures.

    myo_sim.diff takes the same discrete decisions as MuJoCo at every sweep point, so it reproduces both.
    """
    m = mujoco.MjModel.from_xml_string(ONSET_XML)
    mt = MuscleTendonModel(m)
    th = np.linspace(-1.2, 1.2, 4801)
    q = torch.tensor(th[:, None], requires_grad=True)
    out = mt.forward(q)
    g = torch.autograd.grad(out.ten_length.sum(0)[1], q, retain_graph=True)[0][:, 0].numpy()
    g_plain = torch.autograd.grad(out.ten_length.sum(0)[0], q)[0][:, 0].numpy()
    L = out.ten_length.detach().numpy()
    d = mujoco.MjData(m)
    Lm = np.zeros_like(L)
    for i, x in enumerate(th):
        d.qpos[0] = x
        mujoco.mj_forward(m, d)
        Lm[i] = d.ten_length
    np.testing.assert_allclose(L, Lm, atol=1e-12)
    J = mt.dense_ten_J(out.ten_J).detach().numpy()[:, :, 0]
    np.testing.assert_allclose(J[:, 1], g, atol=1e-12)
    w = out.wrapped.numpy()
    step = th[1] - th[0]
    # grazing onset of the plain tendon: continuous length and moment arm
    k = np.flatnonzero(np.diff(w[:, 0].astype(int)))
    assert len(k) >= 1
    for i in k:
        assert abs(L[i + 1, 0] - L[i, 0]) < 1.1 * abs(J[i, 0]) * step
        assert abs(g_plain[i + 1] - g_plain[i]) < 1e-3
    # side-site switch of the sided tendon: the length jumps, the moment arm flips
    k = np.flatnonzero(np.diff(w[:, 1].astype(int)))
    jumps = [i for i in k if abs(L[i + 1, 1] - L[i, 1]) > 0.01]
    assert jumps, "expected a side-site switch in the sweep"
    for i in jumps:
        assert abs(Lm[i + 1, 1] - Lm[i, 1]) > 0.01
        assert np.sign(g[i]) != np.sign(g[i + 1])
        # one-sided derivatives are the branch derivatives
        assert abs((L[i, 1] - L[i - 1, 1]) / step - g[i]) < 1e-3
        assert abs((L[i + 2, 1] - L[i + 1, 1]) / step - g[i + 1]) < 1e-3
