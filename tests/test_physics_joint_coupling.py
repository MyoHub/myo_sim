"""Joint couplings (``mjEQ_JOINT``): projection onto their polynomials.

MuJoCo's own equality residuals are the reference, so the checks do not reuse
the arithmetic of :func:`project_joint_couplings`.
"""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from myo_sim.physics.joint_coupling import (
    JointCouplings,
    project_joint_couplings,
    projected_joint_couplings,
)

# A slider with a nonzero ref drives a hinge (all five coefficients), which drives
# a second hinge; that chained equality is listed first. One hinge is locked
# without a master, and one equality is inactive.
_XML = """
<mujoco>
  <worldbody>
    <body>
      <joint name="master" type="slide" axis="1 0 0" ref="0.3"/>
      <geom size="0.1" mass="1"/>
      <body pos="0 0 0.3">
        <joint name="dep" axis="0 0 1" ref="-0.2"/>
        <geom size="0.1" mass="1"/>
        <body pos="0 0 0.3">
          <joint name="chained" axis="0 1 0" ref="0.1"/>
          <geom size="0.1" mass="1"/>
          <body pos="0 0 0.3">
            <joint name="locked" axis="1 0 0" ref="0.05"/>
            <geom size="0.1" mass="1"/>
            <body pos="0 0 0.3">
              <joint name="loose" axis="0 1 0"/>
              <geom size="0.1" mass="1"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <equality>
    <joint joint1="chained" joint2="dep" polycoef="0.02 -0.7 0.3 0.1 -0.05"/>
    <joint joint1="dep" joint2="master" polycoef="0.1 0.5 -0.4 0.2 0.3"/>
    <joint joint1="locked" polycoef="-0.15 0 0 0 0"/>
    <joint joint1="loose" joint2="master" polycoef="0.3 1 0 0 0" active="false"/>
  </equality>
</mujoco>
"""


def joint_equality_residuals(
    model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """Position and velocity residuals (``efc_pos``, ``efc_vel``) of the active
    ``mjEQ_JOINT`` rows at the state *qpos*, *qvel* (zero velocity if ``None``).

    Args:
        model: Compiled model whose CPU layout *qpos* and *qvel* follow.
        qpos: Positions ``(nq,)``.
        qvel: Velocities ``(nv,)``.

    Returns:
        The two residual vectors, one entry per active joint equality.
    """
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    if qvel is not None:
        data.qvel[:] = qvel
    mujoco.mj_forward(model, data)
    rows = np.flatnonzero(data.efc_type[: data.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    rows = rows[model.eq_type[data.efc_id[rows]] == mujoco.mjtEq.mjEQ_JOINT]
    return data.efc_pos[rows].copy(), data.efc_vel[rows].copy()


def _random_state(
    model: mujoco.MjModel, rng: np.random.Generator, batch: tuple[int, ...] = ()
) -> tuple[np.ndarray, np.ndarray]:
    return (
        model.qpos0 + rng.uniform(-0.5, 0.5, (*batch, model.nq)),
        rng.uniform(-1.0, 1.0, (*batch, model.nv)),
    )


def test_projection_puts_every_active_coupling_on_its_polynomial() -> None:
    """Dependents, chained and locked joints land on their equalities; nothing else moves."""
    model = mujoco.MjModel.from_xml_string(_XML)
    couplings = JointCouplings.from_model(model)
    assert couplings.level_ends == (1, 2)  # "dep" before "chained"
    qpos, qvel = _random_state(model, np.random.default_rng(0))
    assert np.abs(joint_equality_residuals(model, qpos, qvel)[0]).max() > 0.1

    projected, projected_vel = qpos.copy(), qvel.copy()
    project_joint_couplings(couplings, projected, projected_vel)

    pos, vel = joint_equality_residuals(model, projected, projected_vel)
    assert len(pos) == 3  # the inactive equality has no row
    np.testing.assert_allclose(pos, 0.0, atol=1e-12)
    np.testing.assert_allclose(vel, 0.0, atol=1e-12)
    for name in ("master", "loose"):
        q_adr = model.jnt_qposadr[model.joint(name).id]
        v_adr = model.jnt_dofadr[model.joint(name).id]
        assert projected[q_adr] == qpos[q_adr]
        assert projected_vel[v_adr] == qvel[v_adr]


def test_torch_batch_matches_numpy() -> None:
    """The mjlab path (batched Torch tensors) gives the CPU numbers."""
    torch = pytest.importorskip("torch")
    model = mujoco.MjModel.from_xml_string(_XML)
    couplings = JointCouplings.from_model(model)
    qpos, qvel = _random_state(model, np.random.default_rng(1), (8,))
    qpos_t, qvel_t = torch.tensor(qpos), torch.tensor(qvel)
    project_joint_couplings(couplings.map_arrays(torch.as_tensor), qpos_t, qvel_t)
    for q, v in zip(qpos, qvel):
        project_joint_couplings(couplings, q, v)
    np.testing.assert_allclose(qpos_t.numpy(), qpos, rtol=0, atol=1e-15)
    np.testing.assert_allclose(qvel_t.numpy(), qvel, rtol=0, atol=1e-15)


@pytest.mark.parametrize(
    "equalities",
    [
        '<joint joint1="b" joint2="a"/><joint joint1="b" joint2="c"/>',
        '<joint joint1="b" joint2="a"/><joint joint1="a" joint2="b"/>',
    ],
    ids=["two-masters", "cycle"],
)
def test_rejects_over_constrained_and_cyclic_couplings(equalities: str) -> None:
    xml = f"""
    <mujoco>
      <worldbody>
        <body>
          <joint name="a"/><joint name="b" axis="0 1 0"/><joint name="c" axis="1 0 0"/>
          <geom size="0.1"/>
        </body>
      </worldbody>
      <equality>{equalities}</equality>
    </mujoco>
    """
    with pytest.raises(ValueError):
        JointCouplings.from_model(mujoco.MjModel.from_xml_string(xml))


def test_functional_projection_matches_in_place_and_leaves_inputs() -> None:
    """The out-of-place projection (chained couplings listed out of order, nonzero
    refs, a locked joint, an inactive equality) puts every row on its equality."""
    model = mujoco.MjModel.from_xml_string(_XML)
    couplings = JointCouplings.from_model(model)
    qpos, qvel = _random_state(model, np.random.default_rng(2))
    before = qpos.copy(), qvel.copy()
    projected, projected_vel = projected_joint_couplings(couplings, qpos, qvel)
    np.testing.assert_array_equal(qpos, before[0])
    np.testing.assert_array_equal(qvel, before[1])
    pos, vel = joint_equality_residuals(model, projected, projected_vel)
    assert len(pos) == 3
    np.testing.assert_allclose(pos, 0.0, atol=1e-12)
    np.testing.assert_allclose(vel, 0.0, atol=1e-12)
    project_joint_couplings(couplings, qpos, qvel)
    np.testing.assert_array_equal(projected, qpos)
    np.testing.assert_array_equal(projected_vel, qvel)
    assert projected_joint_couplings(couplings, before[0])[1] is None


@pytest.mark.parametrize("backend", ["numpy", "torch", "torch-cuda"])
@pytest.mark.parametrize("case", ["empty-model", "no-equalities", "inactive-only"])
@pytest.mark.parametrize("with_velocity", [False, True])
def test_uncoupled_functional_projection_returns_independent_arrays(backend: str, case: str, with_velocity: bool) -> None:
    if case == "empty-model":
        xml = "<mujoco/>"
    else:
        equalities = '<equality><joint joint1="a" active="false"/></equality>' if case == "inactive-only" else ""
        xml = f'<mujoco><worldbody><body><joint name="a"/><geom size=".1"/></body></worldbody>{equalities}</mujoco>'
    model = mujoco.MjModel.from_xml_string(xml)
    couplings = JointCouplings.from_model(model)
    qpos, qvel = _random_state(model, np.random.default_rng(5), (2, 3))
    if backend == "numpy":
        before = qpos.copy(), qvel.copy()
        out, vel = projected_joint_couplings(couplings, qpos, qvel if with_velocity else None)
        assert out is not qpos
        np.testing.assert_array_equal(out, qpos)
        out += 1
        np.testing.assert_array_equal(qpos, before[0])
        if with_velocity:
            assert vel is not qvel
            np.testing.assert_array_equal(vel, qvel)
            vel += 1
            np.testing.assert_array_equal(qvel, before[1])
        else:
            assert vel is None
        return

    torch = pytest.importorskip("torch")
    device = "cuda" if backend == "torch-cuda" else "cpu"
    if device == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")
    c = couplings.as_torch(device=device, dtype=torch.float64)
    q = torch.tensor(qpos, dtype=torch.float64, device=device, requires_grad=True)
    v = torch.tensor(qvel, dtype=torch.float64, device=device, requires_grad=True) if with_velocity else None
    out, vel = projected_joint_couplings(c, q, v)
    assert out is not q
    torch.testing.assert_close(out, q)
    if with_velocity:
        assert vel is not v
        torch.testing.assert_close(vel, v)
        inputs, loss = (q, v), out.sum() + vel.sum()
    else:
        assert vel is None
        inputs, loss = (q,), out.sum()
    for source, gradient in zip(inputs, torch.autograd.grad(loss, inputs)):
        torch.testing.assert_close(gradient, torch.ones_like(source))
    with torch.no_grad():
        out.add_(1)
        if vel is not None:
            vel.add_(1)
    torch.testing.assert_close(q.detach(), torch.tensor(qpos, dtype=torch.float64, device=device))
    if v is not None:
        torch.testing.assert_close(v.detach(), torch.tensor(qvel, dtype=torch.float64, device=device))


def test_functional_projection_gradients() -> None:
    """Torch gradients through the chain match finite differences and the chain
    rule; overwritten dependents and locked joints get none."""
    torch = pytest.importorskip("torch")
    model = mujoco.MjModel.from_xml_string(_XML)
    couplings = JointCouplings.from_model(model).as_torch(dtype=torch.float64)
    qpos, qvel = _random_state(model, np.random.default_rng(3), (4,))
    qpos_t = torch.tensor(qpos, requires_grad=True)
    qvel_t = torch.tensor(qvel, requires_grad=True)
    assert torch.autograd.gradcheck(lambda q, v: projected_joint_couplings(couplings, q, v), (qpos_t, qvel_t))
    out, out_vel = projected_joint_couplings(couplings, qpos_t, qvel_t)
    np.testing.assert_allclose(
        out.detach().numpy(), projected_joint_couplings(JointCouplings.from_model(model), qpos, qvel)[0], atol=1e-15
    )
    adr = {n: model.jnt_qposadr[model.joint(n).id] for n in
           ("master", "dep", "chained", "locked")}  # fmt: skip
    (grad,) = torch.autograd.grad(out[:, adr["chained"]].sum(), qpos_t)
    # d chained / d master = p2'(dep - dep0) * p1'(master - master0)
    p1, p2 = (np.polynomial.Polynomial(model.eq_data[e, :5]) for e in (1, 0))
    x1 = qpos[:, adr["master"]] - model.qpos0[adr["master"]]
    x2 = out.detach().numpy()[:, adr["dep"]] - model.qpos0[adr["dep"]]
    np.testing.assert_allclose(grad[:, adr["master"]].numpy(), p2.deriv()(x2) * p1.deriv()(x1), rtol=1e-12)
    for name in ("dep", "chained", "locked"):
        assert (grad[:, adr[name]] == 0).all()


def test_as_torch_casts_values_and_keeps_addresses() -> None:
    torch = pytest.importorskip("torch")
    model = mujoco.MjModel.from_xml_string(_XML)
    couplings = JointCouplings.from_model(model).as_torch(dtype=torch.float32)
    assert couplings.coef.dtype == torch.float32
    assert couplings.dep_qpos.dtype == torch.int64
    assert couplings.level_ends == (1, 2)
    qpos, _ = _random_state(model, np.random.default_rng(4), (3,))
    out, _ = projected_joint_couplings(couplings, torch.tensor(qpos, dtype=torch.float32))
    assert out.dtype == torch.float32


@pytest.mark.parametrize("name", ["myolegs", "myoarm_r"])
def test_composed_model_states_land_on_their_couplings(name: str) -> None:
    """Random states of a composed model (knee mechanism, shoulder rhythm) end on every coupling."""
    import myo_sim

    model = myo_sim.load_spec(name).compile()
    couplings = JointCouplings.from_model(model)
    assert len(couplings.dep_qpos) + len(couplings.locked_qpos) > 0
    rng = np.random.default_rng(0)
    for _ in range(3):
        qpos = model.qpos0 + rng.uniform(-0.3, 0.3, model.nq)
        qvel = rng.uniform(-1.0, 1.0, model.nv)
        project_joint_couplings(couplings, qpos, qvel)
        pos, vel = joint_equality_residuals(model, qpos, qvel)
        np.testing.assert_allclose(pos, 0.0, atol=1e-10)
        np.testing.assert_allclose(vel, 0.0, atol=1e-10)
