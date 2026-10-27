"""Opt-in passive joint moments and thenar muscles (myo_sim.build.passive / thenar)."""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.passive import APPLIED, load_passive_moment_table, measured_rows, passive_moment
from myo_sim.build.thenar import add_thenar_muscles, load_thenar_table

MODELS = ("myofullbody", "myotorso_arms", "myohands", "myoarm_r")
THUMB = {
    "r": ("cmc_flexion", "cmc_abduction", "mp_flexion_r", "ip_flexion_r"),
    "l": ("cmc_flexion_l", "cmc_abduction_l", "mp_flexion_l", "ip_flexion_l"),
}
_FIELDS = (
    "qpos0",
    "jnt_range",
    "jnt_stiffness",
    "dof_damping",
    "body_mass",
    "body_pos",
    "actuator_gainprm",
    "actuator_lengthrange",
    "tendon_stiffness",
    "tendon_lengthspring",
    "site_pos",
)


def _segments(part) -> int:
    return 0 if part is None else len(part) if isinstance(part, list) else 1


def _expected_tendons(statuses: tuple[str, ...], sides: int = 2) -> int:
    rows = measured_rows(load_passive_moment_table(), statuses)
    return sides * sum(_segments(r.get("positive")) + _segments(r.get("negative")) for r in rows)


@pytest.fixture(scope="module")
def fullbody() -> mujoco.MjModel:
    return myo_sim.load_spec("myofullbody", passive_moments="assumed", thenar_muscles=True).compile()


@pytest.mark.parametrize("name", MODELS)
def test_opt_out_leaves_the_model_unchanged(name: str) -> None:
    # Compiled fields, not XML: two plain builds already differ in mesh-asset order.
    base = myo_sim.load_spec(name).compile()
    off = myo_sim.load_spec(name, passive_moments=False, thenar_muscles=False).compile()
    assert (off.nu, off.ntendon, off.nsite, off.njnt) == (base.nu, base.ntendon, base.nsite, base.njnt)
    for field in _FIELDS:
        np.testing.assert_array_equal(getattr(off, field), getattr(base, field), err_msg=field)


@pytest.mark.parametrize("name", MODELS)
def test_each_flag_applies(name: str) -> None:
    base = myo_sim.load_spec(name).compile()
    hands = 1 if name == "myoarm_r" else 2
    thenar = myo_sim.load_spec(name, thenar_muscles=True).compile()
    assert thenar.nu == base.nu + 3 * hands and thenar.ntendon == base.ntendon + 3 * hands
    for flag, statuses in ((True, APPLIED["measured"]), ("assumed", APPLIED["assumed"])):
        model = myo_sim.load_spec(name, passive_moments=flag).compile()
        added = [model.tendon(i).name for i in range(model.ntendon) if model.tendon(i).name.startswith("passive_")]
        assert len(added) == _expected_tendons(statuses, hands), name


def test_table_rows_are_sourced() -> None:
    table = load_passive_moment_table()
    assert measured_rows(table)
    for row in table["joints"]:
        assert row["status"] in ("measured", "assumed", "gap", "covered_by_muscles", "validation_only")
        assert row["scales_with"] == "none"
        if row["status"] in ("measured", "assumed"):
            assert row["source"] in table["sources"] and table["sources"][row["source"]]["doi"]
            assert "positive" in row or "negative" in row
            if row["status"] == "assumed":  # a transfer between joints says so
                assert "Assumption" in row["notes"]
        else:  # nothing is added; a gap says where values could come from
            assert "positive" not in row and "negative" not in row
            if row["status"] == "gap":
                assert all(src in table["sources"] for src in row["candidate_sources"])
            else:  # covered by muscles / validation only: the source is kept for reference
                assert row["source"] in table["sources"] and "reference" in row
    thenar = load_thenar_table()
    for muscle in thenar["muscles"]:
        assert muscle["source"] in thenar["sources"] and muscle["architecture_source"] in thenar["sources"]
        assert {muscle[k]["scales_with"] for k in ("force_N", "optimal_fiber_length_m")} == {"mass", "length"}


def test_passive_moments_match_the_table(fullbody: mujoco.MjModel) -> None:
    data = mujoco.MjData(fullbody)
    for row in measured_rows(load_passive_moment_table(), APPLIED["assumed"]):
        for side in ("r", "l"):
            joint = fullbody.joint(row["names"][side])
            for q in np.linspace(*fullbody.jnt_range[joint.id], 7):
                data.qpos[:] = fullbody.qpos0
                data.qvel[:] = 0.0
                data.qpos[joint.qposadr[0]] = q
                mujoco.mj_forward(fullbody, data)
                np.testing.assert_allclose(data.qfrc_passive[joint.dofadr[0]], passive_moment(row, q), atol=1e-9)
    wrist = next(r for r in measured_rows(load_passive_moment_table()) if r["names"]["r"] == "flexion_r")
    np.testing.assert_allclose(passive_moment(wrist, [0.2, -0.2]), [-0.554 * 0.2, 1.021 * 0.2])


def test_cpu_and_mujoco_warp_passive_moments_agree(fullbody: mujoco.MjModel) -> None:
    mjw = pytest.importorskip("mujoco_warp")
    wp = pytest.importorskip("warp")
    data = mujoco.MjData(fullbody)
    rng = np.random.default_rng(0)
    rows = measured_rows(load_passive_moment_table(), APPLIED["assumed"])
    dofs = []
    for row in rows:
        for side in ("r", "l"):
            joint = fullbody.joint(row["names"][side])
            data.qpos[joint.qposadr[0]] = rng.uniform(*fullbody.jnt_range[joint.id])
            dofs.append(joint.dofadr[0])
    mujoco.mj_forward(fullbody, data)
    with wp.ScopedDevice("cpu"):
        model_w = mjw.put_model(fullbody)
        data_w = mjw.put_data(fullbody, data)
        mjw.forward(model_w, data_w)
        warp_passive = data_w.qfrc_passive.numpy()[0]
    np.testing.assert_allclose(warp_passive[dofs], data.qfrc_passive[dofs], atol=1e-5)


def test_left_and_right_hands_are_symmetric(fullbody: mujoco.MjModel) -> None:
    data = mujoco.MjData(fullbody)
    rng = np.random.default_rng(1)
    rows = measured_rows(load_passive_moment_table(), APPLIED["assumed"])
    for row in rows:
        right, left = fullbody.joint(row["names"]["r"]), fullbody.joint(row["names"]["l"])
        np.testing.assert_allclose(fullbody.jnt_range[left.id], fullbody.jnt_range[right.id])
        q = rng.uniform(*fullbody.jnt_range[right.id])
        data.qpos[:] = fullbody.qpos0
        data.qpos[right.qposadr[0]] = data.qpos[left.qposadr[0]] = q
        mujoco.mj_forward(fullbody, data)
        np.testing.assert_allclose(data.qfrc_passive[left.dofadr[0]], data.qfrc_passive[right.dofadr[0]], atol=1e-9)
    for muscle in load_thenar_table()["muscles"]:
        r, left = fullbody.actuator(muscle["name"]).id, fullbody.actuator(muscle["name"] + "_l").id
        np.testing.assert_allclose(fullbody.actuator_gainprm[left], fullbody.actuator_gainprm[r])
        np.testing.assert_allclose(fullbody.actuator_lengthrange[left], fullbody.actuator_lengthrange[r], atol=1e-9)
        for _ in range(5):  # same lengths and moment arms at the same thumb angles
            q = np.array([rng.uniform(*fullbody.jnt_range[fullbody.joint(j).id]) for j in THUMB["r"]])
            right_geo = _tendon_geometry(fullbody, f"{muscle['name']}_tendon", THUMB["r"], q)
            left_geo = _tendon_geometry(fullbody, f"{muscle['name']}_tendon_l", THUMB["l"], q)
            np.testing.assert_allclose(left_geo, right_geo, atol=1e-6)


def _equilibrium(row: dict) -> float:
    part = row.get("positive", row.get("negative"))
    return (part[0] if isinstance(part, list) else part)["onset"]


def test_relaxed_hands_settle_inside_their_ranges() -> None:
    """Without gravity or excitation, joints whose spring rest angle lies inside the range settle off their limits, the same on both hands."""
    model = myo_sim.load_spec("myofullbody", passive_moments=True, thenar_muscles=True).compile()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    rows = measured_rows(load_passive_moment_table())
    rng = np.random.default_rng(2)
    for row in rows:  # random start inside each range, mirrored on the left
        right, left = model.joint(row["names"]["r"]), model.joint(row["names"]["l"])
        data.qpos[right.qposadr[0]] = data.qpos[left.qposadr[0]] = rng.uniform(*model.jnt_range[right.id])
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    for row in rows:
        right, left = model.joint(row["names"]["r"]), model.joint(row["names"]["l"])
        lo, hi = model.jnt_range[right.id]
        q_r, q_l = data.qpos[right.qposadr[0]], data.qpos[left.qposadr[0]]
        assert abs(q_r - q_l) < 1e-3, row["joint"]
        if lo + 0.05 < _equilibrium(row) < hi - 0.05:
            assert min(q_r - lo, hi - q_r) > 0.01, row["joint"]


def test_one_sided_springs_restore_toward_neutral_with_polynomial_terms() -> None:
    row = {
        "positive": [{"onset": 0.1, "stiffness": [1.0, 2.0, 3.0]}, {"onset": 0.5, "stiffness": [2.0, 0.0, 0.0]}],
        "negative": {"onset": -0.2, "stiffness": [0.5, 4.0, 1.0]},
    }
    spec = mujoco.MjSpec.from_string(
        '<mujoco><worldbody><body><joint name="j" range="-2 2"/><geom size=".1"/></body></worldbody></mujoco>'
    )
    from myo_sim.build.passive import add_passive_joint_moments

    add_passive_joint_moments(spec, {"joints": [{**row, "status": "measured", "names": {"r": "j"}}]}, sides=("r",))
    model = spec.compile()
    data = mujoco.MjData(model)
    for q in (-0.7, -0.15, 0.0, 0.05, 0.6):
        data.qpos[0] = q
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.qfrc_passive[0], passive_moment(row, q), atol=1e-12)
        assert data.qfrc_passive[0] * q <= 0.0


def test_subject_scale_multiplies_rows() -> None:
    base = myo_sim.load_spec("myoarm_r", passive_moments=True, thenar_muscles=True).compile()
    scaled = myo_sim.load_spec(
        "myoarm_r",
        passive_moments=True,
        thenar_muscles=True,
        subject_scale={"wrist_flexion": 2.0, "AdP.force": 0.5, "APB.length": 1.1},
    ).compile()
    ten = scaled.tendon("passive_flexion_r_pos").id
    np.testing.assert_allclose(scaled.tendon_stiffness[ten], 2 * base.tendon_stiffness[base.tendon("passive_flexion_r_pos").id])
    assert scaled.actuator_gainprm[scaled.actuator("AdP").id, 2] == pytest.approx(
        0.5 * base.actuator_gainprm[base.actuator("AdP").id, 2]
    )
    span = lambda m: float(np.diff(m.actuator_lengthrange[m.actuator("APB").id])[0])  # noqa: E731
    assert span(scaled) == pytest.approx(1.1 * span(base))  # 0.3 L_fo


def _tendon_geometry(model: mujoco.MjModel, tendon: str, joints: tuple[str, ...], q: np.ndarray) -> np.ndarray:
    """Length and dL/dq of *tendon* at thumb angles *q* (finite differences)."""
    data = mujoco.MjData(model)

    def length(qq: np.ndarray) -> float:
        data.qpos[:] = model.qpos0
        for name, value in zip(joints, qq):
            data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_tendon(model, data)
        return float(data.ten_length[model.tendon(tendon).id])

    eye = np.eye(len(joints)) * 1e-6
    return np.array([length(q)] + [(length(q + e) - length(q - e)) / 2e-6 for e in eye])


def test_thenar_muscles_follow_jacobson_and_copy_the_template() -> None:
    model = myo_sim.load_spec("myotorso_arms", thenar_muscles=True).compile()
    template = model.actuator("OP").id
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    for muscle in load_thenar_table()["muscles"]:
        act = model.actuator(muscle["name"]).id
        assert model.actuator_gainprm[act][2] == pytest.approx(muscle["force_N"]["value"])
        np.testing.assert_array_equal(
            np.delete(model.actuator_gainprm[act][:9], 2), np.delete(model.actuator_gainprm[template][:9], 2)
        )
        np.testing.assert_array_equal(model.actuator_dynprm[act], model.actuator_dynprm[template])
        # Architecture length range: rigid tendon of Barry's slack length, range 0.75-1.05 of L_fo.
        l_fo, alpha = muscle["optimal_fiber_length_m"]["value"], np.radians(muscle["pennation_deg"]["value"])
        l_ts = (data.actuator_length[act] - l_fo * np.cos(alpha)) / 1.033
        np.testing.assert_allclose(model.actuator_lengthrange[act], [l_ts + 0.75 * l_fo, l_ts + 1.05 * l_fo], atol=1e-9)


def test_thenar_lengthrange_modes() -> None:
    computed = myo_sim.load_spec("myoarm_r")
    add_thenar_muscles(computed, lengthrange="computed")
    model = computed.compile()
    rng = np.random.default_rng(1)
    for muscle in load_thenar_table()["muscles"]:  # computed: covers the tendon over the thumb range
        lo, hi = model.actuator_lengthrange[model.actuator(muscle["name"]).id]
        for _ in range(30):
            q = np.array([rng.uniform(*model.jnt_range[model.joint(j).id]) for j in THUMB["r"]])
            assert lo - 1e-4 <= _tendon_geometry(model, f"{muscle['name']}_tendon", THUMB["r"], q)[0] <= hi + 1e-4
    source = myo_sim.load_spec("myoarm_r")
    add_thenar_muscles(source, lengthrange="source")
    model = source.compile()
    for muscle in load_thenar_table()["muscles"]:
        np.testing.assert_allclose(
            model.actuator_lengthrange[model.actuator(muscle["name"]).id], muscle["source_lengthrange_m"]
        )


def test_thenar_moment_arms_act_as_named() -> None:
    """AdP adducts the CMC and flexes the MP joint; APB abducts the CMC (MyoHand signs: +abduction, -MP flexion)."""
    model = myo_sim.load_spec("myoarm_r", thenar_muscles=True).compile()
    q = np.zeros(4)
    arm = {m: -_tendon_geometry(model, f"{m}_tendon", THUMB["r"], q)[1:] for m in ("AdP", "APB", "FPB")}
    assert arm["AdP"][1] < 0 and arm["AdP"][2] < 0
    assert arm["APB"][1] > 0
    assert arm["FPB"][2] < 0


def test_assumed_rows_only_on_request() -> None:
    table = load_passive_moment_table()
    names = lambda m: {m.tendon(i).name for i in range(m.ntendon) if m.tendon(i).name.startswith("passive_")}  # noqa: E731
    measured = names(myo_sim.load_spec("myoarm_r", passive_moments=True).compile())
    assumed = names(myo_sim.load_spec("myoarm_r", passive_moments="assumed").compile())
    extra = assumed - measured
    assert extra and measured < assumed
    assumed_joints = {r["names"]["r"] for r in table["joints"] if r["status"] == "assumed"}
    assert all(any(t.startswith(f"passive_{j}_") for j in assumed_joints) for t in extra)


def test_fitted_rows_reproduce_from_their_source_curves() -> None:
    """The stored springs are what scripts/fit_passive_moments.py fits from each row's source curve."""
    pytest.importorskip("scipy")
    path = Path(__file__).parents[1] / "scripts" / "fit_passive_moments.py"
    spec = importlib.util.spec_from_file_location("fit_passive_moments", path)
    fit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fit)
    model = myo_sim.load_spec("myoarm_r").compile()
    for row in load_passive_moment_table()["joints"]:
        if "source_curve" not in row:
            continue
        joint_range = model.jnt_range[model.joint(row["names"]["r"]).id]
        refit = copy.deepcopy(row)
        fit.fit_row(refit, joint_range)
        q = np.linspace(*joint_range, 50)
        np.testing.assert_allclose(passive_moment(refit, q), passive_moment(row, q), atol=1e-9, err_msg=row["joint"])
        if row["status"] == "measured":
            assert row["fit"]["max_abs_err_Nm"] <= 0.12 * row["fit"]["source_peak_Nm"], row["joint"]
