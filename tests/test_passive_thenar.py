"""Opt-in passive joint moments and thenar muscles (myo_sim.build.passive / thenar)."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build.passive import APPLIED, load_passive_moment_table, measured_rows, passive_moment
from myo_sim.build.thenar import load_thenar_table

MODELS = ("myotorso_arms", "myohands", "myoarm_r")
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


@pytest.mark.parametrize("name", MODELS)
def test_opt_out_leaves_the_model_unchanged(name: str) -> None:
    # Compiled fields, not XML: two plain builds already differ in mesh-asset order.
    base = myo_sim.load_spec(name).compile()
    off = myo_sim.load_spec(name, passive_moments=False, thenar_muscles=False).compile()
    assert (off.nu, off.ntendon, off.nsite, off.njnt) == (base.nu, base.ntendon, base.nsite, base.njnt)
    for field in _FIELDS:
        np.testing.assert_array_equal(getattr(off, field), getattr(base, field), err_msg=field)


def test_table_rows_are_sourced() -> None:
    table = load_passive_moment_table()
    rows = measured_rows(table)
    assert rows, "at least one measured row"
    for row in table["joints"]:
        assert row["status"] in ("measured", "assumed", "gap", "covered_by_muscles", "validation_only")
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
    for muscle in load_thenar_table()["muscles"]:
        assert muscle["source"] in load_thenar_table()["sources"]


def test_passive_moments_match_the_table() -> None:
    model = myo_sim.load_spec("myotorso_arms", passive_moments="assumed").compile()
    data = mujoco.MjData(model)
    rows = measured_rows(load_passive_moment_table(), APPLIED["assumed"])
    for row in rows:
        for side in ("r", "l"):
            joint = model.joint(row["names"][side])
            for q in np.linspace(*model.jnt_range[joint.id], 11):
                data.qpos[:] = model.qpos0
                data.qvel[:] = 0.0
                data.qpos[model.jnt_qposadr[joint.id]] = q
                mujoco.mj_forward(model, data)
                np.testing.assert_allclose(data.qfrc_passive[model.jnt_dofadr[joint.id]], passive_moment(row, q), atol=1e-9)
    # Direction-specific: wrist flexion stiffness 0.554, extension 1.021 Nm/rad (Formica et al. 2012).
    wrist = next(r for r in rows if r["names"]["r"] == "flexion_r")
    np.testing.assert_allclose(passive_moment(wrist, [0.2, -0.2]), [-0.554 * 0.2, 1.021 * 0.2])


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


def test_thenar_muscles_mirror_and_copy_the_template() -> None:
    model = myo_sim.load_spec("myotorso_arms", thenar_muscles=True).compile()
    base = myo_sim.load_spec("myotorso_arms").compile()
    assert model.nu == base.nu + 6
    rng = np.random.default_rng(0)
    template = model.actuator("OP").id
    for muscle in load_thenar_table()["muscles"]:
        for side, suffix in (("r", ""), ("l", "_l")):
            act = model.actuator(muscle["name"] + suffix).id
            assert model.actuator_gainprm[act][2] == muscle["force"]
            np.testing.assert_array_equal(
                np.delete(model.actuator_gainprm[act][:9], 2), np.delete(model.actuator_gainprm[template][:9], 2)
            )
            np.testing.assert_array_equal(model.actuator_dynprm[act], model.actuator_dynprm[template])
        for _ in range(5):  # left hand = mirror of the right: same lengths and moment arms
            q = np.array([rng.uniform(*model.jnt_range[model.joint(j).id]) for j in THUMB["r"]])
            right = _tendon_geometry(model, f"{muscle['name']}_tendon", THUMB["r"], q)
            left = _tendon_geometry(model, f"{muscle['name']}_tendon_l", THUMB["l"], q)
            np.testing.assert_allclose(left, right, atol=1e-6)


def test_thenar_lengthrange_covers_the_tendon_over_the_thumb_range() -> None:
    model = myo_sim.load_spec("myoarm_r", thenar_muscles=True).compile()
    rng = np.random.default_rng(1)
    for muscle in load_thenar_table()["muscles"]:
        lo, hi = model.actuator_lengthrange[model.actuator(muscle["name"]).id]
        for _ in range(50):
            q = np.array([rng.uniform(*model.jnt_range[model.joint(j).id]) for j in THUMB["r"]])
            length = _tendon_geometry(model, f"{muscle['name']}_tendon", THUMB["r"], q)[0]
            assert lo - 1e-4 <= length <= hi + 1e-4
    spec = myo_sim.load_spec("myoarm_r")
    from myo_sim.build.thenar import add_thenar_muscles

    add_thenar_muscles(spec, lengthrange="source")
    model = spec.compile()
    for muscle in load_thenar_table()["muscles"]:
        np.testing.assert_allclose(model.actuator_lengthrange[model.actuator(muscle["name"]).id], muscle["source_lengthrange"])


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
    measured = myo_sim.load_spec("myoarm_r", passive_moments=True).compile()
    assumed = myo_sim.load_spec("myoarm_r", passive_moments="assumed").compile()
    names = lambda m: {m.tendon(i).name for i in range(m.ntendon) if m.tendon(i).name.startswith("passive_")}  # noqa: E731
    extra = names(assumed) - names(measured)
    assert extra and names(measured) < names(assumed)
    assumed_joints = {r["names"]["r"] for r in table["joints"] if r["status"] == "assumed"}
    assert all(any(t.startswith(f"passive_{j}_") for j in assumed_joints) for t in extra)


def test_fitted_rows_reproduce_from_their_source_curves() -> None:
    """The stored springs are what scripts/fit_passive_moments.py fits from each row's source curve."""
    import copy
    import importlib.util
    from pathlib import Path

    pytest.importorskip("scipy")
    path = Path(__file__).parents[1] / "scripts" / "fit_passive_moments.py"
    spec = importlib.util.spec_from_file_location("fit_passive_moments", path)
    fit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fit)
    model = myo_sim.load_spec("myoarm_r").compile()
    for row in load_passive_moment_table()["joints"]:
        if "source_curve" not in row:
            continue
        refit = copy.deepcopy(row)
        fit.fit_row(refit, model.jnt_range[model.joint(row["names"]["r"]).id])
        q = np.linspace(*model.jnt_range[model.joint(row["names"]["r"]).id], 50)
        np.testing.assert_allclose(passive_moment(refit, q), passive_moment(row, q), atol=1e-9, err_msg=row["joint"])
        assert row["fit"]["max_abs_err_Nm"] <= 0.12 * row["fit"]["source_peak_Nm"], row["joint"]
