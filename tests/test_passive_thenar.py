"""Passive joint-moment tendons and thenar muscles: XML fragments, defaults, opt-out and data provenance."""

from __future__ import annotations

import copy
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path
from types import ModuleType

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build import compose
from myo_sim.build.hand_data import (
    load_passive_moment_table,
    load_thenar_table,
    passive_moment,
    passive_tendon_name,
    rows_with_status,
    spring_parameters,
    spring_segments,
)

MODELS = ("myofullbody", "myotorso_arms", "myohands", "myoarm_r")
SCRIPTS = Path(__file__).parents[1] / "scripts"
THUMB = {
    "r": ("cmc_flexion", "cmc_abduction", "mp_flexion_r", "ip_flexion_r"),
    "l": ("cmc_flexion_l", "cmc_abduction_l", "mp_flexion_l", "ip_flexion_l"),
}
OFF = {"thenar_muscles": False, "passive_moments": False}


def _script(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _tendon_count(status: str) -> int:
    rows = rows_with_status(load_passive_moment_table(), (status,))
    return sum(len(spring_segments(r, "positive")) + len(spring_segments(r, "negative")) for r in rows)


def _passive_tendons(model: mujoco.MjModel) -> set[str]:
    return {model.tendon(i).name for i in range(model.ntendon) if model.tendon(i).name.startswith("passive_")}


@pytest.fixture(scope="module")
def fullbody() -> mujoco.MjModel:
    return myo_sim.load_spec("myofullbody", passive_moments="assumed").compile()


def test_fragments_are_generated_from_the_tables() -> None:
    """The XML fragments are what the scripts write from the JSON tables (no hand edits)."""
    fit = _script("fit_passive_moments")
    table = load_passive_moment_table()
    for status, path in fit.FRAGMENTS.items():
        assert path.read_text() == fit.fragment_xml(table, status), path.name
    gen = _script("generate_thenar_fragments")
    thenar = load_thenar_table()
    assert compose.RIGHT_ARM_THENAR_TENDONS_XML.read_text() == gen.tendon_fragment_xml(thenar)
    expected = gen.muscle_fragment_xml(thenar, gen.architecture_lengthranges(thenar))
    assert compose.RIGHT_ARM_THENAR_MUSCLES_XML.read_text() == expected


def test_chain_holds_the_thenar_sites() -> None:
    chain = ET.parse(compose.RIGHT_ARM_CHAIN_XML).getroot()
    parents = {child: parent for parent in chain.iter() for child in parent}
    sites = {s.get("name"): s for s in chain.iter("site")}
    for muscle in load_thenar_table()["muscles"]:
        for i, (body, pos) in enumerate(muscle["sites"], start=1):
            site = sites[f"{muscle['name']}-P{i}_r"]
            assert parents[site].get("name") == f"{body}_r"
            np.testing.assert_array_equal([float(v) for v in site.get("pos").split()], pos)


@pytest.mark.parametrize("name", MODELS)
def test_default_has_thenar_muscles_and_measured_springs(name: str) -> None:
    hands = 1 if name == "myoarm_r" else 2
    off = myo_sim.load_spec(name, **OFF).compile()
    default = myo_sim.load_spec(name).compile()
    assert default.nu == off.nu + 3 * hands
    assert default.nsite == off.nsite  # the thenar sites are always in the chain
    assert len(_passive_tendons(default)) == _tendon_count("measured") * hands
    assert default.ntendon == off.ntendon + (3 + _tendon_count("measured")) * hands
    assumed = myo_sim.load_spec(name, passive_moments="assumed").compile()
    assert len(_passive_tendons(assumed)) == (_tendon_count("measured") + _tendon_count("assumed")) * hands
    thenar_only = myo_sim.load_spec(name, passive_moments=False).compile()
    assert not _passive_tendons(thenar_only) and thenar_only.nu == default.nu


def test_opt_out_restores_the_previous_action_space() -> None:
    assert myo_sim.load_model("myofullbody", **OFF).nu == 416
    hand = myo_sim.load_model("myohand_r", **OFF)
    assert hand.nu == 39 and not _passive_tendons(hand)
    for name in ("AdP", "FPB", "APB"):
        assert mujoco.mj_name2id(hand, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_r") < 0


def test_flags_do_not_leak_between_builds() -> None:
    myo_sim.load_spec("myoarm_r", **OFF)
    assert myo_sim.load_model("myoarm_r").nu == 66
    with pytest.raises(ValueError, match="passive_moments"):
        myo_sim.load_spec("myoarm_r", passive_moments="measured")


def test_both_sides_come_from_the_right_fragments() -> None:
    """Every fragment element appears on both arms; the left one through the mirroring."""
    model = myo_sim.load_model("myoarms", passive_moments="assumed")
    paths = (
        compose.RIGHT_ARM_THENAR_TENDONS_XML,
        compose.RIGHT_ARM_THENAR_MUSCLES_XML,
        compose.RIGHT_ARM_PASSIVE_TENDONS_XML,
        compose.RIGHT_ARM_PASSIVE_ASSUMED_TENDONS_XML,
    )
    for path in paths:
        for element in ET.parse(path).getroot().iter():
            if element.tag not in ("spatial", "fixed", "muscle"):
                continue
            obj = mujoco.mjtObj.mjOBJ_ACTUATOR if element.tag == "muscle" else mujoco.mjtObj.mjOBJ_TENDON
            name = element.get("name")
            left = f"{name[:-2]}_l" if name.endswith("_r") else f"{name}_l"
            assert mujoco.mj_name2id(model, obj, name) >= 0 and mujoco.mj_name2id(model, obj, left) >= 0, name


def test_table_rows_are_sourced() -> None:
    table = load_passive_moment_table()
    assert rows_with_status(table)
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
    for row in rows_with_status(load_passive_moment_table(), ("measured", "assumed")):
        for side in ("r", "l"):
            joint = fullbody.joint(row["names"][side])
            for q in np.linspace(*fullbody.jnt_range[joint.id], 7):
                data.qpos[:] = fullbody.qpos0
                data.qvel[:] = 0.0
                data.qpos[joint.qposadr[0]] = q
                mujoco.mj_forward(fullbody, data)
                np.testing.assert_allclose(data.qfrc_passive[joint.dofadr[0]], passive_moment(row, q), atol=1e-9)
    wrist = next(r for r in rows_with_status(load_passive_moment_table()) if r["names"]["r"] == "flexion_r")
    np.testing.assert_allclose(passive_moment(wrist, [0.2, -0.2]), [-0.554 * 0.2, 1.021 * 0.2])


def test_cpu_and_mujoco_warp_passive_moments_agree(fullbody: mujoco.MjModel) -> None:
    mjw = pytest.importorskip("mujoco_warp")
    wp = pytest.importorskip("warp")
    data = mujoco.MjData(fullbody)
    rng = np.random.default_rng(0)
    dofs = []
    for row in rows_with_status(load_passive_moment_table(), ("measured", "assumed")):
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
    for row in rows_with_status(load_passive_moment_table(), ("measured", "assumed")):
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
    return spring_segments(row, "positive" if "positive" in row else "negative")[0]["onset"]


def test_relaxed_hands_settle_inside_their_ranges() -> None:
    """Without gravity or excitation, joints whose spring rest angle lies inside the range settle off their limits, the same on both hands."""
    model = myo_sim.load_spec("myofullbody").compile()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    rows = rows_with_status(load_passive_moment_table())
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
    tendons = []
    for direction in ("positive", "negative"):
        for i, part in enumerate(spring_segments(row, direction)):
            springlength, stiffness = spring_parameters(direction, part["onset"], part["stiffness"])
            tendons.append(
                f'<fixed name="t{direction}{i}" springlength="{springlength[0]} {springlength[1]}"'
                f' stiffness="{" ".join(map(str, stiffness))}"><joint joint="j" coef="1"/></fixed>'
            )
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><joint name="j" range="-2 2"/><geom size=".1"/></body></worldbody>'
        f"<tendon>{''.join(tendons)}</tendon></mujoco>"
    )
    data = mujoco.MjData(model)
    for q in (-0.7, -0.15, 0.0, 0.05, 0.6):
        data.qpos[0] = q
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.qfrc_passive[0], passive_moment(row, q), atol=1e-12)
        assert data.qfrc_passive[0] * q <= 0.0


def test_subject_scale_multiplies_rows() -> None:
    base = myo_sim.load_spec("myoarm_r").compile()
    scaled = myo_sim.load_spec("myoarm_r", subject_scale={"wrist_flexion": 2.0, "AdP.force": 0.5, "APB.length": 1.1}).compile()
    name = passive_tendon_name("wrist_flexion", "positive")
    stiffness = base.tendon_stiffness[base.tendon(name).id]
    np.testing.assert_allclose(scaled.tendon_stiffness[scaled.tendon(name).id], 2 * stiffness)
    assert scaled.actuator_gainprm[scaled.actuator("AdP").id, 2] == pytest.approx(
        0.5 * base.actuator_gainprm[base.actuator("AdP").id, 2]
    )
    span = lambda m: float(np.diff(m.actuator_lengthrange[m.actuator("APB").id])[0])  # noqa: E731
    assert span(scaled) == pytest.approx(1.1 * span(base))  # 0.3 L_fo
    both = myo_sim.load_spec("myoarms", subject_scale={"wrist_flexion": 2.0}).compile()
    np.testing.assert_allclose(both.tendon_stiffness[both.tendon(f"{name[:-2]}_l").id], 2 * stiffness)


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


def test_thenar_muscles_follow_jacobson_and_share_the_forearm_class() -> None:
    model = myo_sim.load_spec("myotorso_arms").compile()
    reference = model.actuator("OP").id
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    for muscle in load_thenar_table()["muscles"]:
        act = model.actuator(muscle["name"]).id
        assert model.actuator_gainprm[act][2] == pytest.approx(muscle["force_N"]["value"])
        np.testing.assert_array_equal(
            np.delete(model.actuator_gainprm[act][:9], 2), np.delete(model.actuator_gainprm[reference][:9], 2)
        )
        np.testing.assert_array_equal(model.actuator_dynprm[act], model.actuator_dynprm[reference])
        # Architecture length range: rigid tendon of Barry's slack length, range 0.75-1.05 of L_fo.
        l_fo, alpha = muscle["optimal_fiber_length_m"]["value"], np.radians(muscle["pennation_deg"]["value"])
        l_ts = (data.actuator_length[act] - l_fo * np.cos(alpha)) / 1.033
        np.testing.assert_allclose(model.actuator_lengthrange[act], [l_ts + 0.75 * l_fo, l_ts + 1.05 * l_fo], atol=1e-9)


def test_thenar_moment_arms_act_as_named() -> None:
    """AdP adducts the CMC and flexes the MP joint; APB abducts the CMC (MyoHand signs: +abduction, -MP flexion)."""
    model = myo_sim.load_spec("myoarm_r").compile()
    q = np.zeros(4)
    arm = {m: -_tendon_geometry(model, f"{m}_tendon", THUMB["r"], q)[1:] for m in ("AdP", "APB", "FPB")}
    assert arm["AdP"][1] < 0 and arm["AdP"][2] < 0
    assert arm["APB"][1] > 0
    assert arm["FPB"][2] < 0


def test_assumed_rows_only_on_request() -> None:
    table = load_passive_moment_table()
    measured = _passive_tendons(myo_sim.load_spec("myoarm_r").compile())
    assumed = _passive_tendons(myo_sim.load_spec("myoarm_r", passive_moments="assumed").compile())
    extra = assumed - measured
    assert extra and measured < assumed
    assumed_rows = {r["joint"] for r in table["joints"] if r["status"] == "assumed"}
    assert all(any(t.startswith(f"passive_{row}_") for row in assumed_rows) for t in extra)


def test_fitted_rows_reproduce_from_their_source_curves() -> None:
    """The stored springs are what scripts/fit_passive_moments.py fits from each row's source curve."""
    pytest.importorskip("scipy")
    fit = _script("fit_passive_moments")
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
