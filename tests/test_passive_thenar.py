"""Passive joint-moment tendons and hand intrinsic muscles: generated XML blocks, model content and data provenance."""

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
    APPLIED_STATUSES,
    load_intrinsic_table,
    load_passive_moment_table,
    passive_moment,
    read_generated_block,
    rows_with_status,
    spring_parameters,
    spring_segments,
)

MODELS = ("myofullbody", "myotorso_arms", "myohands", "myoarm_r")
SCRIPTS = Path(__file__).parents[1] / "scripts"
THUMB = {
    "r": ("cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r", "ip_flexion_r", "mcp2_flexion_r", "mcp2_abduction_r")
    + ("mcp5_flexion_r", "mcp5_abduction_r", "cmc5_flexion_r"),
    "l": ("cmc_flexion_l", "cmc_abduction_l", "mp_flexion_l", "ip_flexion_l", "mcp2_flexion_l", "mcp2_abduction_l")
    + ("mcp5_flexion_l", "mcp5_abduction_l", "cmc5_flexion_l"),
}


def _script(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _applied_rows() -> list[dict]:
    """Applied rows that carry at least one spring."""
    rows = rows_with_status(load_passive_moment_table(), APPLIED_STATUSES)
    return [r for r in rows if spring_segments(r, "positive") or spring_segments(r, "negative")]


def _tendon_count() -> int:
    return sum(len(spring_segments(r, "positive")) + len(spring_segments(r, "negative")) for r in _applied_rows())


def _passive_tendons(model: mujoco.MjModel) -> set[str]:
    return {model.tendon(i).name for i in range(model.ntendon) if model.tendon(i).name.startswith("passive_")}


@pytest.fixture(scope="module")
def fullbody() -> mujoco.MjModel:
    return myo_sim.load_spec("myofullbody").compile()


def test_generated_blocks_are_what_the_scripts_write() -> None:
    """The generated blocks of the arm tendon and muscle files are the scripts' output from the JSON tables."""
    fit = _script("fit_passive_moments")
    assert read_generated_block(compose.RIGHT_ARM_TENDONS_XML, fit.SCRIPT) == fit.tendon_block(load_passive_moment_table())
    gen = _script("generate_hand_intrinsic_muscles")
    table = load_intrinsic_table()
    ranges = gen.architecture_lengthranges(table)
    assert gen.block_labels(table) == [None, "OP", "DI1d", "ADM"]
    for label in gen.block_labels(table):
        assert read_generated_block(compose.RIGHT_ARM_TENDONS_XML, gen.SCRIPT, label) == gen.tendon_block(table, label)
        assert read_generated_block(compose.RIGHT_ARM_MUSCLES_XML, gen.SCRIPT, label) == gen.muscle_block(table, ranges, label)
    text = compose.RIGHT_ARM_CHAIN_XML.read_text()
    assert gen.chain_with_sites(text, table) == text


def test_chain_holds_the_intrinsic_sites() -> None:
    chain = ET.parse(compose.RIGHT_ARM_CHAIN_XML).getroot()
    parents = {child: parent for parent in chain.iter() for child in parent}
    sites = {s.get("name"): s for s in chain.iter("site")}
    for muscle in load_intrinsic_table()["muscles"]:
        for i, site in enumerate(muscle["sites"], start=1):
            element = sites[f"{muscle['name']}-P{i}_r"]
            assert parents[element].get("name") == site["body"]
            np.testing.assert_allclose([float(v) for v in element.get("pos").split()], site["pos"], atol=1e-9)


def test_intrinsic_fit_reproduces_the_table() -> None:
    """The tabled sites and passive scales are what ``generate_hand_intrinsic_muscles.py --fit`` fits from the start points."""
    pytest.importorskip("scipy")
    gen = _script("generate_hand_intrinsic_muscles")
    table = load_intrinsic_table()
    refit = copy.deepcopy(table)
    gen.fit(refit)
    for old, new in zip(table["muscles"], refit["muscles"]):
        for a, b in zip(old["sites"], new["sites"]):
            np.testing.assert_allclose(b["pos"], a["pos"], atol=2e-6, err_msg=old["name"])
        assert new["fit"]["result"].get("rms_z") == pytest.approx(old["fit"]["result"].get("rms_z"), abs=0.01)


def test_intrinsic_fit_keeps_the_fibre_length_window() -> None:
    """Every fitted row stays inside the fibre-length window over its measurement box (the hard constraint)."""
    table = load_intrinsic_table()
    lo, hi = table["path_fit"]["fibre_length_window"]
    for row in table["muscles"]:
        box = row["fit"]["result"]["fibre_length_measurement_box"]
        assert lo - 0.01 <= box[0] and box[1] <= hi + 0.01, row["name"]
        assert row["fit"]["result"]["fibre_length_full_range"][1] <= table["path_fit"]["full_range_max"] + 0.01, row["name"]


@pytest.mark.parametrize("name", MODELS)
def test_models_carry_the_intrinsic_muscles_and_every_applied_row(name: str) -> None:
    hands = 1 if name == "myoarm_r" else 2
    model = myo_sim.load_spec(name).compile()
    assert len(_passive_tendons(model)) == _tendon_count() * hands
    names = {model.actuator(i).name for i in range(model.nu)}
    for muscle in (m["name"] for m in load_intrinsic_table()["muscles"]):
        assert f"{muscle}_r" in names, muscle
        assert (f"{muscle}_l" in names) == (hands == 2), muscle


def test_both_sides_come_from_the_right_arm_files() -> None:
    """Every generated element appears on both arms; the left one through the mirroring."""
    model = myo_sim.load_model("myoarms")
    labels = (None, "OP", "DI1d", "ADM")
    blocks = [read_generated_block(compose.RIGHT_ARM_TENDONS_XML, "fit_passive_moments.py")]
    blocks += [
        read_generated_block(path, "generate_hand_intrinsic_muscles.py", label)
        for label in labels
        for path in (compose.RIGHT_ARM_TENDONS_XML, compose.RIGHT_ARM_MUSCLES_XML)
    ]
    for block in blocks:
        for element in ET.fromstring(f"<block>{block}</block>").iter():
            if element.tag not in ("spatial", "fixed", "muscle"):
                continue
            obj = mujoco.mjtObj.mjOBJ_ACTUATOR if element.tag == "muscle" else mujoco.mjtObj.mjOBJ_TENDON
            name = element.get("name")
            left = name.replace("_r_tendon", "_l_tendon") if name.endswith("_r_tendon") else f"{name[:-2]}_l"
            assert mujoco.mj_name2id(model, obj, name) >= 0 and mujoco.mj_name2id(model, obj, left) >= 0, name


def test_table_rows_are_sourced() -> None:
    table = load_passive_moment_table()
    assert rows_with_status(table)
    for row in table["joints"]:
        assert row["status"] in ("measured", "assumed", "gap", "covered_by_muscles", "validation_only")
        assert row["scales_with"] == "none"
        if row["status"] in APPLIED_STATUSES:
            assert row["source"] in table["sources"] and table["sources"][row["source"]]["doi"]
            assert "source_curve" in row and "subtract_model_muscles" in row["source_curve"]
            assert "positive" in row or "negative" in row or row["fit"]["spring"].startswith("none")
            if row["status"] == "assumed":  # a transfer between joints says so
                assert "Assumption" in row["notes"]
        else:  # nothing is added; a gap says where values could come from
            assert "positive" not in row and "negative" not in row
            if row["status"] == "gap":
                assert all(src in table["sources"] for src in row["candidate_sources"])
            else:  # covered by muscles / validation only: the source is kept for reference
                assert row["source"] in table["sources"] and "reference" in row
    intrinsic = load_intrinsic_table()
    for muscle in intrinsic["muscles"]:
        assert muscle["architecture_source"] in intrinsic["sources"]
        assert all(
            site["start"]["source"] in intrinsic["sources"] or "(" in site["start"]["source"] for site in muscle["sites"]
        )
        assert all(t["source"] in intrinsic["sources"] for t in muscle["fit"].get("targets", {}).values())
        assert {muscle[k]["scales_with"] for k in ("force_N", "optimal_fiber_length_m")} == {"mass", "length"}


def test_passive_moments_match_the_table(fullbody: mujoco.MjModel) -> None:
    """The springs give the tabled moments; the ligaments (``lig_``, extensor mechanism) come on top of them."""
    ligaments = _script("fit_passive_moments").ligament_moment
    data = mujoco.MjData(fullbody)
    for row in _applied_rows():
        for side in ("r", "l"):
            joint = fullbody.joint(row["names"][side])
            for q in np.linspace(*fullbody.jnt_range[joint.id], 7):
                data.qpos[:] = fullbody.qpos0
                data.qvel[:] = 0.0
                data.qpos[joint.qposadr[0]] = q
                mujoco.mj_forward(fullbody, data)
                springs = data.qfrc_passive[joint.dofadr[0]] - ligaments(fullbody, data, joint.dofadr[0])
                np.testing.assert_allclose(springs, passive_moment(row, q), atol=1e-9)
    wrist = next(r for r in rows_with_status(load_passive_moment_table()) if r["names"]["r"] == "flexion_r")
    # Formica 2012 measured the total: springs plus the wrist muscles at act = 0, in the row's posture. A
    # one-segment linear spring per side cannot follow the muscles' curvature, so the total matches within
    # the error the fit recorded, which stays a small part of the measured peak.
    q = np.array([0.2, -0.2])
    muscles = _script("fit_passive_moments").muscle_baseline(wrist["source_curve"], "flexion_r", q)
    tolerance = wrist["fit"]["max_abs_err_Nm"] + 1e-6
    np.testing.assert_allclose(passive_moment(wrist, q) + muscles, [-0.554 * 0.2, 1.021 * 0.2], rtol=0.0, atol=tolerance)
    assert wrist["fit"]["max_abs_err_Nm"] <= 0.2 * wrist["fit"]["source_peak_Nm"]


def _joint_id(model: mujoco.MjModel, name: str, side: str) -> int:
    """Joint id of the right-side joint *name* on *side* of a composed model (hand models suffix every joint)."""
    base = name[:-2] if name.endswith("_r") else name
    for candidate in (name, f"{base}_r") if side == "r" else (f"{base}_l",):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, candidate) >= 0:
            return model.joint(candidate).id
    raise KeyError(name)


@pytest.mark.parametrize("name", MODELS)
def test_finger_damping_is_the_tabled_value(name: str) -> None:
    """The chain holds each tabled damping (what the script writes) and every model has it on both hands."""
    fit = _script("fit_passive_moments")
    table = load_passive_moment_table()
    text = compose.RIGHT_ARM_CHAIN_XML.read_text()
    assert fit.chain_with_joint_attributes(text, table) == text
    rows = [r for r in table["joints"] if "damping" in r]
    assert len(rows) == 12
    model = myo_sim.load_spec(name).compile()
    for row in rows:
        assert row["damping"]["source"] in table["sources"]
        for side in ("r",) if name == "myoarm_r" else ("r", "l"):
            dof = model.jnt_dofadr[_joint_id(model, row["names"]["r"], side)]
            assert model.dof_damping[dof] == row["damping"]["value"], (row["joint"], side)


@pytest.mark.parametrize("name", MODELS)
def test_tabled_joint_ranges_are_in_every_model(name: str) -> None:
    """The PIP rows' measured range (Leijnse et al. 2010) is the joint range on both hands of every model."""
    table = load_passive_moment_table()
    rows = [r for r in table["joints"] if "range" in r]
    assert {r["names"]["r"] for r in rows} == {f"pm{d}_flexion_r" for d in (2, 3, 4, 5)}
    model = myo_sim.load_spec(name).compile()
    for row in rows:
        assert row["range"]["source"] in table["sources"]
        for side in ("r",) if name == "myoarm_r" else ("r", "l"):
            joint = _joint_id(model, row["names"]["r"], side)
            np.testing.assert_allclose(np.degrees(model.jnt_range[joint]), row["range"]["value_deg"], atol=1e-4)


def test_cpu_and_mujoco_warp_passive_moments_agree(fullbody: mujoco.MjModel) -> None:
    mjw = pytest.importorskip("mujoco_warp")
    wp = pytest.importorskip("warp")
    data = mujoco.MjData(fullbody)
    rng = np.random.default_rng(0)
    dofs = []
    for row in _applied_rows():
        for side in ("r", "l"):
            joint = fullbody.joint(row["names"][side])
            data.qpos[joint.qposadr[0]] = rng.uniform(*fullbody.jnt_range[joint.id])
            data.qvel[joint.dofadr[0]] = rng.uniform(-2.0, 2.0)  # damping counts too
            dofs.append(joint.dofadr[0])
    mujoco.mj_forward(fullbody, data)
    with wp.ScopedDevice("cpu"):
        model_w = mjw.put_model(fullbody)
        # The hand self-contacts need more contact rows than mujoco-warp's default guess (48 overflows).
        data_w = mjw.put_data(fullbody, data, nconmax=max(256, 2 * data.ncon), njmax=max(1024, 2 * data.nefc))
        mjw.forward(model_w, data_w)
        warp_passive = data_w.qfrc_passive.numpy()[0]
    np.testing.assert_allclose(warp_passive[dofs], data.qfrc_passive[dofs], atol=1e-5)


def test_left_and_right_hands_are_symmetric(fullbody: mujoco.MjModel) -> None:
    data = mujoco.MjData(fullbody)
    rng = np.random.default_rng(1)
    for row in _applied_rows():
        right, left = fullbody.joint(row["names"]["r"]), fullbody.joint(row["names"]["l"])
        np.testing.assert_allclose(fullbody.jnt_range[left.id], fullbody.jnt_range[right.id])
        q = rng.uniform(*fullbody.jnt_range[right.id])
        data.qpos[:] = fullbody.qpos0
        data.qpos[right.qposadr[0]] = data.qpos[left.qposadr[0]] = q
        mujoco.mj_forward(fullbody, data)
        np.testing.assert_allclose(data.qfrc_passive[left.dofadr[0]], data.qfrc_passive[right.dofadr[0]], atol=1e-9)
    for muscle in load_intrinsic_table()["muscles"]:
        r, left = fullbody.actuator(muscle["name"] + "_r").id, fullbody.actuator(muscle["name"] + "_l").id
        np.testing.assert_allclose(fullbody.actuator_gainprm[left], fullbody.actuator_gainprm[r])
        np.testing.assert_allclose(fullbody.actuator_lengthrange[left], fullbody.actuator_lengthrange[r], atol=1e-9)
        for _ in range(5):  # same lengths and moment arms at the same thumb angles
            q = np.array([rng.uniform(*fullbody.jnt_range[fullbody.joint(j).id]) for j in THUMB["r"]])
            right_geo = _tendon_geometry(fullbody, f"{muscle['name']}_r_tendon", THUMB["r"], q)
            left_geo = _tendon_geometry(fullbody, f"{muscle['name']}_l_tendon", THUMB["l"], q)
            np.testing.assert_allclose(left_geo, right_geo, atol=1e-6)


def _equilibrium(row: dict) -> float:
    return spring_segments(row, "positive" if "positive" in row else "negative")[0]["onset"]


def test_relaxed_hands_settle_inside_their_ranges() -> None:
    """Without gravity or excitation, joints whose spring rest angle lies inside the range settle off their limits, the same on both hands."""
    model = myo_sim.load_spec("myofullbody").compile()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    rows = _applied_rows()
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


def test_intrinsic_muscles_follow_jacobson_and_share_the_forearm_class() -> None:
    model = myo_sim.load_spec("myotorso_arms").compile()
    table = load_intrinsic_table()
    reference = model.actuator(f"{table['template_muscle']}_r").id
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    for muscle in table["muscles"]:
        act = model.actuator(muscle["name"] + "_r").id
        assert model.actuator_gainprm[act][2] == pytest.approx(muscle["force_N"]["value"])
        fpmax = muscle["fpmax"]["value"] if "fpmax" in muscle else model.actuator_gainprm[reference][7]
        assert model.actuator_gainprm[act][7] == pytest.approx(fpmax)
        np.testing.assert_array_equal(
            np.delete(model.actuator_gainprm[act][:9], [2, 7]), np.delete(model.actuator_gainprm[reference][:9], [2, 7])
        )
        np.testing.assert_array_equal(model.actuator_dynprm[act], model.actuator_dynprm[reference])
        # Architecture length range: rigid tendon of Barry's slack length, range 0.75-1.05 of L_fo.
        l_fo, alpha = muscle["optimal_fiber_length_m"]["value"], np.radians(muscle["pennation_deg"]["value"])
        l_ts = (data.actuator_length[act] - l_fo * np.cos(alpha)) / 1.033
        np.testing.assert_allclose(model.actuator_lengthrange[act], [l_ts + 0.75 * l_fo, l_ts + 1.05 * l_fo], atol=1e-9)


def test_intrinsic_moment_arms_act_as_named() -> None:
    """Signs at the Smutz 1998 neutral (mid-point of the thumb CMC ranges) and index rest, as dL/dq.

    MyoHand: +q extends the thumb CMC and MP and abducts the CMC; +q flexes and radially abducts MCP2.
    Smutz: OP flexes and abducts the CMC; AdPt and AdPo flex and adduct the CMC and flex the MP; APB abducts
    the CMC; FPB flexes the CMC and MP and abducts the CMC. An 1983: DI1 flexes and radially abducts MCP2.
    """
    model = myo_sim.load_spec("myoarm_r").compile()
    q = np.array(
        [np.mean(model.joint("cmc_flexion_r").range), np.mean(model.joint("cmc_abduction_r").range), 0, 0, 0, 0, 0, 0, 0]
    )
    arm = {
        m: _tendon_geometry(model, f"{m}_r_tendon", THUMB["r"], q)[1:]
        for m in ("OP", "AdPt", "AdPo", "APB", "FPBs", "FPBd", "DI1s", "DI1d")
    }
    assert arm["OP"][0] > 0 and arm["OP"][1] < 0
    for m in ("AdPt", "AdPo"):
        assert arm[m][0] > 0 and arm[m][1] > 0 and arm[m][2] > 0, m
    assert arm["APB"][1] < 0
    for m in ("FPBs", "FPBd"):
        assert arm[m][0] > 0 and arm[m][1] < 0 and arm[m][2] > 0, m
    for m in ("DI1s", "DI1d"):
        assert arm[m][4] < 0 and arm[m][5] < 0, m
    assert 0.0 < arm["DI1s"][1] < 5e-3  # small CMC adduction arm (Brand & Hollister 1993)
    # Hypothenar (dL/dq; +q flexes MCP5 and CMC5 and moves digit 5 radially): ADM and FDM flex MCP5 and
    # abduct it ulnarly, all three flex CMC5 (An 1979 for ADM; anatomy for FDM and ODM).
    little = {m: _tendon_geometry(model, f"{m}_r_tendon", THUMB["r"], q)[1:] for m in ("ADM", "FDM", "ODM")}
    for m in ("ADM", "FDM"):
        assert little[m][6] < 0 and little[m][7] > 0, m
    for m in ("ADM", "FDM", "ODM"):
        assert little[m][8] < 0, m


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
        if "rmse_Nm" in row["fit"]:  # the springs never make the total (muscles + springs) worse than the muscles alone
            assert row["fit"]["rmse_Nm"] <= row["fit"]["rmse_without_spring_Nm"], row["joint"]
