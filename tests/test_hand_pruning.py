"""The hand models are the arm pruned: every element of the arm that should survive is there, unchanged.

``myohand_r`` / ``myohands`` (and the bare hand fragments) are derived from the canonical arm by removal
only (``build/hand.py``). An arm element should survive when everything it names is still in the hand:

- actuators listed in ``RIGHT_HAND_ACTUATORS``, and the tendons they drive;
- passive tendons whose length still depends on a joint of the hand;
- equalities, sensors and excludes whose elements all exist in the hand;
- contact pairs whose geoms exist and can still move relative to each other.

Each survivor must have identical parameters; tendon lengths must agree at random hand postures.
"""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.build import compose
from myo_sim.build.hand import (
    OBJ,
    RIGHT_HAND_ACTUATORS,
    TENDON_ATTRIBUTES,
    add_side_suffix,
    base_actuator_name,
    element_references,
    equality_object_type,
    rename_elements,
    tendon_path,
)

ACTUATOR_ATTRIBUTES = (
    "biasprm",
    "biastype",
    "ctrllimited",
    "ctrlrange",
    "dynprm",
    "dyntype",
    "forcelimited",
    "forcerange",
    "gainprm",
    "gaintype",
    "gear",
    "lengthrange",
    "actlimited",
    "actrange",
    "actdim",
    "trntype",
)
EQUALITY_ATTRIBUTES = ("type", "data", "solref", "solimp", "active")
PAIR_ATTRIBUTES = ("condim", "friction", "solref", "solreffriction", "solimp", "margin", "gap")
SENSOR_ATTRIBUTES = ("type", "objtype", "reftype", "noise", "cutoff")


CASES = {
    # The bare arm fragment names torso sites, so the composed arm models are the reference.
    "fragment_r": (lambda: myo_sim.load_spec("myoarm_r"), compose.build_right_hand_from_arm_spec, "r"),
    "fragment_l": (lambda: myo_sim.load_spec("myoarms"), compose.build_left_hand_from_arm_spec, "l"),
    "myohand_r": (lambda: myo_sim.load_spec("myoarm_r"), lambda: myo_sim.load_spec("myohand_r"), "r"),
    "myohands": (lambda: myo_sim.load_spec("myoarms"), lambda: myo_sim.load_spec("myohands"), "rl"),
}


@pytest.fixture(scope="module", params=list(CASES))
def case(request: pytest.FixtureRequest) -> dict:
    build_arm, build_hand, sides = CASES[request.param]
    arm, hand = build_arm(), build_hand()
    return {"arm": arm, "hand": hand, "sides": sides, "arm_model": arm.compile(), "hand_model": hand.compile()}


def _side(name: str) -> str:
    """Side of an arm element: the mirrored left arm's names all end in _l; the right arm's may be unsuffixed."""
    return "l" if name.endswith("_l") else "r"


def _hand_name(name: str, sides: str = "") -> str:
    return add_side_suffix(name, _side(name))


def _values(element: object, attributes: tuple[str, ...]) -> dict:
    return {
        a: np.array(getattr(element, a), dtype=float) if not isinstance(getattr(element, a), str) else getattr(element, a)
        for a in attributes
    }


def _assert_same(name: str, a: dict, b: dict) -> None:
    for key, value in a.items():
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(value, b[key], err_msg=f"{name}: {key}")
        else:
            assert value == b[key], f"{name}: {key}"


def _kept_actuators(case: dict) -> list[object]:
    return [
        a
        for a in case["arm"].actuators
        if _side(a.name) in case["sides"] and base_actuator_name(a.name, _side(a.name)) in RIGHT_HAND_ACTUATORS
    ]


def _hand_joint_dofs(case: dict) -> list[int]:
    """DOFs of the arm model whose joints exist in the hand."""
    model, hand = case["arm_model"], case["hand"]
    return [
        int(model.jnt_dofadr[j])
        for j in range(model.njnt)
        if hand.joint(_hand_name(model.joint(j).name, case["sides"])) is not None
    ]


def _random_postures(model: mujoco.MjModel, n: int, seed: int = 0) -> list[dict[str, float]]:
    rng = np.random.default_rng(seed)
    hinges = [j for j in range(model.njnt) if model.jnt_type[j] in (2, 3)]
    return [
        {model.joint(j).name: rng.uniform(*model.jnt_range[j]) if model.jnt_limited[j] else rng.uniform(-1, 1) for j in hinges}
        for _ in range(n)
    ]


def _passive_survivors(case: dict) -> list[str]:
    """Arm tendons without an actuator whose moment arms are nonzero on a hand joint in some posture."""
    model, data = case["arm_model"], mujoco.MjData(case["arm_model"])
    actuated = {a.target for a in case["arm"].actuators if a.trntype == mujoco.mjtTrn.mjTRN_TENDON}
    dofs = _hand_joint_dofs(case)
    names = set()
    for posture in _random_postures(model, 8):
        for joint, value in posture.items():
            data.qpos[model.jnt_qposadr[model.joint(joint).id]] = value
        mujoco.mj_fwdPosition(model, data)
        jac = np.zeros((model.ntendon, model.nv))
        mujoco.mju_sparse2dense(jac, data.ten_J, model.ten_J_rownnz, model.ten_J_rowadr, model.ten_J_colind)
        for t in range(model.ntendon):
            name = model.tendon(t).name
            fixed_joints = [
                model.joint(model.wrap_objid[w]).name
                for w in range(model.tendon_adr[t], model.tendon_adr[t] + model.tendon_num[t])
                if model.wrap_type[w] == 1
            ]
            on_removed = any(case["hand"].joint(_hand_name(j, case["sides"])) is None for j in fixed_joints)
            if name not in actuated and _side(name) in case["sides"] and not on_removed and np.any(jac[t, dofs] != 0):
                names.add(name)
    return sorted(names)


def test_kept_actuators_are_identical(case: dict) -> None:
    kept = _kept_actuators(case)
    assert len(kept) == len(list(case["hand"].actuators))
    for actuator in kept:
        hand_actuator = case["hand"].actuator(_hand_name(actuator.name, case["sides"]))
        assert hand_actuator is not None, actuator.name
        _assert_same(actuator.name, _values(actuator, ACTUATOR_ATTRIBUTES), _values(hand_actuator, ACTUATOR_ATTRIBUTES))
        assert hand_actuator.target == _hand_name(actuator.target, case["sides"])


def test_surviving_tendons_are_identical(case: dict) -> None:
    expected = {a.target for a in _kept_actuators(case)} | set(_passive_survivors(case))
    hand_tendons = {t.name for t in case["hand"].tendons}
    assert {_hand_name(n, case["sides"]) for n in expected} == hand_tendons
    for name in expected:
        arm_tendon, hand_tendon = case["arm"].tendon(name), case["hand"].tendon(_hand_name(name, case["sides"]))
        _assert_same(name, _values(arm_tendon, TENDON_ATTRIBUTES), _values(hand_tendon, TENDON_ATTRIBUTES))
        renamed = [
            (w[0], _hand_name(w[1], case["sides"]) if w[0] == mujoco.mjtWrap.mjWRAP_JOINT else w[1], *w[2:])
            for w in tendon_path(arm_tendon)
        ]
        assert renamed == tendon_path(hand_tendon), name


def test_tendon_lengths_agree_at_random_hand_postures(case: dict) -> None:
    arm, hand = case["arm_model"], case["hand_model"]
    arm_data, hand_data = mujoco.MjData(arm), mujoco.MjData(hand)
    pairs = [
        (t, hand.tendon(_hand_name(arm.tendon(t).name, case["sides"])).id)
        for t in range(arm.ntendon)
        if mujoco.mj_name2id(hand, OBJ.mjOBJ_TENDON, _hand_name(arm.tendon(t).name, case["sides"])) >= 0
    ]
    assert len(pairs) == hand.ntendon
    for posture in _random_postures(hand, 5, seed=3):
        for joint, value in posture.items():
            hand_data.qpos[hand.jnt_qposadr[hand.joint(joint).id]] = value
            arm_joint = next(n for n in (joint, joint[:-2]) if mujoco.mj_name2id(arm, OBJ.mjOBJ_JOINT, n) >= 0)
            arm_data.qpos[arm.jnt_qposadr[arm.joint(arm_joint).id]] = value
        mujoco.mj_fwdPosition(arm, arm_data)
        mujoco.mj_fwdPosition(hand, hand_data)
        a = np.array([arm_data.ten_length[i] for i, _ in pairs])
        h = np.array([hand_data.ten_length[j] for _, j in pairs])
        np.testing.assert_allclose(h, a, rtol=0, atol=1e-9)


def test_surviving_equalities_and_sensors_are_identical(case: dict) -> None:
    def exists(objtype: object, name: str) -> bool:
        finder = {
            OBJ.mjOBJ_JOINT: case["hand"].joint,
            OBJ.mjOBJ_TENDON: case["hand"].tendon,
            OBJ.mjOBJ_SITE: case["hand"].site,
            OBJ.mjOBJ_BODY: case["hand"].body,
        }
        return (
            finder[objtype](_hand_name(name, case["sides"]) if objtype in (OBJ.mjOBJ_JOINT, OBJ.mjOBJ_TENDON) else name)
            is not None
        )

    expected = [e for e in case["arm"].equalities if all(exists(equality_object_type(e), n) for n in (e.name1, e.name2) if n)]
    assert len(expected) == len(list(case["hand"].equalities))
    for equality in expected:
        hand_equality = case["hand"].equality(equality.name)
        assert hand_equality is not None, equality.name
        _assert_same(equality.name, _values(equality, EQUALITY_ATTRIBUTES), _values(hand_equality, EQUALITY_ATTRIBUTES))
    arm_sensors = [s for s in case["arm"].sensors if exists(s.objtype, s.objname)]
    assert len(arm_sensors) == len(list(case["hand"].sensors))
    for sensor in arm_sensors:
        _assert_same(
            sensor.name, _values(sensor, SENSOR_ATTRIBUTES), _values(case["hand"].sensor(sensor.name), SENSOR_ATTRIBUTES)
        )


def test_surviving_contact_pairs_are_identical(case: dict) -> None:
    hand = case["hand_model"]

    def key(pair: object) -> tuple[str, str]:
        return tuple(sorted((pair.geomname1, pair.geomname2)))

    def mobile(pair: object) -> bool:
        ids = [mujoco.mj_name2id(hand, OBJ.mjOBJ_GEOM, n) for n in (pair.geomname1, pair.geomname2)]
        return min(ids) >= 0 and hand.body_weldid[hand.geom_bodyid[ids[0]]] != hand.body_weldid[hand.geom_bodyid[ids[1]]]

    expected = {key(p): p for p in case["arm"].pairs if mobile(p)}
    hand_pairs = {key(p): p for p in case["hand"].pairs}
    assert set(expected) <= set(hand_pairs)
    for name, pair in expected.items():
        _assert_same(str(name), _values(pair, PAIR_ATTRIBUTES), _values(hand_pairs[name], PAIR_ATTRIBUTES))
    for name in hand_pairs:
        ids = [hand.geom(n).id for n in name]
        assert hand.body_weldid[hand.geom_bodyid[ids[0]]] != hand.body_weldid[hand.geom_bodyid[ids[1]]], name


def test_hand_has_no_dangling_references(case: dict) -> None:
    finder = {
        OBJ.mjOBJ_JOINT: case["hand"].joint,
        OBJ.mjOBJ_TENDON: case["hand"].tendon,
        OBJ.mjOBJ_SITE: case["hand"].site,
        OBJ.mjOBJ_BODY: case["hand"].body,
        OBJ.mjOBJ_GEOM: case["hand"].geom,
    }
    for element, refs in element_references(case["hand"]):
        for objtype, name in refs:
            assert finder[objtype](name) is not None, f"{element.name}: {objtype.name} {name!r}"


RENAME_XML = """
<mujoco>
  <worldbody>
    <body name="a">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom name="g1" size=".1" pos="0 0 .5"/>
      <site name="s1" pos="0 0 .1"/>
      <body name="b" pos="0 0 1">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom name="g2" type="cylinder" size=".05 .1" quat="0.707 0.707 0 0"/>
        <site name="side" pos="0 0.1 0"/>
        <site name="s2" pos="0.2 0 0.3"/>
        <site name="s3" pos="-0.2 0 0.3"/>
      </body>
    </body>
    <camera name="c" mode="targetbody" target="b"/>
  </worldbody>
  <tendon>
    <spatial name="t0"><site site="s1"/><site site="s2"/></spatial>
    <fixed name="t1" stiffness="2" springlength="0.1"><joint joint="j1" coef="1"/><joint joint="j2" coef="-0.5"/></fixed>
    <spatial name="t2" damping="0.3"><site site="s1"/><geom geom="g2" sidesite="side"/><site site="s2"/></spatial>
    <spatial name="t3"><site site="s1"/><site site="s2"/><pulley divisor="2"/><site site="s1"/><site site="s3"/></spatial>
  </tendon>
  <equality>
    <joint name="e1" joint1="j1" joint2="j2" polycoef="0 0.5 0 0 0"/>
    <tendon name="e2" tendon1="t1"/>
    <connect name="e3" body1="b" anchor="0 0 0" body2="a"/>
  </equality>
  <actuator>
    <motor name="m1" joint="j1"/>
    <motor name="m2" tendon="t2"/>
    <motor name="m3" site="s1" gear="0 0 0 0 1 0"/>
  </actuator>
  <sensor>
    <jointpos name="sp" joint="j1"/>
    <tendonpos name="st" tendon="t2"/>
    <framepos name="sf" objtype="site" objname="s2" reftype="body" refname="a"/>
  </sensor>
  <contact><pair geom1="g1" geom2="g2"/><exclude body1="a" body2="b"/></contact>
</mujoco>
"""


def test_rename_elements_keeps_every_reference_and_the_numerics() -> None:
    reference = mujoco.MjSpec.from_string(RENAME_XML).compile()
    spec = mujoco.MjSpec.from_string(RENAME_XML)
    rename_elements(
        spec,
        {
            OBJ.mjOBJ_JOINT: {"j1": "j1_r", "j2": "j2_r"},
            OBJ.mjOBJ_TENDON: {"t1": "t1_r", "t2": "t2_r"},
            OBJ.mjOBJ_GEOM: {"g2": "g2_r"},
            OBJ.mjOBJ_SITE: {"s1": "s1_r", "side": "side_r"},
            OBJ.mjOBJ_BODY: {"a": "a_r", "b": "b_r"},
            OBJ.mjOBJ_ACTUATOR: {"m1": "m1_r"},
        },
    )
    model = spec.compile()
    assert [model.tendon(i).name for i in range(model.ntendon)] == ["t0", "t1_r", "t2_r", "t3"]
    for field in (
        "wrap_type",
        "wrap_objid",
        "wrap_prm",
        "tendon_stiffness",
        "tendon_damping",
        "tendon_lengthspring",
        "eq_obj1id",
        "eq_obj2id",
        "eq_data",
        "actuator_trnid",
        "actuator_gear",
        "sensor_objid",
        "sensor_refid",
        "pair_geom1",
        "pair_geom2",
        "exclude_signature",
        "cam_targetbodyid",
    ):
        np.testing.assert_array_equal(getattr(model, field), getattr(reference, field), err_msg=field)
    data, ref_data = mujoco.MjData(model), mujoco.MjData(reference)
    data.qpos[:] = ref_data.qpos[:] = [0.3, -0.7]
    mujoco.mj_forward(model, data)
    mujoco.mj_forward(reference, ref_data)
    np.testing.assert_array_equal(data.ten_length, ref_data.ten_length)
    np.testing.assert_array_equal(data.sensordata, ref_data.sensordata)
