"""Dependency-safe joint and tendon deletion keeps the spec compilable."""

from __future__ import annotations

import mujoco

import myo_sim
from myo_sim.build.spec_edit import delete_joint, delete_tendon, immobilize_subtree

XML = """
<mujoco>
  <worldbody>
    <body name="b1">
      <geom type="sphere" size="0.05"/>
      <joint name="j1" type="hinge" range="-1 1"/>
      <body name="b2" pos="0 0 0.2">
        <geom type="sphere" size="0.05"/>
        <joint name="j2" type="hinge" range="-1 1"/>
        <body name="b3" pos="0 0 0.2">
          <geom type="sphere" size="0.05"/>
          <joint name="j3" type="hinge" range="-1 1"/>
          <joint name="j4" axis="0 1 0" type="hinge" range="-1 1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t12"><joint joint="j1" coef="1"/><joint joint="j2" coef="1"/></fixed>
    <fixed name="t34"><joint joint="j3" coef="1"/><joint joint="j4" coef="1"/></fixed>
    <fixed name="t4"><joint joint="j4" coef="1"/></fixed>
    <fixed name="t_other"><joint joint="j2" coef="1"/><joint joint="j3" coef="1"/></fixed>
  </tendon>
  <equality>
    <joint joint1="j1" joint2="j2"/>
    <joint joint1="j3" joint2="j4"/>
    <tendon tendon1="t12" tendon2="t34"/>
    <tendon tendon1="t4" tendon2="t_other"/>
  </equality>
  <actuator>
    <motor name="a_j1" joint="j1"/>
    <motor name="a_j2" joint="j2"/>
    <motor name="a_j4" joint="j4"/>
    <motor name="a_t12" tendon="t12"/>
    <motor name="a_t34" tendon="t34"/>
    <motor name="a_t_other" tendon="t_other"/>
  </actuator>
  <sensor>
    <jointpos name="s_j1" joint="j1"/>
    <jointpos name="s_j2" joint="j2"/>
    <jointpos name="s_j4" joint="j4"/>
    <tendonpos name="s_t12" tendon="t12"/>
    <tendonpos name="s_t34" tendon="t34"/>
    <tendonpos name="s_t_other" tendon="t_other"/>
    <actuatorfrc name="s_a_j1" actuator="a_j1"/>
    <actuatorfrc name="s_a_j2" actuator="a_j2"/>
    <actuatorfrc name="s_a_t12" actuator="a_t12"/>
    <actuatorfrc name="s_a_t_other" actuator="a_t_other"/>
  </sensor>
</mujoco>
"""


def _names(spec: mujoco.MjSpec) -> dict[str, set[str]]:
    return {
        "joints": {j.name for j in spec.joints},
        "tendons": {t.name for t in spec.tendons},
        "actuators": {a.name for a in spec.actuators},
        "sensors": {s.name for s in spec.sensors},
    }


def _equality_count(spec: mujoco.MjSpec, kind: mujoco.mjtEq) -> int:
    return sum(e.type == kind for e in spec.equalities)


def test_delete_tendon_removes_only_its_dependents() -> None:
    spec = mujoco.MjSpec.from_string(XML)
    delete_tendon(spec, spec.tendon("t12"))
    spec.compile()
    n = _names(spec)
    assert n["tendons"] == {"t34", "t4", "t_other"}
    assert n["actuators"] == {"a_j1", "a_j2", "a_j4", "a_t34", "a_t_other"}
    assert {"s_t12", "s_a_t12"}.isdisjoint(n["sensors"])
    assert len(n["sensors"]) == 8
    assert _equality_count(spec, mujoco.mjtEq.mjEQ_TENDON) == 1
    assert _equality_count(spec, mujoco.mjtEq.mjEQ_JOINT) == 2
    assert n["joints"] == {"j1", "j2", "j3", "j4"}


def test_delete_joint_removes_only_its_dependents() -> None:
    spec = mujoco.MjSpec.from_string(XML)
    delete_joint(spec, spec.joint("j4"))
    spec.compile()
    n = _names(spec)
    assert n["joints"] == {"j1", "j2", "j3"}
    assert n["tendons"] == {"t12", "t_other"}
    assert n["actuators"] == {"a_j1", "a_j2", "a_t12", "a_t_other"}
    assert n["sensors"] == {"s_j1", "s_j2", "s_t12", "s_t_other", "s_a_j1", "s_a_j2", "s_a_t12", "s_a_t_other"}
    assert _equality_count(spec, mujoco.mjtEq.mjEQ_JOINT) == 1
    assert _equality_count(spec, mujoco.mjtEq.mjEQ_TENDON) == 0


def test_delete_joint_cascades_through_tendon() -> None:
    spec = mujoco.MjSpec.from_string(XML)
    delete_joint(spec, spec.joint("j1"))
    spec.compile()
    n = _names(spec)
    assert n["tendons"] == {"t34", "t4", "t_other"}
    assert "a_t12" not in n["actuators"] and "s_a_t12" not in n["sensors"]
    assert _equality_count(spec, mujoco.mjtEq.mjEQ_TENDON) == 1


def test_immobilize_subtree_returns_traversal_order() -> None:
    spec = mujoco.MjSpec.from_string(XML)
    assert immobilize_subtree(spec, spec.body("b2")) == ["j2", "j3", "j4"]
    model = spec.compile()
    n = _names(spec)
    assert n["joints"] == {"j1"}
    assert n["tendons"] == set()
    assert n["actuators"] == {"a_j1"}
    assert n["sensors"] == {"s_j1", "s_a_j1"}
    assert len(spec.equalities) == 0
    assert model.nbody == 4 and model.ngeom == 3


def test_immobilize_hand_of_composed_arm() -> None:
    spec = myo_sim.load_spec("myoarm_r")
    removed = immobilize_subtree(spec, spec.body("lunate_r"))
    model = spec.compile()
    assert {"deviation_r", "flexion_r"} <= set(removed)
    assert len(removed) == len(set(removed))
    for joint in removed:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint) < 0
