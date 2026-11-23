"""Tendon wrap reads on a copied MjSpec, which crash on MuJoCo 3.10 and earlier."""

import mujoco

from myo_sim.build.hand import tendon_wrap_geom_names

XML = """
<mujoco>
  <worldbody>
    <body name="a">
      <joint name="hinge" axis="0 1 0"/>
      <geom name="cyl" type="cylinder" size="0.02 0.05"/>
      <site name="s1" pos="0.05 0 0"/>
      <site name="side" pos="0 0.05 0"/>
      <body name="b" pos="0 0 -0.2">
        <joint name="hinge2" axis="0 1 0"/>
        <geom size="0.02"/>
        <site name="s2" pos="0.05 0 0"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="sp"><site site="s1"/><geom geom="cyl" sidesite="side"/><site site="s2"/></spatial>
    <fixed name="fx"><joint joint="hinge2" coef="1"/></fixed>
  </tendon>
</mujoco>
"""


def test_wrap_targets_resolve_on_copied_spec() -> None:
    # MuJoCo <= 3.10 leaves the wraps of a copied spec without a model, so these
    # reads segfault (google-deepmind/mujoco#3406). Guards the mujoco>=3.11 floor.
    spec = mujoco.MjSpec.from_string(XML).copy()
    assert tendon_wrap_geom_names(spec) == {"cyl"}
    assert spec.tendon("sp").path[1].sidesite.name == "side"
    assert spec.tendon("fx").path[0].target.name == "hinge2"
