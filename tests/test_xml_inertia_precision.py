# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Native XML roundtrips retain edited moments at the inertia triangle."""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

from myo_sim.build.compose import write_spec_xml


@pytest.mark.parametrize("factor", [0.95, 1.0189, 1.05, 1.1])
def test_edited_planar_inertia_roundtrips_without_balancing(tmp_path: Path, factor: float) -> None:
    spec = mujoco.MjSpec.from_string("""<mujoco><worldbody><body name="flat">
      <joint/><inertial pos="0 0 0" mass="1" diaginertia=".01765 .01219 .00546"/>
    </body></worldbody></mujoco>""")
    moments = np.array([0.01765, 0.01219, 0.00546]) * factor**5
    moments[0] = moments[1] + moments[2]
    spec.body("flat").inertia = moments
    expected = spec.compile()
    path = tmp_path / "edited.xml"

    write_spec_xml(spec, path)
    actual = mujoco.MjModel.from_xml_path(str(path))

    assert ET.parse(path).getroot().find("compiler").get("balanceinertia", "false") == "false"
    np.testing.assert_array_equal(actual.body_inertia, expected.body_inertia)
    np.testing.assert_array_equal(actual.body_mass, expected.body_mass)
    # The source is not nudged inside the triangle to make its serialization valid.
    np.testing.assert_array_equal(spec.body("flat").inertia, moments)


@pytest.mark.parametrize("alignfree", [False, True])
def test_inertia_precision_keeps_native_frames_and_unnamed_body_order(tmp_path: Path, alignfree: bool) -> None:
    spec = mujoco.MjSpec.from_string(f"""<mujoco><compiler angle="radian" alignfree="{str(alignfree).lower()}"/>
      <worldbody><frame pos=".1 .2 .3" euler=".2 .3 .4"><body name="free"><freejoint/>
        <frame pos=".3 .2 .1" euler=".4 .1 .2"><inertial mass="2" pos=".2 .1 .3"
          diaginertia=".1 .2 .25" euler=".1 .2 .3"/></frame>
      </body></frame><body><inertial mass="1" pos="0 0 0" diaginertia=".2 .3 .4"/></body></worldbody>
    </mujoco>""")
    expected = spec.compile()
    path = tmp_path / "frames.xml"

    write_spec_xml(spec, path)
    actual = mujoco.MjModel.from_xml_path(str(path))

    np.testing.assert_array_equal(actual.body_inertia, expected.body_inertia)
    np.testing.assert_allclose(actual.body_ipos, expected.body_ipos, atol=1e-6)
    np.testing.assert_allclose(actual.body_iquat, expected.body_iquat, atol=1e-6)
