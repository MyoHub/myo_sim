"""Imported sensor/contact settings match MuJoCo's native MJCF includes."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from myo_sim.build.utils import add_contact_pairs, add_sensors

_SENSOR_BASE = """
<mujoco>
  <worldbody>
    <body name="reference">
      <joint name="reference_slide" type="slide" axis="1 0 0"/>
      <joint name="reference_hinge" axis="0 0 1"/>
      <geom name="reference_geom" size="0.1"/>
      <site name="reference_site" pos="0.1 0 0"/>
      <camera name="reference_camera" pos="0 0 0.1"/>
    </body>
    <body name="moving" pos="0 1 0">
      <joint name="moving_slide" type="slide" axis="1 0 0" limited="true" range="-1 1"/>
      <joint name="moving_hinge" axis="0 0 1"/>
      <geom name="moving_geom" size="0.1"/>
      <site name="moving_site" pos="0.1 0 0"/>
      <camera name="moving_camera" pos="0 0 0.1"/>
    </body>
  </worldbody>
  {sensors}
</mujoco>
"""


@pytest.mark.parametrize("objtype", ["body", "xbody", "geom", "site", "camera"])
@pytest.mark.parametrize("reftype", ["body", "xbody", "geom", "site", "camera"])
def test_frame_sensor_references_options_and_values_match_native(tmp_path: Path, objtype: str, reftype: str) -> None:
    objname = "moving" if objtype in {"body", "xbody"} else f"moving_{objtype}"
    refname = "reference" if reftype in {"body", "xbody"} else f"reference_{reftype}"
    sensors = (
        f'<sensor><framelinvel name="relative_linear" objtype="{objtype}" objname="{objname}" '
        f'reftype="{reftype}" refname="{refname}" cutoff="4" noise="0.1" user="2 3"/>'
        f'<frameangvel name="relative_angular" objtype="{objtype}" objname="{objname}" '
        f'reftype="{reftype}" refname="{refname}" cutoff="2" noise="0.2" user="4 5"/></sensor>'
    )
    include = tmp_path / "sensors.xml"
    include.write_text(f"<mujocoinclude>{sensors}</mujocoinclude>")
    native = mujoco.MjModel.from_xml_string(_SENSOR_BASE.format(sensors=sensors))
    spec = mujoco.MjSpec.from_string(_SENSOR_BASE.format(sensors=""))

    add_sensors(spec, include)
    imported = spec.compile()

    for attribute in (
        "sensor_type",
        "sensor_objtype",
        "sensor_objid",
        "sensor_reftype",
        "sensor_refid",
        "sensor_cutoff",
        "sensor_noise",
        "sensor_user",
    ):
        np.testing.assert_array_equal(getattr(imported, attribute), getattr(native, attribute), err_msg=attribute)
    native_data, imported_data = mujoco.MjData(native), mujoco.MjData(imported)
    native_data.qpos[:] = imported_data.qpos[:] = [0.1, 0.2, 0.3, 0.4]
    native_data.qvel[:] = imported_data.qvel[:] = [2.0, 0.3, 5.0, 0.8]
    mujoco.mj_forward(native, native_data)
    mujoco.mj_forward(imported, imported_data)
    np.testing.assert_allclose(imported_data.sensordata, native_data.sensordata, atol=1e-12)


def test_unnamed_sensors_keep_native_names_and_defaults(tmp_path: Path) -> None:
    sensors = '<sensor><touch site="moving_site"/><jointlimitfrc joint="moving_slide"/></sensor>'
    include = tmp_path / "sensors.xml"
    include.write_text(f"<mujocoinclude>{sensors}</mujocoinclude>")
    native = mujoco.MjModel.from_xml_string(_SENSOR_BASE.format(sensors=sensors))
    spec = mujoco.MjSpec.from_string(_SENSOR_BASE.format(sensors=""))
    add_sensors(spec, include)
    imported = spec.compile()

    assert [sensor.name for sensor in spec.sensors] == ["", ""]
    for attribute in ("sensor_type", "sensor_objtype", "sensor_objid", "sensor_cutoff", "sensor_noise"):
        np.testing.assert_array_equal(getattr(imported, attribute), getattr(native, attribute))


_CONTACT_BASE = """
<mujoco>
  <default>
    <default class="sticky">
      <pair condim="4" friction="4 3 2 1 0.5" margin="0.03" gap="0.01"
            solref="0.04 0.8" solimp="0.8 0.9 0.002 0.4 3"/>
    </default>
  </default>
  <worldbody>
    <geom name="ground" type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.1"><freejoint/><geom name="ball" size="0.1"/></body>
  </worldbody>
  {contacts}
</mujoco>
"""


@pytest.mark.parametrize("overrides", ["", 'condim="3" friction="0.7 0.6 0.5 0.4 0.3" margin="0.02"'])
def test_contact_class_and_explicit_overrides_match_native(tmp_path: Path, overrides: str) -> None:
    contacts = f'<contact><pair class="sticky" geom1="ball" geom2="ground" {overrides}/></contact>'
    include = tmp_path / "contacts.xml"
    include.write_text(f"<mujocoinclude>{contacts}</mujocoinclude>")
    native = mujoco.MjModel.from_xml_string(_CONTACT_BASE.format(contacts=contacts))
    spec = mujoco.MjSpec.from_string(_CONTACT_BASE.format(contacts=""))

    add_contact_pairs(spec, include)
    imported = spec.compile()

    for attribute in ("pair_dim", "pair_friction", "pair_margin", "pair_gap", "pair_solref", "pair_solimp"):
        np.testing.assert_array_equal(getattr(imported, attribute), getattr(native, attribute), err_msg=attribute)


def test_contact_missing_class_fails_instead_of_using_default_settings(tmp_path: Path) -> None:
    contacts = '<contact><pair class="missing" geom1="ball" geom2="ground"/></contact>'
    include = tmp_path / "contacts.xml"
    include.write_text(f"<mujocoinclude>{contacts}</mujocoinclude>")
    spec = mujoco.MjSpec.from_string(_CONTACT_BASE.format(contacts=""))

    with pytest.raises(ValueError, match="missing"):
        add_contact_pairs(spec, include)
    assert len(spec.pairs) == 0
