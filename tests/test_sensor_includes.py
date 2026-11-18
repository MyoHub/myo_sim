"""add_sensors imports the sensors of an MJCF include, or names the ones it cannot import."""

from pathlib import Path

import mujoco
import pytest

from myo_sim.build.utils import add_sensors

_BASE = """
<mujoco>
  <worldbody>
    <body name="body">
      <joint name="hinge" axis="0 0 1" limited="true" range="-1 1"/>
      <geom size="0.1"/>
      <site name="site"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.mark.parametrize(
    "tag, sensor",
    [
        ("gyro", '<gyro name="gyro" site="site"/>'),
        ("framepos", '<framepos name="position" objtype="body" objname="body"/>'),
    ],
)
def test_add_sensors_rejects_unsupported_sensor_tags(tmp_path: Path, tag: str, sensor: str) -> None:
    include = tmp_path / "sensors.xml"
    include.write_text(f'<mujocoinclude><sensor><touch name="touch" site="site"/>{sensor}</sensor></mujocoinclude>')
    spec = mujoco.MjSpec.from_string(_BASE)

    with pytest.raises(ValueError, match=rf"sensors\.xml.*<{tag}>"):
        add_sensors(spec, include)
    assert not spec.sensors


def test_add_sensors_imports_supported_sensors_like_native_mjcf(tmp_path: Path) -> None:
    sensors = (
        '<sensor><touch name="touch" site="site"/><jointlimitfrc name="limit" joint="hinge"/>'
        '<framelinvel name="linvel" objtype="body" objname="body"/>'
        '<frameangvel name="angvel" objtype="site" objname="site"/></sensor>'
    )
    include = tmp_path / "sensors.xml"
    include.write_text(f"<mujocoinclude>{sensors}</mujocoinclude>")
    native = mujoco.MjModel.from_xml_string(_BASE.replace("</mujoco>", f"{sensors}</mujoco>"))
    spec = mujoco.MjSpec.from_string(_BASE)

    add_sensors(spec, include)
    imported = spec.compile()

    for attribute in ("sensor_type", "sensor_objtype", "sensor_objid", "sensor_dim", "sensor_adr"):
        assert getattr(imported, attribute).tolist() == getattr(native, attribute).tolist(), attribute
    assert [imported.sensor(i).name for i in range(imported.nsensor)] == ["touch", "limit", "linvel", "angvel"]
