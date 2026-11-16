"""attach_checked: composing models fails loudly where MjSpec.attach would drop an element."""

from pathlib import Path

import mujoco
import pytest

from myo_sim.build import compose
from myo_sim.build.utils import attach_checked

CHILD = """<mujoco><worldbody><body name="b"><joint name="j" axis="0 1 0"/><geom size=".05"/>
<site name="s1"/><site name="s2" pos=".1 0 0"/></body></worldbody>
<tendon><spatial name="good"><site site="s1"/><site site="s2"/></spatial>
<spatial name="bad"><site site="s1"/><site site="MISSING"/></spatial></tendon>
<actuator><muscle name="m_good" tendon="good" lengthrange=".05 .2"/>
<muscle name="m_bad" tendon="bad" lengthrange=".05 .2"/></actuator></mujoco>"""


def parent() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string('<mujoco><worldbody><site name="at"/></worldbody></mujoco>')


def test_names_every_dropped_element() -> None:
    spec = parent()
    with pytest.raises(ValueError, match="c_bad") as error:
        attach_checked(spec, mujoco.MjSpec.from_string(CHILD), prefix="c_", site=spec.site("at"))
    assert "('actuators', 'c_m_bad')" in str(error.value) and "('tendons', 'c_bad')" in str(error.value)


def test_keeps_a_complete_child() -> None:
    spec = parent()
    attach_checked(spec, mujoco.MjSpec.from_string(CHILD.replace("MISSING", "s2")), prefix="c_", site=spec.site("at"))
    model = spec.compile()
    assert {model.tendon(i).name for i in range(model.ntendon)} == {"c_good", "c_bad"}


@pytest.mark.parametrize("name", ["myoarm_r", "myofullbody"])
def test_misspelled_arm_site_fails_the_build(name: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    text = compose.RIGHT_ARM_TENDONS_XML.read_text()
    assert 'site="APL_via_r"' in text
    (tmp_path / "tendons.xml").write_text(text.replace('site="APL_via_r"', 'site="APL_via_typo_r"'))
    monkeypatch.setattr(compose, "RIGHT_ARM_TENDONS_XML", tmp_path / "tendons.xml")
    with pytest.raises(ValueError, match="APL_r_tendon"):
        compose.build_spec(name)
