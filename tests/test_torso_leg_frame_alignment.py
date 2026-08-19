"""Regression test for myo_sim#131: leg + rigid torso 90-degree waist twist.

``leg/assets/myolegs_chain.xml``'s pelvis is OpenSim-framed
(``euler="1.57 -1.57 0"``, Y-up/X-anterior). ``legacy/torso/assets/
myotorso_rigid_chain.xml`` used to have no compensating rotation, so any
composition combining the two under a shared parent (e.g. the legacy
``myolegs_osl.xml`` model) produced a 90-degree twist at the waist unless the
composer manually applied a compensating yaw.

Fixed by baking ``euler="0 0 -1.57"`` into the rigid torso's own root body,
so its anterior (+X) axis matches an OpenSim-framed pelvis out of the box.
``myolegs_osl.xml`` previously supplied this yaw itself, wrapping both torso
and legs in one rotated body; that wrapper was moved to cover the legs
include only, since the torso now carries its own correction.

See https://github.com/MyoHub/myo_sim/issues/131.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import myo_sim  # noqa: E402


def _anterior_axis_angle(model, data, body_name_a, body_name_b):
    id_a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name_a)
    id_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name_b)
    assert id_a >= 0 and id_b >= 0
    x_a = data.xmat[id_a].reshape(3, 3)[:, 0]
    x_b = data.xmat[id_b].reshape(3, 3)[:, 0]
    cos_angle = np.clip(np.dot(x_a, x_b), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def test_myolegs_osl_torso_and_pelvis_are_not_twisted():
    path = myo_sim.MODELS_DIR / "legacy" / "osl" / "myolegs_osl.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    angle = _anterior_axis_angle(model, data, "torso", "pelvis")
    assert angle < 1.0, f"torso/pelvis anterior axes are {angle:.1f} deg apart, expected ~0"


def test_rigid_torso_root_body_has_compensating_yaw():
    torso_chain = (myo_sim.MODELS_DIR / "legacy" / "torso" / "assets" / "myotorso_rigid_chain.xml").read_text()
    assert 'body name="torso" euler="0 0 -1.57"' in torso_chain
