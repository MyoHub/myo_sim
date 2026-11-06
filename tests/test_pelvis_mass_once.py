"""The pelvis is counted once.

The torso chain comes from a lumbar spine model whose sacrum carries the whole pelvis mass (its pelvis body is massless),
and the torso ends at the sacrum so that the pelvis comes from the leg chain (#48). In the models that join the two, the
sacrum must be massless. In the torso-only models the sacrum is welded to the world, so its mass changes nothing there.
"""

import mujoco
import pytest

from myo_sim import load_spec
from myo_sim.build.compose import MODEL_REGISTRY

WITH_LEGS = ["myofullbody", "myolegs", "myolegs26", "myolegs_abdomen"]


def body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


@pytest.mark.parametrize("name", WITH_LEGS)
def test_pelvis_counted_once_with_legs(name):
    model = load_spec(name).compile()
    sacrum, pelvis = body_id(model, "sacrum"), body_id(model, "pelvis")
    assert sacrum >= 0 and pelvis >= 0
    assert model.body_mass[pelvis] > 1.0, f"{name}: the leg chain's pelvis should carry the pelvis mass"
    assert model.body_mass[sacrum] == 0.0, f"{name}: sacrum {model.body_mass[sacrum]:.3f} kg beside the pelvis"


def test_sacrum_moves_only_with_legs():
    """A massless sacrum changes nothing only where it is welded to the world; it may move only in the models with legs."""
    for name in sorted(MODEL_REGISTRY):
        model = load_spec(name).compile()
        sacrum = body_id(model, "sacrum")
        if sacrum < 0 or model.body_weldid[sacrum] == 0:
            continue
        assert name in WITH_LEGS, f"{name}: the sacrum moves but the model is not one that brings a pelvis"
