import mujoco
import numpy as np
import pytest

import myo_sim

KNEE_DRIVERS = ("knee_angle_r", "knee_angle_l")


@pytest.fixture(scope="module")
def leg_model():
    model, _ = myo_sim.load("myolegs")
    return model


def knee_couplings(model) -> list[tuple[int, int, np.ndarray]]:
    """(coupled joint, knee joint, polynomial coefficients) for every joint equality driven by a knee angle."""
    drivers = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in KNEE_DRIVERS}
    return [
        (int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq]), model.eq_data[eq, :5])
        for eq in range(model.neq)
        if model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT and int(model.eq_obj2id[eq]) in drivers
    ]


def test_knee_coupled_coordinates_stay_inside_their_ranges(leg_model):
    """A coordinate the knee drives must stay inside its own declared range at 2001 angles across the knee range; otherwise the joint
    limit and the coupling fight wherever the coupling crosses it."""
    couplings = knee_couplings(leg_model)
    assert len(couplings) == 14

    failures = []
    for coupled, knee, coeffs in couplings:
        assert leg_model.jnt_limited[coupled] and leg_model.jnt_limited[knee]
        knee_q = np.linspace(*leg_model.jnt_range[knee], 2001)
        # MuJoCo's joint equality: y - y0 = poly(x - x0), with x0, y0 the joints' qpos0 values
        x0 = leg_model.qpos0[leg_model.jnt_qposadr[knee]]
        y0 = leg_model.qpos0[leg_model.jnt_qposadr[coupled]]
        values = y0 + np.polynomial.polynomial.polyval(knee_q - x0, coeffs)
        low, high = leg_model.jnt_range[coupled]
        if values.min() < low or values.max() > high:
            name = mujoco.mj_id2name(leg_model, mujoco.mjtObj.mjOBJ_JOINT, coupled)
            failures.append((name, (float(values.min()), float(values.max())), (float(low), float(high))))

    assert not failures
