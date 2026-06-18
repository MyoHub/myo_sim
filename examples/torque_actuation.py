"""Examples for building torque-driven and mixed muscle/torque models.

Run from the repository root:

    uv run python examples/torque_actuation.py

`build_model()`'s `actuation` argument selects per-part actuation mode:
  - omitted / None -> every part uses its muscle-driven default
  - a single string (e.g. "torque") -> applied uniformly to every part
  - a dict keyed by PART_TORSO / PART_ARMS / PART_LEGS -> mixes modes
    within one build (parts left out of the dict keep their default)
"""

from myo_sim.build.compose import PART_ARMS, PART_LEGS, PART_TORSO, build_model


def build_torque_fullbody():
    """Full body where every joint is driven by a direct torque motor
    instead of muscles+tendons."""
    model = build_model("myofullbody", actuation="torque")
    print(f"myofullbody (torque): nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}")
    return model


def build_mixed_fullbody():
    """Full body with torque-driven torso + arms, but muscle-driven legs --
    e.g. for locomotion tasks where leg muscle dynamics matter but upper-body
    control can be simplified."""
    model = build_model(
        "myofullbody",
        actuation={PART_TORSO: "torque", PART_ARMS: "torque", PART_LEGS: "muscle"},
    )
    print(f"myofullbody (mixed torso/arms=torque, legs=muscle): nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}")
    return model


def build_muscle_fullbody():
    """Default fully muscle-driven full body, for comparison."""
    model = build_model("myofullbody")
    print(f"myofullbody (muscle, default): nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}")
    return model


if __name__ == "__main__":
    build_muscle_fullbody()
    build_torque_fullbody()
    build_mixed_fullbody()
