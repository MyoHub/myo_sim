"""Model-analysis utilities that operate on a compiled MuJoCo model.

These helpers read structure out of an already-compiled `mujoco.MjModel`; they
do not author or modify MJCF. Import them directly, e.g.::

    from myo_sim.analysis.muscle_groups import discover_muscle_groups
"""

from myo_sim.analysis.muscle_groups import MuscleGroup as MuscleGroup
from myo_sim.analysis.muscle_groups import discover_muscle_groups as discover_muscle_groups
from myo_sim.analysis.muscle_groups import group_for_actuator as group_for_actuator
from myo_sim.analysis.muscle_groups import moment_arm_signatures as moment_arm_signatures
