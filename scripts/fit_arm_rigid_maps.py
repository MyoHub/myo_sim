#!/usr/bin/env python3
"""Write adopted arm rigid-tendon maps into myoarm_r_muscle.xml.

Development / release only. Reads published MoBL-ARMS parameters from
myo_sim.build.references. Does not load or commit motion-capture trajectories.

    uv run python scripts/fit_arm_rigid_maps.py          # print
    uv run python scripts/fit_arm_rigid_maps.py --write  # rewrite XML
"""

from __future__ import annotations

import sys

from myo_sim.build.rigid_conversion import main

if __name__ == "__main__":
    sys.argv = [sys.argv[0], "--region", "arm", *sys.argv[1:]]
    main()
