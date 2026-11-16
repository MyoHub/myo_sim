#!/usr/bin/env python3
"""Write adopted leg rigid-tendon maps into myolegs_muscle.xml.

Development / release only. Reads published Rajagopal et al. (2016) fibre and
tendon lengths from myo_sim.build.references. Does not load or commit the
OpenSim model or any trajectory dataset.

    uv run python scripts/fit_leg_rigid_maps.py          # print
    uv run python scripts/fit_leg_rigid_maps.py --write  # rewrite XML
"""

from __future__ import annotations

import sys

from myo_sim.build.rigid_conversion import main

if __name__ == "__main__":
    sys.argv = [sys.argv[0], "--region", "leg", *sys.argv[1:]]
    main()
