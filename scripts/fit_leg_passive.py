#!/usr/bin/env python3
"""Write the Silder 2007 leg passive slack / fpmax fit into myolegs_muscle.xml.

Development / release only. Source table values live in
myo_sim.build.references.SILDER2007. Does not commit subject-level recordings.

    uv run python scripts/fit_leg_passive.py          # print
    uv run python scripts/fit_leg_passive.py --write  # rewrite XML
"""

from __future__ import annotations

from myo_sim.build.leg_passive import main

if __name__ == "__main__":
    main()
