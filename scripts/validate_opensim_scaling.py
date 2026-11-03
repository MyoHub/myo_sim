# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
#
# Adapted from MyoSuite PR #564 by Mark Colley (M-Colley), Apache-2.0
# (https://github.com/MyoHub/myosuite/pull/564).
"""Compare ``scale_spec`` on MyoLeg with OpenSim's own ModelScaler on Rajagopal2016.

OpenSim runs in its own interpreter and is not a dependency of myo_sim. The script writes OpenSim's
results to ``tests/data/opensim_scaling/reference.json``; ``tests/test_opensim_comparison.py`` then
compares them with ``scale_spec`` without OpenSim.

    python scripts/validate_opensim_scaling.py --opensim-python <python of an OpenSim 4.6 env> [--model Rajagopal2016.osim]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import tempfile
import urllib.request
from pathlib import Path

DATA = Path(__file__).resolve().parents[1] / "tests" / "data" / "opensim_scaling"
MODEL_URL = "https://raw.githubusercontent.com/opensim-org/opensim-models/d9b05d470b1a481c222372c85b75772faf8f7792/Models/Rajagopal/Rajagopal2016.osim"
MODEL_SHA256 = "3f5c5f23e486073f2ad2aa4a4967ffe2fcdd582b1e355512bc54f70c36376bf4"
BODIES = (
    "pelvis",
    "femur_r",
    "tibia_r",
    "patella_r",
    "talus_r",
    "calcn_r",
    "toes_r",
    "femur_l",
    "tibia_l",
    "talus_l",
    "calcn_l",
)
POSITIONS = ("femur_r", "tibia_r", "talus_r", "calcn_r", "toes_r", "femur_l", "tibia_l", "talus_l", "calcn_l", "toes_l")


def model_path(given: str | None) -> Path:
    """The pinned Rajagopal2016 model, downloaded if no path is given; refuses changed bytes."""
    if given:
        path = Path(given)
    else:
        path = Path(tempfile.gettempdir()) / f"Rajagopal2016_{MODEL_SHA256[:12]}.osim"
        if not path.exists():
            with urllib.request.urlopen(MODEL_URL, timeout=60) as response:
                path.write_bytes(response.read())
    if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError(f"{path} does not match the pinned SHA-256.")
    return path


def opensim_half(path: Path) -> dict:
    """Scale the model with OpenSim's ModelScaler; runs inside the OpenSim interpreter."""
    import numpy as np
    import opensim as osim

    osim.Logger.setLevelString("error")
    fixture = json.loads((DATA / "fixture.json").read_text())
    output = {"opensim_version": osim.GetVersionAndDate(), "model_sha256": MODEL_SHA256, "cases": {}}
    body = osim.Body("probe", 10.0, osim.Vec3(0), osim.Inertia(0.1, 0.2, 0.25))
    body.scaleInertialProperties(osim.Vec3(1.2, 0.8, 1.1), True)
    output["inertia_probe"] = {"mass_kg": body.getMass(), "moments": body.getInertia().getMoments().to_numpy().tolist()}
    for case, factors in fixture["cases"].items():
        model = osim.Model(str(path))
        if factors:
            scaler = osim.ModelScaler()
            order = osim.ArrayStr()
            order.append("manualScale")
            scaler.setScalingOrder(order)
            scaler.setPreserveMassDist(False)
            for name, values in factors.items():
                scale = osim.Scale()
                scale.setSegmentName(name)
                scale.setScaleFactors(osim.Vec3(*values))
                scale.setApply(True)
                scaler.addScale(scale)
            if not scaler.processModel(model, "", -1.0):
                raise RuntimeError(f"OpenSim's ModelScaler failed for {case}.")
        state = model.initSystem()
        model.assemble(state)
        model.realizePosition(state)
        pelvis = model.getBodySet().get("pelvis")
        rows: dict = {"positions_m": {}, "mass_kg": {}, "inertia_moments": {}, "muscles": {}}
        for name in POSITIONS:
            location = model.getBodySet().get(name).findStationLocationInAnotherFrame(state, osim.Vec3(0), pelvis)
            rows["positions_m"][name] = np.array(location.to_numpy()).tolist()
        for name in BODIES:
            entry = model.getBodySet().get(name)
            rows["mass_kg"][name] = entry.getMass()
            rows["inertia_moments"][name] = entry.getInertia().getMoments().to_numpy().tolist()
        muscles = model.getMuscles()
        for index in range(muscles.getSize()):
            muscle = muscles.get(index)
            rows["muscles"][muscle.getName()] = {
                "optimal_fibre_length_m": muscle.getOptimalFiberLength(),
                "tendon_slack_length_m": muscle.getTendonSlackLength(),
                "path_length_m": muscle.getLength(state),
            }
        output["cases"][case] = rows
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--opensim-python", help="Python of an environment with OpenSim 4.6.")
    parser.add_argument("--model", help="A local copy of the pinned Rajagopal2016.osim.")
    parser.add_argument("--opensim-half", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    path = model_path(args.model)
    if args.opensim_half:
        print(json.dumps(opensim_half(path)))
        return
    if not args.opensim_python:
        parser.error("--opensim-python is required")
    command = [args.opensim_python, __file__, "--opensim-half", "--model", str(path)]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    reference = json.loads(result.stdout.splitlines()[-1])
    (DATA / "reference.json").write_text(json.dumps(reference, indent=1, sort_keys=True) + "\n")
    print(f"Wrote {DATA / 'reference.json'} with OpenSim {reference['opensim_version']}.")


if __name__ == "__main__":
    main()
