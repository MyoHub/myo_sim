import unittest
from pathlib import Path

import mujoco
import myo_sim
from myo_sim.build.compose import build_model

model_paths = [
    "leg/myolegs.xml",
    "torso/myotorso.xml",
    "torso/myotorso_abdomen.xml",
    "scene/myosuite_scene_noPedestal.xml",
    "scene/myosuite_scene.xml",
]


class TestSims(unittest.TestCase):
    def get_sim(self, model_path: str = None, model_xmlstr=None):
        """
        Get sim using model_path or model_xmlstr.
        """

        # load from path
        if model_path:
            # resolve full path
            if model_path.startswith("/"):
                fullpath = model_path
            else:
                fullpath = str(myo_sim.MODELS_DIR / model_path)
            if not Path(fullpath).exists():
                raise IOError("File %s does not exist" % fullpath)

            # load model
            if model_path.endswith(".mjb"):
                model = mujoco.MjModel.from_binary_path(fullpath)
            elif model_path.endswith(".xml"):
                model = mujoco.MjModel.from_xml_path(fullpath)

        # load from xml string
        elif model_xmlstr:
            model = mujoco.MjModel.from_xml_path(model_xmlstr)
        else:
            raise TypeError("Both model_path and model_xmlstr can't be None")

        return model

    def test_sims(self):
        for model_path in model_paths:
            print("Testing: {}".format(model_path))
            self.get_sim(model_path)


MUSCLEMIMIC_FULLBODY_SPEC = {
    "njnt": 123,
    "nu": 416,
}


class TestMuscleMimicFullBody(unittest.TestCase):
    """Numerical validation: myofullbody must match musclemimic spec exactly."""

    def _load(self, rel_path):
        if rel_path == "myofullbody":
            return build_model("myofullbody")
        return mujoco.MjModel.from_xml_path(str(myo_sim.MODELS_DIR / rel_path))

    def test_myofullbody_joints(self):
        m = self._load("myofullbody")
        self.assertEqual(
            m.njnt,
            MUSCLEMIMIC_FULLBODY_SPEC["njnt"],
            f"myofullbody.xml has {m.njnt} joints, expected {MUSCLEMIMIC_FULLBODY_SPEC['njnt']} (musclemimic spec)",
        )

    def test_myofullbody_actuators(self):
        m = self._load("myofullbody")
        self.assertEqual(
            m.nu,
            MUSCLEMIMIC_FULLBODY_SPEC["nu"],
            f"myofullbody.xml has {m.nu} actuators, expected {MUSCLEMIMIC_FULLBODY_SPEC['nu']} (musclemimic spec)",
        )

    def test_myofullbody_report(self):
        """Print a summary report for PR inclusion."""
        m = self._load("myofullbody")
        report = (
            f"\n=== myofullbody.xml vs musclemimic spec ===\n"
            f"  joints (njnt):    {m.njnt:4d}  expected {MUSCLEMIMIC_FULLBODY_SPEC['njnt']}\n"
            f"  actuators (nu):   {m.nu:4d}  expected {MUSCLEMIMIC_FULLBODY_SPEC['nu']}\n"
            f"  nq:               {m.nq:4d}\n"
            f"  match: {'PASS' if m.njnt == MUSCLEMIMIC_FULLBODY_SPEC['njnt'] and m.nu == MUSCLEMIMIC_FULLBODY_SPEC['nu'] else 'FAIL'}\n"
        )
        print(report)

    def test_part_models_summary(self):
        """Print joint/actuator counts for all musclemimic-derived models."""
        parts = [
            ("myotorso_arm_r", "myotorso_arm_r"),
            ("torso/myotorso.xml", "myotorso"),
            ("myofullbody", "myofullbody"),
        ]
        print("\n=== Part model counts ===")
        for path, name in parts:
            fullpath = myo_sim.MODELS_DIR / path
            if path in {"myofullbody", "myotorso_arm_r"}:
                m = build_model(path)
                print(f"  {name:30s}  njnt={m.njnt:4d}  nu={m.nu:4d}")
            elif fullpath.exists():
                m = mujoco.MjModel.from_xml_path(str(fullpath))
                print(f"  {name:30s}  njnt={m.njnt:4d}  nu={m.nu:4d}")
            else:
                print(f"  {name:30s}  MISSING")


if __name__ == "__main__":
    unittest.main()
