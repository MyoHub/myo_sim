import unittest
import os
import mujoco

model_paths = [
    # Basic models
    "basic/myomuscle.xml",
    # finger models
    "finger/finger_v0.xml",
    "finger/myofinger_v0.xml",
    "finger/motorfinger_v0.xml",
    # elbow models
    "elbow/myoelbow_1dof6muscles_1dofexo.xml",
    "elbow/myoelbow_1dof6muscles.xml",
    "elbow/myoelbow_2dof6muscles.xml",
    "elbow/myoelbow_1dof6muscles_1dofSoftexo_Ideal.xml",
    "elbow/myoelbow_1dof6muscles_1dofSoftexo_sim2.xml",
    # arms
    "arm/myoarm_simple.xml",
    "arm/myoarm.xml",
    "arm/myoarm_bimanual.xml",
    "arm/myoarm_r.xml",
    "arm/myoarm_l.xml",
    # hand models
    "hand/myohand.xml",
    # leg models
    "leg/myolegs.xml",
    "leg/myolegs_abdomen.xml",
    "osl/myolegs_osl.xml",
    # head
    "head/myohead_simple.xml",
    # torso
    "torso/myotorso.xml",
    "torso/myotorso_exosuit.xml",
    "torso/myotorso_rigid.xml",
    "torso/myotorso_abdomen.xml",
    "torso/myotorso_bimanual.xml",
    # full body models
    "body/myobody.xml",
    "body/myoupperbody.xml",
    "body/myofullbody.xml",
    # scene
    "scene/myosuite_scene_noPedestal.xml",
    "scene/myosuite_scene.xml",
    "scene/myosuite_quad.xml",
    "scene/myosuite_logo.xml",
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
                fullpath = os.path.join(os.path.dirname(__file__), model_path)
            if not os.path.exists(fullpath):
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
        fullpath = os.path.join(os.path.dirname(__file__), rel_path)
        return mujoco.MjModel.from_xml_path(fullpath)

    def test_myofullbody_joints(self):
        m = self._load("body/myofullbody.xml")
        self.assertEqual(
            m.njnt,
            MUSCLEMIMIC_FULLBODY_SPEC["njnt"],
            f"myofullbody.xml has {m.njnt} joints, expected {MUSCLEMIMIC_FULLBODY_SPEC['njnt']} " f"(musclemimic spec)",
        )

    def test_myofullbody_actuators(self):
        m = self._load("body/myofullbody.xml")
        self.assertEqual(
            m.nu,
            MUSCLEMIMIC_FULLBODY_SPEC["nu"],
            f"myofullbody.xml has {m.nu} actuators, expected {MUSCLEMIMIC_FULLBODY_SPEC['nu']} " f"(musclemimic spec)",
        )

    def test_myofullbody_report(self):
        """Print a summary report for PR inclusion."""
        m = self._load("body/myofullbody.xml")
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
            ("leg/myolegs.xml", "myolegs"),
            ("arm/myoarm_r.xml", "myoarm_r"),
            ("arm/myoarm_l.xml", "myoarm_l"),
            ("arm/myoarm_bimanual.xml", "myoarm_bimanual"),
            ("torso/myotorso_bimanual.xml", "myotorso_bimanual"),
            ("body/myofullbody.xml", "myofullbody"),
        ]
        print("\n=== Part model counts ===")
        for path, name in parts:
            fullpath = os.path.join(os.path.dirname(__file__), path)
            if os.path.exists(fullpath):
                m = mujoco.MjModel.from_xml_path(fullpath)
                print(f"  {name:30s}  njnt={m.njnt:4d}  nu={m.nu:4d}")
            else:
                print(f"  {name:30s}  MISSING")


if __name__ == "__main__":
    unittest.main()
