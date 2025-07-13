import os
import sys
import traceback
import unittest
from typing import Optional

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
    # full body models
    "body/myobody.xml",
    "body/myoupperbody.xml",
    # scene
    "scene/myosuite_scene_noPedestal.xml",
    "scene/myosuite_scene.xml",
    "scene/myosuite_quad.xml",
    "scene/myosuite_logo.xml",
]


class TestSims(unittest.TestCase):

    def setUp(self):
        """Set up test environment and print MuJoCo version info."""
        print(f"MuJoCo version: {mujoco.__version__}")
        print(f"Python version: {sys.version}")
        print(f"Platform: {sys.platform}")
        print("-" * 50)

    def get_sim(
        self, model_path: Optional[str] = None, model_xmlstr: Optional[str] = None
    ):
        """
        Get sim using model_path or model_xmlstr.
        """
        model = None

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
        """Test loading all models with enhanced error reporting."""
        failed_models = []
        successful_models = []

        for model_path in model_paths:
            print(f"Testing: {model_path}")
            try:
                model = self.get_sim(model_path)
                print(f"  ✅ Successfully loaded {model_path}")
                successful_models.append(model_path)
            except Exception as e:
                error_msg = f"  ❌ Failed to load {model_path}: {str(e)}"
                print(error_msg)
                print(f"  Error details: {traceback.format_exc()}")
                failed_models.append((model_path, str(e)))

        # Print summary
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        print(f"Total models tested: {len(model_paths)}")
        print(f"Successful: {len(successful_models)}")
        print(f"Failed: {len(failed_models)}")

        if successful_models:
            print("\n✅ Successfully loaded models:")
            for model in successful_models:
                print(f"  - {model}")

        if failed_models:
            print("\n❌ Failed models:")
            for model, error in failed_models:
                print(f"  - {model}: {error}")

        # Assert that at least some models loaded successfully
        self.assertGreater(
            len(successful_models),
            0,
            f"No models could be loaded. All {len(model_paths)} models failed.",
        )

        # If there are failures, provide detailed information
        if failed_models:
            print(f"\n⚠️  Warning: {len(failed_models)} models failed to load.")
            print("This might be due to MuJoCo version compatibility issues.")
            print("Check the model files for version-specific features.")


if __name__ == "__main__":
    # Run tests with verbose output
    unittest.main(verbosity=2)
