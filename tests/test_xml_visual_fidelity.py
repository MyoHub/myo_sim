"""Generated XML keeps the compiled materials and site visibility groups."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from myo_sim.build.compose import build_spec, write_spec_xml


@pytest.mark.parametrize("model_name", ["myoarms", "myohands", "myofullbody"])
def test_generated_xml_preserves_asymmetric_materials_and_site_groups(tmp_path: Path, model_name: str) -> None:
    spec = build_spec(model_name)
    for material in spec.materials:
        if material.name == "MatSkin":
            material.rgba = [0.8, 0.1, 0.2, 1.0]
        elif material.name == "MatSkin_l":
            material.rgba = [0.2, 0.8, 0.1, 1.0]
    reference = spec.compile()
    assert reference.geom("humerus_coll_l").matid != reference.geom("humerus_coll_r").matid
    assert reference.site("arm_root_attach_r").group == 0
    assert reference.site("arm_root_attach_l").group == 0
    output_path = tmp_path / f"{model_name}.xml"

    write_spec_xml(spec, output_path)
    generated = mujoco.MjModel.from_xml_path(str(output_path))

    np.testing.assert_array_equal(generated.geom_matid, reference.geom_matid)
    np.testing.assert_array_equal(generated.mat_rgba, reference.mat_rgba)
    np.testing.assert_array_equal(generated.site_group, reference.site_group)
