"""Generated XML keeps the compiled materials and site visibility groups."""

from pathlib import Path

import mujoco
import pytest

from myo_sim.build.compose import build_spec, write_spec_xml

Key = str | tuple[str, int]


def element_ids_by_name(model: mujoco.MjModel, tag: str) -> dict[Key, int]:
    """Geom or site ids keyed by name, or by body name and order among the body's unnamed elements."""
    ids: dict[Key, int] = {}
    unnamed_count: dict[int, int] = {}
    for element_id in range(getattr(model, f"n{tag}")):
        key: Key = getattr(model, tag)(element_id).name
        if not key:
            body_id = int(getattr(model, f"{tag}_bodyid")[element_id])
            key = (model.body(body_id).name, unnamed_count.get(body_id, 0))
            unnamed_count[body_id] = key[1] + 1
        ids[key] = element_id
    return ids


def visual_attributes_by_name(model: mujoco.MjModel) -> dict[str, dict]:
    """Geom material names, site groups and material colours, compared by name rather than by index."""

    def material_name(geom_id: int) -> str:
        material_id = int(model.geom_matid[geom_id])
        return model.mat(material_id).name if material_id >= 0 else ""

    return {
        "geom_material": {key: material_name(i) for key, i in element_ids_by_name(model, "geom").items()},
        "site_group": {key: int(model.site_group[i]) for key, i in element_ids_by_name(model, "site").items()},
        "mat_rgba": {model.mat(i).name: model.mat_rgba[i].tolist() for i in range(model.nmat)},
    }


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

    assert visual_attributes_by_name(generated) == visual_attributes_by_name(reference)


def test_generated_xml_preserves_materials_and_site_groups_in_frames(tmp_path: Path) -> None:
    # MjSpec.to_xml() writes the geoms and sites created in a frame after the body's other ones, and a frame's bodies first.
    size = [0.05, 0, 0]
    spec = mujoco.MjSpec()
    spec.add_material(name="red", rgba=[1, 0, 0, 1])
    spec.add_material(name="green", rgba=[0, 1, 0, 1])
    body = spec.worldbody.add_body(name="body", pos=[0, 0, 1])
    body.add_freejoint()
    body.add_geom(name="plain_first", size=size, material="red")
    frame = body.add_frame(pos=[0.2, 0, 0])
    frame.add_geom(name="in_frame", size=size, material="green")
    frame.add_site(name="site_in_frame", group=4)
    body.add_geom(name="plain_last", size=size, material="red")
    body.add_geom(size=size)
    body.add_site(name="site_plain", group=1)
    body.add_site(group=2)
    body.add_body(name="child_plain").add_site(group=5)
    body.add_frame(name="named_frame").add_body(name="child_in_frame").add_site(group=3)
    reference = spec.compile()
    output_path = tmp_path / "frames.xml"

    write_spec_xml(spec, output_path)
    generated = mujoco.MjModel.from_xml_path(str(output_path))

    assert visual_attributes_by_name(generated) == visual_attributes_by_name(reference)
