# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Editor-independent interchange preserves compiled anatomy and MuJoCo geometry."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import pytest

import myo_sim
from myo_sim.authoring import export_model, joint_sweep, model_fingerprint
from myo_sim.authoring.interchange import _peak_force
from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings


@pytest.mark.parametrize("name,count", [("myoelbow", 6), ("myoarm_r", 63), ("myolegs", 80)])
def test_compiled_muscles_wraps_and_mesh_transforms(tmp_path: Path, name: str, count: int) -> None:
    model = myo_sim.load_spec(name).compile()
    fingerprint = model_fingerprint(model)
    document = export_model(model, tmp_path / "model.json")
    assert len(document["muscles"]) == count
    assert sum(len(t["elements"]) for t in document["tendons"]) == model.nwrap
    assert len(document["joints"]) == model.njnt
    assert len(document["equalities"]) == model.neq
    assert json.loads((tmp_path / "model.json").read_text()) == document
    data = mujoco.MjData(model)
    project_joint_couplings(JointCouplings.from_model(model), data.qpos)
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(document["frames"][0]["geom_pos"], data.geom_xpos)
    np.testing.assert_allclose(document["frames"][0]["site_pos"], data.site_xpos)
    mesh = next(g for g in document["geoms"] if g["type"] == "MESH")
    mid = model.geom_dataid[mesh["id"]]
    np.testing.assert_array_equal(
        mesh["faces"],
        model.mesh_face[model.mesh_faceadr[mid] : model.mesh_faceadr[mid] + model.mesh_facenum[mid]],
    )
    assert model_fingerprint(model) == fingerprint


def test_verified_fullbody_ownership_preserves_generated_left_provenance(tmp_path: Path) -> None:
    model = myo_sim.load_spec("myofullbody").compile()
    fingerprint = model_fingerprint(model)
    document = export_model(model, tmp_path / "fullbody.json", source_model="myofullbody")

    assert document["schema"] == 2
    assert document["patch_schema"] == 1
    assert document["provenance"]["scope"] == "instance"
    assert document["provenance"]["status"] == "verified"
    assert document["provenance"]["source_model"] == "myofullbody"
    bodies = {body["source_name"]: body for body in document["bodies"]}
    sites = {site["source_name"]: site for site in document["sites"]}
    for kind, name in ((bodies, "humerus"), (sites, "DELT1_DELT1-P4")):
        for side in ("r", "l"):
            assert kind[f"{name}_{side}"]["origin"] == dict(
                status="canonical",
                chain_fragment="arm/assets/myoarm_r_chain.xml",
                canonical_name=f"{name}_r",
                generated_left=side == "l",
            )
    assert bodies["head"]["origin"]["chain_fragment"] == "head/assets/myohead_rigid_chain.xml"
    assert bodies["pelvis"]["origin"]["chain_fragment"] == "leg/assets/myolegs_chain.xml"
    assert bodies["Full Body"]["origin"]["status"] == "unknown"
    assert model_fingerprint(model) == fingerprint


@pytest.mark.parametrize("source_model", [None, "myoarm_r"])
def test_custom_named_anatomy_does_not_claim_canonical_ownership(tmp_path: Path, source_model: str | None) -> None:
    # Familiar object names alone cannot establish which source declared them.
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="humerus_l"><geom size=".1"/><site name="DELT1_DELT1-P4_l"/></body></worldbody></mujoco>'
    )
    document = export_model(model, tmp_path / "custom.json", source_model=source_model)

    assert document["provenance"]["status"] == "unknown"
    assert document["provenance"]["source_model"] is None
    for entry in document["bodies"] + document["sites"]:
        assert entry["origin"] == dict(status="unknown", chain_fragment=None, canonical_name=None, generated_left=None)


@pytest.mark.parametrize("shape", ["sphere", "cylinder"])
@pytest.mark.parametrize("sidesite,poses", [(True, [[0.0], [0.3]]), (True, [[0.3]]), (False, [[0.0], [0.3]])])
def test_inside_sidesite_flags_and_warnings_use_native_pose_coordinates(
    tmp_path: Path, shape: str, sidesite: bool, poses: list[list[float]]
) -> None:
    side_attribute = ' sidesite="side"' if sidesite else ""
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><worldbody><geom name="wrap" type="{shape}" size=".1 .2"/>'
        '<site name="start" pos="-.3 0 0"/><site name="end" pos=".3 0 0"/>'
        '<body><joint type="slide" axis="1 0 0"/><geom size=".001" mass=".01"/>'
        '<site name="side" pos="0 0 .04"/></body></worldbody>'
        f'<tendon><spatial name="path"><site site="start"/><geom geom="wrap"{side_attribute}/>'
        '<site site="end"/></spatial></tendon><actuator>'
        '<muscle name="muscle" tendon="path" force="10" lengthrange=".1 .8"/>'
        "</actuator></mujoco>"
    )
    document = export_model(model, tmp_path / f"{shape}.json", qpos=np.asarray(poses))

    element = document["tendons"][0]["elements"][1]
    assert document["muscles"][0]["has_sidesite_constraints"] is sidesite
    assert document["muscles"][0]["force"] == 10
    if not sidesite:
        assert element["sidesite"] is None
        assert all(frame["wrap_flags"] == [] for frame in document["frames"])
        assert document["warnings"] == []
        return
    assert element["sidesite"] == model.site("side").id
    expected_inside = [pose[0] == 0.0 for pose in poses]
    assert [frame["wrap_flags"][0]["sidesite_inside"] for frame in document["frames"]] == expected_inside
    if not any(expected_inside):
        assert document["warnings"] == []
        return
    warning = document["warnings"][0]
    assert warning["code"] == "inside_sidesite"
    assert warning["tendon"] == 0 and warning["element"] == 1
    assert warning["geom"] == model.geom("wrap").id
    assert warning["sidesite"] == model.site("side").id
    assert warning["frames"] == [index for index, inside in enumerate(expected_inside) if inside]
    assert "MuJoCo" in warning["message"]


@pytest.mark.parametrize(
    "geom_rgba,material_rgba",
    [
        (None, None),
        (".2 .3 .4 .6", None),
        (None, "1 0 0 .7"),
        (".5 .5 .5 1", "1 0 0 .7"),
        (".5 .5 .5 .2", "1 0 0 .7"),
        (".2 .3 .4 1", "1 0 0 .7"),
        (".2 .3 .4 .3", "1 0 0 .7"),
        (None, "1 0 0 0"),
    ],
)
def test_exported_geom_rgba_matches_native_appearance(tmp_path: Path, geom_rgba: str | None, material_rgba: str | None) -> None:
    """Material/default colors and explicit RGB/alpha overrides match MuJoCo without OpenGL."""
    asset = f'<asset><material name="color" rgba="{material_rgba}"/></asset>' if material_rgba is not None else ""
    attributes = ' material="color"' if material_rgba is not None else ""
    if geom_rgba is not None:
        attributes += f' rgba="{geom_rgba}"'
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco>{asset}<worldbody><geom name="colored" size=".1"{attributes}/></worldbody></mujoco>'
    )
    fingerprint = model_fingerprint(model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    scene = mujoco.MjvScene(model, maxgeom=10)
    mujoco.mjv_updateScene(
        model, data, mujoco.MjvOption(), mujoco.MjvPerturb(), mujoco.MjvCamera(), mujoco.mjtCatBit.mjCAT_ALL, scene
    )

    document = export_model(model, tmp_path / "colors.json")

    if scene.ngeom == 0:  # MuJoCo omits fully transparent geometry from the scene.
        assert document["geoms"][0]["rgba"][3] == 0
    else:
        assert scene.ngeom == 1
        np.testing.assert_array_equal(document["geoms"][0]["rgba"], scene.geoms[0].rgba)
    assert model_fingerprint(model) == fingerprint


def test_elbow_range_length_derivatives(tmp_path: Path) -> None:
    model = myo_sim.load_spec("myoelbow").compile()
    poses = joint_sweep(model, "r_elbow_flex", 31)
    document = export_model(model, tmp_path / "sweep.json", qpos=poses)
    data = mujoco.MjData(model)
    for pose, frame in zip(poses[1:-1], document["frames"][1:-1]):
        lengths = []
        for sign in (-1, 1):
            data.qpos[:] = pose + sign * 1e-6
            mujoco.mj_forward(model, data)
            lengths.append(data.actuator_length.copy())
        derivative = (lengths[1] - lengths[0]) / 2e-6
        np.testing.assert_allclose(np.asarray(frame["moment_arms"])[:, 0], derivative, atol=1e-7, rtol=1e-5)


def test_couplings_and_joint_types_are_not_collapsed(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body name="root"><freejoint name="free"/><geom size=".1"/>
        <body name="ball_body" pos="0 0 1"><joint name="ball" type="ball"/><geom size=".1"/>
          <body name="a"><joint name="driver" ref="20" range="-60 60"/><geom size=".1"/>
            <body name="b"><joint name="follower" type="slide" ref=".1" range="-.5 .5"/>
              <geom size=".1"/></body></body></body></body></worldbody>
      <equality><joint joint1="follower" joint2="driver" polycoef=".02 .3 .4 0 0"/></equality>
      </mujoco>""")
    poses = np.tile(model.qpos0, (2, 1))
    poses[1, model.joint("driver").qposadr[0]] += 0.2
    document = export_model(model, tmp_path / "coupled.json", qpos=poses)
    assert {j["type"] for j in document["joints"]} == {"FREE", "BALL", "HINGE", "SLIDE"}
    follower = model.joint("follower").qposadr[0]
    np.testing.assert_allclose(
        [f["qpos"][follower] for f in document["frames"]],
        [0.12, 0.12 + 0.3 * 0.2 + 0.4 * 0.2**2],
    )
    with pytest.raises(ValueError, match="independent"):
        joint_sweep(model, "follower")


@pytest.mark.parametrize("poses", [np.zeros((0, 1)), np.zeros((3, 2)), [[np.nan]]])
def test_invalid_poses_rejected(tmp_path: Path, poses: Any) -> None:
    model = myo_sim.load_spec("myoelbow").compile()
    with pytest.raises(ValueError, match="qpos"):
        export_model(model, tmp_path / "bad.json", qpos=poses)


def test_generated_labels_do_not_replace_source_names(tmp_path: Path) -> None:
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <site pos="1 0 0"/><site name="site_0" pos="0 1 0"/>
      </worldbody></mujoco>""")
    document = export_model(model, tmp_path / "unnamed.json")
    assert document["sites"][0]["name"] == document["sites"][1]["name"]
    assert document["sites"][0]["source_name"] == ""
    assert document["sites"][1]["source_name"] == "site_0"


@pytest.mark.parametrize("zero_acceleration", [False, True])
def test_peak_force_matches_native_muscle_gain(zero_acceleration: bool) -> None:
    """Automatic and explicit peak forces agree with MuJoCo at optimal muscle length."""
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body><joint name="q"/><geom size=".1" mass="1"/></body></worldbody>
      <tendon><fixed name="t"><joint joint="q" coef="1"/></fixed></tendon>
      <actuator>
        <muscle name="automatic" tendon="t" force="-1" scale="200" range="0 1" lengthrange="0 1"/>
        <muscle name="explicit" tendon="t" force="123" range="0 1" lengthrange="0 1"/>
        <general name="ordinary" joint="q" gainprm="1 0 -7"/>
      </actuator></mujoco>""")
    if zero_acceleration:
        model.actuator_acc0[0] = 0
    before = model.actuator_gainprm.copy()
    result = _peak_force(model)
    for actuator in (0, 1):
        native = -mujoco.mju_muscleGain(
            1.0, 0.0, model.actuator_lengthrange[actuator], model.actuator_acc0[actuator], model.actuator_gainprm[actuator, :9]
        )
        np.testing.assert_allclose(result[actuator], native)
    assert result[1] == 123
    assert result[2] == -7
    np.testing.assert_array_equal(model.actuator_gainprm, before)


def test_core_import_does_not_load_optional_apps() -> None:
    """The anatomy API imports without optional editors, mesh or training libraries."""
    code = (
        "import sys; import myo_sim.authoring; "
        "unexpected = {'myosuite', 'trimesh', 'bpy', 'torch', 'jax'} & sys.modules.keys(); "
        "assert not unexpected, unexpected"
    )
    subprocess.run(
        [sys.executable, "-c", code], cwd=Path(__file__).resolve().parents[1], check=True, capture_output=True, text=True
    )
