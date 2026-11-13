# Copyright (c) MyoSuite Authors. All rights reserved.
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.
"""Portable native model archives, gated by compiled and kinematic equivalence."""

from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from myo_sim.build.compose import apply_compiled_inertia_precision
from myo_sim.physics.joint_coupling import JointCouplings, project_joint_couplings

from .edits import _compile_source
from .interchange import model_fingerprint


def _bundle_assets(spec: mujoco.MjSpec) -> dict[str, bytes]:
    provided = dict(spec.assets)
    assets = {}
    for elements, directory_attribute in (
        (spec.meshes, "meshdir"),
        (spec.textures, "texturedir"),
        (spec.hfields, None),
        (spec.skins, None),
    ):
        for element in elements:
            compiler = getattr(element, "compiler", spec.compiler)
            directory = getattr(compiler, directory_attribute) if directory_attribute is not None else ""
            for filename in filter(None, (element.file, *getattr(element, "cubefiles", ()))):
                name = Path(filename.replace("\\", "/")).name
                # Native attachment uses the parent's global strippath setting.
                resolved = name if spec.strippath else filename.replace("\\", "/")
                reference = Path(directory) / resolved
                aliases = [data for key, data in provided.items() if Path(key.replace("\\", "/")).name == name]
                if aliases and any(data != aliases[0] for data in aliases[1:]):
                    raise ValueError(f"Ambiguous asset basename {name!r}; use unique source asset names")
                content = next(
                    (provided[key] for key in (str(reference), reference.as_posix(), resolved, name) if key in provided),
                    None,
                )
                if content is None:
                    if aliases:
                        content = aliases[0]
                    else:
                        if compiler is not spec.compiler and not reference.is_absolute():
                            raise ValueError(
                                f"Attached asset {filename!r} has an unavailable child-relative model directory; "
                                "use absolute asset filenames or supply its bytes in the parent spec.assets"
                            )
                        try:
                            content = (Path(spec.modelfiledir) / reference).read_bytes()
                        except FileNotFoundError as exc:
                            raise ValueError(
                                f"Cannot resolve asset {filename!r} from public source paths; an attached child-relative "
                                "model directory may be unavailable. Use absolute asset filenames or supply its bytes "
                                "in the parent spec.assets"
                            ) from exc
                if name in assets and assets[name] != content:
                    raise ValueError(f"Ambiguous asset basename {name!r}; use unique source asset names")
                assets[name] = content
    return assets


def _numbers(value: Any) -> str:
    return " ".join(format(float(v), ".17g") for v in np.asarray(value).reshape(-1))


def _archive_xml(spec: mujoco.MjSpec, model: mujoco.MjModel) -> str:
    """Normalize native serialization on a disposable copy, without editing anatomy."""
    root = ET.fromstring(spec.to_xml())
    definitions = list(root.iter("default"))
    names = [node.get("class", "main") for node in definitions]
    if len(names) != len(set(names)):
        # Attached specs can serialize duplicate classes and unnamed nested main
        # defaults. Rename definitions, then repair XML references: some native
        # bindings retain stale class strings after a default's name changes.
        groups = (
            ("worldbody", "joint", "joints"),
            ("worldbody", "geom", "geoms"),
            ("worldbody", "site", "sites"),
            ("worldbody", "camera", "cameras"),
            ("worldbody", "light", "lights"),
            ("asset", "mesh", "meshes"),
            ("asset", "material", "materials"),
            ("contact", "pair", "pairs"),
            ("deformable", "flex", "flexes"),
            ("tendon", None, "tendons"),
            ("actuator", None, "actuators"),
            ("equality", None, "equalities"),
        )
        references = {
            group: [getattr(element, "classname", None) for element in getattr(spec, group, ())] for _, _, group in groups
        }
        prefix = "__archive_default_"
        while any(name.startswith(prefix) for name in names):
            prefix += "_"
        spec.default.name = prefix + "root"
        replacements = {}
        serial = 0
        for name in dict.fromkeys(names):
            while (default := spec.find_default(name)) is not None:
                default.name = prefix + str(serial)
                replacements.setdefault(name, default.name)
                serial += 1
        spec.default.name = "main"
        root = ET.fromstring(spec.to_xml())
        for container, tag, group in groups:
            section = root.find(container)
            nodes = [] if section is None else list(section.iter(tag)) if tag else list(section)
            if len(nodes) != len(references[group]):
                raise ValueError(f"Native archive cannot match serialized {group} to source elements")
            for node, default in zip(nodes, references[group], strict=True):
                if default is not None:
                    node.set("class", default.name)
                elif node.get("class") in replacements:
                    node.set("class", replacements[node.get("class")])
        # Runtime elements now carry their explicit class; body/frame inheritance
        # strings would otherwise refer to the previous, ambiguous class names.
        for node in root.iter():
            node.attrib.pop("childclass", None)

    # Native XML emits literal whitespace in custom text attributes. Parsing that
    # XML normalizes tabs/newlines/CR to spaces; restore the original strings so
    # ElementTree writes character references that preserve the compiled bytes.
    custom = root.find("custom")
    texts = [] if custom is None else custom.findall("text")
    if len(texts) != len(spec.texts):
        raise ValueError("Native archive changed the source custom text topology")
    for node, text in zip(texts, spec.texts, strict=True):
        node.set("data", text.data)

    # Preserve values the native writer omits for zero-mass attachment frames.
    bodies = list(root.find("worldbody").iter("body"))
    if len(bodies) != model.nbody - 1:
        raise ValueError("Native archive changed the source body topology")
    for body_id, node in enumerate(bodies, 1):
        if model.body_mass[body_id] == 0:
            inertial = node.find("inertial")
            if inertial is None:
                inertial = ET.SubElement(node, "inertial")
            inertial.attrib = dict(
                mass="0",
                pos=_numbers(model.body_ipos[body_id]),
                quat=_numbers(model.body_iquat[body_id]),
                diaginertia=_numbers(model.body_inertia[body_id]),
            )
    apply_compiled_inertia_precision(root, model)
    # MjsTendon has no classname reference in supported MuJoCo versions. Its
    # appearance must be explicit when otherwise-equivalent attached defaults differ.
    tendons = root.find("tendon")
    if tendons is not None:
        for index, node in enumerate(tendons):
            node.set("width", _numbers(model.tendon_width[index]))
            node.set("rgba", _numbers(model.tendon_rgba[index]))
    compiler = root.find("compiler")
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
        compiler.attrib.pop("texturedir", None)
    assets = root.find("asset")
    if assets is not None:
        for node in assets:
            for key in ("file", "fileback", "filedown", "filefront", "fileleft", "fileright", "fileup"):
                if node.get(key):
                    node.set(key, Path(node.get(key).replace("\\", "/")).name)
    return ET.tostring(root, encoding="unicode")


_GEOMETRIC = {
    "body_pos",
    "body_quat",
    "body_ipos",
    "body_iquat",
    "geom_pos",
    "geom_quat",
    "geom_size",
    "geom_aabb",
    "geom_rbound",
    "site_pos",
    "site_quat",
    "site_size",
    "cam_pos",
    "cam_quat",
    "cam_mat0",
    "cam_pos0",
    "cam_poscom0",
    "light_pos",
    "light_dir",
    "light_poscom0",
    "bvh_aabb",
    "jnt_pos",
    "jnt_axis",
    "qpos0",
    "key_qpos",
}
_REPRESENTATION = {
    "paths",
    "mesh_pathadr",
    "tex_pathadr",
    "hfield_pathadr",
    "body_sameframe",
    "geom_sameframe",
    "site_sameframe",
}


def _compare(name: str, before: Any, after: Any, *, atol: float = 1e-12) -> None:
    a, b = np.asarray(before), np.asarray(after)
    equal = a.shape == b.shape and (
        np.allclose(a, b, rtol=1e-5, atol=atol, equal_nan=True) if a.dtype.kind == "f" else np.array_equal(a, b)
    )
    if not equal:
        raise ValueError(f"Native archive equivalence failed for {name}")


def _poses(model: mujoco.MjModel) -> list[np.ndarray]:
    couplings = JointCouplings.from_model(model)
    dependent = set(couplings.dep_qpos.tolist()) | set(couplings.locked_qpos.tolist())
    poses = [model.qpos0.copy(), *[pose.copy() for pose in model.key_qpos[:2]]]
    for direction in (-1, 1):
        pose = model.qpos0.copy()
        for j in range(model.njnt):
            address = model.jnt_qposadr[j]
            if model.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE) and address not in dependent:
                if model.jnt_limited[j]:
                    low, high = model.jnt_range[j]
                    pose[address] = np.clip(pose[address] + direction * 0.1 * (high - low), low, high)
                else:
                    pose[address] += direction * 0.1
        poses.append(pose)
    for pose in poses:
        project_joint_couplings(couplings, pose)
    return poses


def _validate_equivalence(source: mujoco.MjModel, loaded: mujoco.MjModel) -> None:
    if source.names != loaded.names:
        raise ValueError("Native archive equivalence failed for object names")
    if source.text_data != loaded.text_data:
        raise ValueError("Native archive equivalence failed for custom text")
    for name in (
        "nq",
        "nv",
        "nu",
        "na",
        "nbody",
        "njnt",
        "ngeom",
        "nsite",
        "neq",
        "ntendon",
        "nsensor",
        "nsensordata",
        "nmesh",
        "nmat",
        "ncam",
        "nlight",
        "nskin",
        "nflex",
        "nexclude",
        "npair",
        "nkey",
        "ntex",
        "nhfield",
        "nuserdata",
    ):
        _compare(name, getattr(source, name), getattr(loaded, name))
    mesh_geoms = source.geom_type == mujoco.mjtGeom.mjGEOM_MESH
    paired = np.concatenate((source.pair_geom1, source.pair_geom2))
    visual_meshes = not (
        np.any(source.geom_contype[mesh_geoms] | source.geom_conaffinity[mesh_geoms]) or np.any(mesh_geoms[paired])
    )
    for name in dir(source):
        before = getattr(source, name)
        if not name.startswith("_") and isinstance(before, np.ndarray) and name not in _REPRESENTATION:
            # MuJoCo 3.4's file/VFS cache can retain different polygonizations of
            # visual hulls. Original vertices/faces, visual transforms and inferred
            # body inertias remain checked; collision-mesh hulls are never exempt.
            if visual_meshes and name.startswith("mesh_poly"):
                continue
            _compare(name, before, getattr(loaded, name), atol=1e-6 if name in _GEOMETRIC else 1e-12)
    for section in ("opt", "stat"):
        for name in dir(before := getattr(source, section)):
            if not name.startswith("_") and not callable(value := getattr(before, name)):
                _compare(
                    f"{section}.{name}",
                    value,
                    getattr(getattr(loaded, section), name),
                    atol=1e-6 if name == "center" else 1e-12,
                )
    data = [mujoco.MjData(source), mujoco.MjData(loaded)]
    couplings = JointCouplings.from_model(loaded)
    for pose in _poses(source):
        for model, state in zip((source, loaded), data, strict=True):
            state.qpos[:] = pose
            if model is loaded:
                project_joint_couplings(couplings, state.qpos)
            mujoco.mj_forward(model, state)
            if not np.isfinite(state.ten_length).all():
                raise ValueError("Nonfinite native tendon lengths cannot be archived")
        _compare("native tendon lengths", data[0].ten_length, data[1].ten_length, atol=1e-6)
        moments = []
        for model, state in zip((source, loaded), data, strict=True):
            dense = np.zeros((model.ntendon, model.nv))
            if mujoco.mj_isSparse(model):
                layout = model if hasattr(model, "ten_J_rownnz") else state
                mujoco.mju_sparse2dense(
                    dense, state.ten_J.reshape(-1), layout.ten_J_rownnz, layout.ten_J_rowadr, layout.ten_J_colind.reshape(-1)
                )
            else:
                dense[:] = state.ten_J.reshape(dense.shape)
            if not np.isfinite(dense).all():
                raise ValueError("Nonfinite native tendon moment arms cannot be archived")
            moments.append(dense)
        _compare("native tendon moment arms", *moments, atol=1e-5)


def save_model_archive(spec: mujoco.MjSpec, output: str | Path, *, overwrite: bool = False) -> Path:
    """Save a portable native MuJoCo ZIP after reloading and checking equivalence.

    The source is copied. Existing files require ``overwrite=True`` and are
    replaced atomically only after validation. Native XML uses finite precision:
    explicit principal moments retain roundtrip precision; other compiled floats
    allow 1e-5 relative error, geometric quantities also allow
    1e-6 absolute error, and native tendon moments allow 1e-5 absolute error.
    Integer topology, object names and options are checked as well. Derived frame
    flags, visual-only hull polygon caches and external path spellings are
    representation only; raw mesh vertices/faces and body inertias are checked.

    Raises:
        FileExistsError: The destination exists without overwrite permission.
        ValueError: Compilation, normalization or substantive equivalence fails.
    """
    output = Path(output).resolve()

    def check_destination() -> None:
        if output.exists() and (not overwrite or not output.is_file()):
            raise FileExistsError(f"Output exists: {output}; pass overwrite=True to replace a file")

    check_destination()
    candidate = spec.copy()
    source = _compile_source(candidate)
    fingerprint = model_fingerprint(source)
    assets = _bundle_assets(candidate)
    xml = _archive_xml(candidate, source)
    if model_fingerprint(_compile_source(candidate)) != fingerprint:
        raise ValueError("Archive normalization changed the compiled source model")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as folder:
        staged = Path(folder) / output.name
        with zipfile.ZipFile(staged, "w") as archive:
            for name, contents in assets.items():
                archive.writestr(name, contents)
            archive.writestr("__myo_sim_model__.xml", xml)
        loaded = _compile_source(mujoco.MjSpec.from_zip(staged))
        _validate_equivalence(source, loaded)
        check_destination()
        staged.replace(output)
    return output
