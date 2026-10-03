"""Regenerate the right-side HYOID neck fragments from a pinned MyoConverter checkout."""

import argparse
import copy
import hashlib
import json
import struct
import xml.etree.ElementTree as E
from pathlib import Path

import mujoco
import numpy as np


def read_stl(path: Path) -> list:
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count:
        raise ValueError("binary STL required")
    out = []
    for i in range(count):
        v = struct.unpack_from("<12fH", raw, 84 + 50 * i)
        out.append((v[:3], [tuple(v[3 + j * 3 : 6 + j * 3]) for j in range(3)]))
    return out


def components(triangles: list) -> list[list[int]]:
    vertices = {}
    for i, (_, tri) in enumerate(triangles):
        for vertex in tri:
            vertices.setdefault(tuple(round(x, 7) for x in vertex), []).append(i)
    adjacency = [set() for _ in triangles]
    for ids in vertices.values():
        for i in ids:
            adjacency[i].update(ids)
    found = []
    seen = set()
    for start in range(len(triangles)):
        if start in seen:
            continue
        stack = [start]
        seen.add(start)
        component = []
        while stack:
            i = stack.pop()
            component.append(i)
            for j in adjacency[i] - seen:
                seen.add(j)
                stack.append(j)
        found.append(component)
    return found


def center_y(component: list[int], triangles: list) -> float:
    return sum(v[1] for i in component for v in triangles[i][1]) / (3 * len(component))


def write_stl(path: Path, indices: list[int], triangles: list, origin_y: float) -> None:
    with path.open("wb") as stream:
        stream.write(b"MyoHead separated cervical vertebra".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(indices)))
        for i in indices:
            normal, tri = triangles[i]
            values = tuple(normal) + tuple(x for v in tri for x in (v[0], v[1] - origin_y, v[2]))
            stream.write(struct.pack("<12fH", *values, 0))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_checkout", type=Path)
    args = parser.parse_args()
    out = Path(__file__).resolve().parents[1] / "myo_sim/models"
    src = args.source_checkout / "models"
    provenance = json.loads((out / "head/validation/provenance.json").read_text())
    for relative, checksum in provenance["files"].items():
        path = out / "meshes/hat_cervical.stl" if relative.startswith("myo_sim/") else src / relative
        if hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
            raise ValueError(f"Pinned source checksum mismatch: {relative}")
    r = E.parse(src / "mjc/Neck6D/neck6d_cvt3.xml").getroot()
    osim = E.parse(src / "osim/Neck6D/HYOID_1.2_ScaledStrenght_UpdatedInertia_adjusted.osim").getroot()
    # Preserve exact OpenSim neutral offsets and child-frame centers (lost in cvt3).
    for joint in osim.findall(".//CustomJoint"):
        frames = joint.findall("./frames/PhysicalOffsetFrame")
        if len(frames) != 2:
            continue
        child = frames[1].findtext("socket_parent").split("/")[-1]
        body = r.find(f'.//body[@name="{child}"]')
        if body is None:
            continue
        parent = np.fromstring(frames[0].findtext("translation"), sep=" ")
        offset = np.fromstring(frames[1].findtext("translation"), sep=" ")
        body.set("pos", " ".join(f"{v:.12g}" for v in parent - offset))
        for mjjoint in body.findall("joint"):
            mjjoint.set("pos", frames[1].findtext("translation").strip())
    # Compile in Y-up source coordinates to flatten fixed thoracic attachment sites.
    r.find("option").attrib.pop("collision", None)
    for mesh in r.findall(".//mesh"):
        mesh.set("file", str(src / "mjc/Neck6D" / mesh.get("file")))
    r.find('.//body[@name="spine"]').set("quat", "1 0 0 0")
    m = mujoco.MjModel.from_xml_string(E.tostring(r).decode())
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    head_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "skull")
    shift = np.array([0, 0.5, 0]) - d.xpos[head_id]
    right = [a for a in r.find("actuator") if not a.get("name").endswith("_L")]
    sitenames = {s.get("site") for a in right for s in r.find(f'./tendon/spatial[@name="{a.get("tendon")}"]')}

    def rename(name: str) -> str:
        return "myohead_" + name + "_r"

    root = E.Element("mujocoinclude")
    chain = E.SubElement(
        root, "body", name="myohead_base", pos=" ".join(f"{v:.12g}" for v in shift), childclass="myohead_articulated"
    )
    source_chain = copy.deepcopy(r.find('.//body[@name="cerv7"]'))
    for body in source_chain.iter("body"):
        name = body.get("name")
        body.set("name", "myohead_" + name)
        for elem in list(body):
            if (
                elem.tag == "geom"
                or (elem.tag == "site" and elem.get("name") not in sitenames)
                or (elem.tag == "body" and elem.get("name") == "jaw")
            ):
                body.remove(elem)
            elif elem.tag == "joint":
                elem.set("name", "myohead_" + elem.get("name"))
                elem.attrib.pop("user", None)
            elif elem.tag == "site":
                elem.set("name", rename(elem.get("name")))
    # Restore source passive rotational mechanics: exact for sagittal release,
    # first-order XYZ approximation for multi-axis motion (source uses Euler XYZ).
    for bushing in osim.findall(".//BushingForce"):
        child = bushing.findtext("socket_frame2").split("/")[-1]
        body = source_chain if child == "cerv7" else source_chain.find(f'.//body[@name="myohead_{child}"]')
        if body is None:
            raise ValueError(f"Missing cervical bushing body: {child}")
        stiffness = np.fromstring(bushing.findtext("rotational_stiffness"), sep=" ")
        damping = np.fromstring(bushing.findtext("rotational_damping"), sep=" ")
        for joint in body.findall("joint"):
            axis = int(np.argmax(np.abs(np.fromstring(joint.get("axis"), sep=" "))))
            joint.set("stiffness", f"{stiffness[axis]:.12g}")
            joint.set("damping", f"{damping[axis]:.12g}")
            joint.set("springref", "0")
    chain.append(source_chain)
    # Thoracic landmarks stay on the existing rigid torso/head attachment frame.
    for site in r.findall(".//site"):
        if site.get("name") not in sitenames:
            continue
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site.get("name"))
        bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.site_bodyid[sid]))
        if bname.startswith("cerv") or bname == "skull":
            continue
        fixed = copy.deepcopy(site)
        fixed.set("name", rename(site.get("name")))
        fixed.set("pos", " ".join(f"{v:.12g}" for v in d.site_xpos[sid]))
        chain.append(fixed)
    # Existing HAT geometry, unchanged in the neutral torso frame.
    triangles = read_stl(out / "meshes/hat_cervical.stl")
    pieces = components(triangles)
    shells = sorted(sorted(pieces, key=len, reverse=True)[:7], key=lambda c: center_y(c, triangles))
    centres = [center_y(c, triangles) for c in shells]
    levels = [list(component) for component in shells]
    shell_ids = {i for component in shells for i in component}
    for component in pieces:
        if any(i in shell_ids for i in component):
            continue
        y = center_y(component, triangles)
        level = min(range(7), key=lambda i: abs(centres[i] - y))
        levels[level].extend(component)
    for level, indices, y in zip(range(7, 0, -1), levels, centres):
        write_stl(out / f"meshes/hat_cervical_c{level}.stl", indices, triangles, y)
    for level, y in zip(range(7, 0, -1), centres):
        body = source_chain.find(f'.//body[@name="myohead_cerv{level}"]') if level != 7 else source_chain
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"cerv{level}")
        pos = np.array([0, y, 0]) - d.xpos[bid] - shift
        E.SubElement(
            body,
            "geom",
            name=f"myohead_cerv{level}_visual",
            type="mesh",
            mesh=f"myohead_cerv{level}",
            pos=" ".join(f"{v:.12g}" for v in pos),
        )
    rigid = E.parse(out / "head/assets/myohead_rigid_chain.xml").getroot()
    skull = source_chain.find('.//body[@name="myohead_skull"]')
    for geom in rigid.findall('.//body[@name="head"]/geom'):
        skull.append(copy.deepcopy(geom))
    coll = copy.deepcopy(rigid.find('.//geom[@name="hat_cervical_coll"]'))
    pos = np.fromstring(coll.get("pos"), sep=" ") - shift
    coll.set("pos", " ".join(f"{v:.12g}" for v in pos))
    chain.append(coll)
    assets = E.Element("mujocoinclude")
    asset = E.SubElement(assets, "asset")
    for level in range(1, 8):
        E.SubElement(asset, "mesh", name=f"myohead_cerv{level}", file=f"meshes/hat_cervical_c{level}.stl")
    default = E.SubElement(assets, "default")
    bdef = E.SubElement(default, "default", {"class": "myohead_articulated"})
    E.SubElement(bdef, "joint", limited="true", armature="1e-5", damping=".05")
    E.SubElement(bdef, "geom", margin=".001", material="mat_myohead", rgba=".8 .8 .8 1", contype="0", conaffinity="0", mass="0")
    E.SubElement(bdef, "site", size=".001", group="3")
    E.SubElement(bdef, "tendon", width=".001", rgba=".95 .3 .3 1")
    mdef = E.SubElement(default, "default", {"class": "myohead_muscle"})
    gen = copy.deepcopy(r.find('./default/default[@class="muscle"]/general'))
    mdef.append(gen)
    eq = E.SubElement(assets, "equality")
    for equality in r.findall("./equality/joint"):
        if not equality.get("joint1").startswith("aux"):
            continue
        e = copy.deepcopy(equality)
        for key in ("name", "joint1", "joint2"):
            e.set(key, "myohead_" + e.get(key))
        eq.append(e)
    tendons = E.Element("mujocoinclude")
    tendon = E.SubElement(tendons, "tendon")
    actuators = E.Element("mujocoinclude")
    actuator = E.SubElement(actuators, "actuator")
    for a in right:
        t = copy.deepcopy(r.find(f'./tendon/spatial[@name="{a.get("tendon")}"]'))
        t.set("name", rename(a.get("tendon")))
        t.set("class", "myohead_articulated")
        for site in t:
            site.set("site", rename(site.get("site")))
        tendon.append(t)
        a = copy.deepcopy(a)
        # cvt3's Geniohyoid fit implies a negative tendon slack length. Restore
        # the source physical L0/LT/F0 with MuJoCo's standard curve instead.
        parameters = np.fromstring(a.get("gainprm"), sep=" ")
        limits = np.fromstring(a.get("lengthrange"), sep=" ")
        fitted_l0 = (limits[1] - limits[0]) / (parameters[1] - parameters[0])
        if limits[0] - parameters[0] * fitted_l0 < 0:
            muscle = osim.find(f'.//*[@name="{a.get("name")}"]')
            angle = float(muscle.findtext("pennation_angle_at_optimal"))
            l0 = float(muscle.findtext("optimal_fiber_length")) * np.cos(angle)
            lt = float(muscle.findtext("tendon_slack_length"))
            force = float(muscle.findtext("max_isometric_force")) * np.cos(angle)
            parameters = [(limits[0] - lt) / l0, (limits[1] - lt) / l0, force, 1, 0.5, 1.6, 1.5, 1.3, 1.2, 0]
            for attribute in ("gainprm", "biasprm"):
                a.set(attribute, " ".join(f"{value:.12g}" for value in parameters))
        a.set("name", rename(a.get("name")))
        a.set("tendon", rename(a.get("tendon")))
        a.set("class", "myohead_muscle")
        actuator.append(a)
    for name, xml in [
        ("myohead_chain", root),
        ("myohead_assets", assets),
        ("myohead_r_tendon", tendons),
        ("myohead_r_muscle", actuators),
    ]:
        xml.insert(
            0,
            E.Comment(
                " Derived from MIT-licensed HYOID / Neck6D; see ../validation/README.md and ../validation/LICENSE.Neck6D. "
            ),
        )
        E.indent(xml, space="    ")
        E.ElementTree(xml).write(out / f"head/assets/{name}.xml", encoding="unicode")
    val = out / "head/validation"
    val.mkdir(exist_ok=True)
    (val / "LICENSE.Neck6D").write_bytes((src / "osim/Neck6D/license").read_bytes())
    print("right muscles", len(right), "shift", shift, "sites", len(sitenames))


if __name__ == "__main__":
    main()
