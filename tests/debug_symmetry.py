"""
MJCF symmetry checker (include-aware)

Checks left/right symmetry in a MuJoCo MJCF model by parsing a main XML and all
`<include file="...">` sub-XMLs.

Default behavior:
- Only prints elements with symmetry discrepancies
- Pauses after each printed item
- Skips elements without a mirror counterpart

Supports:
- Bodies / joints: `_r` vs `_l`
- Sites / geoms: `<name>` vs `<name>_left`

Examples:
  python symmetry_check.py main.xml --joints
  python symmetry_check.py main.xml --joints --print-all --no-pause
  python symmetry_check.py main.xml --joints --show-single
"""

import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R


#some finger geoms have euler angles for wrapping that are different 
#in order to achieve muscle symmetry
GEOM_EULER_SIGNFLIP_EXCEPTIONS = {
    "MPthumb_wrap",
    "2ndmcp_ellipsoid_wrap",
    "Secondpm_wrap",
    "3rdmcp_ellipsoid_wrap",
    "4thmcp_ellipsoid_wrap",
    "Fourthpm_wrap",
    "Fourthmd_wrap",
    "5thmcp_ellipsoid_wrap"
}

def euler_signflip_match(elem1, elem2, tol=1e-6):
    """
    Return True if Euler angles satisfy:
      euler1 ≈ -euler2  (elementwise)
    """
    _, _, euler1 = get_orientation_info(elem1)
    _, _, euler2 = get_orientation_info(elem2)

    if euler1 is None or euler2 is None:
        return False

    e1 = np.array(euler1, dtype=float)
    e2 = np.array(euler2, dtype=float)

    return np.allclose(e1, -e2, atol=tol, rtol=0.0)

def _iter_includes(root: ET.Element):
    """Yield file attribute values from <include file="..."> tags."""
    for inc in root.iter("include"):
        f = inc.attrib.get("file")
        if f:
            yield f

def has_discrepancy(
    pos1, ori1, pos2, ori2, *,
    is_joint=False,
    pos_tol=1e-8,
    ori_tol_rad=1e-6,
):
    """
    Return True if mirrored comparison exceeds tolerance.
    """
    try:
        if is_joint:
            # --- joint pos ---
            if pos1 is not None and pos2 is not None:
                pos2 = pos2.copy()
                pos2[2] *= -1
                if np.linalg.norm(pos1 - pos2) > pos_tol:
                    return True

            # --- joint axis ---
            if ori1 is not None and ori2 is not None:
                axis2 = ori2.copy()
                axis2[0] *= -1
                axis2[1] *= -1
                if np.linalg.norm(ori1 - axis2) > pos_tol:
                    return True

        else:
            # --- position ---
            if pos1 is not None and pos2 is not None:
                pos2 = pos2.copy()
                pos2[2] *= -1
                if np.linalg.norm(pos1 - pos2) > pos_tol:
                    return True

            # --- orientation ---
            if ori1 is not None and ori2 is not None:
                q1 = ori1 / np.linalg.norm(ori1)
                q2 = ori2.copy()
                q2[1] *= -1
                q2[2] *= -1
                q2 /= np.linalg.norm(q2)

                if np.dot(q1, q2) < 0:
                    q2 = -q2

                ang = 2 * np.arccos(np.clip(np.dot(q1, q2), -1.0, 1.0))
                if ang > ori_tol_rad:
                    return True

        return False

    except Exception:
        # If comparison fails, treat as discrepancy
        return True

def collect_mjcf_files(main_xml: str | Path):
    """
    Recursively collect all MJCF XML files reachable via <include file="...">.

    Returns:
      files: list[Path] in discovery order (main first)
    """
    main_xml = Path(main_xml).expanduser().resolve()
    seen: set[Path] = set()
    order: list[Path] = []

    def dfs(xml_path: Path):
        xml_path = xml_path.expanduser().resolve()
        if xml_path in seen:
            return
        if not xml_path.exists():
            raise FileNotFoundError(f"Included XML not found: {xml_path}")
        seen.add(xml_path)
        order.append(xml_path)

        tree = ET.parse(xml_path)
        root = tree.getroot()

        base_dir = xml_path.parent
        for rel in _iter_includes(root):
            inc_path = (base_dir / rel).resolve()
            dfs(inc_path)

    dfs(main_xml)
    return order


def parse_float_list(text):
    return list(map(float, text.strip().split()))

def get_orientation_info(elem):
    quat = axisangle = euler = None

    if 'euler' in elem.attrib:
        euler = parse_float_list(elem.attrib['euler'])
        quat = R.from_euler('xyz', euler).as_quat()

    elif 'quat' in elem.attrib:
        quat = parse_float_list(elem.attrib['quat'])
        euler = R.from_quat(quat).as_euler('xyz', degrees=True)

    elif 'axisangle' in elem.attrib:
        axisangle = parse_float_list(elem.attrib['axisangle'])
        axis = np.array(axisangle[:3])
        angle = axisangle[3]
        if np.linalg.norm(axis) > 0:
            quat = R.from_rotvec(axis / np.linalg.norm(axis) * angle).as_quat()
            euler = R.from_quat(quat).as_euler('xyz', degrees=True)

    return quat, axisangle, euler

def get_pos_and_ori(elem):
    pos = elem.attrib.get('pos', 'N/A')
    quat, axisangle, euler = get_orientation_info(elem)
    return pos, quat, axisangle, euler

def safe_parse_array(val):
    """
    Convert:
      - "0.1 0.2 0.3" → np.array([0.1, 0.2, 0.3])
      - [0.1, 0.2, 0.3] → np.array([0.1, 0.2, 0.3])
      - "N/A" or None → None
    """
    if val is None:
        return None
    if isinstance(val, str):
        if val.strip().upper() == "N/A" or val.strip() == "":
            return None
        return np.array(list(map(float, val.strip().split())), dtype=float)
    if isinstance(val, (list, tuple, np.ndarray)):
        return np.array(val, dtype=float)
    raise ValueError(f"Unsupported type for conversion: {type(val)}")


def parse_elements_from_files(xml_files):
    """
    Parse bodies/geoms/sites/joints from multiple XML files.

    Returns:
      bodies, geoms, sites, joints: dict[str, (ET.Element, Path)]
        mapping element name -> (element, source_file)
    """
    bodies, geoms, sites, joints = {}, {}, {}, {}

    def add_unique(dst, name, elem, src: Path):
        # Keep first occurrence by default; warn on duplicates
        if name in dst:
            prev_src = dst[name][1]
            if prev_src != src:
                print(f"[WARN] Duplicate name '{name}' in:\n  - {prev_src}\n  - {src}\n  Keeping first.")
            return
        dst[name] = (elem, src)

    for xml_path in xml_files:
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for body in root.iter('body'):
            name = body.attrib.get('name')
            if name:
                add_unique(bodies, name, body, xml_path)

        for geom in root.iter('geom'):
            name = geom.attrib.get('name')
            if name:
                add_unique(geoms, name, geom, xml_path)

        for site in root.iter('site'):
            name = site.attrib.get('name')
            if name:
                add_unique(sites, name, site, xml_path)

        for joint in root.iter('joint'):
            name = joint.attrib.get('name')
            if name:
                add_unique(joints, name, joint, xml_path)

    return bodies, geoms, sites, joints


def print_pair(
    name1, elem1, name2=None, elem2=None, *,
    is_joint=False, src1=None, src2=None,
    print_all=False, pause=True, skip_single = True,
):
    # ---- FIRST: extract values silently ----
    def extract_info(elem, is_joint=False):
        if is_joint:
            pos = safe_parse_array(elem.attrib.get('pos', 'N/A'))
            axis = safe_parse_array(elem.attrib.get('axis', 'N/A'))
            return pos, axis
        else:
            pos, quat, _, _ = get_pos_and_ori(elem)
            return safe_parse_array(pos), safe_parse_array(quat)

    pos1, ori1 = extract_info(elem1, is_joint)

    if name2 and elem2 is not None:
        pos2, ori2 = extract_info(elem2, is_joint)

        discrep = has_discrepancy(
            pos1, ori1, pos2, ori2, is_joint=is_joint
        )

        if (discrep and (not is_joint) and name1 is not None):
            base = name1[:-5] if name1.endswith("_left") else name1
            if base in GEOM_EULER_SIGNFLIP_EXCEPTIONS:
                if euler_signflip_match(elem1, elem2):
                    discrep = False

        if not print_all and not discrep:
            return "clean"
    else:
        # No counterpart
        if skip_single and not print_all:
            return  "single" # default: skip singletons entirely
        pos2a = pos2b = None
        discrep = True  # if we're printing all, show it

    print("=" * 60)

    def print_info(name, elem, is_joint=False, src=None):
        src_s = f"  [src: {src}]" if src is not None else ""
        if is_joint:
            pos = elem.attrib.get('pos', 'N/A')
            axis = elem.attrib.get('axis', 'N/A')
            rng = elem.attrib.get('range', 'N/A')
            print(f"{name}:{src_s}")
            print(f"  type  : {elem.attrib.get('type', 'N/A')}")
            print(f"  pos   : {pos}")
            print(f"  axis  : {axis}")
            print(f"  range : {rng}")
        else:
            pos, quat, axisangle, euler = get_pos_and_ori(elem)
            print(f"{name}:{src_s}")
            print(f"  pos       : {pos}")
            print(f"  quat      : {quat if quat is not None else 'N/A'}")
            print(f"  axisangle : {axisangle if axisangle is not None else 'N/A'}")
            print(f"  euler xyz : {euler if euler is not None else 'N/A'}")

    print_info(name1, elem1, is_joint, src1)

    if name2 and elem2 is not None:
        print("\nvs\n")
        print_info(name2, elem2, is_joint, src2)
    
    if discrep:
        q1 = ori1 / np.linalg.norm(ori1)
        q2 = ori2 / np.linalg.norm(ori2)
        q2m = q2.copy()
        q2m[1] *= -1
        q2m[2] *= -1
        ang = 2 * np.arccos(np.clip(np.dot(q1, q2m), -1.0, 1.0))
        print(f"[DEBUG] angle diff (deg): {np.degrees(ang)}")
    
    if pause:
        input("\nPress Enter to continue...\n")
    
    
    return "discrepancy" if discrep else "clean"

def resolve_checks(args):
    checks = {
        "bodies": args.bodies,
        "joints": args.joints,
        "geoms": args.geoms,
        "sites": args.sites,
    }
    if not any(checks.values()):
        for k in checks:
            checks[k] = True
    return checks

def main(xml_path, checks, *, include=True, print_all=False, pause=True, skip_single=True):
    counts = {
        "discrepancy": 0,
        "clean": 0,
        "single": 0,
    }
    
    # Collect files: main + sub-xmls (via <include>)
    if include:
        xml_files = collect_mjcf_files(xml_path)
    else:
        xml_files = [Path(xml_path).expanduser().resolve()]

    print("[INFO] Parsing files in this order:")
    for p in xml_files:
        print(f"  - {p}")

    bodies, geoms, sites, joints = parse_elements_from_files(xml_files)

    # --- Bodies ---
    if checks["bodies"]:
        for name, (elem, src) in bodies.items():
            if name.endswith('_l'):
                continue
            if name.endswith('_r'):
                base = name[:-2]
                mirror_name = f"{base}_l"
                mirror = bodies.get(mirror_name)
                mirror_elem, mirror_src = (mirror[0], mirror[1]) if mirror else (None, None)
                status = print_pair(name, elem, mirror_name, mirror_elem, is_joint=False, src1=src, src2=mirror_src, print_all=print_all, pause=pause, skip_single=True)
            else:
                status = print_pair(name, elem, is_joint=False, src1=src, print_all=print_all, pause=pause, skip_single=True)
            if status is not None:
                counts[status] += 1
    # --- Sites ---
    if checks["sites"]:
        for name, (elem, src) in sites.items():
            if name.endswith('_left'):
                continue
            mirror_name = f"{name}_left"
            mirror = sites.get(mirror_name)
            mirror_elem, mirror_src = (mirror[0], mirror[1]) if mirror else (None, None)
            status = print_pair(name, elem, mirror_name, mirror_elem, is_joint=False, src1=src, src2=mirror_src, print_all=print_all, pause=pause, skip_single=True)
            if status is not None:
                counts[status] += 1

    # --- Geoms ---
    if checks["geoms"]:
        for name, (elem, src) in geoms.items():
            if name.endswith('_left'):
                continue
            mirror_name = f"{name}_left"
            mirror = geoms.get(mirror_name)
            mirror_elem, mirror_src = (mirror[0], mirror[1]) if mirror else (None, None)
            status = print_pair(name, elem, mirror_name, mirror_elem, is_joint=False, src1=src, src2=mirror_src, print_all=print_all, pause=pause, skip_single=True)
            if status is not None:
                counts[status] += 1

    # --- Joints ---
    if checks["joints"]:
        for name, (elem, src) in joints.items():
            if name.endswith('_l'):
                continue
            if name.endswith('_r'):
                base = name[:-2]
                mirror_name = f"{base}_l"
                mirror = joints.get(mirror_name)
                mirror_elem, mirror_src = (mirror[0], mirror[1]) if mirror else (None, None)
                status = print_pair(name, elem, mirror_name, mirror_elem, is_joint=True, src1=src, src2=mirror_src, print_all=print_all, pause=pause, skip_single=True)
            else:
                status = print_pair(name, elem, is_joint=True, src1=src, print_all=print_all, pause=pause, skip_single=True)
            if status is not None:
                counts[status] += 1
    
    total_pairs = counts["discrepancy"] + counts["clean"]

    print("\n" + "=" * 60)
    print("[SUMMARY]")
    print(f"  Checked pairs : {total_pairs}")
    print(f"  Discrepancies : {counts['discrepancy']}")
    print(f"  Clean pairs   : {counts['clean']}")
    print(f"  Single skipped: {counts['single']}")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("xml_path", help="Path to the MAIN MJCF XML file")

    parser.add_argument("--bodies", action="store_true", help="Check body symmetry")
    parser.add_argument("--joints", action="store_true", help="Check joint symmetry")
    parser.add_argument("--geoms",  action="store_true", help="Check geom symmetry")
    parser.add_argument("--sites",  action="store_true", help="Check site symmetry")

    parser.add_argument(
        "--print-all",
        action="store_true",
        help="Print all elements, not only those with discrepancies",
    )
    parser.add_argument(
        "--no-pause",
        action="store_true",
        help="Do not pause with Enter between printed items",
    )

    parser.add_argument(
        "--no-include",
        action="store_true",
        help="Do NOT traverse <include file=...>; parse only the provided xml",
    )

    args = parser.parse_args()
    checks = resolve_checks(args)
    main(
        args.xml_path,
        checks,
        include=(not args.no_include),
        print_all=args.print_all,
        pause=(not args.no_pause),
    )
