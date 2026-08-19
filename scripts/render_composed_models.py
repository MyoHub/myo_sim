"""Render every registered composed model to a PNG for human inspection.

Usage:
    uv run python scripts/render_composed_models.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import myo_sim  # noqa: E402
from myo_sim.build.compose import MODEL_REGISTRY, build_spec  # noqa: E402

OUT_DIR = ROOT / "docs" / "images" / "models"
WIDTH, HEIGHT = 900, 900

# Legacy static-XML models outside MODEL_REGISTRY/build_spec: (output name, path, keyframe).
# These typically define no lights of their own (they're meant to be embedded in a scene
# that provides one), so they also get a brighter headlight than the registry sweep below.
STATIC_XML_TARGETS: dict[str, tuple[Path, str | None]] = {
    "myolegs_osl": (myo_sim.MODELS_DIR / "legacy" / "osl" / "myolegs_osl.xml", "stand"),
}


def _character_bounds(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, float]:
    """Bounding sphere over geoms attached to an actual body, excluding worldbody scene props."""
    mins = np.full(3, np.inf)
    maxs = np.full(3, -np.inf)
    for gid in range(model.ngeom):
        if model.geom_bodyid[gid] == 0:  # worldbody: floor/walls/pedestal/logo, not the character
            continue
        pos = data.geom_xpos[gid]
        size = model.geom_size[gid].max() + 0.05
        mins = np.minimum(mins, pos - size)
        maxs = np.maximum(maxs, pos + size)
    if not np.all(np.isfinite(mins)):
        return model.stat.center, model.stat.extent
    center = (mins + maxs) / 2.0
    extent = float(np.linalg.norm(maxs - mins)) / 2.0
    return center, extent


def _character_forward_azimuth(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """World-frame heading (deg) of the character's own anterior (+X) axis.

    Different build strategies apply different root body orientations (e.g.
    build_torso_abdomen_spec's TORSO_ABDOMEN_ROOT_QUAT), so a fixed world
    azimuth points the camera at a different side of the character depending
    on which strategy built it. Body 1 -- the top of each spec's own
    kinematic tree -- always has +X as anterior, so deriving the camera
    azimuth from it keeps every render's framing consistent regardless of
    the model's absolute orientation in world space.
    """
    forward = data.xmat[1].reshape(3, 3)[:, 0]
    return float(np.degrees(np.arctan2(forward[1], forward[0])))


def render_model(model: mujoco.MjModel, out_path: Path, key_name: str | None = None) -> None:
    data = mujoco.MjData(model)
    if key_name is not None:
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key_name)
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    center, extent = _character_bounds(model, data)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = center
    cam.distance = 2.3 * extent
    cam.azimuth = _character_forward_azimuth(model, data) + 120
    cam.elevation = -15

    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False

    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        renderer.update_scene(data, camera=cam, scene_option=opt)
        pixels = renderer.render()

    from PIL import Image

    Image.fromarray(pixels).save(out_path)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name in sorted(MODEL_REGISTRY):
        try:
            spec = build_spec(name)
            spec.visual.global_.offwidth = WIDTH
            spec.visual.global_.offheight = HEIGHT
            model = spec.compile()
        except Exception as exc:  # noqa: BLE001
            print(f"SKIP {name}: {exc}")
            continue
        out_path = OUT_DIR / f"{name}.png"
        render_model(model, out_path)
        print(f"wrote {out_path}")

    for name, (xml_path, key_name) in sorted(STATIC_XML_TARGETS.items()):
        try:
            spec = mujoco.MjSpec.from_file(str(xml_path))
            spec.visual.global_.offwidth = WIDTH
            spec.visual.global_.offheight = HEIGHT
            spec.visual.headlight.ambient = [0.4, 0.4, 0.4]
            spec.visual.headlight.diffuse = [0.7, 0.7, 0.7]
            model = spec.compile()
        except Exception as exc:  # noqa: BLE001
            print(f"SKIP {name}: {exc}")
            continue
        out_path = OUT_DIR / f"{name}.png"
        render_model(model, out_path, key_name=key_name)
        print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
