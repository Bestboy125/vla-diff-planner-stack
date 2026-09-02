#!/usr/bin/env python3
"""Isaac Sim 6 headless RGB-D and camera-pose validation."""

import argparse
import json
from pathlib import Path

from isaacsim import SimulationApp


parser = argparse.ArgumentParser()
parser.add_argument("--output-dir", required=True)
parser.add_argument("--warmup-frames", type=int, default=120)
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
args = parser.parse_args()

simulation_app = SimulationApp(
    {
        "headless": True,
        "width": args.width,
        "height": args.height,
        "renderer": "RayTracedLighting",
    }
)

import numpy as np
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.camera import Camera


def main() -> None:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.scene.add(
        VisualCuboid(
            prim_path="/World/RedCube",
            name="red_cube",
            position=np.array([0.0, 0.0, 1.0]),
            scale=np.array([1.2, 1.2, 1.2]),
            color=np.array([1.0, 0.05, 0.05]),
        )
    )
    world.scene.add(
        VisualCuboid(
            prim_path="/World/GreenCube",
            name="green_cube",
            position=np.array([2.0, 1.0, 0.6]),
            scale=np.array([0.8, 0.8, 0.8]),
            color=np.array([0.05, 1.0, 0.05]),
        )
    )

    camera = Camera(
        prim_path="/World/UavCamera",
        position=np.array([0.0, 0.0, 8.0]),
        frequency=20,
        resolution=(args.width, args.height),
    )

    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    for _ in range(args.warmup_frames):
        world.step(render=True)

    rgba = np.asarray(camera.get_rgba())
    depth = np.asarray(camera.get_current_frame().get("distance_to_image_plane"))
    position, orientation = camera.get_world_pose()

    expected_rgba = (args.height, args.width, 4)
    expected_depth = (args.height, args.width)
    if rgba.shape != expected_rgba:
        raise RuntimeError(f"unexpected RGBA shape: {rgba.shape}; expected {expected_rgba}")
    if depth.shape != expected_depth:
        raise RuntimeError(f"unexpected depth shape: {depth.shape}; expected {expected_depth}")

    rgb = rgba[:, :, :3]
    if rgb.dtype != np.uint8:
        scale = 255.0 if float(np.max(rgb)) <= 1.0 else 1.0
        rgb = np.clip(rgb * scale, 0, 255).astype(np.uint8)

    valid = np.isfinite(depth) & (depth > 0)
    values = depth[valid]
    if float(rgb.std()) < 1.0:
        raise RuntimeError("rendered RGB image is effectively uniform")
    if values.size == 0:
        raise RuntimeError("depth image has no finite positive samples")

    Image.fromarray(rgb, mode="RGB").save(output_dir / "rgb.png")
    np.save(output_dir / "depth.npy", depth.astype(np.float32))

    near = float(np.percentile(values, 2))
    far = float(np.percentile(values, 98))
    span = max(far - near, 1e-6)
    preview = np.zeros(depth.shape, dtype=np.uint8)
    preview[valid] = np.clip((far - depth[valid]) / span * 255.0, 0, 255).astype(np.uint8)
    Image.fromarray(preview, mode="L").save(output_dir / "depth_preview.png")

    metadata = {
        "status": "pass",
        "isaac_sim": "6.0.1",
        "coordinate_convention": "Isaac Sim world frame; metres; quaternion wxyz",
        "resolution": {"width": args.width, "height": args.height},
        "rgb_std": float(rgb.std()),
        "depth": {
            "valid_pixels": int(values.size),
            "min_m": float(values.min()),
            "max_m": float(values.max()),
        },
        "camera_pose": {
            "position_m": np.asarray(position).tolist(),
            "orientation_wxyz": np.asarray(orientation).tolist(),
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(metadata, indent=2, ensure_ascii=False))


try:
    main()
finally:
    simulation_app.close()
