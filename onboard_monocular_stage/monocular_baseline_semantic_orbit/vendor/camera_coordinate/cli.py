"""Offline two-view inference. All paths in the JSON are relative to that JSON."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import cv2
import numpy as np
from . import Observation, RelativeTargetEstimator, TargetTemplate
from .matching import _LightGlueFeatureEngine


def read_image(path):
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Cannot decode image: {path}")
    return image


def run(config_path, device=None):
    path = Path(config_path).resolve()
    config = json.loads(path.read_text(encoding="utf-8"))
    def observation(key):
        data = config[key]
        return Observation(
            image=read_image(path.parent / data["image"]),
            intrinsics=data["intrinsics"],
            rotation_world_from_cv=data["rotation_world_from_cv"],
            timestamp_ns=data.get("timestamp_ns", 0),
        )
    a, b = observation("a"), observation("b")
    t = config["template"]
    template = TargetTemplate(read_image(path.parent / t["image"]), tuple(t["bbox_xyxy"]))
    estimator = RelativeTargetEstimator(feature_engine=_LightGlueFeatureEngine(device))
    result = estimator.estimate(template, a, b, np.asarray(config["baseline_a_camera_m"]))
    output = asdict(result)
    output.update(coordinate_frame="camera_b_optical", axes=["right", "down", "forward"],
                  coordinate_unit="m", covariance_unit="m^2", timestamp_ns=b.timestamp_ns)
    output["warnings"] = (["bbox_center fallback: two detected box centres are not guaranteed to be the same 3D point"]
                          if result.estimate_method == "bbox_center" else [])
    return output


def main():
    parser = argparse.ArgumentParser(description="Estimate target XYZ in camera B; no vehicle commands")
    parser.add_argument("input", type=Path, help="Input JSON")
    parser.add_argument("--output", type=Path, help="Output JSON; otherwise stdout")
    parser.add_argument("--device", default=None, help="cpu, cuda, cuda:0; default auto")
    args = parser.parse_args()
    try:
        result = run(args.input, args.device)
        text = json.dumps(result, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x.item(),
                          ensure_ascii=False, indent=2, allow_nan=False)
        if args.output:
            args.output.write_text(text + "\n", encoding="utf-8")
        else:
            print(text)
    except Exception as exc:
        print(f"Estimation failed: {exc}", file=sys.stderr)
        return 1
    return 0
