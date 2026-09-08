"""Local weights only; never downloads or changes the global Torch cache."""
import os
from pathlib import Path
import torch


def load_weights(filename):
    directory = Path(os.environ.get("CAMERA_COORDINATE_WEIGHTS", str(Path(__file__).resolve().parents[1] / "weights")))
    path = directory / filename
    if not path.is_file():
        raise FileNotFoundError(f"Missing local model weights: {path}")
    return torch.load(path, map_location="cpu", weights_only=True)
