"""Camera observations independent of any simulator or vehicle SDK."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class Observation:
    image: np.ndarray
    rotation_world_from_cv: np.ndarray
    intrinsics: np.ndarray
    timestamp_ns: int = 0

    def __post_init__(self):
        if not isinstance(self.image, np.ndarray) or self.image.dtype != np.uint8:
            raise ValueError("image must be a uint8 BGR numpy array")
        if self.image.ndim != 3 or self.image.shape[2] != 3 or min(self.image.shape[:2]) < 2:
            raise ValueError("image must have shape HxWx3")
        k = np.array(self.intrinsics, dtype=np.float64, copy=True)
        r = np.array(self.rotation_world_from_cv, dtype=np.float64, copy=True)
        if k.shape != (3, 3) or not np.isfinite(k).all():
            raise ValueError("intrinsics must be a finite 3x3 matrix")
        if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
            raise ValueError("intrinsics must be a calibrated pinhole matrix with positive focal lengths")
        if r.shape != (3, 3) or not np.isfinite(r).all():
            raise ValueError("rotation_world_from_cv must be a finite 3x3 matrix")
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1, atol=1e-5):
            raise ValueError("rotation_world_from_cv must be a proper rotation matrix")
        object.__setattr__(self, "intrinsics", k)
        object.__setattr__(self, "rotation_world_from_cv", r)


def undistort_bgr(image, intrinsics, distortion, new_intrinsics=None):
    """OpenCV pinhole distortion only; returns (rectified_image, rectified_K).

    Select the template box AFTER rectification. Fisheye cameras require the
    camera-specific rectification supplied by the caller.
    """
    k = np.asarray(intrinsics, dtype=np.float64)
    new_k = k.copy() if new_intrinsics is None else np.asarray(new_intrinsics, dtype=np.float64)
    d = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if d.size not in (4, 5, 8, 12, 14) or not np.isfinite(d).all():
        raise ValueError("Expected 4, 5, 8, 12 or 14 finite OpenCV pinhole distortion coefficients")
    return cv2.undistort(image, k, d, None, new_k), new_k.copy()
