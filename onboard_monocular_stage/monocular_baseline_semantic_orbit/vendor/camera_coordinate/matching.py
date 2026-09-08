from __future__ import annotations
from dataclasses import dataclass
from typing import Any
import cv2
import numpy as np


@dataclass(frozen=True)
class TemplateMatch:
    """一个模板关键点到实时帧关键点的匹配。"""

    template_index: int
    template_point: tuple[float, float]
    live_point: tuple[float, float]
    score: float


@dataclass(frozen=True)
class TemplateMatchSet:
    """模板与一张实时帧之间通过几何筛选的匹配。"""

    matches: list[TemplateMatch]
    live_bbox: tuple[float, float, float, float]


def filter_template_matches(
    template_keypoints: np.ndarray,
    live_keypoints: np.ndarray,
    match_indices: np.ndarray,
    scores: np.ndarray,
    template_bbox: tuple[float, float, float, float],
    *,
    ransac_threshold_px: float = 5.0,
) -> TemplateMatchSet:
    """筛选模板框内匹配，并用单应 RANSAC 去除明显误匹配。"""
    x1, y1, x2, y2 = map(float, template_bbox)
    if not (0.0 <= x1 < x2 and 0.0 <= y1 < y2):
        raise ValueError("template_bbox 必须是有效的 [x1, y1, x2, y2]")

    candidates: list[TemplateMatch] = []
    for pair_index, (template_index, live_index) in enumerate(match_indices):
        template_index = int(template_index)
        live_index = int(live_index)
        if not (
            0 <= template_index < len(template_keypoints)
            and 0 <= live_index < len(live_keypoints)
        ):
            continue
        tx, ty = map(float, template_keypoints[template_index])
        if not (x1 <= tx <= x2 and y1 <= ty <= y2):
            continue
        lx, ly = map(float, live_keypoints[live_index])
        score = float(scores[pair_index]) if pair_index < len(scores) else 0.0
        candidates.append(TemplateMatch(
            template_index,
            (tx, ty),
            (lx, ly),
            score,
        ))

    if len(candidates) < 4:
        raise ValueError(f"模板框内匹配不足: {len(candidates)}/4")

    template_points = np.asarray(
        [match.template_point for match in candidates], dtype=np.float32
    )
    live_points = np.asarray(
        [match.live_point for match in candidates], dtype=np.float32
    )
    _homography, inlier_mask = cv2.findHomography(
        template_points,
        live_points,
        cv2.RANSAC,
        ransac_threshold_px,
    )
    if inlier_mask is None:
        raise ValueError("模板匹配无法形成稳定几何区域")
    inliers = [
        match
        for match, keep in zip(candidates, inlier_mask.ravel())
        if bool(keep)
    ]
    if len(inliers) < 4:
        raise ValueError(f"模板几何内点不足: {len(inliers)}/4")

    live_inliers = np.asarray([match.live_point for match in inliers])
    live_bbox = (
        float(np.min(live_inliers[:, 0])),
        float(np.min(live_inliers[:, 1])),
        float(np.max(live_inliers[:, 0])),
        float(np.max(live_inliers[:, 1])),
    )
    return TemplateMatchSet(inliers, live_bbox)


class _LightGlueFeatureEngine:
    """把仓库内的 SuperPoint/LightGlue 封装为 NumPy 数组接口。"""

    def __init__(self, device: str | None = None) -> None:
        try:
            import torch
            from .lightglue import LightGlue, SuperPoint
            from .lightglue.utils import numpy_image_to_torch, rbd
        except ImportError as exc:
            raise RuntimeError(
                "SuperPoint/LightGlue 导入失败: "
                f"{exc}. 请检查 torch、torchvision、kornia 和运行目录"
            ) from exc

        self._torch = torch
        self._to_tensor = numpy_image_to_torch
        self._rbd = rbd
        self._device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self._extractor = SuperPoint(max_num_keypoints=2048).eval().to(self._device)
        self._matcher = LightGlue(features="superpoint").eval().to(self._device)

    def extract(self, image: np.ndarray) -> dict[str, Any]:
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        tensor = self._to_tensor(rgb).to(self._device)
        with self._torch.inference_mode():
            return self._extractor.extract(tensor)

    def match(
        self,
        template_features: dict[str, Any],
        live_features: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self._torch.inference_mode():
            result = self._matcher({
                "image0": template_features,
                "image1": live_features,
            })
        features0, features1, result = [
            self._rbd(value)
            for value in (template_features, live_features, result)
        ]
        return (
            features0["keypoints"].detach().cpu().numpy(),
            features1["keypoints"].detach().cpu().numpy(),
            result["matches"].detach().cpu().numpy(),
            result["scores"].detach().cpu().numpy(),
        )


class LightGlueTemplateMatcher:
    """缓存模板特征，并把同一模板关键点匹配到多张实时帧。"""

    def __init__(
        self,
        template_image: np.ndarray,
        template_bbox: tuple[float, float, float, float],
        device: str | None = None,
        *,
        feature_engine: Any | None = None,
    ) -> None:
        if not isinstance(template_image, np.ndarray) or template_image.ndim != 3:
            raise ValueError("template_image 必须是 HxWxC 图像")
        height, width = template_image.shape[:2]
        x1, y1, x2, y2 = map(float, template_bbox)
        if not (0.0 <= x1 < x2 <= width and 0.0 <= y1 < y2 <= height):
            raise ValueError("template_bbox 超出模板图像范围")
        self._bbox = (x1, y1, x2, y2)
        self._engine = feature_engine or _LightGlueFeatureEngine(device)
        self._template_features = self._engine.extract(template_image)

    def match(self, live_image: np.ndarray) -> TemplateMatchSet:
        if not isinstance(live_image, np.ndarray) or live_image.ndim != 3:
            raise ValueError("live_image 必须是 HxWxC 图像")
        live_features = self._engine.extract(live_image)
        arrays = self._engine.match(self._template_features, live_features)
        return filter_template_matches(*arrays, self._bbox)
