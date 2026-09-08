from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .inputs import Observation


@dataclass(frozen=True)
class TargetTemplate:
    image: np.ndarray
    bbox_xyxy: tuple[float, float, float, float]


@dataclass(frozen=True)
class EstimateResult:
    target_camera: np.ndarray
    covariance_camera: np.ndarray
    range_std_m: float
    lateral_std_m: float
    bbox_in_b: tuple[int, int, int, int]
    common_matches: int
    inliers: int
    reprojection_error_px: float
    identity_confidence: float
    confidence: float
    estimate_method: str


class RelativeTargetEstimator:
    """Two-view estimator backed by the bundled SuperPoint/LightGlue."""

    def __init__(
        self,
        min_matches: int = 4,
        max_reprojection_error_px: float = 6.0,
        max_target_range_m: float = 2500.0,
        bbox_expand: float = 1.35,
        feature_engine: object | None = None,
    ) -> None:
        self.min_matches = int(min_matches)
        self.max_reprojection_error_px = float(max_reprojection_error_px)
        self.max_target_range_m = float(max_target_range_m)
        self.bbox_expand = float(bbox_expand)
        self.backend_name = "SuperPoint/LightGlue"
        self._feature_engine = feature_engine
        self._cached_template: TargetTemplate | None = None
        self._cached_matcher = None
        self.last_debug: dict[str, object] = {}

    def estimate(
        self,
        template: TargetTemplate,
        a: Observation,
        b: Observation,
        baseline_a_camera: np.ndarray,
    ) -> EstimateResult:
        self.last_debug = {}
        baseline = np.asarray(baseline_a_camera, dtype=np.float64)
        if baseline.shape != (3,) or not np.isfinite(baseline).all():
            raise ValueError("baseline_a_camera must be a finite vector of shape (3,) in metres")
        if np.linalg.norm(baseline) < 1e-6:
            raise ValueError("A nonzero measured metric camera baseline is required")
        if template.image.size == 0:
            raise ValueError("目标模板为空")
        matcher = self._get_template_matcher(template)
        self.last_debug = {
            "stage": "template_to_a",
            "matches_a": [],
            "matches_b": [],
            "common_ids": [],
        }
        try:
            match_set_a = matcher.match(a.image)
        except Exception as exc:
            self.last_debug.update(error=str(exc))
            raise RuntimeError(f"模板→A 匹配失败：{exc}") from exc
        matches_a = match_set_a.matches
        self.last_debug.update(
            stage="template_to_b",
            matches_a=matches_a,
            bbox_a=match_set_a.live_bbox,
        )
        try:
            match_set_b = matcher.match(b.image)
        except Exception as exc:
            self.last_debug.update(error=str(exc))
            raise RuntimeError(f"模板→B 匹配失败：{exc}") from exc
        matches_b = match_set_b.matches
        by_id_a = {match.template_index: match for match in matches_a}
        by_id_b = {match.template_index: match for match in matches_b}
        common_ids = sorted(by_id_a.keys() & by_id_b.keys())
        self.last_debug.update(
            stage="common_template_ids",
            matches_b=matches_b,
            common_ids=common_ids,
            bbox_a=match_set_a.live_bbox,
            bbox_b=match_set_b.live_bbox,
        )

        direct_matches: list[object] = []
        if len(common_ids) < self.min_matches:
            try:
                direct_matches = self._match_verified_rois(
                    a,
                    b,
                    match_set_a.live_bbox,
                    match_set_b.live_bbox,
                )
            except Exception as exc:
                self.last_debug.update(direct_error=str(exc))
        self.last_debug.update(direct_matches=direct_matches)

        if len(direct_matches) >= self.min_matches:
            pts_a = np.float64([match.template_point for match in direct_matches])
            pts_b = np.float64([match.live_point for match in direct_matches])
            estimate_method = "direct_ab"
        elif len(common_ids) >= self.min_matches:
            pts_t = np.float64([by_id_a[index].template_point for index in common_ids])
            pts_a = np.float64([by_id_a[index].live_point for index in common_ids])
            pts_b = np.float64([by_id_b[index].live_point for index in common_ids])
            selected = self._select_target_cluster(pts_t, template.bbox_xyxy)
            if len(selected) >= self.min_matches:
                pts_a, pts_b = pts_a[selected], pts_b[selected]
            estimate_method = "anchor_common"
        else:
            pts_a = np.asarray([self._bbox_center(match_set_a.live_bbox)], dtype=np.float64)
            pts_b = np.asarray([self._bbox_center(match_set_b.live_bbox)], dtype=np.float64)
            estimate_method = "bbox_center"

        if len(pts_a) >= 8:
            _, fundamental_mask = cv2.findFundamentalMat(
                pts_a,
                pts_b,
                cv2.FM_RANSAC,
                1.5,
                0.995,
            )
            if fundamental_mask is not None:
                mask = fundamental_mask.reshape(-1).astype(bool)
                pts_a, pts_b = pts_a[mask], pts_b[mask]
        required_points = 1 if estimate_method == "bbox_center" else self.min_matches
        if len(pts_a) < required_points:
            raise RuntimeError("两视图几何校验后的内点不足")

        points_camera, errors = self._triangulate(
            pts_a,
            pts_b,
            a,
            b,
            baseline_a_camera,
        )
        ranges = np.linalg.norm(points_camera, axis=1)
        valid = np.isfinite(points_camera).all(axis=1)
        valid &= np.isfinite(errors)
        valid &= errors <= self.max_reprojection_error_px
        valid &= ranges <= self.max_target_range_m
        points_camera, errors = points_camera[valid], errors[valid]
        if len(points_camera) < required_points:
            raise RuntimeError(
                f"三角化有效点不足：{len(points_camera)} < {required_points}；"
                "请增大横向基线或检查检测框"
            )

        target = np.median(points_camera, axis=0)
        median_error = float(np.median(errors))
        if estimate_method == "bbox_center":
            bbox = self._bbox_xyxy_to_xywh(match_set_b.live_bbox, b.image.shape[1], b.image.shape[0])
        else:
            bbox = self._expanded_bbox(pts_b[valid], b.image.shape[1], b.image.shape[0])
        covariance = self._estimate_covariance(
            pts_a[valid],
            pts_b[valid],
            a,
            b,
            baseline_a_camera,
            median_error,
            estimate_method,
            required_points,
        )
        unit_target = target / max(float(np.linalg.norm(target)), 1e-9)
        range_variance = float(unit_target @ covariance @ unit_target)
        range_std = float(np.sqrt(max(range_variance, 0.0)))
        lateral_variance = max(float(np.trace(covariance)) - range_variance, 0.0)
        lateral_std = float(np.sqrt(lateral_variance))
        identity_confidence = self._identity_confidence(match_set_a, match_set_b)
        count_score = min(1.0, len(points_camera) / 30.0)
        error_score = float(np.exp(-median_error / 2.0))
        relative_range_std = range_std / max(float(np.linalg.norm(target)), 1e-9)
        uncertainty_score = float(np.exp(-relative_range_std / 0.15))
        geometry_confidence = 0.4 * count_score + 0.3 * error_score + 0.3 * uncertainty_score
        if estimate_method == "bbox_center":
            geometry_confidence *= 0.45
        confidence = float(np.clip(min(identity_confidence, geometry_confidence), 0.0, 1.0))
        self.last_debug.update(
            stage="success",
            estimate_method=estimate_method,
            inliers=len(points_camera),
            reprojection_error_px=median_error,
            covariance_camera=covariance,
            range_std_m=range_std,
            lateral_std_m=lateral_std,
            identity_confidence=identity_confidence,
        )
        return EstimateResult(
            target_camera=target,
            covariance_camera=covariance,
            range_std_m=range_std,
            lateral_std_m=lateral_std,
            bbox_in_b=bbox,
            common_matches=len(pts_a),
            inliers=len(points_camera),
            reprojection_error_px=median_error,
            identity_confidence=identity_confidence,
            confidence=confidence,
            estimate_method=estimate_method,
        )

    def _match_verified_rois(
        self,
        a: Observation,
        b: Observation,
        bbox_a: tuple[float, float, float, float],
        bbox_b: tuple[float, float, float, float],
    ) -> list[object]:
        from .matching import LightGlueTemplateMatcher

        matcher = LightGlueTemplateMatcher(
            a.image,
            bbox_a,
            feature_engine=self._feature_engine,
        )
        matches = matcher.match(b.image).matches
        x1, y1, x2, y2 = self._expand_xyxy(
            bbox_b,
            b.image.shape[1],
            b.image.shape[0],
            1.8,
        )
        return [
            match
            for match in matches
            if x1 <= match.live_point[0] <= x2 and y1 <= match.live_point[1] <= y2
        ]

    def _estimate_covariance(
        self,
        pts_a: np.ndarray,
        pts_b: np.ndarray,
        a: Observation,
        b: Observation,
        baseline_a_camera: np.ndarray,
        reprojection_error_px: float,
        estimate_method: str,
        required_points: int,
    ) -> np.ndarray:
        rng = np.random.default_rng(20260831)
        pixel_sigma = float(np.clip(max(reprojection_error_px, 0.75), 0.75, 3.0))
        samples: list[np.ndarray] = []
        for _ in range(64):
            indices = rng.integers(0, len(pts_a), len(pts_a))
            sample_a = pts_a[indices] + rng.normal(0.0, pixel_sigma, (len(indices), 2))
            sample_b = pts_b[indices] + rng.normal(0.0, pixel_sigma, (len(indices), 2))
            baseline = np.asarray(baseline_a_camera, dtype=np.float64) * (
                1.0 + rng.normal(0.0, 0.02)
            )
            points, errors = self._triangulate(sample_a, sample_b, a, b, baseline)
            valid = np.isfinite(points).all(axis=1) & np.isfinite(errors)
            if int(np.count_nonzero(valid)) >= required_points:
                samples.append(np.median(points[valid], axis=0))
        distance = float(np.linalg.norm(np.median(self._triangulate(
            pts_a, pts_b, a, b, baseline_a_camera
        )[0], axis=0)))
        if len(samples) >= 4:
            covariance = np.cov(np.asarray(samples), rowvar=False)
        else:
            covariance = np.eye(3, dtype=np.float64) * (0.25 * max(distance, 1.0)) ** 2
        lateral_floor = (0.01 * max(distance, 1.0)) ** 2
        depth_floor = (0.02 * max(distance, 1.0)) ** 2
        if estimate_method == "bbox_center":
            lateral_floor = (0.05 * max(distance, 1.0)) ** 2
            depth_floor = (0.15 * max(distance, 1.0)) ** 2
        return np.asarray(covariance, dtype=np.float64) + np.diag(
            [lateral_floor, lateral_floor, depth_floor]
        )

    @staticmethod
    def _identity_confidence(match_set_a: object, match_set_b: object) -> float:
        count = min(len(match_set_a.matches), len(match_set_b.matches))
        count_score = min(1.0, count / 15.0)
        area_a = max(
            (match_set_a.live_bbox[2] - match_set_a.live_bbox[0])
            * (match_set_a.live_bbox[3] - match_set_a.live_bbox[1]),
            1.0,
        )
        area_b = max(
            (match_set_b.live_bbox[2] - match_set_b.live_bbox[0])
            * (match_set_b.live_bbox[3] - match_set_b.live_bbox[1]),
            1.0,
        )
        scale_score = min(area_a, area_b) / max(area_a, area_b)
        return float(np.clip(0.75 * count_score + 0.25 * scale_score, 0.0, 1.0))

    @staticmethod
    def _bbox_center(bbox: tuple[float, float, float, float]) -> tuple[float, float]:
        return ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)

    @staticmethod
    def _expand_xyxy(
        bbox: tuple[float, float, float, float],
        width: int,
        height: int,
        factor: float,
    ) -> tuple[float, float, float, float]:
        center_x, center_y = RelativeTargetEstimator._bbox_center(bbox)
        half_width = max((bbox[2] - bbox[0]) * factor / 2.0, 12.0)
        half_height = max((bbox[3] - bbox[1]) * factor / 2.0, 12.0)
        return (
            float(np.clip(center_x - half_width, 0, width - 1)),
            float(np.clip(center_y - half_height, 0, height - 1)),
            float(np.clip(center_x + half_width, 1, width)),
            float(np.clip(center_y + half_height, 1, height)),
        )

    def _bbox_xyxy_to_xywh(
        self,
        bbox: tuple[float, float, float, float],
        width: int,
        height: int,
    ) -> tuple[int, int, int, int]:
        x1, y1, x2, y2 = self._expand_xyxy(bbox, width, height, self.bbox_expand)
        return int(x1), int(y1), max(1, int(x2 - x1)), max(1, int(y2 - y1))

    def _get_template_matcher(self, template: TargetTemplate):
        if self._cached_template is template and self._cached_matcher is not None:
            return self._cached_matcher
        from .matching import (
            LightGlueTemplateMatcher,
        )

        self._cached_matcher = LightGlueTemplateMatcher(
            template.image,
            template.bbox_xyxy,
            feature_engine=self.feature_engine,
        )
        self._cached_template = template
        return self._cached_matcher

    @property
    def feature_engine(self):
        if self._feature_engine is None:
            from .matching import _LightGlueFeatureEngine

            self._feature_engine = _LightGlueFeatureEngine()
        return self._feature_engine

    def _select_target_cluster(
        self,
        points: np.ndarray,
        bbox: tuple[float, float, float, float],
    ) -> np.ndarray:
        if len(points) <= self.min_matches:
            return np.arange(len(points))
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        radius = max(18.0, 0.18 * min(width, height))
        unvisited = set(range(len(points)))
        components: list[list[int]] = []
        while unvisited:
            seed = unvisited.pop()
            component = [seed]
            queue = [seed]
            while queue:
                current = queue.pop()
                candidates = list(unvisited)
                if not candidates:
                    break
                distances = np.linalg.norm(points[candidates] - points[current], axis=1)
                neighbors = [candidates[i] for i in np.where(distances <= radius)[0]]
                for neighbor in neighbors:
                    unvisited.remove(neighbor)
                    component.append(neighbor)
                    queue.append(neighbor)
            components.append(component)
        center = np.array([(bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0])
        diagonal = max(float(np.hypot(width, height)), 1.0)

        def score(component: list[int]) -> float:
            cluster_center = np.mean(points[component], axis=0)
            center_prior = 1.0 - min(1.0, np.linalg.norm(cluster_center - center) / diagonal)
            return len(component) * (0.55 + 0.45 * center_prior)

        best = max(components, key=score)
        return np.asarray(sorted(best), dtype=np.int32)

    def _triangulate(
        self,
        pts_a: np.ndarray,
        pts_b: np.ndarray,
        a: Observation,
        b: Observation,
        baseline_a_camera: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        p_a, p_b, rotation_b_from_a = self._relative_projections(
            a,
            b,
            baseline_a_camera,
        )
        homogeneous = cv2.triangulatePoints(p_a, p_b, pts_a.T, pts_b.T)
        points_a = (homogeneous[:3] / homogeneous[3:4]).T
        baseline = np.asarray(baseline_a_camera, dtype=np.float64)
        points_b = (rotation_b_from_a @ (points_a - baseline).T).T
        depth_a = points_a[:, 2]
        depth_b = points_b[:, 2]
        errors_a = self._reprojection_errors(points_a, pts_a, p_a)
        errors_b = self._reprojection_errors(points_a, pts_b, p_b)
        errors = 0.5 * (errors_a + errors_b)
        errors[(depth_a <= 0.0) | (depth_b <= 0.0)] = np.inf
        return points_b, errors

    @staticmethod
    def _relative_projections(
        a: Observation,
        b: Observation,
        baseline_a_camera: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        baseline = np.asarray(baseline_a_camera, dtype=np.float64).reshape(3)
        rotation_a_from_b = a.rotation_world_from_cv.T @ b.rotation_world_from_cv
        rotation_b_from_a = rotation_a_from_b.T
        p_a = a.intrinsics @ np.hstack([np.eye(3), np.zeros((3, 1))])
        translation_b = -rotation_b_from_a @ baseline.reshape(3, 1)
        p_b = b.intrinsics @ np.hstack([rotation_b_from_a, translation_b])
        return p_a, p_b, rotation_b_from_a

    @staticmethod
    def _reprojection_errors(points: np.ndarray, measured: np.ndarray, projection: np.ndarray) -> np.ndarray:
        homogeneous = np.hstack([points, np.ones((len(points), 1), dtype=np.float64)])
        projected = (projection @ homogeneous.T).T
        projected = projected[:, :2] / projected[:, 2:3]
        return np.linalg.norm(projected - measured, axis=1)

    def _expanded_bbox(self, points: np.ndarray, width: int, height: int) -> tuple[int, int, int, int]:
        low = np.min(points, axis=0)
        high = np.max(points, axis=0)
        center = 0.5 * (low + high)
        size = np.maximum((high - low) * self.bbox_expand, np.array([24.0, 24.0]))
        x1 = int(np.clip(center[0] - size[0] / 2.0, 0, width - 1))
        y1 = int(np.clip(center[1] - size[1] / 2.0, 0, height - 1))
        x2 = int(np.clip(center[0] + size[0] / 2.0, x1 + 1, width))
        y2 = int(np.clip(center[1] + size[1] / 2.0, y1 + 1, height))
        return x1, y1, x2 - x1, y2 - y1

