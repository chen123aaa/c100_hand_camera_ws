#!/usr/bin/env python3
"""AprilTag 36h11 detection and square-tag pose estimation helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import cv2
import numpy as np


@dataclass(frozen=True)
class TagDetection:
    tag_id: int
    corners: np.ndarray
    center: np.ndarray


@dataclass(frozen=True)
class TagPose:
    rotation_vector: np.ndarray
    translation_m: np.ndarray


class TagSizeResolver:
    """Resolve physical black-square edge length in metres."""

    def __init__(self, default_size_m: float, size_by_id: Mapping[int, float]) -> None:
        self.default_size_m = float(default_size_m)
        self.size_by_id = {int(key): float(value) for key, value in size_by_id.items()}

    def size_for(self, tag_id: int) -> float:
        size = self.size_by_id.get(int(tag_id), self.default_size_m)
        if size <= 0.0:
            raise ValueError('tag_size must be greater than zero')
        return size


def create_detector() -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def detect_apriltags(
    image_bgr: np.ndarray,
    target_id: int = -1,
    detector: cv2.aruco.ArucoDetector | None = None,
) -> list[TagDetection]:
    if image_bgr is None or image_bgr.size == 0:
        return []
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY) if image_bgr.ndim == 3 else image_bgr
    corners_list, ids, _ = (detector or create_detector()).detectMarkers(gray)
    if ids is None:
        return []

    detections = []
    for corners, raw_id in zip(corners_list, ids.reshape(-1)):
        tag_id = int(raw_id)
        if target_id >= 0 and tag_id != target_id:
            continue
        points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        detections.append(TagDetection(tag_id, points, points.mean(axis=0)))
    return detections


def estimate_tag_pose(
    corners: np.ndarray,
    tag_size_m: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> TagPose:
    size = float(tag_size_m)
    if size <= 0.0:
        raise ValueError('tag_size_m must be greater than zero')
    half = size / 2.0
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    success, rotation, translation = cv2.solvePnP(
        object_points,
        np.asarray(corners, dtype=np.float64).reshape(4, 2),
        np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
        np.asarray(distortion, dtype=np.float64).reshape(-1),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        raise RuntimeError('AprilTag pose estimation failed')
    return TagPose(rotation.reshape(3), translation.reshape(3))


def rotation_vector_to_quaternion(rotation_vector: np.ndarray) -> tuple[float, float, float, float]:
    matrix, _ = cv2.Rodrigues(np.asarray(rotation_vector, dtype=np.float64).reshape(3))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        return (
            float((matrix[2, 1] - matrix[1, 2]) / scale),
            float((matrix[0, 2] - matrix[2, 0]) / scale),
            float((matrix[1, 0] - matrix[0, 1]) / scale),
            float(scale / 4.0),
        )

    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        return (
            float(scale / 4.0),
            float((matrix[0, 1] + matrix[1, 0]) / scale),
            float((matrix[0, 2] + matrix[2, 0]) / scale),
            float((matrix[2, 1] - matrix[1, 2]) / scale),
        )
    if index == 1:
        scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        return (
            float((matrix[0, 1] + matrix[1, 0]) / scale),
            float(scale / 4.0),
            float((matrix[1, 2] + matrix[2, 1]) / scale),
            float((matrix[0, 2] - matrix[2, 0]) / scale),
        )
    scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
    return (
        float((matrix[0, 2] + matrix[2, 0]) / scale),
        float((matrix[1, 2] + matrix[2, 1]) / scale),
        float(scale / 4.0),
        float((matrix[1, 0] - matrix[0, 1]) / scale),
    )
