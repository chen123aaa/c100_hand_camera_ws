import cv2
import numpy as np
import pytest

from c100_hand_camera.apriltag_detector import (
    TagSizeResolver,
    detect_apriltags,
    estimate_tag_pose,
)


def make_tag_scene(tag_id=0, marker_pixels=240):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_pixels)
    scene = np.full((480, 640), 255, dtype=np.uint8)
    y0 = (scene.shape[0] - marker_pixels) // 2
    x0 = (scene.shape[1] - marker_pixels) // 2
    scene[y0:y0 + marker_pixels, x0:x0 + marker_pixels] = marker
    return cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR)


def test_detects_only_requested_apriltag_id():
    image = make_tag_scene(tag_id=0)

    detections = detect_apriltags(image, target_id=0)
    rejected = detect_apriltags(image, target_id=1)

    assert len(detections) == 1
    assert detections[0].tag_id == 0
    assert detections[0].corners.shape == (4, 2)
    assert rejected == []


def test_size_resolver_requires_explicit_choice_for_duplicate_id():
    resolver = TagSizeResolver(default_size_m=0.0, size_by_id={})

    with pytest.raises(ValueError, match='tag_size'):
        resolver.size_for(0)

    assert TagSizeResolver(default_size_m=0.03, size_by_id={}).size_for(0) == pytest.approx(0.03)
    assert TagSizeResolver(default_size_m=0.03, size_by_id={0: 0.05}).size_for(0) == pytest.approx(0.05)


def test_pose_distance_scales_with_selected_physical_tag_size():
    detection = detect_apriltags(make_tag_scene(), target_id=0)[0]
    camera_matrix = np.array(
        [[302.7048929342819, 0.0, 298.6791162335703],
         [0.0, 302.2019683382837, 234.9714757570849],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros(5, dtype=np.float64)

    pose_3cm = estimate_tag_pose(detection.corners, 0.03, camera_matrix, distortion)
    pose_5cm = estimate_tag_pose(detection.corners, 0.05, camera_matrix, distortion)

    assert pose_3cm.translation_m[2] > 0.0
    assert pose_5cm.translation_m[2] / pose_3cm.translation_m[2] == pytest.approx(5.0 / 3.0, rel=0.03)
