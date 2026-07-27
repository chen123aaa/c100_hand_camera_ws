#!/usr/bin/env python3
"""ROS2 AprilTag 36h11 detector for the C100 hand camera."""

from __future__ import annotations

import json
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .apriltag_detector import (
    TagSizeResolver,
    create_detector,
    detect_apriltags,
    estimate_tag_pose,
    rotation_vector_to_quaternion,
)


class C100AprilTagDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('c100_apriltag_detector')
        self.declare_parameter('image_topic', '/c100_hand_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/c100_hand_camera/camera_info')
        self.declare_parameter('debug_image_topic', '/c100_apriltag/debug_image')
        self.declare_parameter('detection_topic', '/c100_apriltag/detection')
        self.declare_parameter('pose_topic', '/c100_apriltag/pose')
        self.declare_parameter('family', '36h11')
        self.declare_parameter('target_id', 0)
        self.declare_parameter('tag_size', 0.03)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_child_frame', 'apriltag_0')
        self.declare_parameter('log_interval', 1.0)

        family = str(self.get_parameter('family').value).lower()
        if family != '36h11':
            raise ValueError('Only AprilTag family 36h11 is supported')
        self.target_id = int(self.get_parameter('target_id').value)
        self.tag_size = TagSizeResolver(float(self.get_parameter('tag_size').value), {})
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.tf_child_frame = str(self.get_parameter('tf_child_frame').value)
        self.log_interval = max(float(self.get_parameter('log_interval').value), 0.1)

        self.bridge = CvBridge()
        self.detector = create_detector()
        self.camera_matrix: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.camera_frame = ''
        self.last_log_time = 0.0
        self.was_detected = False

        self.debug_pub = self.create_publisher(
            Image, str(self.get_parameter('debug_image_topic').value), 10
        )
        self.detection_pub = self.create_publisher(
            String, str(self.get_parameter('detection_topic').value), 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('pose_topic').value), 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._image_callback,
            10,
        )
        self.get_logger().info(
            f'检测 AprilTag 36h11 ID={self.target_id}, tag_size={self.tag_size.size_for(self.target_id):.3f} m'
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id

    def _image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug = frame.copy()
        detections = detect_apriltags(frame, self.target_id, self.detector)

        if self.camera_matrix is None or self.distortion is None:
            cv2.putText(debug, 'Waiting for camera_info', (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        elif detections:
            for detection in detections:
                self._publish_detection(detection, msg, debug)
            self.was_detected = True
        else:
            cv2.putText(debug, f'AprilTag 36h11 ID={self.target_id}: not detected', (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            if self.was_detected:
                self.get_logger().info(f'AprilTag ID={self.target_id} lost')
            self.was_detected = False

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def _publish_detection(self, detection, image_msg: Image, debug: np.ndarray) -> None:
        size = self.tag_size.size_for(detection.tag_id)
        pose = estimate_tag_pose(detection.corners, size, self.camera_matrix, self.distortion)
        quaternion = rotation_vector_to_quaternion(pose.rotation_vector)
        frame_id = image_msg.header.frame_id or self.camera_frame

        pose_msg = PoseStamped()
        pose_msg.header = image_msg.header
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(pose.translation_m[0])
        pose_msg.pose.position.y = float(pose.translation_m[1])
        pose_msg.pose.position.z = float(pose.translation_m[2])
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quaternion
        self.pose_pub.publish(pose_msg)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = pose_msg.header
            transform.child_frame_id = self.tf_child_frame
            transform.transform.translation.x = pose_msg.pose.position.x
            transform.transform.translation.y = pose_msg.pose.position.y
            transform.transform.translation.z = pose_msg.pose.position.z
            transform.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

        payload = {
            'family': '36h11',
            'id': detection.tag_id,
            'tag_size_m': size,
            'center_px': [round(float(value), 2) for value in detection.center],
            'position_m': [round(float(value), 5) for value in pose.translation_m],
            'frame_id': frame_id,
        }
        result = String()
        result.data = json.dumps(payload, ensure_ascii=False)
        self.detection_pub.publish(result)

        corners = detection.corners.astype(np.int32)
        cv2.polylines(debug, [corners], True, (0, 255, 0), 2, cv2.LINE_AA)
        center = tuple(np.round(detection.center).astype(int))
        cv2.circle(debug, center, 4, (0, 0, 255), -1)
        cv2.drawFrameAxes(debug, self.camera_matrix, self.distortion, pose.rotation_vector, pose.translation_m, size * 0.5, 2)
        distance = float(np.linalg.norm(pose.translation_m))
        text = f'36h11 ID={detection.tag_id} size={size * 100:.1f}cm distance={distance:.3f}m'
        text_origin = (int(corners[:, 0].min()), max(24, int(corners[:, 1].min()) - 10))
        cv2.putText(debug, text, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        now = time.monotonic()
        if now - self.last_log_time >= self.log_interval:
            x, y, z = pose.translation_m
            self.get_logger().info(
                f'检测到 ID={detection.tag_id}: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, distance={distance:.3f} m'
            )
            self.last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = C100AprilTagDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
