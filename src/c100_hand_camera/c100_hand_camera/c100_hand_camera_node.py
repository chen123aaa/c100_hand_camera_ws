#!/usr/bin/env python3
"""ROS2 image publisher for the WHEELTEC C100 hand camera."""

from __future__ import annotations

from typing import Union

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class C100HandCameraNode(Node):
    """Publish C100 USB camera frames and default camera info."""

    def __init__(self) -> None:
        super().__init__('c100_hand_camera_node')

        self.declare_parameter('device', '')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('use_mjpeg', True)
        self.declare_parameter('frame_id', 'c100_hand_camera_link')
        self.declare_parameter('image_topic', '/c100_hand_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/c100_hand_camera/camera_info')
        self.declare_parameter(
            'camera_matrix',
            [
                302.7048929342819, 0.0, 298.6791162335703,
                0.0, 302.2019683382837, 234.9714757570849,
                0.0, 0.0, 1.0,
            ],
        )
        self.declare_parameter('distortion_model', 'plumb_bob')
        self.declare_parameter(
            'distortion_coefficients',
            [
                0.0615721521591663,
                0.09610650401456461,
                0.0004141072132062723,
                0.0008935300620624921,
                0.02529521259640314,
            ],
        )

        self.device = str(self.get_parameter('device').value).strip()
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.use_mjpeg = bool(self.get_parameter('use_mjpeg').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self.camera_matrix = [float(v) for v in self.get_parameter('camera_matrix').value]
        self.distortion_model = str(self.get_parameter('distortion_model').value)
        self.distortion_coefficients = [
            float(v) for v in self.get_parameter('distortion_coefficients').value
        ]

        self._validate_camera_params()
        if not self.device:
            raise RuntimeError(
                'Missing required device parameter. Detect devices first, then launch with '
                'device:=/dev/videoX, for example device:=/dev/video2.'
            )

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)

        self.cap = self._open_capture(self.device)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open C100 camera device {self.device!r}. "
                "Run ./scripts/list_video_devices.sh after plugging in the camera."
            )

        self.get_logger().info(
            'Opened C100 hand camera: '
            f'device={self.device!r}, requested={self.width}x{self.height}@{self.fps:.1f}, '
            f'actual={self._capture_description()}'
        )
        self.get_logger().info(
            f'Publishing image={image_topic}, camera_info={camera_info_topic}, frame_id={self.frame_id}'
        )

        timer_period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(timer_period, self._publish_frame)

    def _validate_camera_params(self) -> None:
        if len(self.camera_matrix) != 9:
            raise ValueError('camera_matrix must contain 9 values')
        if len(self.distortion_coefficients) not in (4, 5, 8, 12, 14):
            raise ValueError('distortion_coefficients must contain a valid ROS CameraInfo D array')

    def _open_capture(self, device: Union[str, int]) -> cv2.VideoCapture:
        source: Union[str, int] = int(device) if str(device).isdigit() else str(device)
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)

        if self.use_mjpeg:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        return cap

    def _capture_description(self) -> str:
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc_value = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = ''.join(chr((fourcc_value >> (8 * i)) & 0xFF) for i in range(4))
        return f'{width}x{height}@{fps:.1f}, fourcc={fourcc!r}'

    def _camera_info_msg(self, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = self.distortion_model
        msg.d = self.distortion_coefficients
        msg.k = self.camera_matrix

        fx = self.camera_matrix[0]
        fy = self.camera_matrix[4]
        cx = self.camera_matrix[2]
        cy = self.camera_matrix[5]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def _publish_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning('Failed to read frame from C100 camera')
            return

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(self._camera_info_msg(stamp))

    def destroy_node(self) -> None:
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = C100HandCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
