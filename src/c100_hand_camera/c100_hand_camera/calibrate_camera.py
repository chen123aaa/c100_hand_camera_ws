#!/usr/bin/env python3
"""Terminal ROS2 camera calibration from the C100 image topic."""

from __future__ import annotations

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class C100CalibrationNode(Node):
    """Collect chessboard samples from an image topic and write camera intrinsics."""

    def __init__(self) -> None:
        super().__init__('c100_calibration_node')

        self.declare_parameter('image_topic', '/c100_hand_camera/image_raw')
        self.declare_parameter('debug_image_topic', '/c100_calibration/debug_image')
        self.declare_parameter('board_cols', 9)
        self.declare_parameter('board_rows', 6)
        self.declare_parameter('square_size_m', 0.025)
        self.declare_parameter('target_samples', 25)
        self.declare_parameter('output_path', 'calibration/c100_hand_camera_calibrated.yaml')
        self.declare_parameter('shutdown_on_complete', False)
        self.declare_parameter('log_every_n_frames', 30)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.board_cols = int(self.get_parameter('board_cols').value)
        self.board_rows = int(self.get_parameter('board_rows').value)
        self.square_size_m = float(self.get_parameter('square_size_m').value)
        self.target_samples = int(self.get_parameter('target_samples').value)
        self.output_path = Path(str(self.get_parameter('output_path').value))
        self.shutdown_on_complete = bool(self.get_parameter('shutdown_on_complete').value)
        self.log_every_n_frames = max(1, int(self.get_parameter('log_every_n_frames').value))

        if self.board_cols <= 0 or self.board_rows <= 0:
            raise ValueError('board_cols and board_rows must be positive')
        if self.square_size_m <= 0.0:
            raise ValueError('square_size_m must be positive')
        if self.target_samples < 5:
            raise ValueError('target_samples must be at least 5')

        self.pattern_size = (self.board_cols, self.board_rows)
        self.object_template = self._make_object_template()
        self.object_points: list[np.ndarray] = []
        self.image_points: list[np.ndarray] = []
        self.latest_corners: np.ndarray | None = None
        self.latest_stamp = None
        self.frame_count = 0
        self.image_size: tuple[int, int] | None = None
        self.completed = False

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.capture_service = self.create_service(
            Trigger,
            '/c100_calibration/capture_sample',
            self.capture_sample_callback,
        )
        self.calibrate_service = self.create_service(
            Trigger,
            '/c100_calibration/calibrate',
            self.calibrate_callback,
        )
        self.reset_service = self.create_service(
            Trigger,
            '/c100_calibration/reset',
            self.reset_callback,
        )

        self.get_logger().info(
            'C100 calibration started: '
            f'image_topic={self.image_topic}, board={self.board_cols}x{self.board_rows}, '
            f'square={self.square_size_m:.4f}m, target_samples={self.target_samples}'
        )
        self.get_logger().info(f'Publishing calibration debug image: {self.debug_image_topic}')
        self.get_logger().info(
            'Move the chessboard through different positions and angles. '
            'Call /c100_calibration/capture_sample to capture the current detected chessboard.'
        )
        self.get_logger().info(
            'Services: /c100_calibration/capture_sample, '
            '/c100_calibration/calibrate, /c100_calibration/reset'
        )

    def _make_object_template(self) -> np.ndarray:
        objp = np.zeros((self.board_cols * self.board_rows, 3), np.float32)
        objp[:, :2] = (
            np.mgrid[0:self.board_cols, 0:self.board_rows]
            .T.reshape(-1, 2)
            .astype(np.float32)
            * self.square_size_m
        )
        return objp

    def image_callback(self, msg: Image) -> None:
        if self.completed:
            return

        self.frame_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Failed to convert image: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.image_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(
            gray,
            self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if not found:
            self.latest_corners = None
            self.latest_stamp = None
            self._publish_debug_image(frame, msg, found=False, corners=None)
            if self.frame_count % self.log_every_n_frames == 0:
                self.get_logger().info(
                    f'Waiting for {self.board_cols}x{self.board_rows} chessboard corners... '
                    f'samples={len(self.image_points)}/{self.target_samples}'
                )
            return

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.1,
        )
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        self.latest_corners = corners.copy()
        self.latest_stamp = msg.header.stamp
        self._publish_debug_image(frame, msg, found=True, corners=corners)

        if self.frame_count % self.log_every_n_frames == 0:
            self.get_logger().info(
                f'Chessboard detected. Call capture_sample service to capture. '
                f'samples={len(self.image_points)}/{self.target_samples}'
            )

    def _publish_debug_image(self, frame: np.ndarray, source_msg: Image, found: bool, corners) -> None:
        debug = frame.copy()
        if found and corners is not None:
            cv2.drawChessboardCorners(debug, self.pattern_size, corners, found)

        status = 'FOUND' if found else 'SEARCHING'
        text = f'{status} samples={len(self.image_points)}/{self.target_samples}'
        color = (0, 255, 0) if found else (0, 165, 255)
        cv2.putText(
            debug,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA,
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = source_msg.header
        self.debug_image_pub.publish(debug_msg)

    def capture_sample_callback(self, _request, response):
        success, message = self.capture_current_sample()
        response.success = success
        response.message = message
        return response

    def calibrate_callback(self, _request, response):
        success, message = self.calibrate_and_save()
        response.success = success
        response.message = message
        return response

    def reset_callback(self, _request, response):
        self.object_points.clear()
        self.image_points.clear()
        self.completed = False
        response.success = True
        response.message = 'Calibration samples reset.'
        self.get_logger().info(response.message)
        return response

    def capture_current_sample(self) -> tuple[bool, str]:
        if self.completed:
            message = 'Calibration already completed. Call reset service to start over.'
            self.get_logger().info(message)
            return False, message

        corners = None if self.latest_corners is None else self.latest_corners.copy()

        if corners is None:
            message = 'No chessboard is currently detected; sample not captured.'
            self.get_logger().warning(message)
            return False, message

        self.object_points.append(self.object_template.copy())
        self.image_points.append(corners)

        message = f'Captured calibration sample {len(self.image_points)}/{self.target_samples}'
        self.get_logger().info(message)

        if len(self.image_points) >= self.target_samples:
            success, calibrate_message = self.calibrate_and_save()
            return success, f'{message}. {calibrate_message}'

        return True, message

    def calibrate_and_save(self) -> tuple[bool, str]:
        if self.completed:
            message = 'Calibration already completed.'
            self.get_logger().info(message)
            return True, message
        if len(self.image_points) < 5:
            message = f'Need at least 5 samples before calibration; current={len(self.image_points)}'
            self.get_logger().warning(message)
            return False, message
        if self.image_size is None:
            message = 'Cannot calibrate without image size'
            self.get_logger().error(message)
            return False, message

        reprojection_error, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            self.object_points,
            self.image_points,
            self.image_size,
            None,
            None,
        )

        dist = dist_coeffs.reshape(-1).astype(float).tolist()
        matrix = camera_matrix.reshape(-1).astype(float).tolist()
        self._write_ros_yaml(matrix, dist, reprojection_error)
        self.completed = True

        self.get_logger().info(f'Calibration complete. Reprojection error: {reprojection_error:.6f}')
        self.get_logger().info(f'camera_matrix: {matrix}')
        self.get_logger().info(f'distortion_coefficients: {dist}')
        message = f'Wrote calibration YAML: {self.output_path.resolve()}'
        self.get_logger().info(message)

        if self.shutdown_on_complete:
            self.get_logger().info('Calibration finished; shutting down node.')
            rclpy.shutdown()
        return True, message

    def _write_ros_yaml(self, camera_matrix: list[float], dist_coeffs: list[float], error: float) -> None:
        path = self.output_path
        if not path.is_absolute():
            path = Path.cwd() / path
        path.parent.mkdir(parents=True, exist_ok=True)

        width, height = self.image_size if self.image_size is not None else (0, 0)
        path.write_text(
            self._format_yaml(camera_matrix, dist_coeffs, width, height, error),
            encoding='utf-8',
        )
        self.output_path = path

    def _format_yaml(
        self,
        camera_matrix: list[float],
        dist_coeffs: list[float],
        width: int,
        height: int,
        error: float,
    ) -> str:
        matrix_text = self._format_float_list(camera_matrix, indent=6, per_line=3)
        dist_text = self._format_float_list(dist_coeffs, indent=6, per_line=5)
        return (
            '# Generated by c100_calibrate_camera.\n'
            f'# reprojection_error: {error:.9f}\n'
            'c100_hand_camera_node:\n'
            '  ros__parameters:\n'
            f'    width: {width}\n'
            f'    height: {height}\n'
            '    distortion_model: "plumb_bob"\n'
            '    camera_matrix:\n'
            f'{matrix_text}\n'
            '    distortion_coefficients:\n'
            f'{dist_text}\n'
        )

    @staticmethod
    def _format_float_list(values: list[float], indent: int, per_line: int) -> str:
        prefix = ' ' * indent
        lines = [prefix + '[']
        for index in range(0, len(values), per_line):
            chunk = values[index:index + per_line]
            suffix = ',' if index + per_line < len(values) else ''
            lines.append(prefix + ' ' + ', '.join(f'{value:.15g}' for value in chunk) + suffix)
        lines.append(prefix + ']')
        return '\n'.join(lines)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = C100CalibrationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
