#!/usr/bin/env python3
"""
BlueBoat ArUco Marker Detector for Stonefish + ROS 2 Humble.

Detects DICT_4X4_50 markers:
    - IDs 11–22
    - IDs 31–42

Subscribes:
    /blueboat/camera/image_raw/image_color

Publishes:
    /blueboat/aruco/ids (std_msgs/Int32MultiArray)

This is an optional workshop perception diagnostic.
It is NOT part of race judging.

Requirements:
    - rclpy
    - sensor_msgs
    - std_msgs
    - cv_bridge
    - OpenCV with cv2.aruco
"""

import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge, CvBridgeError


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('blueboat_aruco_detector')

        # --------------------------------------------------
        # ROS 2 parameters
        # --------------------------------------------------

        self.declare_parameter(
            'image_topic',
            '/blueboat/camera/image_raw/image_color'
        )

        self.image_topic = (
            self.get_parameter('image_topic')
            .get_parameter_value()
            .string_value
        )

        # --------------------------------------------------
        # Check OpenCV ArUco availability
        # --------------------------------------------------

        if not hasattr(cv2, 'aruco'):
            raise RuntimeError(
                'OpenCV ArUco module unavailable. '
                'Install an OpenCV build with ArUco support.'
            )

        # --------------------------------------------------
        # ArUco dictionary
        # --------------------------------------------------

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        # Support newer and older OpenCV versions.
        if hasattr(cv2.aruco, 'ArucoDetector'):

            # OpenCV >= 4.7
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary
            )

        else:

            # Older OpenCV
            self.detector = None

        # --------------------------------------------------
        # Valid marker IDs
        # --------------------------------------------------

        self.valid_ids = set(range(11, 23)) | set(range(31, 43))

        # --------------------------------------------------
        # ROS 2 bridge
        # --------------------------------------------------

        self.bridge = CvBridge()

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------

        self.pub = self.create_publisher(
            Int32MultiArray,
            '/blueboat/aruco/ids',
            10
        )

        # --------------------------------------------------
        # Camera subscriber
        # --------------------------------------------------

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.callback,
            qos_profile_sensor_data
        )

        # --------------------------------------------------
        # Logging
        # --------------------------------------------------

        self.last_report = None

        self.get_logger().info(
            'BlueBoat ArUco detector initialized.'
        )

        self.get_logger().info(
            f'Camera topic: {self.image_topic}'
        )

        self.get_logger().info(
            'Dictionary: DICT_4X4_50'
        )

        self.get_logger().info(
            'Watching checkpoint IDs: 11–22 and 31–42'
        )

    # ======================================================
    # Image callback
    # ======================================================

    def callback(self, msg):

        # --------------------------------------------------
        # ROS Image -> OpenCV
        # --------------------------------------------------

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except (CvBridgeError, Exception) as exc:

            self.get_logger().error(
                f'Image conversion failed: {exc}'
            )

            return

        # --------------------------------------------------
        # Convert to grayscale
        # --------------------------------------------------

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        # --------------------------------------------------
        # Detect markers
        # --------------------------------------------------

        if self.detector is not None:

            corners, ids, rejected = (
                self.detector.detectMarkers(gray)
            )

        else:

            corners, ids, rejected = (
                cv2.aruco.detectMarkers(
                    gray,
                    self.dictionary
                )
            )

        # --------------------------------------------------
        # Filter valid marker IDs
        # --------------------------------------------------

        seen = []

        if ids is not None:

            seen = sorted(
                set(
                    int(marker_id)
                    for marker_id in ids.flatten()
                    if int(marker_id) in self.valid_ids
                )
            )

        # --------------------------------------------------
        # Publish visible marker IDs
        # --------------------------------------------------

        result = Int32MultiArray()

        result.data = seen

        self.pub.publish(result)

        # --------------------------------------------------
        # Log detected markers (maximum 1 Hz)
        # --------------------------------------------------

        now = self.get_clock().now().nanoseconds * 1e-9

        if seen:

            if (
                self.last_report is None
                or now - self.last_report >= 1.0
                or now < self.last_report
            ):

                self.get_logger().info(
                    f'Visible ArUco checkpoint IDs: {seen}'
                )

                self.last_report = now


# ==========================================================
# Main
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = ArucoDetector()

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
