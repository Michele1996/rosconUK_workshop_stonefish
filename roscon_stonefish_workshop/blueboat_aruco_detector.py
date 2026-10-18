#!/usr/bin/env python3
"""Optional live check: verify DICT_4X4_50 markers in the simulated RGB stream.

This is an optional workshop perception diagnostic, NOT part of race judging.
Requires cv_bridge and an OpenCV build with cv2.aruco.
"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('blueboat_aruco_detector')
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV ArUco unavailable: install python3-opencv with aruco or opencv-contrib-python')
        self.declare_parameter('image_topic', '/blueboat/camera/image_raw')
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Int32MultiArray, '/blueboat/aruco/ids', 10)
        self.create_subscription(Image, self.callback,
                                 self.get_parameter('image_topic').value, 10)
        self.last_report = -1e10
        self.get_logger().info('Watching DICT_4X4_50 IDs 11–22 and 31–42 on %s' %
                               self.get_parameter('image_topic').value)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            _corners, ids, _rejected = self.detector.detectMarkers(gray)
        else:
            _corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
        seen = sorted(int(x) for x in ids.flatten() if 11 <= int(x) <= 22 or 31 <= int(x) <= 42) if ids is not None else []
        result = Int32MultiArray()
        result.data = seen
        self.pub.publish(result)
        now = self.get_clock().now().nanoseconds * 1e-9
        if seen and now - self.last_report >= 1.0:
            self.get_logger().info('Visible ArUco checkpoint IDs: %s' % seen)
            self.last_report = now


def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
