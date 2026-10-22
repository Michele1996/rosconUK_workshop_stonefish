#!/usr/bin/env python3
"""PARTICIPANT EXERCISE: partially completed image-based BlueBoat steering.

Allowed input: *live* RGB camera and (optionally) robot navigation sensors.
No course JSON, waypoint CSV, precomputed trajectory, or judge internals.

Implemented:
  * ROS2 camera subscriber / cv_bridge, image staleness failsafe.
  * Basic yellow-buoy HSV segmentation (requires tuning in Stonefish).
  * Optional ArUco IDs for displaying perceived checkpoint labels.
  * Differential steering command from a target image pixel, and Twist output.

Students complete the TODOs for robust gate association and mission autonomy.
The example starts DISARMED (enable_motion:=false): no surprise propulsion.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .control_utils import clamp
from .fair_play import FairPlayViolation, guard_participant_route_parameters, reject_privileged_topic


# Pure computer-vision helpers are separated from ROS callbacks for unit tests.
def yellow_buoy_candidates(bgr, min_area_px=90):
    """Return (pixel_x, pixel_y, bounding_box_area) sorted largest first.

    A starting point only; yellow signs, reflections and light variation can
    also match this colour threshold. All coordinates originate in the image.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # TODO(vision 1): calibrate this yellow HSV range using real Stonefish frames.
    mask = cv2.inRange(hsv, (15, 65, 60), (40, 255, 255))
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if h < 5 or w < 5:
            continue
        detections.append((x+w/2.0, y+h/2.0, float(w*h)))
    return sorted(detections, key=lambda d: d[2], reverse=True)


def initial_gate_target(candidates, width):
    """Deliberately incomplete: midpoint of the two largest yellow blobs.

    This does NOT know which blobs belong to the same gate or their orientation.
    Returning None holds the vessel stopped rather than inventing a target.
    """
    if len(candidates) < 2:
        return None
    # TODO(navigation 2): match LEFT/RIGHT buoys belonging to the next gate.
    # Current heuristic may pick two buoys along the same boundary, or signs.
    x_a, _, _ = candidates[0]
    x_b, _, _ = candidates[1]
    if abs(x_a-x_b) < 0.05*width:
        return None
    return 0.5*(x_a+x_b)


def pixel_steering_command(target_x, width, kp, max_yaw):
    """Basic visual servo: desired image x = width/2; positive = turn right."""
    if target_x is None or width <= 0:
        return 0.0
    error = (float(target_x) - 0.5*width) / (0.5*width)
    return clamp(float(kp)*error, -abs(float(max_yaw)), abs(float(max_yaw)))


class BlueBoatCameraSteering(Node):
    def __init__(self):
        super().__init__('blueboat_camera_steering')
        guard_participant_route_parameters(self)
        self.declare_parameter('image_topic', '/blueboat/camera/image_raw/image_color')
        self.declare_parameter('cmd_topic', '/blueboat/cmd_vel')
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('surge_speed', 0.20)
        self.declare_parameter('kp_image_yaw', 0.5)
        self.declare_parameter('max_yaw', 0.35)
        self.declare_parameter('min_blob_area_px', 90)
        self.declare_parameter('camera_timeout_s', 0.65)
        self.declare_parameter('show_marker_ids', True)
        reject_privileged_topic(self.get_parameter('image_topic').value, parameter='image_topic')
        self.bridge = CvBridge()
        self.last_frame_monotonic = None
        self.target_x = None
        self.image_width = 0
        self.last_ids = ()
        self.last_report = 0.0
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.on_image, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.control_tick)
        self.dictionary = None
        self.detector = None
        if hasattr(cv2, 'aruco'):
            self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            if hasattr(cv2.aruco, 'ArucoDetector'):
                self.detector = cv2.aruco.ArucoDetector(self.dictionary)
        self.get_logger().info(
            'Camera steering EXERCISE ready (enable_motion=false). '
            'FAIR PLAY: no course JSON, CSV, route files or judge feedback.')

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if bgr is None or bgr.size == 0:
                raise ValueError('empty image')
            # Only observed image pixels feed the candidate gate target.
            blobs = yellow_buoy_candidates(
                bgr, int(self.get_parameter('min_blob_area_px').value))
            self.image_width = bgr.shape[1]
            self.target_x = initial_gate_target(blobs, self.image_width)
            self.last_frame_monotonic = time.monotonic()
            self.last_ids = ()
            if (self.dictionary is not None and
                    self.get_parameter('show_marker_ids').value):
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                if self.detector is not None:
                    _, ids, _ = self.detector.detectMarkers(gray)
                else:
                    _, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
                if ids is not None:
                    self.last_ids = tuple(sorted(
                        set(int(v) for v in ids.flatten()
                            if 11 <= int(v) <= 22 or 31 <= int(v) <= 42)))
            # TODO(perception 3): track detections across frames, decode arrow
            # directions, and reject signs/reflections misclassified as buoys.
            now = time.monotonic()
            if now-self.last_report >= 2.0:
                self.get_logger().info(
                    f'Yellow candidates={len(blobs)}, target_px={self.target_x}, '
                    f'ArUco={self.last_ids}')
                self.last_report = now
        except Exception as exc:
            self.target_x = None
            self.last_frame_monotonic = None
            self.get_logger().warning(f'Camera frame rejected; stopping: {exc}')

    def control_tick(self):
        out = Twist()  # Zero output is the default failsafe.
        age = (float('inf') if self.last_frame_monotonic is None else
               time.monotonic()-self.last_frame_monotonic)
        if (not bool(self.get_parameter('enable_motion').value) or
                age > float(self.get_parameter('camera_timeout_s').value) or
                self.target_x is None):
            self.pub.publish(out)
            return
        # TODO(navigation 4): associate signs/ArUco IDs with the correct CP,
        # advance only after live observations confirm checkpoint passage.
        # TODO(control 5): reduce surge around sharp turns and when uncertain;
        # add filtering and dynamic obstacle response for robust racing.
        # Positive Twist.linear.x requests FORWARD motion; the thruster bridge
        # handles the model-specific native propeller polarity.
        out.linear.x = clamp(float(self.get_parameter('surge_speed').value), 0.0, 0.4)
        out.angular.z = pixel_steering_command(
            self.target_x, self.image_width,
            self.get_parameter('kp_image_yaw').value,
            self.get_parameter('max_yaw').value)
        # TODO(control 6): verify camera mirroring and yaw polarity in Stonefish.
        self.pub.publish(out)

    def stop(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BlueBoatCameraSteering()
        rclpy.spin(node)
    except FairPlayViolation:
        raise SystemExit(2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
