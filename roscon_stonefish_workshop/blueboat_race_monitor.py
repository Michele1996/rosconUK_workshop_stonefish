#!/usr/bin/env python3
"""Local official race judge. Scores only after explicit race_start arm.

Practice crosses no gate and never creates a report. Attempts are reserved on
arming and persist across restarts. DNF or abandoned reservations consume slots.
"""
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .race_engine import RaceEngine
from .race_geometry import load_course
from .race_records import AttemptStore, DEFAULT_RESULTS, sha256_file


class RaceMonitor(Node):
    def __init__(self):
        super().__init__('blueboat_race_monitor')
        self.declare_parameter('course_file', '')
        self.declare_parameter('odom_topic', '/blueboat/race/ground_truth_odom')
        self.declare_parameter('team', 'Team 1')
        self.declare_parameter('results_dir', DEFAULT_RESULTS)
        self.declare_parameter('max_attempts', 3)
        course_file = self.get_parameter('course_file').value
        if not course_file:
            raise ValueError('course_file launch parameter is required')
        course = load_course(course_file, purpose='judge')
        self.engine = RaceEngine(course)
        self.engine.disarm()  # ALWAYS practice until a team explicitly arms.
        self.team = str(self.get_parameter('team').value)
        self.store = AttemptStore(
            self.get_parameter('results_dir').value, self.team,
            course.get('course_version', 'unknown'), sha256_file(course_file),
            self.get_parameter('max_attempts').value)
        self.last_trace_sim = float('-inf')
        self.last_log_sim = float('-inf')
        self.last_status = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/blueboat/race/status', qos)
        self.create_service(SetBool, '/blueboat/race/arm', self.arm_cb)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 20)
        self.get_logger().info(
            f'BlueBoat judge: team={self.team}, PRACTICE mode; '
            f'{self.store.remaining}/3 official attempts remain; result dir={self.store.root}')

    def arm_cb(self, request, response):
        if not request.data:
            if self.store.current:
                self.store.finalize('DNF_CANCELLED', self.last_status)
            self.engine.disarm()
            response.success = True
            response.message = 'Practice mode. Active official attempt cancelled (DNF).'
            return response
        if self.engine.state != 'practice' or self.store.current:
            response.success = False
            response.message = 'Already armed. Stop this attempt first, or restart the simulator.'
            return response
        if self.engine.last_xy is None:
            response.success = False
            response.message = 'Waiting for BlueBoat odometry.'
            return response
        origin = self.engine.course['boat_start_xyz_m']
        if math.dist(self.engine.last_xy, origin[:2]) > 1.5:
            response.success = False
            response.message = 'Boat must be within 1.5 m of spawn; restart Stonefish to reset.'
            return response
        try:
            record = self.store.start(self.engine.last_t)
        except (OSError, RuntimeError) as err:
            response.success = False
            response.message = str(err)
            return response
        self.engine.arm()
        self.last_trace_sim = float('-inf')
        response.success = True
        response.message = (
            f'Official attempt {record["attempt_number"]}/3 ARMED. '
            'Cross the green START gate forward to begin timing.')
        self.get_logger().info(response.message)
        return response

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        if self.engine.last_t is not None and stamp < self.engine.last_t - 1e-4:
            if self.store.current:
                self.store.finalize('DNF_CLOCK_RESET', self.last_status)
            self.engine.disarm()
            self.last_trace_sim = float('-inf')
            self.last_log_sim = float('-inf')
        status = self.engine.update((p.x, p.y), stamp)
        self.last_status = status
        if self.store.current:
            # Capture ordered event times. Events are emitted on the sample after crossing.
            for event in status['events']:
                if event == 'START':
                    self.store.current['status'] = 'RUNNING'
                    self.store.checkpoint('START', self.engine.start_time)
                elif event == 'FINISH':
                    self.store.checkpoint('FINISH', self.engine.finish_time)
                elif event.endswith(' passed') and event.startswith('CP'):
                    self.store.checkpoint(event.split()[0], stamp)
            if stamp - self.last_trace_sim >= 0.2 or status['events']:
                self.store.trace(stamp, p.x, p.y, status)
                self.last_trace_sim = stamp
            if status['state'] == 'finished':
                self.store.finalize('FINISHED', status)
            elif status['state'] == 'timed_out':
                self.store.finalize('DNF_TIMEOUT', status)
        # Keep full geometric diagnostics INSIDE the judge and saved records.
        # Never publish the signed/cross-track geometric oracle over the public
        # status topic; participants must estimate route geometry from sensors.
        public_status = {key: value for key, value in status.items()
                         if key not in ('distance_from_center_m',
                                        'nearest_course_s_m', 'outside')}
        public_status.update({
            'team': self.team,
            'attempts_used': self.store.consumed,
            'attempts_remaining': self.store.remaining,
            'official_attempt_id': self.store.current['attempt_id'] if self.store.current else None,
            'local_results_directory': str(self.store.root),
        })
        message = String()
        message.data = json.dumps(public_status, separators=(',', ':'))
        self.status_pub.publish(message)
        if status['events'] or stamp - self.last_log_sim >= 5:
            self.get_logger().info(
                f'[{status["state"]}] {status["race_time_s"]:.1f}s '
                f'+{status["penalty_s"]:.1f}s | next={status["next_gate"]} '
                f'| official attempts remaining={self.store.remaining} '
                f'| events={status["events"]}')
            self.last_log_sim = stamp


def main():
    rclpy.init()
    node = RaceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
