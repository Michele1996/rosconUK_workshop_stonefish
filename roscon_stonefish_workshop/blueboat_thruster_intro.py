#!/usr/bin/env python3
"""Introductory direct two-thruster example (adapter MUST be disabled).

Run with: ros2 run roscon_stonefish_workshop blueboat_thruster_intro --demo
or:       ros2 run roscon_stonefish_workshop blueboat_thruster_intro --right -0.15 --left -0.15 --duration 3

BlueBoat SCN thruster order: [right, left]. Commands normalized in [-1, +1].
Signs depend on final physical orientation; test gently and visually confirm.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DirectMotors(Node):
    def __init__(self, topic):
        super().__init__('blueboat_thruster_intro')
        self.pub = self.create_publisher(Float64MultiArray, topic, 10)

    def move(self, right, left, duration, hz=20):
        assert math.isfinite(right) and math.isfinite(left) and math.isfinite(duration)
        assert -1 <= right <= 1 and -1 <= left <= 1
        start = time.monotonic()
        period = 1.0 / hz
        while rclpy.ok() and time.monotonic() - start < duration:
            msg = Float64MultiArray()
            msg.data = [float(right), float(left)]
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(period)

    def stop(self):
        self.move(0.0, 0.0, 0.6)


def main():
    parser = argparse.ArgumentParser(description='BlueBoat thruster familiarisation (direct motor commands)')
    parser.add_argument('--right', type=float, default=-0.15)
    parser.add_argument('--left', type=float, default=-0.15)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--demo', action='store_true', help='Gentle straight / differential / stop sequence')
    parser.add_argument('--topic', default='/blueboat/controller/thruster_setpoints_sim')
    args = parser.parse_args()
    if args.duration <= 0 or args.duration > 15:
        parser.error('Use --duration between 0 and 15 seconds')
    if max(abs(args.right), abs(args.left)) > 0.3:
        parser.error('For this introductory demo, keep both setpoints within [-0.3, 0.3]')
    rclpy.init()
    motor = DirectMotors(args.topic)
    try:
        if args.demo:
            motor.get_logger().info('Gentle test: straight -> stop -> turn -> stop')
            motor.move(-0.15, -0.15, 2.5)
            motor.move(0., 0., 1.)
            motor.move(-0.1, -0.2, 2.)
        else:
            motor.get_logger().info(f'Direct setpoints [right,left]=[{args.right:.2f},{args.left:.2f}] for {args.duration:.1f}s')
            motor.move(args.right, args.left, args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        motor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
