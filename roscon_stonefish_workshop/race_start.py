"""Local wrapper: arm a three-attempt official race; no shared ROS network."""
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def main():
    parser = argparse.ArgumentParser(description='Arm or cancel a local official BlueBoat attempt')
    parser.add_argument('--disarm', action='store_true', help='Cancel current attempt (consumes one slot)')
    args = parser.parse_args()
    rclpy.init()
    node = Node('blueboat_race_start_cli')
    try:
        client = node.create_client(SetBool, '/blueboat/race/arm')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Race judge unavailable: launch BlueBoat workshop first')
        req = SetBool.Request()
        req.data = not args.disarm
        result = client.call_async(req)
        rclpy.spin_until_future_complete(node, result, timeout_sec=10)
        if not result.done() or result.result() is None:
            raise RuntimeError('Race judge did not respond')
        print(('OK: ' if result.result().success else 'NOT ARMED: ') + result.result().message)
        if not result.result().success:
            raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
