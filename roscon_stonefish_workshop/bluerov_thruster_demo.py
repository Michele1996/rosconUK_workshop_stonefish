#!/usr/bin/env python3
"""Safe direct-thruster familiarisation for the Stonefish BlueROV2 Heavy.

No position control or thruster adapter should be active during tests.
The 8 thrusters are published in *the same order as the scenario actuators*.
Values are normalized and individually limited to [-0.25,+0.25].
The sign of motion must be confirmed in the actual simulator.
"""
import argparse
import math
import time

TOPIC = '/bluerov/controller/thruster_setpoints_sim'
THRUSTERS = (
    'FrontRight', 'FrontLeft', 'BackRight', 'BackLeft',
    'DiveFrontRight', 'DiveFrontLeft', 'DiveBackRight', 'DiveBackLeft',
)
# Same low-level matrix as thruster_adapter.py. This is a *test pattern*, not
# proof of calibrated body directions; use the single-motor test to verify.
AXES = {
    'surge': (1,1,-1,-1,0,0,0,0),
    'sway': (1,-1,1,-1,0,0,0,0),
    'yaw': (1,-1,-1,1,0,0,0,0),
    'heave': (0,0,0,0,1,-1,-1,1),
}
LIMIT = 0.25

def make_pattern(thruster=None, axis=None, value=0.25):
    """Return an 8-value normalized actuator command."""
    if not math.isfinite(value) or abs(value)>LIMIT:
        raise ValueError(f'Choose a finite --value between {-LIMIT} and +{LIMIT}')
    if bool(thruster is not None) == bool(axis is not None):
        raise ValueError('Choose exactly one of --thruster or --axis')
    if thruster is not None:
        if thruster not in THRUSTERS:
            raise ValueError('Unknown thruster: '+str(thruster))
        arr=[0.0]*len(THRUSTERS)
        arr[THRUSTERS.index(thruster)]=float(value)
        return arr
    if axis not in AXES:
        raise ValueError('Unknown axis: '+str(axis))
    return [float(v)*value for v in AXES[axis]]


def main():
    ap=argparse.ArgumentParser(description='Direct 8-thruster BlueROV2 workshop demo (no controller)')
    choice=ap.add_mutually_exclusive_group(required=True)
    choice.add_argument('--demo',action='store_true',help='Pulse the eight thrusters, one at a time')
    choice.add_argument('--thruster',choices=THRUSTERS,help='Pulse one thruster by name')
    choice.add_argument('--axis',choices=tuple(AXES),help='Test a collective motion pattern')
    ap.add_argument('--value',type=float,default=0.25,help='Normalized setpoint; maximum magnitude 0.25')
    ap.add_argument('--duration',type=float,default=1.5,help='Seconds per pulse, at most 6.0')
    ap.add_argument('--topic',default=TOPIC)
    args=ap.parse_args()
    if not math.isfinite(args.duration) or not 0.05<=args.duration<=6.0:
        ap.error('--duration must be between 0.05 and 6.0 seconds')
    if not math.isfinite(args.value) or not 0.0<abs(args.value)<=LIMIT:
        ap.error(f'--value must be nonzero and within [-{LIMIT}, +{LIMIT}]')
    sequence=[(n, make_pattern(thruster=n,value=args.value)) for n in THRUSTERS] if args.demo else [
        (args.thruster or args.axis, make_pattern(thruster=args.thruster,axis=args.axis,value=args.value))]
    import rclpy
    from std_msgs.msg import Float64MultiArray
    rclpy.init()
    node=rclpy.create_node('bluerov_thruster_demo')
    pub=node.create_publisher(Float64MultiArray,args.topic,10)
    def pulse(values, seconds):
        end=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:
            msg=Float64MultiArray(); msg.data=[float(x) for x in values]
            pub.publish(msg)
            time.sleep(0.05)
    zeros=[0.0]*8
    try:
        node.get_logger().info('Direct mode: no PID/allocator active. Topic: '+args.topic)
        node.get_logger().info('Order: '+', '.join(THRUSTERS))
        pulse(zeros,0.3)
        for label,pattern in sequence:
            node.get_logger().info(f'Testing {label}: {[round(v,3) for v in pattern]}')
            pulse(pattern,args.duration)
            pulse(zeros,1.0)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted; stopping all motors.')
    finally:
        pulse(zeros,0.7)
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
