"""ROS-independent waypoint guidance and BlueROV2 allocation math."""
import math
from dataclasses import dataclass
import numpy as np

ALLOCATION = np.asarray([
    [1,1,1,0],[1,-1,-1,0],[-1,1,-1,0],[-1,-1,1,0],
    [0,0,0,1],[0,0,0,-1],[0,0,0,-1],[0,0,0,1],
], dtype=float)

def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def body_forward_velocity(vx_world, vy_world, yaw):
    return math.cos(yaw) * vx_world + math.sin(yaw) * vy_world


def allocate(surge, sway, yaw, heave, limit=0.65):
    """User's 8x4 joystick allocation, with proportional common scaling."""
    cmd = np.asarray([surge, sway, yaw, heave], dtype=float)
    out = ALLOCATION @ cmd
    peak = float(np.max(np.abs(out)))
    if peak > limit:
        out *= limit / peak
    return out.tolist()


@dataclass(frozen=True)
class WaypointGains:
    xy_tolerance: float = 0.65
    turn_threshold_rad: float = 0.52  # ~34 degrees
    kp_forward: float = 0.28
    kd_forward: float = 0.13
    max_forward: float = 0.58
    kp_yaw: float = 0.30
    kd_yaw: float = 0.22
    max_yaw: float = 0.28
    kp_depth: float = 1.20
    ki_depth: float = 0.10
    kd_depth: float = 0.30
    max_heave: float = 0.50
    depth_deadband: float = 0.06
    # Original Logitech joystick directions are not guaranteed to be NED signs.
    # These are startup estimates; confirm via --axis yaw / heave in Stonefish.
    yaw_sign: float = -1.0
    heave_sign: float = -1.0


def waypoint_efforts(x, y, z, yaw, gx, gy, gz, goal_yaw,
                     forward_speed=0.0, down_speed=0.0, yaw_rate=0.0,
                     depth_integral=0.0, turn_depth=None, gains=None):
    """Pure, ROS-independent body commands for a single NED waypoint.

    Sign-independent terms are defined by NED kinematics; yaw_sign/heave_sign
    convert NED requests to the existing hardware/joystick motor convention.
    """
    g = gains or WaypointGains()
    dx, dy = gx-x, gy-y
    dxy = math.hypot(dx, dy)
    course_yaw = math.atan2(dy, dx) if dxy > g.xy_tolerance else goal_yaw
    yaw_error = wrap_pi(course_yaw-yaw)
    mode = 'SETTLE' if dxy <= g.xy_tolerance else (
        'TURN' if abs(yaw_error) > g.turn_threshold_rad else 'MOVE')

    # Avoid rushing ahead when the target lies behind the vehicle. No sway or
    # simultaneous aggressive translation+rotation is needed for this exercise.
    surge = 0.0
    if mode == 'MOVE':
        alignment = clamp((g.turn_threshold_rad-abs(yaw_error)) /
                          g.turn_threshold_rad, 0.0, 1.0)
        # Automatic low-speed approach inside 2 m.  Fast segments still use
        # the full available surge; a near-goal speed limit prevents overshoot.
        approach_limit = g.max_forward * clamp(dxy / 2.0, 0.15, 1.0)
        surge = clamp(g.kp_forward * dxy - g.kd_forward * forward_speed,
                      0.0, approach_limit) * alignment

    # Angular deadband and rate damping avoid chattering at the final heading.
    # Keep rate damping active even inside the deadband to brake residual spin.
    yaw_p = 0.0 if abs(yaw_error) < math.radians(2.0) else g.kp_yaw * yaw_error
    yaw_cmd = g.yaw_sign * clamp(yaw_p - g.kd_yaw * yaw_rate,
                                  -g.max_yaw, g.max_yaw)

    # Do not suddenly dive/climb while rotating on the spot to face waypoint 0.
    # Keep depth at the moment we entered TURN, then track waypoint depth in MOVE.
    desired_depth = turn_depth if mode == 'TURN' and turn_depth is not None else gz
    ez = desired_depth-z
    # Small deadband prevents a near-neutrally buoyant ROV constantly hunting.
    if abs(ez) <= g.depth_deadband and abs(down_speed) < 0.035:
        heave_cmd = 0.0
    else:
        heave_cmd = g.heave_sign * clamp(
            g.kp_depth * ez + g.ki_depth * depth_integral - g.kd_depth * down_speed,
            -g.max_heave, g.max_heave)
    return (float(surge), 0.0, float(yaw_cmd), float(heave_cmd)), mode, dxy, ez, yaw_error




class YawDivergenceGuard:
    """Simulation-only yaw polarity check from closed-loop odometry.

    During *pure rotation* (TURN or SETTLE), if a meaningful correction
    consistently increases heading error, reverse the assumed yaw polarity
    once. If the reversed polarity also diverges, inhibit yaw and require
    inspection of the motor mapping/odometry. Never learn during translation,
    yaw braking, small corrections, or the turn's immediate motor transient.
    """

    def __init__(self, sign=-1.0, reverse_after_s=0.55, lock_after_s=1.5):
        if sign not in (-1.0, 1.0):
            raise ValueError('yaw_sign must be -1 or +1')
        self.sign = float(sign)
        self.reversed_once = False
        self.locked = False
        self.diverging_since = None
        self.ignore_until = 0.0
        self.reverse_after_s = float(reverse_after_s)
        self.lock_after_s = float(lock_after_s)

    def update(self, now, mode, yaw_error, yaw_rate, motor_yaw):
        """Return 'REVERSED', 'LOCKED', or None; new sign in ``sign``."""
        if self.locked:
            return None
        # motor_yaw is in actuator convention; recover intended NED command
        # using the currently assumed polarity.
        correcting = yaw_error * motor_yaw * self.sign > 0.0
        if (now < self.ignore_until or mode not in ('TURN', 'SETTLE') or
                abs(yaw_error) < math.radians(10) or
                abs(yaw_rate) < 0.035 or abs(motor_yaw) < 0.08 or
                not correcting or yaw_error * yaw_rate >= -0.003):
            self.diverging_since = None
            return None
        if self.diverging_since is None:
            self.diverging_since = now
            return None
        limit = self.lock_after_s if self.reversed_once else self.reverse_after_s
        if now - self.diverging_since < limit:
            return None
        self.diverging_since = None
        if not self.reversed_once:
            self.sign *= -1.0
            self.reversed_once = True
            # Allow propeller dynamics and vehicle inertia to settle.
            self.ignore_until = now + 1.0
            return 'REVERSED'
        self.locked = True
        return 'LOCKED'
