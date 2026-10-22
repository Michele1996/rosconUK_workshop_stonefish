"""Workshop FAIR PLAY guard for the *supplied* participant interfaces.

This is an educational, in-process rule enforcement mechanism, NOT a sandbox.
On a participant-controlled computer it cannot intercept arbitrary Python
open(), editor reads, modified code, direct ROS data, or extracted scene geometry.
Only organiser-controlled execution can provide enforceable isolation.
"""
import sys


class FairPlayViolation(RuntimeError):
    """Access to organiser-only course data was requested by participant code."""


def violation(reason, source=None):
    detail = str(source) if source is not None else 'restricted navigation source'
    message = (
        '\n' + '=' * 76 + '\n'
        'FAIR PLAY VIOLATION — BLUEBOAT GRAND PRIX\n\n'
        'Precomputed race coordinates, course JSON/CSV, generated waypoint files,\n'
        'judge internals and reconstructed checkpoint positions are NOT permitted\n'
        'as participant navigation inputs. Use camera, ArUco, IMU, GPS or odometry.\n\n'
        f'Reason: {reason}\n'
        f'Blocked source: {detail}\n\n'
        'The LOCAL RACE JUDGE and local practice/official scoring remain enabled.\n'
        'This participant controller is stopping now.\n'
        + '=' * 76
    )
    print(message, file=sys.stderr, flush=True)
    raise FairPlayViolation(message)


def reject_route_file(value, *, parameter='course_file'):
    """Reject ANY supplied route/waypoint file through participant entrypoints.

    Rejecting the parameter rather than looking only for the original basename
    catches renamed JSON, CSV and YAML files through these same interfaces.
    """
    if value is None or (isinstance(value, str) and not value.strip()):
        return
    violation(f'Participant parameter {parameter!r} must not provide a precomputed route', value)


def guard_participant_route_parameters(node):
    for param in ('course_file', 'waypoints_file', 'route_file', 'track_file'):
        node.declare_parameter(param, '')
        reject_route_file(node.get_parameter(param).value, parameter=param)


def reject_privileged_topic(topic, *, parameter='odom_topic'):
    """Prevent supplied sensor starters from subscribing to judge-only topics."""
    normalized = str(topic).strip().lower()
    if ('/blueboat/race/' in normalized or 'ground_truth' in normalized or
            'nearest_course_s_m' in normalized):
        violation(f'Participant sensor parameter {parameter!r} points to a judge topic', topic)
