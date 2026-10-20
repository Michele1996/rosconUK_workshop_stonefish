"""Challenge 3 replaceable algorithms for BUST and SHIPWRECK.

The full autonomous CAT reference is in inspection_reference.py with a known
obstacle prior. BUST and SHIPWRECK deliberately do NOT have ready-made
inspection/return solutions. Run autonomy_mode:=starter to exercise the safe
SEARCH prefix and then implement the functions below.

Inputs can be ROS 2 sensor observations, e.g. camera RGB, stereo/sonar, IMU,
pressure and DVL. Avoid consulting the hidden evaluator ground truth.
"""
from .autonomy_geometry import OBSTACLES, SITES


def search_waypoints(search_region, fov_degrees=70, distance_m=3.0):
    """EXERCISE: generate coverage-aware exploration waypoints in NED XY.

    Args: search_region=(xmin, ymin, xmax, ymax) in metres;
          fov_degrees=horizontal camera FOV; distance_m=stand-off estimate.
    Return: iterable of (x,y,z,yaw) safe camera viewpoints.
    """
    raise NotImplementedError('Implement a search planner for bust/wreck')


def discover_target(camera_observations, target):
    """EXERCISE: use *actual* camera images to detect/identify the target.

    Return True only after sufficient visual evidence is obtained. Do not
    substitute a known target position for visual discovery.
    """
    if target not in ('bust','wreck','cat'):
        raise ValueError('target must be cat, bust or wreck')
    raise NotImplementedError('Implement visual target detection')


def update_obstacles(sonar_observations, map_prior=OBSTACLES):
    """EXERCISE: construct/refresh local collision map from sensor observations.

    A prior map is provided to compare against, not as a required solution.
    Return your safe obstacle representation and uncertainty margins.
    """
    raise NotImplementedError('Implement online obstacle inference')


def plan_safe_route(current_pose, goal_pose, obstacle_map):
    """EXERCISE: collision-check and route around detected 3D obstacles."""
    raise NotImplementedError('Implement safe route planning')


def choose_next_best_views(target, camera_model, observed_geometry, obstacle_map):
    """EXERCISE: choose viewpoints maximizing newly visible target surface.

    Account for visibility, minimum stereo baseline, overlap, occlusion,
    travel cost, and minimum clearance. Return list of (x,y,z,yaw).
    """
    raise NotImplementedError('Implement a 3D-aware next-best-view planner')


def should_return_home(battery_budget, elapsed_s, coverage, risk):
    """EXERCISE: decide when to stop inspecting and safely return."""
    raise NotImplementedError('Implement mission termination and safe return')
