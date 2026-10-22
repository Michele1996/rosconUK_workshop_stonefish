"""Organiser geometry for Stonefish course generation and local scoring.

Pure Python: these functions can be unit-tested without a ROS 2 installation.
The source of truth for the course is config/blueboat_race.json.
"""
import json
import math
from pathlib import Path

from .fair_play import violation


def load_course(path, *, purpose=None):
    """Load private course data for official scoring, scene generation or tests.

    Participant calls to this SUPPLIED API explicitly fail with the fair-play
    notice. Purpose tags are cooperative, NOT an access-control boundary.
    """
    if purpose not in ('judge', 'scene_generation', 'offline_test'):
        violation('Loading organiser course geometry for navigation is prohibited', path)
    with open(Path(path).expanduser(), encoding='utf-8') as stream:
        course = json.load(stream)
    if len(course['centerline_xy_m']) < 2:
        raise ValueError('The race course needs at least two centreline points')
    if not 0 < course['boat_boundary_margin_m'] < course['half_width_m']:
        raise ValueError('Boundary margin must be smaller than half-width')
    return course


def _smooth_points(course):
    """Smooth race bends using a cubic Hermite centreline, cached per course.

    Tangents are matched across control points, so the buoy lanes do not
    produce sharp normal discontinuities or cross each other at corners.
    """
    if '_smooth_points' not in course:
        if course.get('centerline_interpolation') == 'sampled':
            course['_smooth_points'] = [tuple(map(float, p)) for p in course['centerline_xy_m']]
            return course['_smooth_points']
        controls = [tuple(map(float, p)) for p in course['centerline_xy_m']]
        derivatives = []
        for i in range(len(controls)):
            if i == 0:
                dx = controls[1][0]-controls[0][0]
                dy = controls[1][1]-controls[0][1]
            elif i == len(controls)-1:
                dx = controls[-1][0]-controls[-2][0]
                dy = controls[-1][1]-controls[-2][1]
            else:
                dx = 0.5*(controls[i+1][0]-controls[i-1][0])
                dy = 0.5*(controls[i+1][1]-controls[i-1][1])
            derivatives.append((dx,dy))
        points = [controls[0]]
        for i in range(len(controls)-1):
            a, b = controls[i], controls[i+1]
            m0, m1 = derivatives[i], derivatives[i+1]
            for step in range(1, 25):
                t = step/24.0
                h00 = 2*t**3-3*t**2+1
                h10 = t**3-2*t**2+t
                h01 = -2*t**3+3*t**2
                h11 = t**3-t**2
                points.append(tuple(h00*a[k]+h10*m0[k]+h01*b[k]+h11*m1[k]
                                    for k in (0,1)))
        course['_smooth_points'] = points
    return course['_smooth_points']


def segments(course):
    points = _smooth_points(course)
    start = 0.0
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if length < 1e-8:
            raise ValueError('Centreline contains duplicated consecutive points')
        tangent = ((b[0]-a[0])/length, (b[1]-a[1])/length)
        yield a, b, start, length, tangent
        start += length


def length(course):
    return sum(seg[3] for seg in segments(course))


def sample_course(course, s):
    """Return (xy, unit tangent, unit left normal) at route arc length s."""
    segs = list(segments(course))
    s = max(0.0, min(length(course), float(s)))
    for i, (a, _b, before, segment_len, tangent) in enumerate(segs):
        if s <= before+segment_len or i == len(segs)-1:
            dt = min(segment_len, max(0.0, s-before))
            xy = (a[0]+dt*tangent[0], a[1]+dt*tangent[1])
            return xy, tangent, (-tangent[1], tangent[0])
    raise AssertionError('Route sample failed')


def nearest_centerline(course, xy):
    """Return (minimum point-to-polyline distance, arc length of nearest point)."""
    x, y = xy
    nearest = (float('inf'), 0.0)
    for a, _b, before, seg_len, tangent in segments(course):
        along = max(0.0, min(seg_len, (x-a[0])*tangent[0]+(y-a[1])*tangent[1]))
        px, py = a[0]+tangent[0]*along, a[1]+tangent[1]*along
        d = math.hypot(x-px, y-py)
        if d < nearest[0]:
            nearest = (d, before+along)
    return nearest


def gates(course):
    specs = [('START', course['start_gate_s_m'])]
    specs.extend(('CP%02d' % (i+1), s) for i, s in enumerate(course['checkpoint_s_m']))
    specs.append(('FINISH', course['finish_gate_s_m']))
    if any(b[1] <= a[1] for a, b in zip(specs, specs[1:])):
        raise ValueError('Gates must be strictly increasing along the centreline')
    if specs[0][1] <= 0 or specs[-1][1] >= length(course):
        raise ValueError('Start/finish gates must be interior to the centreline')
    return [(name, float(s), *sample_course(course, s)) for name, s in specs]


def crossing_fraction(old_xy, new_xy, gate, half_width):
    """Forward crossing of a gate plane in the track, with sub-frame fraction.

    The crossing is accepted ONLY when the position intersecting the gate plane
    lies within the gate's width; crossing a gate's infinite extension fails.
    """
    _name, _s, center, forward, side = gate
    ax = (old_xy[0]-center[0])*forward[0]+(old_xy[1]-center[1])*forward[1]
    bx = (new_xy[0]-center[0])*forward[0]+(new_xy[1]-center[1])*forward[1]
    if not (ax < 0.0 <= bx) or bx == ax:
        return None
    fraction = -ax/(bx-ax)
    px = old_xy[0] + (new_xy[0]-old_xy[0])*fraction
    py = old_xy[1] + (new_xy[1]-old_xy[1])*fraction
    lateral = abs((px-center[0])*side[0]+(py-center[1])*side[1])
    if lateral > half_width:
        return None
    return fraction
