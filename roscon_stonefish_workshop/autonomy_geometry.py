"""Static-map reference planning for Challenge 3 (XY geometry in Stonefish NED).

Known obstacle coordinates are supplied to the REFERENCE solution, not estimated
by the camera. All distances are metres. For realistic autonomy replace these
priors using stereo/sonar/depth perception (see autonomy_exercises.py).
"""
import heapq
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Obstacle:
    name: str
    x: float
    y: float
    radius: float           # circumscribing circle + collision margin is added below
    top_z: float = 5.0      # NED: smaller z is shallower
    bottom_z: float = 10.5

# Must match the solid geometry in bluerov2_autonomy_obstacles.scn.
# Conservative circular envelopes around rectangular occluders and pillars.
OBSTACLES = (
    Obstacle('CatApproachBlock', -14.0, -6.0, 1.35),
    Obstacle('CatOccluder', -12.5, -8.6, 1.45),
    Obstacle('CatPillar', -7.6, -9.1, 1.05),
    Obstacle('BustApproachBlock', -12.8, 1.0, 1.55),
    Obstacle('BustOccluder', -12.8, 7.7, 1.25),
    Obstacle('BustPillar', -7.0, 7.5, 1.05),
    Obstacle('EastTransitBlock', 3.0, 2.0, 1.65),
    Obstacle('WreckWestScreen', 13.0, 2.0, 1.35),
    Obstacle('WreckWestPillar', 13.0, 12.0, 1.35),
    Obstacle('WreckEastPillar', 24.0, -2.0, 1.35),
    Obstacle('WreckEastScreen', 25.0, 12.0, 1.35),
)
SITES = {'cat': (-10., -6.), 'bust': (-10., 5.), 'wreck': (19., 6.)}
# Approximate keep-out envelopes for objects; shipwreck uses an AABB rather
# than a circle because of its long axis. Geometry NOT used as ground truth of
# the surface reconstruction; only protects reference navigation.


def point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay
    t = 0.0 if dx*dx+dy*dy < 1e-12 else max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/(dx*dx+dy*dy)))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))


def point_clear(x, y, z=7.0, margin=0.65, obstacles=OBSTACLES, target_exclusion=True):
    if not (-28.0 <= x <= 34.0 and -22.0 <= y <= 25.0):
        return False
    for o in obstacles:
        if o.top_z-margin < z < o.bottom_z+margin and math.hypot(x-o.x, y-o.y) < o.radius+margin:
            return False
    if target_exclusion:
        for name, (cx, cy) in SITES.items():
            if name == 'wreck':
                if 15.5-margin < x < 22.5+margin and -3.8-margin < y < 15.8+margin:
                    return False
            elif math.hypot(x-cx, y-cy) < 1.15+margin:
                return False
    return True


def segment_clear(a, b, z=7.0, margin=0.65, obstacles=OBSTACLES, target_exclusion=True):
    steps = max(2, int(math.ceil(math.dist(a[:2], b[:2])/0.32)))
    for i in range(steps+1):
        f = i/steps
        x, y = a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1])
        if not point_clear(x,y,z,margin,obstacles,target_exclusion):
            return False
    return True


def visible(a, b, z=7.0, obstacles=OBSTACLES):
    """2D occlusion proxy, NOT a camera-frustum/ray-triangle renderer."""
    for o in obstacles:
        if o.top_z <= z <= o.bottom_z and point_segment_distance(o.x,o.y,*a[:2],*b[:2]) < o.radius:
            return False
    return True


def astar(start, goal, z=7.0, cell=0.8, margin=0.55, obstacles=OBSTACLES):
    """Eight-connected A* on conservative occupancy grid, with LOS smoothing."""
    start = (float(start[0]),float(start[1])); goal = (float(goal[0]),float(goal[1]))
    if not point_clear(*start,z,margin,obstacles):
        raise ValueError('A* start position inside keep-out region: '+repr(start))
    if not point_clear(*goal,z,margin,obstacles):
        raise ValueError('A* goal position inside keep-out region: '+repr(goal))
    if segment_clear(start,goal,z,margin,obstacles):
        return [start,goal]
    ox, oy = -28.0, -22.0
    def key(p): return (int(round((p[0]-ox)/cell)),int(round((p[1]-oy)/cell)))
    def coord(k): return (ox+k[0]*cell,oy+k[1]*cell)
    s, g = key(start), key(goal)
    directions=[(dx,dy) for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
    # Grid snapping must NOT place source/destination inside an obstacle.
    def nearby_safe(p,k):
        for di in range(-4,5):
            for dj in range(-4,5):
                kk=(k[0]+di,k[1]+dj);q=coord(kk)
                if point_clear(*q,z,margin,obstacles) and segment_clear(p,q,z,margin,obstacles):
                    yield kk, math.dist(p,q)
    source=dict(nearby_safe(start,s)); dest=dict(nearby_safe(goal,g))
    if not source or not dest:raise RuntimeError('No accessible grid cells near start or destination')
    todo=[];costs={};parent={};visited=set()
    for k,dist in source.items():
        costs[k]=dist;parent[k]=None
        heapq.heappush(todo,(dist+math.dist(coord(k),goal),k))
    end=None
    while todo:
        _,u=heapq.heappop(todo)
        if u in visited:continue
        visited.add(u)
        if u in dest:end=u;break
        for dx,dy in directions:
            v=(u[0]+dx,u[1]+dy);pv=coord(v)
            if not point_clear(*pv,z,margin,obstacles):continue
            p0=coord(u)
            if not segment_clear(p0,pv,z,margin,obstacles):continue
            candidate=costs[u]+math.hypot(dx,dy)*cell
            if candidate<costs.get(v,1e30):
                costs[v]=candidate;parent[v]=u
                heapq.heappush(todo,(candidate+math.dist(pv,goal),v))
        if len(visited)>12000:break
    if end is None:raise RuntimeError('No collision-free route to '+repr(goal))
    cells=[end]
    while parent[cells[-1]] is not None:cells.append(parent[cells[-1]])
    route=[start]+[coord(x) for x in reversed(cells)]+[goal]
    smooth=[route[0]];i=0
    while i<len(route)-1:
        for j in range(len(route)-1,i,-1):
            if segment_clear(route[i],route[j],z,margin,obstacles):break
        smooth.append(route[j]);i=j
    return smooth


def view_candidates(target, heights=(7.1, 8.0)):
    if target != 'cat':
        raise ValueError('Reference viewpoint candidates are provided for CAT only')
    cx,cy=SITES[target]; result=[]
    for z in heights:
        for i in range(18):
            angle=2*math.pi*i/18
            result.append((cx+3.6*math.cos(angle),cy+3.6*math.sin(angle),z))
    return result


def select_views(target, min_views=14, obstacle_map=OBSTACLES):
    """Greedy map-based viewpoint selection: novel azimuth/height or ship-side cell.

    Scoring includes occlusion and minimum clearance; it is a planar visibility
    approximation, NOT an online measured mesh-completeness estimate.
    """
    if target != 'cat':
        raise ValueError('Reference next-best-view is CAT-only; implement bust/wreck planning')
    cx,cy=SITES[target]; candidates=view_candidates(target)
    selected=[]; cells=set()
    while candidates and len(selected)<min_views:
        scored=[]
        for p in candidates:
            if not point_clear(*p,margin=.55,obstacles=obstacle_map):continue
            if not visible(p,(cx,cy),p[2],obstacle_map):continue
            az=(math.atan2(p[1]-cy,p[0]-cx)+2*math.pi)%(2*math.pi)
            cell=(int(round(az/(2*math.pi)*12))%12,round(p[2]))
            unseen=cell not in cells
            if not unseen:continue
            travel=math.dist((p[0],p[1]),(selected[-1][0],selected[-1][1])) if selected else 0.0
            # Prefer viewpoints providing novel view sectors but nearby to minimize travel.
            score=100.0-0.75*travel
            scored.append((score,cell,p))
        if not scored:break
        _,cell,p=max(scored,key=lambda v:v[0]);selected.append(p);cells.add(cell);candidates.remove(p)
    return selected
