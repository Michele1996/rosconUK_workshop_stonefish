"""Deterministic ordered-gate race timing and boundary penalties (ROS-free)."""
from .race_geometry import gates, nearest_centerline, crossing_fraction


class RaceEngine:
    def __init__(self, course):
        self.course = course
        self.gates = gates(course)
        self.allowed_center_distance = (course['half_width_m'] -
                                        course['boat_boundary_margin_m'])
        self.armed = True  # Backwards-compatible pure-python test default.
        self.reset()

    def arm(self):
        self.armed = True
        self.reset()

    def disarm(self):
        self.armed = False
        self.reset()

    def reset(self):
        self.state = 'waiting_for_start' if self.armed else 'practice'
        self.next_index = 0
        self.start_time = None
        self.finish_time = None
        self.penalty = 0.0
        self.excursions = 0
        self.offtrack_time = 0.0
        self.outside = False
        self.last_xy = None
        self.last_t = None
        self.events = []

    def update(self, xy, timestamp):
        xy = (float(xy[0]), float(xy[1]))
        now = float(timestamp)
        self.events = []
        if self.last_t is not None and now < self.last_t - 1e-4:
            # Simulator time reset, not just loss of an odometry sample.
            self.reset()
            self.events.append('Simulation clock reset')
        if self.last_t is not None and now <= self.last_t:
            return self.status(now, xy)
        dist, arc = nearest_centerline(self.course, xy)
        if not self.armed:
            # Practice is deliberately scoreless. Even passing START or CP gates
            # cannot create a run until an instructor arms the judge.
            self.last_xy, self.last_t = xy, now
            return self.status(now, xy, dist=dist, arc=arc)
        is_out = (dist > self.allowed_center_distance if not self.outside
                  else dist > (self.allowed_center_distance -
                               self.course['offtrack_reentry_hysteresis_m']))
        if self.state == 'running' and self.last_t is not None:
            dt = now-self.last_t
            if is_out and not self.outside:
                self.excursions += 1
                self.penalty += self.course['offtrack_entry_penalty_s']
                self.events.append('OFF TRACK: +%.1fs' % self.course['offtrack_entry_penalty_s'])
            if self.outside or is_out:
                self.offtrack_time += dt
                self.penalty += dt * self.course['offtrack_penalty_per_second']
            if self.outside and not is_out:
                self.events.append('Returned inside track')
        self.outside = is_out
        if self.state in ('waiting_for_start', 'running') and self.last_xy is not None:
            gate = self.gates[self.next_index]
            frac = crossing_fraction(self.last_xy, xy, gate, self.allowed_center_distance)
            if frac is not None:
                crossing_t = self.last_t + frac * (now-self.last_t)
                if self.next_index == 0:
                    self.state = 'running'
                    self.start_time = crossing_t
                    self.events.append('START')
                elif self.next_index == len(self.gates)-1:
                    self.state = 'finished'
                    self.finish_time = crossing_t
                    self.events.append('FINISH')
                else:
                    self.events.append('%s passed' % gate[0])
                self.next_index += 1
        if (self.state == 'running' and self.start_time is not None and
                now-self.start_time > self.course['max_race_time_s']):
            self.state = 'timed_out'
            self.finish_time = self.start_time+self.course['max_race_time_s']
            self.events.append('DNF: time limit exceeded')
        self.last_xy, self.last_t = xy, now
        return self.status(now, xy, dist=dist, arc=arc)

    def status(self, timestamp, xy, dist=None, arc=None):
        if dist is None:
            dist, arc = nearest_centerline(self.course, xy)
        if self.start_time is None:
            elapsed = 0.0
        else:
            end = self.finish_time if self.finish_time is not None else timestamp
            elapsed = max(0.0, end-self.start_time)
        next_gate = (self.gates[self.next_index][0]
                     if self.next_index < len(self.gates) else None)
        return {
            'state': self.state,
            'official_armed': self.armed,
            'race_started': self.start_time is not None,
            'next_gate': next_gate,
            'gates_passed': self.next_index,
            'gates_total': len(self.gates),
            'race_time_s': round(elapsed, 3),
            'penalty_s': round(self.penalty, 3),
            'adjusted_time_s': round(elapsed+self.penalty, 3),
            'offtrack_excursions': self.excursions,
            'offtrack_duration_s': round(self.offtrack_time, 3),
            'outside': self.outside,
            'distance_from_center_m': round(dist, 3),
            'nearest_course_s_m': round(arc, 3),
            'events': list(self.events),
        }
