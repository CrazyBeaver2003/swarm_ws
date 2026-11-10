"""Curvature bounded paths on a flat, obstacle free ENU map.

Analytic Dubins construction adapted from this project's gpt_test reference.
Tracking has been replaced with bounded arc-length progress; a loop may not
be skipped merely because a later segment lies near the aircraft.

Yaw is mathematical ENU yaw (zero East, positive towards North). PX4 course
conversion belongs at the output boundary. Six Dubins words are evaluated using
the classical analytic construction; no simulator or GPS position is consumed.
"""
from dataclasses import dataclass
import math

TAU = 2.0 * math.pi


def wrap(angle):
    return (angle + math.pi) % TAU - math.pi


def positive(angle):
    return angle % TAU


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Route:
    points: tuple
    radius: float
    length: float
    word: str
    goal: tuple
    loiter_center: tuple
    working_altitude: float


def _words(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    cab = math.cos(a - b)
    result = []

    def straight(word, p2, calculate):
        if p2 >= -1e-10:
            p = math.sqrt(max(p2, 0.0))
            t, q = calculate(p)
            result.append((word, (positive(t), p, positive(q))))

    straight('LSL', 2 + d*d - 2*cab + 2*d*(sa-sb),
             lambda p: (-a + math.atan2(cb-ca, d+sa-sb),
                        b - math.atan2(cb-ca, d+sa-sb)))
    straight('RSR', 2 + d*d - 2*cab + 2*d*(-sa+sb),
             lambda p: (a - math.atan2(ca-cb, d-sa+sb),
                        -b + math.atan2(ca-cb, d-sa+sb)))
    straight('LSR', -2 + d*d + 2*cab + 2*d*(sa+sb),
             lambda p: (-a + math.atan2(-ca-cb, d+sa+sb) - math.atan2(-2, p),
                        -b + math.atan2(-ca-cb, d+sa+sb) - math.atan2(-2, p)))
    straight('RSL', d*d - 2 + 2*cab - 2*d*(sa+sb),
             lambda p: (a - math.atan2(ca+cb, d-sa-sb) + math.atan2(2, p),
                        b - math.atan2(ca+cb, d-sa-sb) + math.atan2(2, p)))
    value = (6-d*d+2*cab+2*d*(sa-sb))/8
    if abs(value) <= 1.0 + 1e-10:
        p = positive(TAU-math.acos(max(-1.0, min(1.0, value))))
        t = positive(a-math.atan2(ca-cb, d-sa+sb)+p/2)
        result.append(('RLR', (t, p, positive(a-b-t+p))))
    value = (6-d*d+2*cab+2*d*(-sa+sb))/8
    if abs(value) <= 1.0 + 1e-10:
        p = positive(TAU-math.acos(max(-1.0, min(1.0, value))))
        t = positive(-a-math.atan2(ca-cb, d+sa-sb)+p/2)
        result.append(('LRL', (t, p, positive(b-a-t+p))))
    return result


def dubins(start, finish, radius=350.0, sample_step=8.0):
    """Return shortest six-word Dubins path, sampled by exact arc integration."""
    values = (start.x, start.y, start.yaw, finish.x, finish.y, finish.yaw,
              radius, sample_step)
    if not all(math.isfinite(v) for v in values) or radius < 120 or not 0.5 <= sample_step <= 50.0:
        raise ValueError('finite poses, radius >=120 m and positive sample step required')
    dx, dy = finish.x-start.x, finish.y-start.y
    theta = math.atan2(dy, dx)
    options = _words(positive(start.yaw-theta), positive(finish.yaw-theta),
                     math.hypot(dx, dy)/radius)
    word, lengths = min(options, key=lambda option: sum(option[1]))
    x, y, yaw = start.x, start.y, start.yaw
    points = [Pose2(x, y, wrap(yaw))]
    for mode, normalized_length in zip(word, lengths):
        remaining = normalized_length*radius
        curvature = {'L': 1/radius, 'R': -1/radius, 'S': 0.0}[mode]
        while remaining > 1e-9:
            ds = min(sample_step, remaining)
            if curvature:
                new_yaw = yaw+curvature*ds
                x += (math.sin(new_yaw)-math.sin(yaw))/curvature
                y += (math.cos(yaw)-math.cos(new_yaw))/curvature
                yaw = new_yaw
            else:
                x += ds*math.cos(yaw)
                y += ds*math.sin(yaw)
            points.append(Pose2(x, y, wrap(yaw)))
            remaining -= ds
    return tuple(points), sum(lengths)*radius, word


def plan_mission(start, goal, working_altitude=70.0, radius=350.0):
    """Reach the clicked goal, then enter a tangent CCW terminal orbit.

    Flat terrain is declared explicitly; height is chosen above that terrain.
    An orbit rather than stopping at goal respects fixed-wing kinematics.
    """
    if (len(goal) != 2 or not all(math.isfinite(v) and abs(v) < 10000 for v in goal)
            or not math.isfinite(working_altitude) or not 40 <= working_altitude <= 250):
        raise ValueError('goal must be inside the 20 km map and height 40..250 m')
    bearing = math.atan2(goal[1]-start.y, goal[0]-start.x)
    if math.hypot(goal[0]-start.x, goal[1]-start.y) < 1.0:
        bearing = start.yaw
    end = Pose2(goal[0], goal[1], bearing)
    center = goal[0]-radius*math.sin(bearing), goal[1]+radius*math.cos(bearing)
    points, length, word = dubins(start, end, radius)
    return Route(points, radius, length, word, tuple(goal), center, working_altitude)


class RouteFollower:
    """Local segment projection with progress bounded by elapsed flight time."""

    def __init__(self, route, arrival_radius=25.0):
        if not math.isfinite(arrival_radius) or not 10.0 <= arrival_radius <= 100.0:
            raise ValueError('Arrival radius must be 10..100 m')
        self.route = route
        self.arrival_radius = arrival_radius
        self.cumulative = [0.0]
        for a, b in zip(route.points, route.points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.hypot(b.x-a.x, b.y-a.y))
        self.progress = 0.0
        self.reached = False
        self.cross_track_error = 0.0

    def desired_course(self, x, y, speed, dt, lookahead=70.0):
        if not all(math.isfinite(v) for v in (x, y, speed, dt, lookahead)):
            raise ValueError('Finite tracking inputs required')
        if speed < 5.0 or not 0.0 < dt <= 0.5 or lookahead <= 0.0:
            raise ValueError('Moving aircraft and dt in (0, 0.5] required')
        points = self.route.points
        if len(points) == 1:
            self.cross_track_error = math.hypot(x-points[0].x, y-points[0].y)
            self.reached = self.cross_track_error <= self.arrival_radius
            return math.atan2(math.cos(points[0].yaw), math.sin(points[0].yaw))
        upper = min(self.cumulative[-1], self.progress + max(1e-6, speed*dt*2.0))
        lower = max(0.0, self.progress-10.0)
        best = None
        for index, (a, b) in enumerate(zip(points, points[1:])):
            begin, end = self.cumulative[index:index+2]
            if end < lower or begin > upper:
                continue
            length = end-begin
            if length <= 1e-10:
                continue
            dx, dy = b.x-a.x, b.y-a.y
            t = ((x-a.x)*dx + (y-a.y)*dy) / (length*length)
            t = min(min(1.0, (upper-begin)/length),
                    max(max(0.0, (lower-begin)/length), t))
            px, py = a.x+t*dx, a.y+t*dy
            distance = math.hypot(x-px, y-py)
            candidate = (distance, begin+t*length)
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is not None:
            self.cross_track_error = best[0]
            self.progress = max(self.progress, best[1])
        total = self.cumulative[-1]
        self.reached = (
            total-self.progress <= max(30.0, self.arrival_radius*2.0)
            and math.hypot(x-self.route.goal[0], y-self.route.goal[1]) <= self.arrival_radius
        )
        target_s = min(total, self.progress+lookahead)
        for index in range(len(points)-1):
            if self.cumulative[index+1] >= target_s:
                a, b = points[index:index+2]
                length = self.cumulative[index+1]-self.cumulative[index]
                fraction = 0.0 if length <= 1e-10 else (target_s-self.cumulative[index])/length
                tx, ty = a.x+(b.x-a.x)*fraction, a.y+(b.y-a.y)*fraction
                if math.hypot(tx-x, ty-y) < 1e-6:
                    return math.atan2(math.cos(b.yaw), math.sin(b.yaw))
                return math.atan2(tx-x, ty-y)
        return math.atan2(math.cos(points[-1].yaw), math.sin(points[-1].yaw))
