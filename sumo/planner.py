"""Hardware-independent closed-loop planner. Distances are mm, angles radians.

World axes: +x right, +y up; positive yaw is counterclockwise. A positive
wheel command means forward. This convention is shared with vision and demo.
"""

from collections import deque
from dataclasses import dataclass, field
import heapq
import math
import time


@dataclass(frozen=True)
class Vec:
    x: float
    y: float

    def __add__(self, other):
        return Vec(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vec(self.x * scalar, self.y * scalar)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def norm(self):
        return math.hypot(self.x, self.y)

    def unit(self):
        return self * (1.0 / max(self.norm(), 1e-9))

    def angle(self):
        return math.atan2(self.y, self.x)


def direction(angle):
    return Vec(math.cos(angle), math.sin(angle))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def segment_distance(point, start, end):
    delta = end - start
    t = max(0.0, min(1.0, (point - start).dot(delta) / max(delta.dot(delta), 1e-9)))
    return (point - (start + delta * t)).norm()


def rectangle_corners(position, heading, width, length):
    forward = direction(heading) * (length / 2)
    side = direction(heading + math.pi / 2) * (width / 2)
    return [position + forward + side, position + forward - side,
            position - forward - side, position - forward + side]


def rectangles_overlap(a, a_heading, a_width, a_length,
                       b, b_heading, b_width, b_length, margin=0):
    """Separating axis test; touching boxes are treated as separated.

    Expanding projections by margin conservatively inflates the forbidden area.
    Also works for the swept rectangle of a straight differential-drive motion.
    """
    af, al = direction(a_heading), direction(a_heading + math.pi / 2)
    bf, bl = direction(b_heading), direction(b_heading + math.pi / 2)
    delta = b - a
    for axis in (af, al, bf, bl):
        span = (abs(af.dot(axis)) * a_length / 2 + abs(al.dot(axis)) * a_width / 2
                + abs(bf.dot(axis)) * b_length / 2 + abs(bl.dot(axis)) * b_width / 2)
        if abs(delta.dot(axis)) >= span + margin - 1e-6:
            return False
    return True


@dataclass
class Pose:
    position: Vec
    heading: float
    timestamp: float
    velocity: Vec = field(default_factory=lambda: Vec(0, 0))


@dataclass
class Config:
    arena_width: float = 840.0   # Outer dimensions, including the white border.
    arena_height: float = 840.0
    white_border: float = 50.0
    robot_width: float = 180.0
    robot_length: float = 210.0
    enemy_width: float = 180.0  # Both robots confirmed 180 x 210 mm; independently editable.
    enemy_length: float = 210.0
    edge_margin: float = 25.0
    clearance: float = 12.0
    wheel_base: float = 120.0
    max_wheel_speed: float = 550.0  # Measured mm/s at PWM 255, unloaded estimate initially.
    max_pwm: int = 255
    min_pwm: int = 60
    left_gain: float = 1.0
    right_gain: float = 1.0
    stall_seconds: float = 1.3
    stall_distance: float = 22.0
    frame_timeout: float = 0.35

    @property
    def black_half_width(self):
        return self.arena_width / 2 - self.white_border

    @property
    def black_half_height(self):
        return self.arena_height / 2 - self.white_border

    @property
    def robot_radius(self):
        return math.hypot(self.robot_width, self.robot_length) / 2

    @property
    def enemy_radius(self):
        return math.hypot(self.enemy_width, self.enemy_length) / 2

    @property
    def contact(self):
        return self.robot_radius + self.enemy_radius

    def center_bounds(self, heading, extra=0):
        """Front stays in black; rear may use white up to the outer field edge.

        The feasible center rectangle is asymmetric and depends on heading.
        A rear corner on white is not an error or a reason to reverse.
        """
        c, s = math.cos(heading), math.sin(heading)
        fx, fy = c * self.robot_length / 2, s * self.robot_length / 2
        sx, sy = abs(s) * self.robot_width / 2, abs(c) * self.robot_width / 2
        bx, by = self.black_half_width - self.edge_margin - extra, self.black_half_height - self.edge_margin - extra
        ox, oy = self.arena_width / 2 - 4 - extra, self.arena_height / 2 - 4 - extra
        low_x, high_x = max(-bx - fx + sx, -ox + fx + sx), min(bx - fx - sx, ox + fx - sx)
        low_y, high_y = max(-by - fy + sy, -oy + fy + sy), min(by - fy - sy, oy + fy - sy)
        return Vec(low_x, low_y), Vec(high_x, high_y)

    def safe_limits(self, heading, extra=0):
        return self.center_bounds(heading, extra)[1]

    def inside(self, position, heading, extra=0):
        low, high = self.center_bounds(heading, extra)
        return low.x - 1e-6 <= position.x <= high.x + 1e-6 and low.y - 1e-6 <= position.y <= high.y + 1e-6

    def boundary_clearance(self, position, heading):
        """Distance from the nearest BODY corner to the black/white boundary."""
        return self.clearances(position, heading)[1] - self.white_border

    def clearances(self, position, heading):
        c, s = math.cos(heading), math.sin(heading)
        fx, fy = c * self.robot_length / 2, s * self.robot_length / 2
        sx, sy = abs(s) * self.robot_width / 2, abs(c) * self.robot_width / 2
        front = min(self.black_half_width - abs(position.x + fx) - sx,
                    self.black_half_height - abs(position.y + fy) - sy)
        outer = min(self.arena_width / 2 - abs(position.x) - abs(fx) - sx,
                    self.arena_height / 2 - abs(position.y) - abs(fy) - sy)
        return front, outer

    def front_clearance(self, position, heading):
        return self.clearances(position, heading)[0]

    def outer_clearance(self, position, heading):
        return self.clearances(position, heading)[1]

    def boundary_slack(self, position, heading):
        front, outer = self.clearances(position, heading)
        return min(front - self.edge_margin, outer - 4)

    def turn_inside(self, position, begin, end):
        finish = begin + wrap(end - begin)
        low, high = sorted((begin, finish))
        angles = [begin, finish]
        for alpha in (math.atan2(self.robot_width, self.robot_length),
                      math.atan2(self.robot_length, self.robot_width)):
            for k in range(math.floor(low / math.pi) - 1, math.ceil(high / math.pi) + 2):
                for value in (k * math.pi + alpha, k * math.pi - alpha):
                    if low <= value <= high:
                        angles.append(value)
        return all(self.inside(position, heading) for heading in angles)

    def outside(self, position):
        return abs(position.x) > self.arena_width / 2 or abs(position.y) > self.arena_height / 2

    def clip(self, position, heading, extra=0):
        low, high = self.center_bounds(heading, extra)
        return Vec(max(low.x, min(high.x, position.x)), max(low.y, min(high.y, position.y)))

    def boundary_target(self, position, ray, heading):
        low, high = self.center_bounds(heading)
        distances = []
        for value, component, lower, upper in ((position.x, ray.x, low.x, high.x), (position.y, ray.y, low.y, high.y)):
            if abs(component) > 1e-8:
                distances.append(((upper if component > 0 else lower) - value) / component)
        return position + ray * max(0, min(distances, default=0))

    def contact_distance(self, own_heading, enemy_heading, ray, margin=0):
        """Exact center separation at first OBB contact along a given ray."""
        af, al = direction(own_heading), direction(own_heading + math.pi / 2)
        bf, bl = direction(enemy_heading), direction(enemy_heading + math.pi / 2)
        distances = []
        for axis in (af, al, bf, bl):
            projected = abs(ray.dot(axis))
            if projected < 1e-8:
                continue
            span = (abs(af.dot(axis)) * self.robot_length / 2 + abs(al.dot(axis)) * self.robot_width / 2
                    + abs(bf.dot(axis)) * self.enemy_length / 2 + abs(bl.dot(axis)) * self.enemy_width / 2)
            distances.append((span + margin) / projected)
        return min(distances)

    def validate(self):
        values = vars(self).values()
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in values):
            raise ValueError("All planner parameters must be finite and positive")
        if min(self.black_half_width, self.black_half_height) <= max(self.robot_radius, self.enemy_radius) + self.edge_margin:
            raise ValueError("Black area too small for the configured robot sizes and margin")
        if not 1 <= self.min_pwm <= self.max_pwm <= 255:
            raise ValueError("Require 1 <= min PWM <= max PWM <= 255")


@dataclass
class Plan:
    state: str
    reason: str
    left: int = 0
    right: int = 0
    path: list = field(default_factory=list)
    target: Vec | None = None
    side: int = 0
    assist: int = 0  # 0 vision/recovery, 1 vision + IR attack, 2 onboard autonomous states.
    approach: list = field(default_factory=list)  # Full tactical curve, ending at the opponent aim point.


class Planner:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.cfg.validate()
        self.reset()

    def reset(self):
        self.state = "FLANK"
        self.side = 0
        self.side_changed = -100.0
        self.retreat_target = None
        self.retreat_started = 0.0
        self.history = deque()
        self.last_plan = Plan("IDLE", "Waiting for Start")
        self.last_own = self.last_enemy = None
        self.estimated_own = None
        self.last_update = None
        self.route_at = -100.0
        self.guidance = []
        self.route_enemy = None
        self.reflank_until = 0.0
        self.blocked_at = None
        self.started_at = None
        self.search_started = None
        self.result_since = {"WIN": None, "LOSS": None}
        self.result_frames = {"WIN": set(), "LOSS": set()}
        self.result = None
        self.pressure_started = self.last_progress_at = None
        self.last_retreat_at = -1000.0
        self.progress_anchor = None

    def clip(self, point, heading=0, extra=0):
        return self.cfg.clip(point, heading, extra)

    def motion_clear(self, start, end, enemy):
        """Check the entire rectangle swept by one straight path segment."""
        cfg = self.cfg
        heading = (end - start).angle()
        if not cfg.inside(start, heading) or not cfg.inside(end, heading):
            return False
        return not rectangles_overlap((start + end) * 0.5, heading, cfg.robot_width,
                                      cfg.robot_length + (end - start).norm(),
                                      enemy.position, enemy.heading, cfg.enemy_width,
                                      cfg.enemy_length, cfg.clearance)

    def pivot_clear(self, point, begin, end, enemy):
        cfg = self.cfg
        if not cfg.turn_inside(point, begin, end):
            return False
        angle = wrap(end - begin)
        count = max(1, math.ceil(abs(angle) / math.radians(10)))
        # Pad samples by the maximum corner displacement between sample angles.
        # This bounds the continuous turn, rather than checking only its endpoints.
        padding = cfg.robot_radius * abs(angle) / count / 2
        for i in range(count + 1):
            heading = begin + angle * i / count
            if rectangles_overlap(point, heading, cfg.robot_width, cfg.robot_length,
                                  enemy.position, enemy.heading, cfg.enemy_width,
                                  cfg.enemy_length, 5 + padding):
                return False
        return True

    def route(self, own, goal, enemy, deadline=None):
        """A* over incoming/outgoing path edges for rectangular footprints.

        Each edge validates a swept body and each junction validates its turn.
        The black rectangular arena is convex, so endpoint footprint checks cover
        the whole straight segment. Heading is retained in each search state.
        """
        cfg = self.cfg
        start = own.position
        nodes = [start, goal]
        tangent = direction((enemy.position - goal).angle() + math.pi / 2)
        for sign in (-1, 1):
            for distance in (60, 120, 180, 240, 300):
                point = goal + tangent * (sign * distance)
                if cfg.inside(point, tangent.angle()):
                    nodes.append(point)
        # Let a robot close to a wall translate a little before its first pivot.
        for distance in (20, 40, 70):
            point = start + direction(own.heading) * distance
            if cfg.inside(point, own.heading):
                nodes.append(point)
        for i in range(32):
            angle = i * 2 * math.pi / 32
            ray = direction(angle)
            radius = cfg.contact_distance(angle + math.pi / 2, enemy.heading, ray, cfg.clearance) + 16
            point = enemy.position + ray * radius
            if cfg.inside(point, angle + math.pi / 2):
                nodes.append(point)
        initial = (-1, 0)
        distances, parents = {initial: 0.0}, {}
        queue = [((goal - start).norm(), 0.0, initial)]
        edges = {}
        while queue:
            if deadline is not None and time.perf_counter() >= deadline:
                return []
            _, cost, state = heapq.heappop(queue)
            if cost > distances[state]:
                continue
            previous, i = state
            incoming = own.heading if previous < 0 else (nodes[i] - nodes[previous]).angle()
            if i == 1:
                # At the final flank, turning can intentionally make side contact.
                # The whole turn must still stay off the white border.
                if not cfg.turn_inside(goal, incoming, (enemy.position - goal).angle()):
                    continue
                result = []
                while state != initial:
                    result.append(nodes[state[1]])
                    state = parents[state]
                return list(reversed(result))
            for j in range(1, len(nodes)):
                if j == i or j == previous:
                    continue
                a, b = nodes[i], nodes[j]
                outgoing = (b - a).angle()
                next_cost = cost + (b - a).norm() + abs(wrap(outgoing - incoming)) * 15
                next_state = (i, j)
                if next_cost >= distances.get(next_state, float("inf")):
                    continue
                key = tuple(sorted((i, j)))
                if key not in edges:
                    edges[key] = self.motion_clear(a, b, enemy)
                if not edges[key] or not self.pivot_clear(a, incoming, outgoing, enemy):
                    continue
                distances[next_state] = next_cost
                parents[next_state] = state
                heapq.heappush(queue, (next_cost + (goal - b).norm(), next_cost, next_state))
        return []

    def wheels(self, pose, target, reverse=False, pushing=False):
        """Pure pursuit seed: unequal forward wheels produce an actual arc."""
        delta = target - pose.position
        if delta.norm() < 3:
            return 0, 0
        error = wrap(delta.angle() + (math.pi if reverse else 0) - pose.heading)
        speed = self.cfg.max_pwm * (1.0 if pushing else 0.8)
        if reverse:
            speed = -self.cfg.max_pwm
        if abs(error) > 1.45:
            turn = math.copysign(self.cfg.min_pwm + 10, error)
            return round(-turn), round(turn)
        curvature = 2 * math.sin(error) / max(110, delta.norm())
        return self._pwm_pair(speed * (1 - curvature * self.cfg.wheel_base / 2),
                              speed * (1 + curvature * self.cfg.wheel_base / 2))

    def _pwm_pair(self, left, right):
        cfg = self.cfg
        left, right = left * cfg.left_gain, right * cfg.right_gain
        scale = max(1, abs(left) / cfg.max_pwm, abs(right) / cfg.max_pwm)
        def quantize(value):
            value /= scale
            if abs(value) < 8:
                return 0
            return round(math.copysign(max(cfg.min_pwm, min(cfg.max_pwm, abs(value))), value))
        return quantize(left), quantize(right)

    def advance(self, pose, left, right, dt, timestamp=None):
        """Exact constant-wheel integration, also used for short tag dropouts."""
        cfg = self.cfg
        v = (left + right) / 2 / 255 * cfg.max_wheel_speed
        omega = (right - left) / 255 * cfg.max_wheel_speed / cfg.wheel_base
        heading = pose.heading + omega * dt
        if abs(omega) < 1e-6:
            position = pose.position + direction(pose.heading) * (v * dt)
        else:
            radius = v / omega
            position = pose.position + Vec(math.sin(heading) - math.sin(pose.heading),
                                           math.cos(pose.heading) - math.cos(heading)) * radius
        return Pose(position, wrap(heading), pose.timestamp if timestamp is None else timestamp)

    def rollout(self, own, left, right, enemy=None, attack=False, recovery=False, horizon=0.45):
        """Validate the curved body sweep, not just the center or target point.

        Samples are padded by maximum inter-sample corner travel. An 80 ms tail
        covers the command lease/communication delay after the scoring horizon.
        """
        cfg = self.cfg
        dt = 0.04
        speed = abs(left + right) / 2 / 255 * cfg.max_wheel_speed
        omega = abs(right - left) / 255 * cfg.max_wheel_speed / cfg.wheel_base
        pad = (speed + omega * cfg.robot_radius) * dt / 2 + 1
        initial = cfg.boundary_slack(own.position, own.heading)
        pose, poses = own, [own]
        for i in range(math.ceil((horizon + 0.08) / dt)):
            pose = self.advance(pose, left, right, dt)
            front, outer = cfg.clearances(pose.position, pose.heading)
            clearance = min(front - cfg.edge_margin, outer - 4)
            if recovery:
                if outer < 3 or clearance < initial - 1 or (i >= 3 and clearance < initial + 1):
                    return []
            elif outer < 4 + pad or (not attack and front < cfg.edge_margin + pad):
                return []
            if enemy:
                # Track the moving obstacle. Intentional contact is allowed only
                # on our forward face; no reversing into the opponent to flank.
                lead = enemy.velocity * min((i + 1) * dt, 0.25)
                if lead.norm() > 60:
                    lead = lead.unit() * 60
                ep = enemy.position + lead
                overlap = rectangles_overlap(pose.position, pose.heading, cfg.robot_width,
                                             cfg.robot_length, ep, enemy.heading,
                                             cfg.enemy_width, cfg.enemy_length, cfg.clearance + pad)
                toward = ep - pose.position
                forward_contact = (attack and left + right > 0
                                   and direction(pose.heading).dot(toward.unit()) > 0.60)
                # Existing physical contact must allow a retreat to open space.
                escaping = (left + right < 0 and (ep - pose.position).norm()
                            > (enemy.position - own.position).norm() + 0.2)
                if overlap and not (forward_contact or escaping):
                    return []
            poses.append(pose)
        return poses

    def approach_curve(self, own, enemy, waypoints=()):
        """Round the selected route all the way to the opponent aim point.

        The endpoint on the enemy is an attack aim, not a promise that our center
        can occupy its tag. Validate body clearance until first intended contact;
        the final part of the line depicts the direction of the subsequent push.
        """
        cfg = self.cfg
        nodes = [own.position]
        for point in [*waypoints, enemy.position]:
            if (point - nodes[-1]).norm() > 2:
                nodes.append(point)
        if len(nodes) < 2:
            return nodes
        for rounding in (0.30, 0.18, 0.08, 0.0):
            tangents = []
            for i, point in enumerate(nodes):
                if i == 0:
                    tangent = direction(own.heading)
                    span = (nodes[1] - point).norm()
                elif i == len(nodes) - 1:
                    tangent = (point - nodes[i-1]).unit()
                    span = (point - nodes[i-1]).norm()
                else:
                    incoming, outgoing = point - nodes[i-1], nodes[i+1] - point
                    tangent = (incoming.unit() + outgoing.unit()).unit()
                    span = min(incoming.norm(), outgoing.norm())
                tangents.append(tangent * min(75, span * rounding))
            result, valid, contact = [nodes[0]], True, False
            for i, (a, b) in enumerate(zip(nodes, nodes[1:])):
                c, d = a + tangents[i], b - tangents[i+1]
                samples = max(4, math.ceil(((c-a).norm() + (d-c).norm() + (b-d).norm()) / 12))
                for step in range(1, samples + 1):
                    t = step / samples
                    u = 1 - t
                    point = a * (u**3) + c * (3*u*u*t) + d * (3*u*t*t) + b * (t**3)
                    tangent = (c-a) * (3*u*u) + (d-c) * (6*u*t) + (b-d) * (3*t*t)
                    heading = tangent.angle() if tangent.norm() > .01 else (b-a).angle()
                    overlap = rectangles_overlap(point, heading, cfg.robot_width, cfg.robot_length,
                                                 enemy.position, enemy.heading, cfg.enemy_width, cfg.enemy_length, 2)
                    if i == len(nodes)-2 and overlap:
                        contact = True
                    if not contact and (not cfg.inside(point, heading) or overlap):
                        valid = False
                        break
                    result.append(point)
                if not valid:
                    break
            if valid:
                result[-1] = enemy.position
                return result
        # A tight route can require a pivot. Keep that honest in the display
        # instead of drawing a smoothed shortcut through the opponent or edge.
        return nodes

    def _motion(self, state, reason, own, target, enemy=None, attack=False,
                reverse=False, recovery=False, assist=0):
        cfg = self.cfg
        error = wrap((target - own.position).angle() + (math.pi if reverse else 0) - own.heading)
        candidates = {self.wheels(own, target, reverse, attack)}
        # Forward arcs are preferred. Pivots are a fallback for targets behind us
        # or a genuinely tight corridor, never a mandatory waypoint maneuver.
        fractions = (1.0,) if attack or reverse else (0.35, 0.55, 0.8, 1.0)
        for fraction in fractions:
            speed = cfg.max_pwm * fraction * (-1 if reverse else 1)
            for turn in (-1.35, -0.85, -0.45, 0, 0.45, 0.85, 1.35):
                candidates.add(self._pwm_pair(speed * (1 - turn), speed * (1 + turn)))
        if not reverse:
            for sign in (-1, 1):
                candidates.add((-sign * cfg.min_pwm, sign * cfg.min_pwm))
        best = None
        for left, right in sorted(candidates):
            # During contact the live front-floor sensors veto white immediately.
            # Validate a short full-power lease instead of slowing merely because
            # a long forecast would eventually touch the white border.
            horizon = 0.12 if reverse else 0.10 if attack else 0.35
            poses = self.rollout(own, left, right, enemy, attack, recovery, horizon)
            if not poses:
                continue
            end = poses[min(len(poses) - 1, round(horizon / 0.04))]
            distance = (target - end.position).norm()
            heading_error = abs(wrap((target - end.position).angle()
                                    + (math.pi if reverse else 0) - end.heading))
            translating = abs(left + right) / 2
            pivot = left * right < 0
            score = distance + heading_error * 48
            score += 45 if pivot and abs(error) < 1.5 else 8 if pivot else 0
            score += (abs(left - self.last_plan.left) + abs(right - self.last_plan.right)) * 0.025
            score -= translating * 0.07
            if attack or reverse:
                score += (cfg.max_pwm - translating) * 0.65
            if recovery:
                score -= cfg.boundary_slack(end.position, end.heading) * 2
            if best is None or score < best[0]:
                best = (score, left, right, poses)
        if best is None:
            if attack:
                self.blocked_at = None
                self.last_plan = self._search(own, enemy, self.last_update or own.timestamp, False, False, {})
                self.last_plan.state = "PRESSURE_IR"
                self.last_plan.reason = "Keep contact pressure with onboard IR; do not retreat for a forecasted blocked arc"
                if enemy:
                    self.last_plan.approach = self.approach_curve(own, enemy)
                self.state = "PRESSURE_IR"
                return self.last_plan
            if self.blocked_at is None:
                self.blocked_at = self.last_update
            if not recovery:
                inward = self.clip(own.position * .5, own.heading, 10)
                backward = (inward - own.position).dot(direction(own.heading)) < 0
                return self._motion("MAKE_SPACE", "Open turning room near edge; onboard IR remains available",
                                    own, inward, reverse=backward, recovery=True)
            self.last_plan = self._search(own, enemy, self.last_update or own.timestamp,
                                         False, True, {})
            self.last_plan.state = "LOCAL_ESCAPE"
            self.last_plan.reason = "Vision corridor constrained: onboard walking search / IR / floor states"
        else:
            self.blocked_at = None
            _, left, right, poses = best
            # Display the exact commanded arc. Global guidance is deliberately
            # separate from this prediction; it is re-evaluated on every frame.
            self.last_plan = Plan(state, reason, left, right,
                                  [p.position for p in poses], target, self.side, assist)
            if enemy and not reverse and not recovery:
                self.last_plan.approach = self.approach_curve(own, enemy, self.guidance if state == "FLANK" else ())
        self.state = state
        return self.last_plan

    def _search(self, own, enemy, now, own_missing, enemy_missing, sensors):
        if self.search_started is None:
            self.search_started = now
        fast = min(140, self.cfg.max_pwm)
        slow = round(fast * .8)
        leg = int((now - self.search_started) / .85) % 4
        left, right = ((fast, fast) if leg in (0, 3) else
                       (slow, fast) if leg == 1 else (fast, slow))
        # Mode 2 delegates actual search, IR tracking and floor avoidance to the
        # ESP32. These nominal wheels are for demo/visualisation only.
        self.last_plan = Plan("LOCAL_SEARCH", "Walking search on ESP32: IR/floor states; vision resumes automatically",
                              left, right, [own.position] if own else [], None, self.side, 2)
        return self.last_plan

    def _flank_route(self, own, enemy, now):
        cfg = self.cfg
        changed = (self.route_enemy is None or (enemy.position - self.route_enemy.position).norm() > 35
                   or abs(wrap(enemy.heading - self.route_enemy.heading)) > 0.25)
        if now - self.route_at < 0.30 and not changed:
            return self.guidance
        candidates = []
        deadline = time.perf_counter() + 0.040
        for side in (-1, 1):
            # Include front quarters: a rear stance outside the arena is simply
            # discarded, rather than making it a compulsory objective.
            for degrees in (65, 100, 140, 180):
                if time.perf_counter() >= deadline:
                    break
                ray = direction(enemy.heading + side * math.radians(degrees))
                facing = wrap(ray.angle() + math.pi)
                spacing = cfg.contact_distance(facing, enemy.heading, ray) + 42
                goal = enemy.position + ray * spacing
                if not cfg.inside(goal, facing):
                    continue
                path = self.route(own, goal, enemy, deadline)
                if not path:
                    continue
                length = sum((b - a).norm() for a, b in zip([own.position, *path], path))
                outward = (enemy.position - goal).unit().dot(enemy.position.unit())
                score = length - outward * 90 + max(0, math.cos(math.radians(degrees))) * 45
                score += abs(wrap((path[0] - own.position).angle() - own.heading)) * 25
                if self.side and side != self.side:
                    score += 65 if now - self.side_changed < 0.8 else 20
                candidates.append((score, side, path))
        self.route_at, self.route_enemy = now, enemy
        if candidates:
            _, side, self.guidance = min(candidates, key=lambda c: c[0])
            if side != self.side:
                self.side, self.side_changed = side, now
        else:
            self.guidance = []
        return self.guidance

    def update(self, own, enemy, now, sensors=None):
        cfg = self.cfg
        sensors = sensors or {}
        for pose in (own, enemy):
            if pose is not None and not all(math.isfinite(v) for v in
                    (pose.position.x, pose.position.y, pose.heading, pose.timestamp,
                     pose.velocity.x, pose.velocity.y)):
                return Plan("INVALID", "Invalid pose")
            if pose is not None and pose.timestamp > now + 0.05:
                return Plan("INVALID", "Observation timestamp is in the future")
        dt = min(0.10, max(0, now - self.last_update)) if self.last_update is not None else 0
        self.last_update = now
        own_missing = own is None or now - own.timestamp > cfg.frame_timeout
        enemy_missing = enemy is None or now - enemy.timestamp > cfg.frame_timeout
        if not own_missing:
            self.last_own = own
            self.estimated_own = own
        elif self.estimated_own is not None:
            self.estimated_own = self.advance(self.estimated_own,
                                            sensors.get("left", self.last_plan.left),
                                            sensors.get("right", self.last_plan.right), dt)
        if not enemy_missing:
            self.last_enemy = enemy
        if own_missing or enemy_missing:
            self.history.clear()
            self.result_since = {"WIN": None, "LOSS": None}
            self.result_frames = {"WIN": set(), "LOSS": set()}
            return self._search(self.estimated_own, self.last_enemy, now, own_missing, enemy_missing, sensors)
        if self.search_started is not None:
            self.search_started = None
            self.route_at = -100
        if self.result:
            return Plan(self.result, "Match finished: confirmed tag outside the arena")
        for result, pose in (("LOSS", own), ("WIN", enemy)):
            if cfg.outside(pose.position):
                if self.result_since[result] is None:
                    self.result_since[result] = now
                self.result_frames[result].add(pose.timestamp)
                if now - self.result_since[result] >= .35 and len(self.result_frames[result]) >= 3:
                    self.result = result
                    return Plan(result, "Match finished: confirmed tag outside the arena")
                return Plan("RESULT_CHECK", "Confirming tag outside arena across consecutive fresh frames")
            self.result_since[result] = None
            self.result_frames[result].clear()
        if self.started_at is None:
            self.started_at = now
        clearance = cfg.boundary_slack(own.position, own.heading)
        if clearance < 0 or (self.state == "EDGE_RETURN" and clearance < 8):
            self.history.clear()
            target = self.clip(own.position, own.heading, 40)
            reverse = (target - own.position).dot(direction(own.heading)) < 0
            return self._motion("EDGE_RETURN", "Front approaches white or body approaches outer edge; return inward",
                                own, target, reverse=reverse, recovery=True)

        delta = enemy.position - own.position
        separation = cfg.contact_distance(own.heading, enemy.heading, delta.unit())
        distance = delta.norm()
        ir = int(sensors.get("ir", 0)) & 31
        trying_forward = (sensors.get("left", self.last_plan.left)
                          + sensors.get("right", self.last_plan.right)) / 2 > cfg.min_pwm * 0.8
        contact = distance < separation + 55 or bool(ir & 14)
        if contact and trying_forward and self.state != "RETREAT":
            if self.pressure_started is None:
                self.pressure_started = self.last_progress_at = now
                self.progress_anchor = enemy.position
            self.history.append((now, own.position, enemy.position))
            while len(self.history) > 1 and self.history[1][0] <= now - cfg.stall_seconds:
                self.history.popleft()
        else:
            self.history.clear()
            self.pressure_started = self.last_progress_at = None
            self.progress_anchor = None
        progress = False
        if self.history:
            _, old_own, old_enemy = self.history[0]
            anchor = self.progress_anchor or old_enemy
            push_progress = max((enemy.position - old_enemy).dot(delta.unit()),
                                (enemy.position - anchor).dot(delta.unit()))
            old_edge = min(cfg.arena_width / 2 - abs(old_enemy.x), cfg.arena_height / 2 - abs(old_enemy.y))
            new_edge = min(cfg.arena_width / 2 - abs(enemy.position.x), cfg.arena_height / 2 - abs(enemy.position.y))
            threshold = max(5, cfg.stall_distance * .30)
            progress = push_progress >= threshold or old_edge - new_edge >= threshold
            if progress:
                self.last_progress_at = now
                self.progress_anchor = enemy.position
        enemy_corners = rectangle_corners(enemy.position, enemy.heading, cfg.enemy_width, cfg.enemy_length)
        enemy_edge = min(min(cfg.arena_width / 2 - abs(p.x), cfg.arena_height / 2 - abs(p.y)) for p in enemy_corners)
        finishing = enemy_edge < cfg.white_border + 15
        stall_wait = max(3.2, cfg.stall_seconds * 2.5) if finishing else max(2.0, cfg.stall_seconds)
        cooldown = 6.0 if finishing else 4.0
        stalled = (bool(self.history) and now - self.history[0][0] >= cfg.stall_seconds
                   and self.last_progress_at is not None and now - self.last_progress_at >= stall_wait
                   and not progress and now - self.last_retreat_at >= cooldown
                   and (own.position - self.history[0][1]).norm() < cfg.stall_distance
                   and (enemy.position - self.history[0][2]).norm() < cfg.stall_distance)
        blocked = self.blocked_at is not None and now - self.blocked_at > 0.3
        if stalled or blocked:
            self.state, self.retreat_started = "RETREAT", now
            self.last_retreat_at = now
            self.retreat_target = self.clip(own.position - direction(own.heading) * 85, own.heading)
            self.side = -self.side if self.side else 1
            self.side_changed = now
            self.guidance = []
            self.history.clear()
            self.blocked_at = None
        if self.state == "RETREAT":
            if ((self.retreat_target - own.position).norm() > 22 and now - self.retreat_started < 0.70
                    and (self.retreat_target - own.position).dot(direction(own.heading)) < -15):
                return self._motion("RETREAT", "Stall confirmed: brief withdrawal, then curve to another angle",
                                    own, self.retreat_target, enemy, reverse=True)
            self.reflank_until, self.state = now + 0.9, "FLANK"
            self.route_at = -100

        lead = enemy.velocity * 0.22
        if lead.norm() > 70:
            lead = lead.unit() * 70
        predicted = Pose(enemy.position + lead, enemy.heading, now, enemy.velocity)
        aligned = direction(own.heading).dot(delta.unit()) > 0.60
        frontal = direction(enemy.heading).dot((own.position - enemy.position).unit()) > 0.45
        # Immediate real contact takes precedence over a distant flank waypoint.
        immediate = aligned and distance < separation + 90
        opening = now - self.started_at < 2.0
        actual_contact = distance < separation + 18
        if ((ir and distance < separation + 200)
                or (immediate and now >= self.reflank_until and (not opening or actual_contact))):
            state = "ATTACK_FRONT" if frontal else "PUSH"
            reason = ("Opponent advancing toward edge: maintain pressure" if progress else
                      "Opponent near drop: finish the push before considering retreat" if finishing else
                      "IR contact: close and steer into target" if ir else
                      "Available contact: drive through, do not wait at a waypoint")
            return self._motion(state, reason, own, predicted.position, enemy, attack=True, assist=1)

        path = self._flank_route(own, predicted, now)
        if path:
            # Remove passed waypoints and look through the next corner. The local
            # arc search validates the actual sweep; it never requires arrival at
            # one exact point before starting the next maneuver.
            while len(path) > 1 and (path[0] - own.position).norm() < 85:
                path.pop(0)
            if (path[-1] - own.position).norm() < 85:
                return self._motion("INTERCEPT", "Flank reached: immediately close the gap", own,
                                    predicted.position, enemy, attack=True, assist=1)
            target = path[0]
            if len(path) > 1 and (target - own.position).norm() < 135:
                target = target + (path[1] - target).unit() * min(65, (path[1] - target).norm())
            plan = self._motion("FLANK", "Follow a body-safe arc; adjust to opponent motion and IR", own,
                                target, enemy, assist=1)
            if plan.state != "CLEARANCE_HOLD":
                return plan
        # No legal flank is not a reason to reverse indefinitely. Take the current
        # approach, including head-on pressure when the opponent's rear is at white.
        return self._motion("ATTACK_FRONT" if frontal else "INTERCEPT",
                            "No useful flank corridor: take the available direct attack",
                            own, predicted.position, enemy, attack=True, assist=1)
