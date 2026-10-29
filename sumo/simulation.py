"""Illustrative differential-drive demo, not a prediction of real pushing force."""

import math
from .planner import Pose, Vec, direction, wrap


class Simulation:
    def __init__(self, config):
        self.cfg = config
        self.reset()

    def reset(self, head_on=False):
        cfg = self.cfg
        contact = (cfg.robot_length + cfg.enemy_length) / 2
        self.own = Pose(Vec(-contact / 2 if head_on else -cfg.black_half_width * 0.38,
                            0 if head_on else -cfg.black_half_height * 0.54), 0, 0)
        self.enemy = Pose(Vec(contact / 2 if head_on else cfg.black_half_width * 0.22,
                              0 if head_on else cfg.black_half_height * 0.27), math.pi, 0)

    def step(self, left, right, dt, now, behavior=1):
        cfg = self.cfg
        dt = min(0.08, max(0, dt))
        a, b = left / 255 * cfg.max_wheel_speed, right / 255 * cfg.max_wheel_speed
        self.own.heading = wrap(self.own.heading + (b - a) / cfg.wheel_base * dt)
        old_own, old_enemy = self.own.position, self.enemy.position
        move = direction(self.own.heading) * ((a + b) / 2 * dt)
        self.own.position += move
        if behavior:
            desired = (self.own.position - self.enemy.position).angle()
            self.enemy.heading = wrap(self.enemy.heading + max(-1.5 * dt, min(1.5 * dt, wrap(desired - self.enemy.heading))))
        if behavior == 2:
            self.enemy.position += direction(self.enemy.heading) * (110 * dt)
        delta = self.enemy.position - self.own.position
        contact = cfg.contact_distance(self.own.heading, self.enemy.heading, delta.unit())
        if delta.norm() < contact:
            normal = delta.unit() if delta.norm() > 1 else Vec(1, 0)
            penetration = contact - delta.norm()
            frontal = direction(self.enemy.heading).dot(normal * -1) > 0.55
            if frontal:
                self.own.position -= normal * penetration
            else:
                self.enemy.position += normal * penetration * 0.80
                self.own.position -= normal * penetration * 0.20
        self.own.velocity = (self.own.position - old_own) * (1 / max(dt, 1e-6))
        self.enemy.velocity = (self.enemy.position - old_enemy) * (1 / max(dt, 1e-6))
        self.own.timestamp = self.enemy.timestamp = now
        return self.own, self.enemy
