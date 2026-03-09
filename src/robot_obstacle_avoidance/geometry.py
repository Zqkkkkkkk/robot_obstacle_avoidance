from __future__ import annotations

import math

from robot_obstacle_avoidance.models import RobotState


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def heading_to(ax: float, ay: float, bx: float, by: float) -> float:
    return math.atan2(by - ay, bx - ax)


def integrate_unicycle(state: RobotState, velocity: float, yaw_rate: float, dt: float) -> RobotState:
    next_theta = wrap_angle(state.theta + yaw_rate * dt)
    next_x = state.x + velocity * math.cos(state.theta) * dt
    next_y = state.y + velocity * math.sin(state.theta) * dt
    return RobotState(x=next_x, y=next_y, theta=next_theta, v=velocity, omega=yaw_rate)
