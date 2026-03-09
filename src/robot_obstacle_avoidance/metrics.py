from __future__ import annotations

from robot_obstacle_avoidance.geometry import distance, wrap_angle
from robot_obstacle_avoidance.models import RobotState


def path_length(trajectory: list[RobotState]) -> float:
    if len(trajectory) < 2:
        return 0.0
    total = 0.0
    for previous, current in zip(trajectory, trajectory[1:]):
        total += distance(previous.x, previous.y, current.x, current.y)
    return total


def smoothness(trajectory: list[RobotState]) -> float:
    if len(trajectory) < 3:
        return 0.0
    total = 0.0
    for first, second in zip(trajectory, trajectory[1:]):
        total += abs(wrap_angle(second.theta - first.theta))
    return total
