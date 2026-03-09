from __future__ import annotations

from abc import ABC, abstractmethod

from robot_obstacle_avoidance.environment import Environment
from robot_obstacle_avoidance.models import Pose, RobotState, Scenario


class Planner(ABC):
    name: str

    @abstractmethod
    def reset(self, scenario: Scenario, environment: Environment) -> None:
        raise NotImplementedError

    @abstractmethod
    def next_state(self, state: RobotState, goal: Pose, environment: Environment, dt: float) -> RobotState:
        raise NotImplementedError
