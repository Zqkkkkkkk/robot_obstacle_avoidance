from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, TypeAlias


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "theta": self.theta}


@dataclass
class RobotState:
    x: float
    y: float
    theta: float = 0.0
    v: float = 0.0
    omega: float = 0.0

    def as_pose(self) -> Pose:
        return Pose(self.x, self.y, self.theta)


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float

    kind: str = field(default="circle", init=False)

    def to_dict(self) -> dict[str, float | str]:
        return {"kind": self.kind, "x": self.x, "y": self.y, "radius": self.radius}


@dataclass(frozen=True)
class RectangleObstacle:
    x: float
    y: float
    width: float
    height: float

    kind: str = field(default="rectangle", init=False)

    def to_dict(self) -> dict[str, float | str]:
        return {
            "kind": self.kind,
            "x": self.x,
            "y": self.y,
            "width": self.width,
            "height": self.height,
        }


StaticObstacle: TypeAlias = CircleObstacle | RectangleObstacle


@dataclass(frozen=True)
class DynamicCircleObstacle:
    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0
    min_x: float | None = None
    max_x: float | None = None
    min_y: float | None = None
    max_y: float | None = None
    label: str = "dynamic"

    kind: str = field(default="dynamic_circle", init=False)

    def to_dict(self) -> dict[str, float | str | None]:
        return {
            "kind": self.kind,
            "x": self.x,
            "y": self.y,
            "radius": self.radius,
            "vx": self.vx,
            "vy": self.vy,
            "min_x": self.min_x,
            "max_x": self.max_x,
            "min_y": self.min_y,
            "max_y": self.max_y,
            "label": self.label,
        }


Obstacle: TypeAlias = StaticObstacle


@dataclass(frozen=True)
class RuntimeObstacleEvent:
    activate_time: float
    obstacle: Obstacle
    label: str = "runtime"

    kind: str = field(default="runtime_obstacle_event", init=False)

    def to_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind,
            "activate_time": self.activate_time,
            "label": self.label,
            "obstacle": self.obstacle.to_dict(),
        }


@dataclass(frozen=True)
class Scenario:
    name: str
    width: float
    height: float
    start: Pose
    goal: Pose
    obstacles: tuple[Obstacle, ...]
    dynamic_obstacles: tuple[DynamicCircleObstacle, ...] = ()
    runtime_obstacle_events: tuple[RuntimeObstacleEvent, ...] = ()
    max_steps: int = 300
    grid_resolution: float = 0.5
    goal_tolerance: float = 0.35
    robot_radius: float = 0.25
    safety_margin: float = 0.15

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "width": self.width,
            "height": self.height,
            "start": self.start.to_dict(),
            "goal": self.goal.to_dict(),
            "obstacles": [obstacle.to_dict() for obstacle in self.obstacles],
            "dynamic_obstacles": [obstacle.to_dict() for obstacle in self.dynamic_obstacles],
            "runtime_obstacle_events": [event.to_dict() for event in self.runtime_obstacle_events],
            "max_steps": self.max_steps,
            "grid_resolution": self.grid_resolution,
            "goal_tolerance": self.goal_tolerance,
            "robot_radius": self.robot_radius,
            "safety_margin": self.safety_margin,
        }


def obstacle_from_dict(payload: Mapping[str, object]) -> Obstacle:
    kind = str(payload.get("kind", "circle")).lower()
    x = _read_float(payload, "x")
    y = _read_float(payload, "y")

    if kind == "circle":
        radius = _read_positive_float(payload, "radius")
        return CircleObstacle(x=x, y=y, radius=radius)

    if kind == "rectangle":
        width = _read_positive_float(payload, "width")
        height = _read_positive_float(payload, "height")
        return RectangleObstacle(x=x, y=y, width=width, height=height)

    raise ValueError(f"Unsupported obstacle kind: {kind}")


def runtime_obstacle_event_from_dict(payload: Mapping[str, object]) -> RuntimeObstacleEvent:
    raw_activate_time = payload.get("activate_time")
    if raw_activate_time is None:
        raise ValueError("Runtime obstacle event field 'activate_time' is required")

    try:
        activate_time = float(raw_activate_time)
    except (TypeError, ValueError) as exc:
        raise ValueError("Runtime obstacle event field 'activate_time' must be a number") from exc

    if activate_time < 0.0:
        raise ValueError("Runtime obstacle event field 'activate_time' must be non-negative")

    raw_obstacle = payload.get("obstacle")
    if not isinstance(raw_obstacle, Mapping):
        raise ValueError("Runtime obstacle event field 'obstacle' must be an object")

    label = str(payload.get("label", "runtime"))
    return RuntimeObstacleEvent(
        activate_time=activate_time,
        obstacle=obstacle_from_dict(raw_obstacle),
        label=label,
    )


def _read_float(payload: Mapping[str, object], key: str) -> float:
    if key not in payload:
        raise ValueError(f"Obstacle field '{key}' is required")

    try:
        return float(payload[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Obstacle field '{key}' must be a number") from exc


def _read_positive_float(payload: Mapping[str, object], key: str) -> float:
    value = _read_float(payload, key)
    if value <= 0.0:
        raise ValueError(f"Obstacle field '{key}' must be greater than zero")
    return value


@dataclass
class SimulationResult:
    algorithm: str
    scenario: str
    success: bool
    collision: bool
    steps: int
    travel_time: float
    path_length: float
    min_clearance: float
    smoothness: float
    final_distance_to_goal: float
    trajectory: list[RobotState] = field(default_factory=list)
    dynamic_obstacle_history: list[list[dict[str, float | str | None]]] = field(default_factory=list)
    notes: dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        return {
            "algorithm": self.algorithm,
            "scenario": self.scenario,
            "success": self.success,
            "collision": self.collision,
            "steps": self.steps,
            "travel_time": self.travel_time,
            "path_length": self.path_length,
            "min_clearance": self.min_clearance,
            "smoothness": self.smoothness,
            "final_distance_to_goal": self.final_distance_to_goal,
            "trajectory": [
                {
                    "x": state.x,
                    "y": state.y,
                    "theta": state.theta,
                    "v": state.v,
                    "omega": state.omega,
                }
                for state in self.trajectory
            ],
            "dynamic_obstacle_history": self.dynamic_obstacle_history,
            "notes": self.notes,
        }
