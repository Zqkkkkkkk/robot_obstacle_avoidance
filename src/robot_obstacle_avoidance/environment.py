from __future__ import annotations

import math

from robot_obstacle_avoidance.geometry import clamp, distance
from robot_obstacle_avoidance.models import CircleObstacle, DynamicCircleObstacle, Obstacle, Pose, RectangleObstacle, RobotState, Scenario


class Environment:
    def __init__(self, scenario: Scenario) -> None:
        self.scenario = scenario
        self.current_time = 0.0

    def advance_time(self, dt: float) -> None:
        self.current_time += dt

    def active_obstacles(self, time_offset: float = 0.0) -> tuple[Obstacle, ...]:
        return self.scenario.obstacles + self.active_runtime_obstacles() + self.active_dynamic_obstacle_snapshots(time_offset=time_offset)

    def active_runtime_obstacles(self) -> tuple[Obstacle, ...]:
        return tuple(
            event.obstacle
            for event in self.scenario.runtime_obstacle_events
            if event.activate_time <= self.current_time + 1e-9
        )

    def active_dynamic_obstacle_snapshots(self, time_offset: float = 0.0) -> tuple[CircleObstacle, ...]:
        return tuple(self._dynamic_obstacle_snapshot(obstacle, time_offset=time_offset) for obstacle in self.scenario.dynamic_obstacles)

    def dynamic_obstacle_snapshot_dicts(self, time_offset: float = 0.0) -> list[dict[str, float | str | None]]:
        snapshots = []
        for source, snapshot in zip(self.scenario.dynamic_obstacles, self.active_dynamic_obstacle_snapshots(time_offset=time_offset)):
            snapshots.append(
                {
                    "kind": "dynamic_circle",
                    "x": snapshot.x,
                    "y": snapshot.y,
                    "radius": snapshot.radius,
                    "vx": source.vx,
                    "vy": source.vy,
                    "label": source.label,
                }
            )
        return snapshots

    def in_bounds(self, x: float, y: float, buffer: float = 0.0) -> bool:
        return buffer <= x <= self.scenario.width - buffer and buffer <= y <= self.scenario.height - buffer

    def obstacle_fits_in_bounds(self, obstacle: Obstacle, buffer: float = 0.0) -> bool:
        if isinstance(obstacle, CircleObstacle):
            return self.in_bounds(obstacle.x, obstacle.y, obstacle.radius + buffer)

        half_width = obstacle.width / 2.0 + buffer
        half_height = obstacle.height / 2.0 + buffer
        return (
            half_width <= obstacle.x <= self.scenario.width - half_width
            and half_height <= obstacle.y <= self.scenario.height - half_height
        )

    def distance_to_goal(self, state: RobotState | Pose) -> float:
        return distance(state.x, state.y, self.scenario.goal.x, self.scenario.goal.y)

    def distance_to_obstacle_surface(self, obstacle: Obstacle, x: float, y: float) -> float:
        if isinstance(obstacle, CircleObstacle):
            return distance(x, y, obstacle.x, obstacle.y) - obstacle.radius

        dx = abs(x - obstacle.x) - obstacle.width / 2.0
        dy = abs(y - obstacle.y) - obstacle.height / 2.0
        outside_distance = math.hypot(max(dx, 0.0), max(dy, 0.0))
        inside_distance = min(max(dx, dy), 0.0)
        return outside_distance + inside_distance

    def nearest_point_on_obstacle(self, obstacle: Obstacle, x: float, y: float) -> tuple[float, float]:
        if isinstance(obstacle, CircleObstacle):
            dx = x - obstacle.x
            dy = y - obstacle.y
            magnitude = math.hypot(dx, dy)
            if magnitude <= 1e-9:
                return obstacle.x + obstacle.radius, obstacle.y
            scale = obstacle.radius / magnitude
            return obstacle.x + dx * scale, obstacle.y + dy * scale

        return (
            clamp(x, obstacle.x - obstacle.width / 2.0, obstacle.x + obstacle.width / 2.0),
            clamp(y, obstacle.y - obstacle.height / 2.0, obstacle.y + obstacle.height / 2.0),
        )

    def obstacle_center(self, obstacle: Obstacle) -> tuple[float, float]:
        return obstacle.x, obstacle.y

    def distance_between_obstacles(self, first: Obstacle, second: Obstacle) -> float:
        if isinstance(first, CircleObstacle) and isinstance(second, CircleObstacle):
            return distance(first.x, first.y, second.x, second.y) - first.radius - second.radius

        if isinstance(first, RectangleObstacle) and isinstance(second, RectangleObstacle):
            dx = abs(first.x - second.x) - (first.width + second.width) / 2.0
            dy = abs(first.y - second.y) - (first.height + second.height) / 2.0
            outside_distance = math.hypot(max(dx, 0.0), max(dy, 0.0))
            inside_distance = min(max(dx, dy), 0.0)
            return outside_distance + inside_distance

        if isinstance(first, RectangleObstacle) and isinstance(second, CircleObstacle):
            first, second = second, first

        nearest_x = clamp(first.x, second.x - second.width / 2.0, second.x + second.width / 2.0)
        nearest_y = clamp(first.y, second.y - second.height / 2.0, second.y + second.height / 2.0)
        return distance(first.x, first.y, nearest_x, nearest_y) - first.radius

    def distance_to_nearest_obstacle_surface(self, x: float, y: float) -> float:
        active_obstacles = self.active_obstacles()
        if not active_obstacles:
            return float("inf")
        return min(self.distance_to_obstacle_surface(obstacle, x, y) for obstacle in active_obstacles)

    def distance_to_nearest_obstacle_surface_at(self, x: float, y: float, time_offset: float = 0.0) -> float:
        active_obstacles = self.active_obstacles(time_offset=time_offset)
        if not active_obstacles:
            return float("inf")
        return min(self.distance_to_obstacle_surface(obstacle, x, y) for obstacle in active_obstacles)

    def distance_to_nearest_boundary(self, x: float, y: float) -> float:
        return min(x, y, self.scenario.width - x, self.scenario.height - y)

    def distance_to_nearest_constraint(self, x: float, y: float) -> float:
        return min(self.distance_to_nearest_obstacle_surface(x, y), self.distance_to_nearest_boundary(x, y))

    def distance_to_nearest_constraint_at(self, x: float, y: float, time_offset: float = 0.0) -> float:
        return min(self.distance_to_nearest_obstacle_surface_at(x, y, time_offset=time_offset), self.distance_to_nearest_boundary(x, y))

    def robot_clearance(self, x: float, y: float) -> float:
        return self.distance_to_nearest_constraint(x, y) - self.scenario.robot_radius

    def robot_clearance_at(self, x: float, y: float, time_offset: float = 0.0) -> float:
        return self.distance_to_nearest_constraint_at(x, y, time_offset=time_offset) - self.scenario.robot_radius

    def is_state_valid(self, state: RobotState | Pose, buffer: float = 0.0, time_offset: float = 0.0) -> bool:
        return self.in_bounds(state.x, state.y, buffer) and self.distance_to_nearest_obstacle_surface_at(state.x, state.y, time_offset=time_offset) > buffer

    def pose_to_grid(self, pose: RobotState | Pose) -> tuple[int, int]:
        resolution = self.scenario.grid_resolution
        return int(round(pose.x / resolution)), int(round(pose.y / resolution))

    def grid_to_pose(self, cell: tuple[int, int]) -> Pose:
        resolution = self.scenario.grid_resolution
        return Pose(x=cell[0] * resolution, y=cell[1] * resolution, theta=0.0)

    def _dynamic_obstacle_snapshot(self, obstacle: DynamicCircleObstacle, time_offset: float = 0.0) -> CircleObstacle:
        absolute_time = self.current_time + time_offset
        min_x = obstacle.min_x if obstacle.min_x is not None else obstacle.radius
        max_x = obstacle.max_x if obstacle.max_x is not None else self.scenario.width - obstacle.radius
        min_y = obstacle.min_y if obstacle.min_y is not None else obstacle.radius
        max_y = obstacle.max_y if obstacle.max_y is not None else self.scenario.height - obstacle.radius
        x = self._reflected_coordinate(obstacle.x, obstacle.vx, min_x, max_x, absolute_time)
        y = self._reflected_coordinate(obstacle.y, obstacle.vy, min_y, max_y, absolute_time)
        return CircleObstacle(x=x, y=y, radius=obstacle.radius)

    def _reflected_coordinate(self, origin: float, velocity: float, lower: float, upper: float, time_value: float) -> float:
        if upper <= lower or abs(velocity) <= 1e-9:
            return clamp(origin, lower, upper)

        span = upper - lower
        period = 2.0 * span
        shifted = (origin - lower) + velocity * time_value
        wrapped = shifted % period
        if wrapped <= span:
            return lower + wrapped
        return upper - (wrapped - span)
