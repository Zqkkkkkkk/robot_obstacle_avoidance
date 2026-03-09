from __future__ import annotations

import heapq
import math

from robot_obstacle_avoidance.algorithms.base import Planner
from robot_obstacle_avoidance.environment import Environment
from robot_obstacle_avoidance.geometry import clamp, distance, heading_to, integrate_unicycle, wrap_angle
from robot_obstacle_avoidance.models import Pose, RobotState, Scenario


class AStarPlanner(Planner):
    name = "astar"

    def __init__(self) -> None:
        self.max_speed = 1.0
        self.max_turn_rate = 1.5
        self.lookahead_distance = 0.4
        self.waypoints: list[Pose] = []
        self.current_waypoint_index = 0
        self.active_inflation_buffer = 0.0
        self.last_runtime_obstacle_count = 0

    def reset(self, scenario: Scenario, environment: Environment) -> None:
        self.scenario = scenario
        self.waypoints = self.compute_path(self.scenario.start, self.scenario.goal, environment) or [self.scenario.goal]
        self.current_waypoint_index = 0
        self.last_runtime_obstacle_count = len(environment.active_runtime_obstacles())

    def next_state(self, state: RobotState, goal: Pose, environment: Environment, dt: float) -> RobotState:
        self.refresh_plan_from_state(state, goal, environment)
        if not self.waypoints:
            return RobotState(state.x, state.y, state.theta, 0.0, 0.0)

        while self.current_waypoint_index < len(self.waypoints) - 1:
            waypoint = self.waypoints[self.current_waypoint_index]
            if distance(state.x, state.y, waypoint.x, waypoint.y) > self.lookahead_distance:
                break
            self.current_waypoint_index += 1

        target = self.waypoints[min(self.current_waypoint_index, len(self.waypoints) - 1)]
        desired_heading = heading_to(state.x, state.y, target.x, target.y)
        heading_error = wrap_angle(desired_heading - state.theta)
        yaw_rate = clamp(2.0 * heading_error, -self.max_turn_rate, self.max_turn_rate)
        corridor_clearance = max(environment.robot_clearance(state.x, state.y), 0.0)
        heading_scale = clamp(1.0 - abs(heading_error) / math.pi, 0.2, 1.0)
        clearance_scale = clamp(corridor_clearance / 1.0, 0.35, 1.0)
        speed = self.max_speed * heading_scale * clearance_scale
        candidate = integrate_unicycle(state, speed, yaw_rate, dt)
        if environment.is_state_valid(candidate, self.scenario.robot_radius, time_offset=dt):
            return candidate

        self.refresh_plan_from_state(state, goal, environment, force=True)
        if self.waypoints:
            target = self.waypoints[min(self.current_waypoint_index, len(self.waypoints) - 1)]
            desired_heading = heading_to(state.x, state.y, target.x, target.y)
            heading_error = wrap_angle(desired_heading - state.theta)
            yaw_rate = clamp(2.0 * heading_error, -self.max_turn_rate, self.max_turn_rate)
            candidate = integrate_unicycle(state, speed, yaw_rate, dt)
            if environment.is_state_valid(candidate, self.scenario.robot_radius, time_offset=dt):
                return candidate
        return integrate_unicycle(state, 0.0, yaw_rate, dt)

    def compute_path(self, start_pose: Pose, goal_pose: Pose, environment: Environment) -> list[Pose] | None:
        inflation_levels = [
            self.scenario.robot_radius + self.scenario.safety_margin,
            self.scenario.robot_radius + self.scenario.safety_margin * 0.5,
            self.scenario.robot_radius + self.scenario.safety_margin * 0.2,
            self.scenario.robot_radius,
        ]

        for inflation_buffer in inflation_levels:
            path = self._solve_with_inflation(environment, inflation_buffer, start_pose, goal_pose)
            if path:
                self.active_inflation_buffer = inflation_buffer
                return path

        return None

    def refresh_plan_from_state(self, state: RobotState, goal: Pose, environment: Environment, force: bool = False) -> bool:
        runtime_obstacle_count = len(environment.active_runtime_obstacles())
        needs_replan = force or runtime_obstacle_count != self.last_runtime_obstacle_count or self._path_is_blocked(state, environment)
        if not needs_replan:
            return False

        start_pose = Pose(state.x, state.y, state.theta)
        path = self.compute_path(start_pose, goal, environment)
        self.waypoints = path or [goal]
        self.current_waypoint_index = 0
        self.last_runtime_obstacle_count = runtime_obstacle_count
        return path is not None

    def _solve_with_inflation(
        self,
        environment: Environment,
        inflation_buffer: float,
        start_pose: Pose,
        goal_pose: Pose,
    ) -> list[Pose] | None:
        start = environment.pose_to_grid(start_pose)
        goal = environment.pose_to_grid(goal_pose)
        open_set: list[tuple[float, tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, start))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        closed: set[tuple[int, int]] = set()

        neighbor_offsets = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            if current == goal:
                return self._reconstruct_path(came_from, current, environment, start_pose, goal_pose)

            closed.add(current)
            current_pose = environment.grid_to_pose(current)
            for dx, dy in neighbor_offsets:
                neighbor = (current[0] + dx, current[1] + dy)
                neighbor_pose = environment.grid_to_pose(neighbor)
                if not environment.is_state_valid(neighbor_pose, inflation_buffer):
                    continue
                if not environment.in_bounds(neighbor_pose.x, neighbor_pose.y, self.scenario.robot_radius):
                    continue

                tentative = g_score[current] + distance(current_pose.x, current_pose.y, neighbor_pose.x, neighbor_pose.y)
                if tentative >= g_score.get(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heuristic = distance(neighbor_pose.x, neighbor_pose.y, goal_pose.x, goal_pose.y)
                heapq.heappush(open_set, (tentative + heuristic, neighbor))

        return None

    def _reconstruct_path(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        current: tuple[int, int],
        environment: Environment,
        start_pose: Pose,
        goal_pose: Pose,
    ) -> list[Pose]:
        cells = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()
        path = [environment.grid_to_pose(cell) for cell in cells]
        path[0] = start_pose
        if distance(path[-1].x, path[-1].y, goal_pose.x, goal_pose.y) > 1e-9:
            path.append(goal_pose)
        return path

    def _path_is_blocked(self, state: RobotState, environment: Environment) -> bool:
        if not self.waypoints:
            return True

        runtime_obstacle_count = len(environment.active_runtime_obstacles())
        if runtime_obstacle_count != self.last_runtime_obstacle_count:
            return True

        check_points = [Pose(state.x, state.y, state.theta)]
        final_index = min(len(self.waypoints), self.current_waypoint_index + 4)
        check_points.extend(self.waypoints[self.current_waypoint_index:final_index])
        for point in check_points:
            if not environment.is_state_valid(point, self.scenario.robot_radius):
                return True

        for start_point, end_point in zip(check_points, check_points[1:]):
            if not self._segment_is_clear(start_point, end_point, environment):
                return True

        return False

    def _segment_is_clear(self, start_point: Pose, end_point: Pose, environment: Environment) -> bool:
        segment_length = distance(start_point.x, start_point.y, end_point.x, end_point.y)
        if segment_length <= 1e-9:
            return True

        sample_spacing = max(self.scenario.grid_resolution * 0.5, 0.1)
        sample_count = max(2, int(math.ceil(segment_length / sample_spacing)))
        for sample_index in range(1, sample_count):
            ratio = sample_index / sample_count
            sample = Pose(
                x=start_point.x + (end_point.x - start_point.x) * ratio,
                y=start_point.y + (end_point.y - start_point.y) * ratio,
                theta=0.0,
            )
            if not environment.is_state_valid(sample, self.scenario.robot_radius):
                return False
        return True
