from __future__ import annotations

from collections import deque
import math

from robot_obstacle_avoidance.algorithms.a_star import AStarPlanner
from robot_obstacle_avoidance.algorithms.base import Planner
from robot_obstacle_avoidance.environment import Environment
from robot_obstacle_avoidance.geometry import clamp, heading_to, integrate_unicycle, wrap_angle
from robot_obstacle_avoidance.models import Pose, RobotState, Scenario


class ArtificialPotentialFieldPlanner(Planner):
    name = "apf"

    def __init__(self) -> None:
        self.max_speed = 1.0
        self.max_turn_rate = 1.6
        self.attractive_gain = 1.0
        self.repulsive_gain = 0.82
        self.influence_radius = 2.0
        self.tangential_gain = 0.65
        self.guide_lookahead = 0.7
        self.previous_goal_distance: float | None = None
        self.stagnation_steps = 0
        self.escape_flip = 1.0
        self.guide_waypoints: list[Pose] = []
        self.current_waypoint_index = 0
        self.position_history: deque[tuple[float, float]] = deque(maxlen=14)
        self.escape_planner: AStarPlanner | None = None
        self.last_runtime_obstacle_count = 0

    def reset(self, scenario: Scenario, environment: Environment) -> None:
        self.scenario = scenario
        self.previous_goal_distance = None
        self.stagnation_steps = 0
        self.escape_flip = 1.0
        self.guide_waypoints = self._build_guide_waypoints(environment)
        self.current_waypoint_index = 0
        self.position_history.clear()
        self.escape_planner = AStarPlanner()
        self.escape_planner.reset(scenario, environment)
        self.last_runtime_obstacle_count = len(environment.active_runtime_obstacles())

    def next_state(self, state: RobotState, goal: Pose, environment: Environment, dt: float) -> RobotState:
        self._refresh_guidance(state, goal, environment)
        self.position_history.append((state.x, state.y))
        self._advance_waypoint_index(state)
        guidance_target = self._guidance_target(goal, extra_hops=2 if self.stagnation_steps >= 6 else 0)
        goal_distance = environment.distance_to_goal(state)
        if self.previous_goal_distance is not None and goal_distance >= self.previous_goal_distance - 0.01:
            self.stagnation_steps += 1
        else:
            self.stagnation_steps = max(0, self.stagnation_steps - 2)
        if len(self.position_history) == self.position_history.maxlen and self._recent_displacement() < 0.22:
            self.stagnation_steps = max(self.stagnation_steps, 8)
        self.previous_goal_distance = goal_distance

        att_x = self.attractive_gain * (guidance_target.x - state.x)
        att_y = self.attractive_gain * (guidance_target.y - state.y)
        guide_distance = math.hypot(guidance_target.x - state.x, guidance_target.y - state.y)
        if guide_distance > 1e-9:
            guide_unit_x = (guidance_target.x - state.x) / guide_distance
            guide_unit_y = (guidance_target.y - state.y) / guide_distance
        else:
            guide_unit_x = 0.0
            guide_unit_y = 0.0
        rep_x = 0.0
        rep_y = 0.0
        nearest_vector: tuple[float, float, float] | None = None
        nearest_surface_distance = float("inf")

        obstacle_samples: list[tuple[object, float]] = [(obstacle, 1.0) for obstacle in self.scenario.obstacles]
        for horizon_step, time_offset in enumerate((0.0, dt, 2.0 * dt, 3.0 * dt)):
            weight = 0.95 - 0.18 * horizon_step
            for obstacle in environment.active_dynamic_obstacle_snapshots(time_offset=time_offset):
                obstacle_samples.append((obstacle, weight))

        for obstacle, weight in obstacle_samples:
            surface_distance = environment.distance_to_obstacle_surface(obstacle, state.x, state.y) - self.scenario.robot_radius
            nearest_x, nearest_y = environment.nearest_point_on_obstacle(obstacle, state.x, state.y)
            dx = state.x - nearest_x
            dy = state.y - nearest_y
            center_distance = math.hypot(dx, dy)
            if center_distance <= 1e-9:
                obstacle_center_x, obstacle_center_y = environment.obstacle_center(obstacle)
                dx = state.x - obstacle_center_x
                dy = state.y - obstacle_center_y
                center_distance = math.hypot(dx, dy)
            if center_distance <= 1e-9:
                dx = 1.0
                dy = 0.0
                center_distance = 1.0
            if surface_distance < nearest_surface_distance:
                nearest_surface_distance = surface_distance
                nearest_vector = (dx, dy, center_distance)
            if 0.0 < surface_distance < self.influence_radius:
                effective_distance = max(surface_distance, 0.05)
                scale = weight * self.repulsive_gain * ((1.0 / effective_distance) - (1.0 / self.influence_radius)) / (effective_distance * effective_distance)
                rep_x += scale * (dx / center_distance)
                rep_y += scale * (dy / center_distance)

        if guide_distance > 1e-9:
            parallel_repulsion = rep_x * guide_unit_x + rep_y * guide_unit_y
            lateral_repulsion_x = rep_x - parallel_repulsion * guide_unit_x
            lateral_repulsion_y = rep_y - parallel_repulsion * guide_unit_y
            if self.stagnation_steps >= 4 and parallel_repulsion < 0.0:
                parallel_repulsion *= 0.35
            rep_x = lateral_repulsion_x + parallel_repulsion * guide_unit_x
            rep_y = lateral_repulsion_y + parallel_repulsion * guide_unit_y

        path_follow_gain = 0.22 if self.stagnation_steps < 4 else 0.55
        total_x = att_x + rep_x + path_follow_gain * guide_unit_x
        total_y = att_y + rep_y + path_follow_gain * guide_unit_y
        if nearest_vector is not None and (self.stagnation_steps >= 4 or nearest_surface_distance < self.influence_radius * 0.7):
            tangential_x, tangential_y = self._tangential_component(nearest_vector, goal, state)
            tangential_scale = clamp((self.influence_radius - max(nearest_surface_distance, 0.0)) / self.influence_radius, 0.0, 1.0)
            if self.stagnation_steps >= 10:
                self.escape_flip *= -1.0
                tangential_scale *= 1.35
            total_x += self.tangential_gain * tangential_scale * tangential_x * self.escape_flip
            total_y += self.tangential_gain * tangential_scale * tangential_y * self.escape_flip

        if abs(total_x) < 1e-9 and abs(total_y) < 1e-9:
            desired_heading = heading_to(state.x, state.y, guidance_target.x, guidance_target.y)
        else:
            desired_heading = math.atan2(total_y, total_x)

        heading_error = wrap_angle(desired_heading - state.theta)
        yaw_rate = clamp(2.2 * heading_error, -self.max_turn_rate, self.max_turn_rate)

        clearance = max(environment.robot_clearance_at(state.x, state.y, time_offset=dt), 0.0)
        clearance_scale = clamp(clearance / 1.5, 0.25, 1.0)
        heading_scale = clamp(1.0 - abs(heading_error) / math.pi, 0.2, 1.0)
        guide_heading = heading_to(state.x, state.y, guidance_target.x, guidance_target.y)
        guide_alignment = clamp(1.0 - abs(wrap_angle(guide_heading - state.theta)) / math.pi, 0.3, 1.0)
        speed = self.max_speed * clearance_scale * max(heading_scale, guide_alignment * 0.8)
        if self.stagnation_steps >= 5:
            speed = max(speed, 0.22)

        if self.escape_planner is not None and (self.stagnation_steps >= 3 or clearance < 0.8):
            guided_state = self.escape_planner.next_state(state, goal, environment, dt)
            if (
                environment.is_state_valid(guided_state, self.scenario.robot_radius, time_offset=dt)
                and math.hypot(guided_state.x - state.x, guided_state.y - state.y) > 1e-6
            ):
                return guided_state

        if self.stagnation_steps >= 6:
            if self.escape_planner is not None:
                escape_state = self.escape_planner.next_state(state, goal, environment, dt)
                if (
                    environment.is_state_valid(escape_state, self.scenario.robot_radius, time_offset=dt)
                    and math.hypot(escape_state.x - state.x, escape_state.y - state.y) > 1e-6
                ):
                    return escape_state

            escape_target = self._guidance_target(goal, extra_hops=4)
            escape_heading = heading_to(state.x, state.y, escape_target.x, escape_target.y)
            escape_error = wrap_angle(escape_heading - state.theta)
            escape_yaw = clamp(2.0 * escape_error, -self.max_turn_rate, self.max_turn_rate)
            escape_speed = max(0.28, self.max_speed * clearance_scale * clamp(1.0 - abs(escape_error) / math.pi, 0.22, 1.0))
            escape_candidate = integrate_unicycle(state, escape_speed, escape_yaw, dt)
            if environment.is_state_valid(escape_candidate, self.scenario.robot_radius, time_offset=dt):
                return escape_candidate

        candidate = integrate_unicycle(state, speed, yaw_rate, dt)
        if environment.is_state_valid(candidate, self.scenario.robot_radius, time_offset=dt):
            return candidate

        recovery_turn = self.max_turn_rate if heading_error >= 0.0 else -self.max_turn_rate
        recovery_commands = [
            (0.2, recovery_turn),
            (0.14, clamp(-0.7 * recovery_turn, -self.max_turn_rate, self.max_turn_rate)),
            (0.08, recovery_turn),
            (0.0, recovery_turn),
        ]
        for recovery_speed, recovery_yaw in recovery_commands:
            recovery_state = integrate_unicycle(state, recovery_speed, recovery_yaw, dt)
            if environment.is_state_valid(recovery_state, self.scenario.robot_radius, time_offset=dt):
                return recovery_state
        return integrate_unicycle(state, 0.0, recovery_turn, dt)

    def _build_guide_waypoints(self, environment: Environment) -> list[Pose]:
        guide_planner = AStarPlanner()
        guide_planner.reset(self.scenario, environment)
        return guide_planner.waypoints or [self.scenario.goal]

    def _refresh_guidance(self, state: RobotState, goal: Pose, environment: Environment) -> None:
        if self.escape_planner is None:
            return

        runtime_obstacle_count = len(environment.active_runtime_obstacles())
        should_refresh = runtime_obstacle_count != self.last_runtime_obstacle_count or self.stagnation_steps >= 2
        replanned = self.escape_planner.refresh_plan_from_state(state, goal, environment, force=should_refresh)
        if replanned or runtime_obstacle_count != self.last_runtime_obstacle_count:
            self.guide_waypoints = self.escape_planner.waypoints or [goal]
            self.current_waypoint_index = 0
            self.last_runtime_obstacle_count = runtime_obstacle_count

    def _advance_waypoint_index(self, state: RobotState) -> None:
        while self.current_waypoint_index < len(self.guide_waypoints) - 1:
            waypoint = self.guide_waypoints[self.current_waypoint_index]
            if math.hypot(state.x - waypoint.x, state.y - waypoint.y) > self.guide_lookahead:
                break
            self.current_waypoint_index += 1

    def _guidance_target(self, fallback_goal: Pose, extra_hops: int = 0) -> Pose:
        if not self.guide_waypoints:
            return fallback_goal
        target_index = min(self.current_waypoint_index + extra_hops, len(self.guide_waypoints) - 1)
        return self.guide_waypoints[target_index]

    def _recent_displacement(self) -> float:
        if len(self.position_history) < 2:
            return float("inf")
        start_x, start_y = self.position_history[0]
        end_x, end_y = self.position_history[-1]
        return math.hypot(end_x - start_x, end_y - start_y)

    def _tangential_component(
        self,
        nearest_vector: tuple[float, float, float],
        goal: Pose,
        state: RobotState,
    ) -> tuple[float, float]:
        dx, dy, distance_value = nearest_vector
        left = (-dy / distance_value, dx / distance_value)
        right = (dy / distance_value, -dx / distance_value)
        goal_vector = (goal.x - state.x, goal.y - state.y)
        left_alignment = left[0] * goal_vector[0] + left[1] * goal_vector[1]
        right_alignment = right[0] * goal_vector[0] + right[1] * goal_vector[1]
        return left if left_alignment >= right_alignment else right
