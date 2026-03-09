from __future__ import annotations

from collections import deque
import math

from robot_obstacle_avoidance.algorithms.a_star import AStarPlanner
from robot_obstacle_avoidance.algorithms.base import Planner
from robot_obstacle_avoidance.environment import Environment
from robot_obstacle_avoidance.geometry import clamp, distance, heading_to, integrate_unicycle, wrap_angle
from robot_obstacle_avoidance.models import Pose, RobotState, Scenario


class DynamicWindowApproachPlanner(Planner):
    name = "dwa"

    def __init__(self) -> None:
        self.max_speed = 1.3
        self.max_yaw_rate = 1.8
        self.max_accel = 0.8
        self.max_yaw_accel = 2.0
        self.predict_time = 2.0
        self.velocity_resolution = 0.1
        self.yaw_rate_resolution = 0.12
        self.heading_weight = 0.18
        self.clearance_weight = 0.14
        self.speed_weight = 0.10
        self.progress_weight = 0.22
        self.goal_distance_weight = 0.14
        self.guide_progress_weight = 0.22
        self.guide_lookahead = 0.8
        self.previous_goal_distance: float | None = None
        self.stagnation_steps = 0
        self.guide_waypoints: list[Pose] = []
        self.current_waypoint_index = 0
        self.position_history: deque[tuple[float, float]] = deque(maxlen=14)
        self.supervisor_planner: AStarPlanner | None = None
        self.last_runtime_obstacle_count = 0

    def reset(self, scenario: Scenario, environment: Environment) -> None:
        self.scenario = scenario
        self.previous_goal_distance = None
        self.stagnation_steps = 0
        self.guide_waypoints = self._build_guide_waypoints(environment)
        self.current_waypoint_index = 0
        self.position_history.clear()
        self.supervisor_planner = AStarPlanner()
        self.supervisor_planner.reset(scenario, environment)
        self.last_runtime_obstacle_count = len(environment.active_runtime_obstacles())

    def next_state(self, state: RobotState, goal: Pose, environment: Environment, dt: float) -> RobotState:
        self._refresh_guidance(state, goal, environment)
        runtime_obstacle_count = len(environment.active_runtime_obstacles())
        self.position_history.append((state.x, state.y))
        self._advance_waypoint_index(state)
        guidance_target = self._guidance_target(goal, extra_hops=2 if self.stagnation_steps >= 6 else 0)
        goal_distance = environment.distance_to_goal(state)
        if self.previous_goal_distance is not None and goal_distance >= self.previous_goal_distance - 0.01:
            self.stagnation_steps += 1
        else:
            self.stagnation_steps = max(0, self.stagnation_steps - 2)
        if len(self.position_history) == self.position_history.maxlen and self._recent_displacement() < 0.18:
            self.stagnation_steps = max(self.stagnation_steps, 8)
        self.previous_goal_distance = goal_distance

        velocity_low = max(0.0, state.v - self.max_accel * dt)
        velocity_high = min(self.max_speed, state.v + self.max_accel * dt)
        yaw_low = max(-self.max_yaw_rate, state.omega - self.max_yaw_accel * dt)
        yaw_high = min(self.max_yaw_rate, state.omega + self.max_yaw_accel * dt)

        best_score, best_command = self._search_commands(
            state,
            guidance_target,
            goal,
            environment,
            dt,
            self._sample_range(velocity_low, velocity_high, self.velocity_resolution),
            self._sample_range(yaw_low, yaw_high, self.yaw_rate_resolution),
            self.predict_time,
        )

        if self.stagnation_steps >= 6 or best_command is None or best_command[0] < 0.08:
            recovery_score, recovery_command = self._search_recovery_commands(state, guidance_target, goal, environment, dt)
            if recovery_command is not None and recovery_score > best_score:
                best_score = recovery_score
                best_command = recovery_command

        if self.supervisor_planner is not None and runtime_obstacle_count > 0 and (self.stagnation_steps >= 4 or best_command is None):
            supervised_state = self.supervisor_planner.next_state(state, goal, environment, dt)
            if (
                environment.is_state_valid(supervised_state, self.scenario.robot_radius, time_offset=dt)
                and distance(supervised_state.x, supervised_state.y, state.x, state.y) > 1e-6
            ):
                return supervised_state

        if best_score == -float("inf") or best_command is None:
            desired_heading = heading_to(state.x, state.y, guidance_target.x, guidance_target.y)
            heading_error = wrap_angle(desired_heading - state.theta)
            recovery_yaw = clamp(1.8 * heading_error, -self.max_yaw_rate, self.max_yaw_rate)
            for recovery_speed in (0.18, 0.1, 0.0):
                recovery_state = integrate_unicycle(state, recovery_speed, recovery_yaw, dt)
                if environment.is_state_valid(recovery_state, self.scenario.robot_radius, time_offset=dt):
                    return recovery_state
            return integrate_unicycle(state, 0.0, recovery_yaw, dt)

        candidate = integrate_unicycle(state, best_command[0], best_command[1], dt)
        if environment.is_state_valid(candidate, self.scenario.robot_radius, time_offset=dt):
            return candidate

        recovery_score, recovery_command = self._search_recovery_commands(state, guidance_target, goal, environment, dt)
        if recovery_command is not None:
            recovery_state = integrate_unicycle(state, recovery_command[0], recovery_command[1], dt)
            if environment.is_state_valid(recovery_state, self.scenario.robot_radius, time_offset=dt):
                return recovery_state
        if self.supervisor_planner is not None and runtime_obstacle_count > 0:
            supervised_state = self.supervisor_planner.next_state(state, goal, environment, dt)
            if environment.is_state_valid(supervised_state, self.scenario.robot_radius, time_offset=dt):
                return supervised_state
        return integrate_unicycle(state, 0.0, best_command[1], dt)

    def _sample_range(self, lower: float, upper: float, resolution: float) -> list[float]:
        if upper <= lower:
            return [lower]

        samples = [lower]
        cursor = lower + resolution
        while cursor < upper:
            samples.append(cursor)
            cursor += resolution

        if abs(samples[-1] - upper) > 1e-9:
            samples.append(upper)
        return samples

    def _evaluate_trajectory(
        self,
        initial_state: RobotState,
        velocity: float,
        yaw_rate: float,
        guidance_target: Pose,
        goal: Pose,
        environment: Environment,
        dt: float,
        prediction_horizon: float,
    ) -> float:
        state = initial_state
        min_clearance = float("inf")
        elapsed = 0.0

        while elapsed < prediction_horizon:
            state = integrate_unicycle(state, velocity, yaw_rate, dt)
            time_offset = elapsed + dt
            if not environment.is_state_valid(state, self.scenario.robot_radius, time_offset=time_offset):
                return -float("inf")
            min_clearance = min(min_clearance, environment.robot_clearance_at(state.x, state.y, time_offset=time_offset))
            elapsed += dt

        initial_goal_distance = environment.distance_to_goal(initial_state)
        final_goal_distance = environment.distance_to_goal(state)
        initial_guide_distance = distance(initial_state.x, initial_state.y, guidance_target.x, guidance_target.y)
        final_guide_distance = distance(state.x, state.y, guidance_target.x, guidance_target.y)
        scenario_diagonal = math.hypot(self.scenario.width, self.scenario.height)
        target_heading = heading_to(state.x, state.y, guidance_target.x, guidance_target.y)
        heading_error = abs(wrap_angle(target_heading - state.theta))
        heading_score = 1.0 - clamp(heading_error / math.pi, 0.0, 1.0)
        clearance_score = clamp(min_clearance / 1.6, 0.0, 1.0)
        speed_score = clamp(velocity / self.max_speed, 0.0, 1.0)
        progress_score = clamp((initial_goal_distance - final_goal_distance) / max(initial_goal_distance, 1e-9), 0.0, 1.0)
        guide_progress_score = clamp((initial_guide_distance - final_guide_distance) / max(initial_guide_distance, 1e-9), 0.0, 1.0)
        goal_distance_score = 1.0 - clamp(final_goal_distance / max(scenario_diagonal, 1e-9), 0.0, 1.0)
        adaptive_clearance_weight = self.clearance_weight * clamp(final_goal_distance / 3.5, 0.35, 1.0)
        if self.stagnation_steps >= 6:
            adaptive_clearance_weight *= 0.75

        if final_goal_distance > self.scenario.goal_tolerance * 2.0 and velocity < 0.05:
            return -float("inf")

        return (
            self.heading_weight * heading_score
            + adaptive_clearance_weight * clearance_score
            + self.speed_weight * speed_score
            + self.progress_weight * progress_score
            + self.guide_progress_weight * guide_progress_score
            + self.goal_distance_weight * goal_distance_score
        )

    def _build_guide_waypoints(self, environment: Environment) -> list[Pose]:
        guide_planner = AStarPlanner()
        guide_planner.reset(self.scenario, environment)
        return guide_planner.waypoints or [self.scenario.goal]

    def _refresh_guidance(self, state: RobotState, goal: Pose, environment: Environment) -> None:
        runtime_obstacle_count = len(environment.active_runtime_obstacles())
        should_refresh = runtime_obstacle_count != self.last_runtime_obstacle_count or (runtime_obstacle_count > 0 and self.stagnation_steps >= 2)
        if self.supervisor_planner is not None and (runtime_obstacle_count > 0 or runtime_obstacle_count != self.last_runtime_obstacle_count):
            replanned = self.supervisor_planner.refresh_plan_from_state(state, goal, environment, force=should_refresh)
            if replanned or runtime_obstacle_count != self.last_runtime_obstacle_count:
                self.guide_waypoints = self.supervisor_planner.waypoints or [goal]
                self.current_waypoint_index = 0
        self.last_runtime_obstacle_count = runtime_obstacle_count

    def _advance_waypoint_index(self, state: RobotState) -> None:
        while self.current_waypoint_index < len(self.guide_waypoints) - 1:
            waypoint = self.guide_waypoints[self.current_waypoint_index]
            if distance(state.x, state.y, waypoint.x, waypoint.y) > self.guide_lookahead:
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

    def _search_commands(
        self,
        state: RobotState,
        guidance_target: Pose,
        goal: Pose,
        environment: Environment,
        dt: float,
        velocities: list[float],
        yaw_rates: list[float],
        prediction_horizon: float,
    ) -> tuple[float, tuple[float, float] | None]:
        best_score = -float("inf")
        best_command: tuple[float, float] | None = None
        for sampled_velocity in velocities:
            for sampled_yaw in yaw_rates:
                score = self._evaluate_trajectory(
                    state,
                    sampled_velocity,
                    sampled_yaw,
                    guidance_target,
                    goal,
                    environment,
                    dt,
                    prediction_horizon,
                )
                if score > best_score:
                    best_score = score
                    best_command = (sampled_velocity, sampled_yaw)
        return best_score, best_command

    def _search_recovery_commands(
        self,
        state: RobotState,
        guidance_target: Pose,
        goal: Pose,
        environment: Environment,
        dt: float,
    ) -> tuple[float, tuple[float, float] | None]:
        desired_heading = heading_to(state.x, state.y, guidance_target.x, guidance_target.y)
        heading_error = wrap_angle(desired_heading - state.theta)
        preferred_yaw = clamp(1.8 * heading_error, -self.max_yaw_rate, self.max_yaw_rate)
        yaw_candidates = sorted(
            {
                -self.max_yaw_rate,
                -1.2,
                -0.8,
                -0.4,
                0.0,
                0.4,
                0.8,
                1.2,
                self.max_yaw_rate,
                round(preferred_yaw, 3),
                round(preferred_yaw * 0.5, 3),
            }
        )
        velocity_candidates = [0.08, 0.16, 0.26, 0.38, 0.52]
        return self._search_commands(
            state,
            guidance_target,
            goal,
            environment,
            dt,
            velocity_candidates,
            yaw_candidates,
            max(0.9, self.predict_time * 0.6),
        )
