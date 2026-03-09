from __future__ import annotations

from robot_obstacle_avoidance.algorithms import AStarPlanner, ArtificialPotentialFieldPlanner, DynamicWindowApproachPlanner
from robot_obstacle_avoidance.environment import Environment
from robot_obstacle_avoidance.metrics import path_length, smoothness
from robot_obstacle_avoidance.models import RobotState, RuntimeObstacleEvent, Scenario, SimulationResult


PLANNERS = {
    "astar": AStarPlanner,
    "apf": ArtificialPotentialFieldPlanner,
    "dwa": DynamicWindowApproachPlanner,
}


def available_algorithms() -> tuple[str, ...]:
    return tuple(PLANNERS.keys())


def run_all_simulations(scenario: Scenario, dt: float = 0.1) -> list[SimulationResult]:
    return [run_simulation(algorithm, scenario, dt=dt) for algorithm in available_algorithms()]


def run_simulation(algorithm: str, scenario: Scenario, dt: float = 0.1) -> SimulationResult:
    try:
        planner = PLANNERS[algorithm]()
    except KeyError as exc:
        raise ValueError(f"Unknown algorithm: {algorithm}") from exc

    environment = Environment(scenario)
    planner.reset(scenario, environment)
    state = RobotState(scenario.start.x, scenario.start.y, scenario.start.theta, 0.0, 0.0)
    trajectory = [RobotState(state.x, state.y, state.theta, state.v, state.omega)]
    dynamic_obstacle_history = [environment.dynamic_obstacle_snapshot_dicts()]
    collision = False
    success = False
    min_clearance = environment.robot_clearance(state.x, state.y)
    runtime_events = tuple(sorted(scenario.runtime_obstacle_events, key=lambda event: event.activate_time))
    next_runtime_event_index = 0

    for step in range(1, scenario.max_steps + 1):
        if environment.distance_to_goal(state) <= scenario.goal_tolerance:
            success = True
            result_steps = step - 1
            break

        next_runtime_event_index = _validate_runtime_obstacle_events(
            state,
            environment,
            runtime_events,
            next_runtime_event_index,
        )

        state = planner.next_state(state, scenario.goal, environment, dt)
        environment.advance_time(dt)
        current_clearance = environment.robot_clearance(state.x, state.y)
        min_clearance = min(min_clearance, current_clearance)
        trajectory.append(RobotState(state.x, state.y, state.theta, state.v, state.omega))
        dynamic_obstacle_history.append(environment.dynamic_obstacle_snapshot_dicts())

        if not environment.is_state_valid(state, scenario.robot_radius):
            collision = True
            result_steps = step
            break
    else:
        result_steps = scenario.max_steps

    if not collision and environment.distance_to_goal(state) <= scenario.goal_tolerance:
        success = True

    return SimulationResult(
        algorithm=algorithm,
        scenario=scenario.name,
        success=success,
        collision=collision,
        steps=result_steps,
        travel_time=result_steps * dt,
        path_length=path_length(trajectory),
        min_clearance=min_clearance,
        smoothness=smoothness(trajectory),
        final_distance_to_goal=environment.distance_to_goal(state),
        trajectory=trajectory,
        dynamic_obstacle_history=dynamic_obstacle_history,
        notes={
            "goal_tolerance": scenario.goal_tolerance,
            "robot_radius": scenario.robot_radius,
            "max_steps": scenario.max_steps,
            "dynamic_obstacle_count": len(scenario.dynamic_obstacles),
            "runtime_obstacle_event_count": len(scenario.runtime_obstacle_events),
        },
    )


def _validate_runtime_obstacle_events(
    state: RobotState,
    environment: Environment,
    runtime_events: tuple[RuntimeObstacleEvent, ...],
    next_runtime_event_index: int,
) -> int:
    while next_runtime_event_index < len(runtime_events):
        event = runtime_events[next_runtime_event_index]
        if event.activate_time > environment.current_time + 1e-9:
            break

        _validate_runtime_obstacle_event(
            event,
            state,
            environment,
            runtime_events[:next_runtime_event_index],
        )
        next_runtime_event_index += 1
    return next_runtime_event_index


def _validate_runtime_obstacle_event(
    event: RuntimeObstacleEvent,
    state: RobotState,
    environment: Environment,
    prior_runtime_events: tuple[RuntimeObstacleEvent, ...],
) -> None:
    obstacle = event.obstacle
    if not environment.obstacle_fits_in_bounds(obstacle):
        raise ValueError(f"Runtime obstacle '{event.label}' is out of bounds at t={event.activate_time:.2f}s")

    robot_buffer = environment.scenario.robot_radius + environment.scenario.safety_margin * 0.25
    if environment.distance_to_obstacle_surface(obstacle, state.x, state.y) <= robot_buffer:
        raise ValueError(
            f"Runtime obstacle '{event.label}' overlaps the robot at t={event.activate_time:.2f}s"
        )

    obstacle_buffer = environment.scenario.safety_margin * 0.15
    active_constraints = (
        environment.scenario.obstacles
        + tuple(runtime_event.obstacle for runtime_event in prior_runtime_events)
        + environment.active_dynamic_obstacle_snapshots()
    )
    for active_obstacle in active_constraints:
        if environment.distance_between_obstacles(obstacle, active_obstacle) <= obstacle_buffer:
            raise ValueError(
                f"Runtime obstacle '{event.label}' overlaps an active obstacle at t={event.activate_time:.2f}s"
            )

    reachability_planner = AStarPlanner()
    reachability_planner.reset(environment.scenario, environment)
    reachable_path = reachability_planner.compute_path(state.as_pose(), environment.scenario.goal, environment)
    if not reachable_path:
        raise ValueError(
            f"Runtime obstacle '{event.label}' would make the goal unreachable at t={event.activate_time:.2f}s"
        )
