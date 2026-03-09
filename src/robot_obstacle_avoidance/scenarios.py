from __future__ import annotations

from dataclasses import replace

from robot_obstacle_avoidance.models import CircleObstacle, DynamicCircleObstacle, Obstacle, Pose, RectangleObstacle, RuntimeObstacleEvent, Scenario


def scenario_names() -> tuple[str, ...]:
    return ("open_field", "slalom", "narrow_passage", "dynamic_crossing")


def build_scenario(
    name: str,
    extra_obstacles: tuple[Obstacle, ...] | None = None,
    runtime_obstacle_events: tuple[RuntimeObstacleEvent, ...] | None = None,
) -> Scenario:
    if name == "open_field":
        scenario = Scenario(
            name=name,
            width=10.0,
            height=10.0,
            start=Pose(1.0, 1.0, 0.0),
            goal=Pose(9.0, 9.0, 0.0),
            obstacles=(
                RectangleObstacle(2.8, 6.9, 1.6, 0.8),
                CircleObstacle(4.9, 5.5, 0.68),
                RectangleObstacle(7.4, 2.7, 1.0, 1.2),
                CircleObstacle(7.7, 7.3, 0.4),
            ),
            max_steps=280,
            grid_resolution=0.5,
        )

    elif name == "slalom":
        scenario = Scenario(
            name=name,
            width=12.0,
            height=8.0,
            start=Pose(1.0, 4.0, 0.0),
            goal=Pose(11.0, 4.0, 0.0),
            obstacles=(
                CircleObstacle(2.9, 2.1, 0.85),
                RectangleObstacle(4.7, 5.7, 1.4, 1.0),
                CircleObstacle(6.6, 2.2, 0.9),
                RectangleObstacle(8.5, 5.6, 1.4, 1.0),
                CircleObstacle(9.7, 2.5, 0.55),
            ),
            max_steps=320,
            grid_resolution=0.4,
        )

    elif name == "narrow_passage":
        scenario = Scenario(
            name=name,
            width=12.0,
            height=10.0,
            start=Pose(1.2, 5.0, 0.0),
            goal=Pose(10.8, 5.0, 0.0),
            obstacles=(
                RectangleObstacle(4.5, 2.1, 2.0, 2.7),
                RectangleObstacle(4.5, 7.9, 2.0, 2.7),
                RectangleObstacle(7.5, 2.1, 2.0, 2.7),
                RectangleObstacle(7.5, 7.9, 2.0, 2.7),
                CircleObstacle(6.0, 5.0, 0.45),
            ),
            max_steps=360,
            grid_resolution=0.35,
        )

    elif name == "dynamic_crossing":
        scenario = Scenario(
            name=name,
            width=14.0,
            height=8.0,
            start=Pose(1.0, 4.0, 0.0),
            goal=Pose(13.0, 4.0, 0.0),
            obstacles=(
                RectangleObstacle(4.2, 0.9, 2.2, 1.2),
                RectangleObstacle(4.2, 7.1, 2.2, 1.2),
                RectangleObstacle(9.8, 0.9, 2.2, 1.2),
                RectangleObstacle(9.8, 7.1, 2.2, 1.2),
            ),
            dynamic_obstacles=(
                DynamicCircleObstacle(5.8, 2.1, 0.45, vx=0.0, vy=1.15, min_y=2.1, max_y=5.9, label="crosser_a"),
                DynamicCircleObstacle(8.2, 5.9, 0.48, vx=0.0, vy=-1.05, min_y=2.1, max_y=5.9, label="crosser_b"),
                DynamicCircleObstacle(10.6, 3.0, 0.36, vx=-0.95, vy=0.0, min_x=9.2, max_x=12.0, label="crosser_c"),
            ),
            max_steps=340,
            grid_resolution=0.35,
        )

    else:
        raise ValueError(f"Unknown scenario: {name}")

    updated_obstacles = scenario.obstacles + extra_obstacles if extra_obstacles else scenario.obstacles
    updated_runtime_events = runtime_obstacle_events if runtime_obstacle_events is not None else scenario.runtime_obstacle_events
    if updated_obstacles != scenario.obstacles or updated_runtime_events != scenario.runtime_obstacle_events:
        return replace(
            scenario,
            obstacles=updated_obstacles,
            runtime_obstacle_events=updated_runtime_events,
        )
    return scenario
