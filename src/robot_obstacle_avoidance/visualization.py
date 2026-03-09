from __future__ import annotations

from pathlib import Path

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

from robot_obstacle_avoidance.models import CircleObstacle, DynamicCircleObstacle, RectangleObstacle, Scenario, SimulationResult


COLORS = {
    "astar": "#0f766e",
    "apf": "#c2410c",
    "dwa": "#1d4ed8",
}


def create_result_figure(scenario: Scenario, result: SimulationResult) -> Figure:
    figure, axes = plt.subplots(figsize=(8, 8))
    _draw_environment(axes, scenario)
    _draw_trajectory(axes, result)
    axes.set_title(f"{result.algorithm.upper()} on {scenario.name}")
    _add_metrics_box(axes, [result])
    figure.tight_layout()
    return figure


def create_comparison_figure(scenario: Scenario, results: list[SimulationResult]) -> Figure:
    figure, axes = plt.subplots(figsize=(8.5, 8.5))
    _draw_environment(axes, scenario)
    for result in results:
        _draw_trajectory(axes, result)
    axes.set_title(f"Algorithm Comparison on {scenario.name}")
    _add_metrics_box(axes, results)
    figure.tight_layout()
    return figure


def save_result_plot(scenario: Scenario, result: SimulationResult, output_path: str) -> None:
    figure = create_result_figure(scenario, result)
    _save_figure(figure, output_path)


def save_comparison_plot(scenario: Scenario, results: list[SimulationResult], output_path: str) -> None:
    figure = create_comparison_figure(scenario, results)
    _save_figure(figure, output_path)


def save_result_animation(scenario: Scenario, result: SimulationResult, output_path: str, fps: int = 12) -> None:
    figure, animation = create_result_animation(scenario, result, fps=fps)
    _save_animation(figure, animation, output_path, fps=fps)


def save_comparison_animation(
    scenario: Scenario,
    results: list[SimulationResult],
    output_path: str,
    fps: int = 12,
) -> None:
    figure, animation = create_comparison_animation(scenario, results, fps=fps)
    _save_animation(figure, animation, output_path, fps=fps)


def show_result_plot(scenario: Scenario, result: SimulationResult) -> None:
    figure = create_result_figure(scenario, result)
    figure.show()
    plt.show()


def show_comparison_plot(scenario: Scenario, results: list[SimulationResult]) -> None:
    figure = create_comparison_figure(scenario, results)
    figure.show()
    plt.show()


def create_result_animation(scenario: Scenario, result: SimulationResult, fps: int = 12) -> tuple[Figure, FuncAnimation]:
    figure, axes = plt.subplots(figsize=(8, 8))
    _draw_environment(axes, scenario)
    axes.set_title(f"{result.algorithm.upper()} animated playback on {scenario.name}")
    _add_metrics_box(axes, [result])
    color = COLORS.get(result.algorithm, "#374151")
    line, = axes.plot([], [], color=color, linewidth=2.4, zorder=4, label=result.algorithm)
    marker, = axes.plot([], [], marker="o", color=color, markersize=7, zorder=6)
    dynamic_patches = _create_dynamic_patches(axes, result.dynamic_obstacle_history[0] if result.dynamic_obstacle_history else [])
    axes.legend(loc="lower right")
    animation = FuncAnimation(
        figure,
        func=lambda frame: _animate_single_frame(frame, result, line, marker, dynamic_patches),
        frames=_frame_count_for_result(result),
        interval=max(1, int(1000 / max(fps, 1))),
        blit=False,
        repeat=True,
    )
    figure.tight_layout()
    return figure, animation


def create_comparison_animation(
    scenario: Scenario,
    results: list[SimulationResult],
    fps: int = 12,
) -> tuple[Figure, FuncAnimation]:
    figure, axes = plt.subplots(figsize=(8.5, 8.5))
    _draw_environment(axes, scenario)
    axes.set_title(f"Animated comparison on {scenario.name}")
    _add_metrics_box(axes, results)

    lines: list[Line2D] = []
    markers: list[Line2D] = []
    for result in results:
        color = COLORS.get(result.algorithm, "#374151")
        line, = axes.plot([], [], color=color, linewidth=2.2, zorder=4, label=result.algorithm)
        marker, = axes.plot([], [], marker="o", color=color, markersize=6, zorder=6)
        lines.append(line)
        markers.append(marker)

    longest_history_result = max(results, key=lambda item: len(item.dynamic_obstacle_history), default=None)
    dynamic_patches = _create_dynamic_patches(
        axes,
        longest_history_result.dynamic_obstacle_history[0] if longest_history_result and longest_history_result.dynamic_obstacle_history else [],
    )
    axes.legend(loc="lower right")
    frame_count = max(_frame_count_for_result(result) for result in results)
    animation = FuncAnimation(
        figure,
        func=lambda frame: _animate_comparison_frame(frame, results, lines, markers, dynamic_patches),
        frames=frame_count,
        interval=max(1, int(1000 / max(fps, 1))),
        blit=False,
        repeat=True,
    )
    figure.tight_layout()
    return figure, animation


def show_result_animation(scenario: Scenario, result: SimulationResult, fps: int = 12) -> None:
    _, animation = create_result_animation(scenario, result, fps=fps)
    globals()["_active_animation"] = animation
    plt.show()


def show_comparison_animation(scenario: Scenario, results: list[SimulationResult], fps: int = 12) -> None:
    _, animation = create_comparison_animation(scenario, results, fps=fps)
    globals()["_active_animation"] = animation
    plt.show()


def _draw_environment(axes: Axes, scenario: Scenario) -> None:
    axes.add_patch(
        Rectangle((0.0, 0.0), scenario.width, scenario.height, fill=False, linewidth=2.0, edgecolor="#111827")
    )

    for obstacle in scenario.obstacles:
        if isinstance(obstacle, CircleObstacle):
            axes.add_patch(
                Circle((obstacle.x, obstacle.y), obstacle.radius, facecolor="#cbd5e1", edgecolor="#475569", linewidth=1.6)
            )
        elif isinstance(obstacle, RectangleObstacle):
            axes.add_patch(
                Rectangle(
                    (obstacle.x - obstacle.width / 2.0, obstacle.y - obstacle.height / 2.0),
                    obstacle.width,
                    obstacle.height,
                    facecolor="#e2e8f0",
                    edgecolor="#475569",
                    linewidth=1.6,
                    hatch="//",
                )
            )

    for dynamic_obstacle in scenario.dynamic_obstacles:
        _draw_dynamic_obstacle(axes, dynamic_obstacle.x, dynamic_obstacle.y, dynamic_obstacle.radius)

    axes.scatter([scenario.start.x], [scenario.start.y], c="#16a34a", s=90, marker="o", label="start", zorder=5)
    axes.scatter([scenario.goal.x], [scenario.goal.y], c="#dc2626", s=110, marker="*", label="goal", zorder=5)
    axes.set_xlim(-0.5, scenario.width + 0.5)
    axes.set_ylim(-0.5, scenario.height + 0.5)
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("x / m")
    axes.set_ylabel("y / m")
    axes.grid(True, alpha=0.25, linestyle="--")


def _draw_trajectory(axes: Axes, result: SimulationResult) -> None:
    xs = [state.x for state in result.trajectory]
    ys = [state.y for state in result.trajectory]
    color = COLORS.get(result.algorithm, "#374151")
    label = f"{result.algorithm} ({'ok' if result.success else 'fail'})"
    axes.plot(xs, ys, color=color, linewidth=2.2, label=label, zorder=4)
    axes.scatter([xs[-1]], [ys[-1]], color=color, s=40, zorder=6)


def _frame_count_for_result(result: SimulationResult) -> int:
    return max(2, len(result.trajectory))


def _animate_single_frame(
    frame: int,
    result: SimulationResult,
    line: Line2D,
    marker: Line2D,
    dynamic_patches: list[Circle],
) -> tuple[object, ...]:
    capped_frame = min(frame + 1, len(result.trajectory))
    xs = [state.x for state in result.trajectory[:capped_frame]]
    ys = [state.y for state in result.trajectory[:capped_frame]]
    line.set_data(xs, ys)
    marker.set_data([xs[-1]], [ys[-1]])
    _update_dynamic_patches(dynamic_patches, _snapshot_at(result.dynamic_obstacle_history, frame))
    return (line, marker, *dynamic_patches)


def _animate_comparison_frame(
    frame: int,
    results: list[SimulationResult],
    lines: list[Line2D],
    markers: list[Line2D],
    dynamic_patches: list[Circle],
) -> tuple[object, ...]:
    artists: list[object] = []
    for result, line, marker in zip(results, lines, markers):
        capped_frame = min(frame + 1, len(result.trajectory))
        xs = [state.x for state in result.trajectory[:capped_frame]]
        ys = [state.y for state in result.trajectory[:capped_frame]]
        line.set_data(xs, ys)
        marker.set_data([xs[-1]], [ys[-1]])
        artists.extend([line, marker])
    longest_history_result = max(results, key=lambda item: len(item.dynamic_obstacle_history), default=None)
    _update_dynamic_patches(
        dynamic_patches,
        _snapshot_at(longest_history_result.dynamic_obstacle_history if longest_history_result else [], frame),
    )
    artists.extend(dynamic_patches)
    return tuple(artists)


def _add_metrics_box(axes: Axes, results: list[SimulationResult]) -> None:
    summary_lines = []
    for result in results:
        summary_lines.append(
            f"{result.algorithm}: success={result.success}, path={result.path_length:.2f} m, clearance={result.min_clearance:.2f} m"
        )

    axes.text(
        1.02,
        0.98,
        "\n".join(summary_lines),
        transform=axes.transAxes,
        va="top",
        fontsize=9,
        bbox={"facecolor": "#f8fafc", "edgecolor": "#cbd5e1", "boxstyle": "round,pad=0.4"},
    )
    axes.legend(loc="lower right")


def _create_dynamic_patches(axes: Axes, snapshots: list[dict[str, float | str | None]]) -> list[Circle]:
    patches: list[Circle] = []
    for snapshot in snapshots:
        patch = Circle(
            (float(snapshot["x"]), float(snapshot["y"])),
            float(snapshot["radius"]),
            facecolor="#93c5fd",
            edgecolor="#2563eb",
            linewidth=2.0,
            alpha=0.85,
            linestyle="--",
            zorder=5,
        )
        axes.add_patch(patch)
        patches.append(patch)
    return patches


def _update_dynamic_patches(patches: list[Circle], snapshots: list[dict[str, float | str | None]]) -> None:
    for patch, snapshot in zip(patches, snapshots):
        patch.center = (float(snapshot["x"]), float(snapshot["y"]))
        patch.radius = float(snapshot["radius"])


def _snapshot_at(history: list[list[dict[str, float | str | None]]], frame: int) -> list[dict[str, float | str | None]]:
    if not history:
        return []
    return history[min(frame, len(history) - 1)]


def _draw_dynamic_obstacle(axes: Axes, x: float, y: float, radius: float) -> None:
    axes.add_patch(
        Circle(
            (x, y),
            radius,
            facecolor="#93c5fd",
            edgecolor="#2563eb",
            linewidth=2.0,
            linestyle="--",
            alpha=0.7,
            zorder=3,
        )
    )


def _save_figure(figure: Figure, output_path: str) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(figure)


def _save_animation(figure: Figure, animation: FuncAnimation, output_path: str, fps: int) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = PillowWriter(fps=max(fps, 1))
    animation.save(path, writer=writer)
    plt.close(figure)
