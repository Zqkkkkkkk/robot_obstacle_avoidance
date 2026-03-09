from __future__ import annotations

import argparse
import json
from typing import Any

from robot_obstacle_avoidance.scenarios import build_scenario, scenario_names
from robot_obstacle_avoidance.simulation import available_algorithms, run_all_simulations, run_simulation
from robot_obstacle_avoidance.visualization import (
    save_comparison_animation,
    save_comparison_plot,
    save_result_animation,
    save_result_plot,
    show_comparison_animation,
    show_comparison_plot,
    show_result_animation,
    show_result_plot,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run robot obstacle avoidance simulations.")
    parser.add_argument("--algorithm", choices=available_algorithms(), default="astar")
    parser.add_argument("--scenario", choices=scenario_names(), default="open_field")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation time step.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output.")
    parser.add_argument("--plot", action="store_true", help="Show a matplotlib plot.")
    parser.add_argument("--save-plot", type=str, help="Save a matplotlib figure to a file.")
    parser.add_argument("--compare-all", action="store_true", help="Run and plot all algorithms on the selected scenario.")
    parser.add_argument("--animate", action="store_true", help="Show an animated trajectory playback window.")
    parser.add_argument("--save-animation", type=str, help="Save an animated GIF to a file.")
    parser.add_argument("--fps", type=int, default=12, help="Animation playback frame rate.")
    return parser


def print_result(result: Any) -> None:
    if isinstance(result, list):
        for index, item in enumerate(result, start=1):
            if index > 1:
                print("-" * 40)
            print_result(item)
        return

    print(f"Algorithm: {result.algorithm}")
    print(f"Scenario: {result.scenario}")
    print(f"Success: {result.success}")
    print(f"Collision: {result.collision}")
    print(f"Steps: {result.steps}")
    print(f"Travel time: {result.travel_time:.2f} s")
    print(f"Path length: {result.path_length:.2f} m")
    print(f"Min clearance: {result.min_clearance:.2f} m")
    print(f"Smoothness: {result.smoothness:.2f} rad")
    print(f"Final distance to goal: {result.final_distance_to_goal:.2f} m")


def exit_code_for_results(results: list[Any]) -> int:
    return 0 if all(result.success for result in results) else 1


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    scenario = build_scenario(args.scenario)

    if args.compare_all:
        results = run_all_simulations(scenario, dt=args.dt)

        if args.json:
            print(json.dumps([result.to_dict() for result in results], indent=2))
        else:
            print_result(results)

        if args.save_plot:
            save_comparison_plot(scenario, results, args.save_plot)
            print(f"Saved comparison plot to: {args.save_plot}")

        if args.plot:
            show_comparison_plot(scenario, results)

        if args.save_animation:
            save_comparison_animation(scenario, results, args.save_animation, fps=args.fps)
            print(f"Saved comparison animation to: {args.save_animation}")

        if args.animate:
            show_comparison_animation(scenario, results, fps=args.fps)

        return exit_code_for_results(results)

    result = run_simulation(args.algorithm, scenario, dt=args.dt)

    if args.json:
        print(json.dumps(result.to_dict(), indent=2))
    else:
        print_result(result)

    if args.save_plot:
        save_result_plot(scenario, result, args.save_plot)
        print(f"Saved plot to: {args.save_plot}")

    if args.plot:
        show_result_plot(scenario, result)

    if args.save_animation:
        save_result_animation(scenario, result, args.save_animation, fps=args.fps)
        print(f"Saved animation to: {args.save_animation}")

    if args.animate:
        show_result_animation(scenario, result, fps=args.fps)

    return 0 if result.success else 1

