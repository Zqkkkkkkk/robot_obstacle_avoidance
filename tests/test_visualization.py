import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from robot_obstacle_avoidance.scenarios import build_scenario
from robot_obstacle_avoidance.simulation import run_all_simulations, run_simulation
from robot_obstacle_avoidance.visualization import (
    save_comparison_animation,
    save_comparison_plot,
    save_result_animation,
    save_result_plot,
)


class VisualizationTests(unittest.TestCase):
    def test_single_result_plot_is_saved(self) -> None:
        scenario = build_scenario("open_field")
        result = run_simulation("astar", scenario)
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "astar.png"
            save_result_plot(scenario, result, str(output_path))
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)

    def test_comparison_plot_is_saved(self) -> None:
        scenario = build_scenario("open_field")
        results = run_all_simulations(scenario)
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "comparison.png"
            save_comparison_plot(scenario, results, str(output_path))
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)

    def test_single_result_animation_is_saved(self) -> None:
        scenario = build_scenario("open_field")
        result = run_simulation("astar", scenario)
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "astar.gif"
            save_result_animation(scenario, result, str(output_path), fps=8)
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)

    def test_comparison_animation_is_saved(self) -> None:
        scenario = build_scenario("open_field")
        results = run_all_simulations(scenario)
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "comparison.gif"
            save_comparison_animation(scenario, results, str(output_path), fps=8)
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()