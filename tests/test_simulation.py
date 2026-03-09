import unittest
from dataclasses import replace

from robot_obstacle_avoidance.models import CircleObstacle, RectangleObstacle, RuntimeObstacleEvent
from robot_obstacle_avoidance.scenarios import build_scenario
from robot_obstacle_avoidance.simulation import run_all_simulations, run_simulation


class SimulationTests(unittest.TestCase):
    def test_astar_reaches_goal_in_open_field(self) -> None:
        result = run_simulation("astar", build_scenario("open_field"), dt=0.1)
        self.assertTrue(result.success)
        self.assertFalse(result.collision)

    def test_astar_reaches_goal_in_narrow_passage(self) -> None:
        result = run_simulation("astar", build_scenario("narrow_passage"), dt=0.1)
        self.assertTrue(result.success)
        self.assertFalse(result.collision)

    def test_apf_produces_nonempty_trajectory(self) -> None:
        result = run_simulation("apf", build_scenario("open_field"), dt=0.1)
        self.assertGreater(len(result.trajectory), 2)
        self.assertFalse(result.collision)

    def test_apf_reaches_goal_in_narrow_passage(self) -> None:
        result = run_simulation("apf", build_scenario("narrow_passage"), dt=0.1)
        self.assertTrue(result.success)
        self.assertFalse(result.collision)

    def test_dwa_produces_nonempty_trajectory(self) -> None:
        result = run_simulation("dwa", build_scenario("open_field"), dt=0.1)
        self.assertGreater(len(result.trajectory), 2)
        self.assertFalse(result.collision)
        self.assertTrue(result.success)

    def test_dwa_reaches_goal_in_narrow_passage(self) -> None:
        result = run_simulation("dwa", build_scenario("narrow_passage"), dt=0.1)
        self.assertTrue(result.success)
        self.assertFalse(result.collision)

    def test_dwa_handles_dynamic_crossing(self) -> None:
        result = run_simulation("dwa", build_scenario("dynamic_crossing"), dt=0.1)
        self.assertTrue(result.success)
        self.assertFalse(result.collision)
        self.assertEqual(len(result.dynamic_obstacle_history), len(result.trajectory))
        self.assertGreater(len(result.dynamic_obstacle_history[0]), 0)
        self.assertNotEqual(result.dynamic_obstacle_history[0][0]["y"], result.dynamic_obstacle_history[-1][0]["y"])

    def test_dynamic_crossing_highlights_dwa_advantage(self) -> None:
        results = {result.algorithm: result for result in run_all_simulations(build_scenario("dynamic_crossing"), dt=0.1)}
        self.assertTrue(results["dwa"].success)
        self.assertLess(results["dwa"].final_distance_to_goal, results["apf"].final_distance_to_goal)
        self.assertLess(results["dwa"].final_distance_to_goal, results["astar"].final_distance_to_goal)

    def test_runtime_obstacle_overlapping_robot_is_rejected(self) -> None:
        scenario = replace(
            build_scenario("open_field"),
            runtime_obstacle_events=(
                RuntimeObstacleEvent(
                    activate_time=0.0,
                    obstacle=CircleObstacle(1.0, 1.0, 0.45),
                    label="invalid_overlap",
                ),
            ),
        )
        with self.assertRaises(ValueError):
            run_simulation("dwa", scenario, dt=0.1)

    def test_runtime_obstacle_event_is_counted(self) -> None:
        scenario = replace(
            build_scenario("open_field"),
            runtime_obstacle_events=(
                RuntimeObstacleEvent(
                    activate_time=1.2,
                    obstacle=CircleObstacle(5.8, 4.0, 0.35),
                    label="surprise_blocker",
                ),
            ),
        )
        result = run_simulation("dwa", scenario, dt=0.1)
        self.assertEqual(result.notes["runtime_obstacle_event_count"], 1)

    def test_all_planners_reach_goal_with_reachable_sudden_obstacle(self) -> None:
        scenario = replace(
            build_scenario("open_field"),
            runtime_obstacle_events=(
                RuntimeObstacleEvent(
                    activate_time=1.8,
                    obstacle=CircleObstacle(3.2, 3.3, 0.55),
                    label="sudden_blocker",
                ),
            ),
        )
        for algorithm in ("astar", "apf", "dwa"):
            with self.subTest(algorithm=algorithm):
                result = run_simulation(algorithm, scenario, dt=0.1)
                self.assertTrue(result.success)
                self.assertFalse(result.collision)

    def test_runtime_obstacle_that_blocks_all_paths_is_rejected(self) -> None:
        empty_field = replace(build_scenario("open_field"), obstacles=())
        scenario = replace(
            empty_field,
            runtime_obstacle_events=(
                RuntimeObstacleEvent(
                    activate_time=0.0,
                    obstacle=RectangleObstacle(5.0, 5.0, 0.7, 9.6),
                    label="sealed_wall",
                ),
            ),
        )
        with self.assertRaises(ValueError):
            run_simulation("astar", scenario, dt=0.1)


if __name__ == "__main__":
    unittest.main()
