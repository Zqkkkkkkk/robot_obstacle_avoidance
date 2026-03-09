import unittest

from robot_obstacle_avoidance.scenarios import build_scenario
from robot_obstacle_avoidance.webapp import create_app


class WebAppTests(unittest.TestCase):
    def setUp(self) -> None:
        self.app = create_app()
        self.app.testing = True
        self.client = self.app.test_client()

    def test_index_page_loads(self) -> None:
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        self.assertIn(b"Robot Obstacle Avoidance Lab", response.data)

    def test_meta_endpoint_returns_algorithms_and_scenarios(self) -> None:
        response = self.client.get("/api/meta")
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertIn("astar", payload["algorithms"])
        self.assertGreaterEqual(len(payload["scenarios"]), 3)
        self.assertTrue(any(scenario["name"] == "dynamic_crossing" for scenario in payload["scenarios"]))

    def test_simulate_endpoint_returns_single_result(self) -> None:
        response = self.client.get("/api/simulate?algorithm=astar&scenario=open_field&dt=0.1")
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["result"]["algorithm"], "astar")
        self.assertEqual(payload["scenario"]["name"], "open_field")
        self.assertIn("dynamic_obstacle_history", payload["result"])

    def test_dynamic_scenario_returns_moving_obstacle_history(self) -> None:
        response = self.client.get("/api/simulate?algorithm=dwa&scenario=dynamic_crossing&dt=0.1")
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertGreater(len(payload["scenario"]["dynamic_obstacles"]), 0)
        self.assertGreater(len(payload["result"]["dynamic_obstacle_history"]), 1)
        self.assertNotEqual(
            payload["result"]["dynamic_obstacle_history"][0][0]["y"],
            payload["result"]["dynamic_obstacle_history"][-1][0]["y"],
        )

    def test_compare_endpoint_returns_all_results(self) -> None:
        response = self.client.get("/api/compare?scenario=open_field&dt=0.1")
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(len(payload["results"]), 3)

    def test_compare_endpoint_accepts_runtime_obstacle_events(self) -> None:
        response = self.client.post(
            "/api/compare",
            json={
                "scenario": "open_field",
                "dt": 0.1,
                "runtime_obstacle_events": [
                    {
                        "activate_time": 1.5,
                        "label": "surprise_blocker",
                        "obstacle": {"kind": "circle", "x": 5.8, "y": 4.0, "radius": 0.35},
                    }
                ],
            },
        )
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(len(payload["scenario"]["runtime_obstacle_events"]), 1)
        self.assertEqual(len(payload["results"]), 3)

    def test_simulate_endpoint_accepts_custom_obstacles(self) -> None:
        response = self.client.post(
            "/api/simulate",
            json={
                "algorithm": "astar",
                "scenario": "open_field",
                "dt": 0.1,
                "custom_obstacles": [
                    {"kind": "rectangle", "x": 2.2, "y": 8.2, "width": 0.8, "height": 0.6},
                    {"kind": "circle", "x": 8.1, "y": 2.1, "radius": 0.45},
                ],
            },
        )
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(
            len(payload["scenario"]["obstacles"]),
            len(build_scenario("open_field").obstacles) + 2,
        )
        self.assertEqual(payload["scenario"]["obstacles"][-2]["kind"], "rectangle")
        self.assertEqual(payload["scenario"]["obstacles"][-1]["kind"], "circle")

    def test_simulate_endpoint_accepts_runtime_obstacle_events(self) -> None:
        response = self.client.post(
            "/api/simulate",
            json={
                "algorithm": "dwa",
                "scenario": "open_field",
                "dt": 0.1,
                "runtime_obstacle_events": [
                    {
                        "activate_time": 1.5,
                        "label": "surprise_blocker",
                        "obstacle": {"kind": "circle", "x": 5.8, "y": 4.0, "radius": 0.35},
                    }
                ],
            },
        )
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(len(payload["scenario"]["runtime_obstacle_events"]), 1)
        self.assertEqual(payload["result"]["notes"]["runtime_obstacle_event_count"], 1)

    def test_runtime_obstacle_overlapping_robot_returns_bad_request(self) -> None:
        response = self.client.post(
            "/api/simulate",
            json={
                "algorithm": "dwa",
                "scenario": "open_field",
                "dt": 0.1,
                "runtime_obstacle_events": [
                    {
                        "activate_time": 0.0,
                        "label": "invalid_overlap",
                        "obstacle": {"kind": "circle", "x": 1.0, "y": 1.0, "radius": 0.45},
                    }
                ],
            },
        )
        self.assertEqual(response.status_code, 400)

    def test_invalid_algorithm_returns_bad_request(self) -> None:
        response = self.client.get("/api/simulate?algorithm=bad&scenario=open_field")
        self.assertEqual(response.status_code, 400)

    def test_invalid_dt_returns_bad_request(self) -> None:
        response = self.client.get("/api/simulate?algorithm=astar&scenario=open_field&dt=0")
        self.assertEqual(response.status_code, 400)

    def test_invalid_custom_obstacle_returns_bad_request(self) -> None:
        response = self.client.post(
            "/api/compare",
            json={
                "scenario": "open_field",
                "custom_obstacles": [{"kind": "circle", "x": 4.0, "y": 5.0, "radius": -1.0}],
            },
        )
        self.assertEqual(response.status_code, 400)


if __name__ == "__main__":
    unittest.main()