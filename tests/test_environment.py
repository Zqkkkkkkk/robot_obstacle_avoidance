import unittest

from robot_obstacle_avoidance.environment import Environment
from robot_obstacle_avoidance.models import Pose, RectangleObstacle, Scenario


class EnvironmentTests(unittest.TestCase):
    def test_rectangle_distance_is_negative_inside_obstacle(self) -> None:
        scenario = Scenario(
            name="rect_test",
            width=12.0,
            height=12.0,
            start=Pose(1.0, 1.0),
            goal=Pose(10.0, 10.0),
            obstacles=(RectangleObstacle(5.0, 5.0, 2.0, 4.0),),
        )
        environment = Environment(scenario)
        self.assertAlmostEqual(environment.distance_to_nearest_obstacle_surface(5.0, 5.0), -1.0)

    def test_nearest_point_on_rectangle_is_clamped_to_edge(self) -> None:
        obstacle = RectangleObstacle(5.0, 5.0, 2.0, 4.0)
        scenario = Scenario(
            name="rect_test",
            width=12.0,
            height=12.0,
            start=Pose(1.0, 1.0),
            goal=Pose(10.0, 10.0),
            obstacles=(obstacle,),
        )
        environment = Environment(scenario)
        self.assertEqual(environment.nearest_point_on_obstacle(obstacle, 8.0, 5.5), (6.0, 5.5))


if __name__ == "__main__":
    unittest.main()