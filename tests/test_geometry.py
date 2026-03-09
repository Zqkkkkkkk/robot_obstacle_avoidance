import math
import unittest

from robot_obstacle_avoidance.geometry import distance, wrap_angle


class GeometryTests(unittest.TestCase):
    def test_distance_matches_pythagoras(self) -> None:
        self.assertAlmostEqual(distance(0.0, 0.0, 3.0, 4.0), 5.0)

    def test_wrap_angle_normalizes_large_value(self) -> None:
        wrapped = wrap_angle(3.0 * math.pi)
        self.assertAlmostEqual(wrapped, math.pi)


if __name__ == "__main__":
    unittest.main()
