"""Restricted regions affect planning without changing the observed obstacle map."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.search import ObservedMap  # noqa: E402


class TestRestrictedMap(unittest.TestCase):
    def setUp(self):
        self.grid = ObservedMap()
        self.grid.free = {(x, y) for x in range(-30, 31) for y in range(-30, 31)}

    def test_route_goes_around_a_restricted_observation_position(self):
        center, radius = (0.0, 0.0), 0.5
        costs, previous = self.grid.routes((1.5, 0.0), [(center, radius)])
        goal = self.grid.cell((-1.5, 0.0))
        self.assertIn(goal, costs)
        self.assertGreater(costs[goal], 3.0)
        path = [goal]
        while previous[path[-1]] is not None:
            path.append(previous[path[-1]])
        self.assertTrue(all(math.dist(self.grid.point(cell), center) > radius + 0.15 for cell in path))
        self.assertIn((0, 0), self.grid.free)
        self.assertNotIn((0, 0), self.grid.occupied)

    def test_start_inside_or_too_near_region_is_not_recovered_into_a_safe_cell(self):
        for position in ((0.0, 0.0), (0.64, 0.0)):
            with self.subTest(position=position), self.assertRaisesRegex(RuntimeError, "restricted area"):
                self.grid.routes(position, [((0.0, 0.0), 0.5)])
        costs, _ = self.grid.routes((0.8, 0.0), [((0.0, 0.0), 0.5)])
        self.assertIn(self.grid.cell((0.8, 0.0)), costs)

    def test_removing_restriction_restores_shortest_path(self):
        unrestricted, _ = self.grid.routes((1.5, 0.0))
        restricted, _ = self.grid.routes((1.5, 0.0), [((0.0, 0.0), 0.5)])
        goal = self.grid.cell((-1.5, 0.0))
        self.assertAlmostEqual(unrestricted[goal], 3.0)
        self.assertGreater(restricted[goal], unrestricted[goal])

    def test_generator_does_not_drop_any_of_multiple_active_restrictions(self):
        zones = [((-1.0, 0.0), 0.3), ((1.0, 0.0), 0.3)]
        safe = self.grid.traversable(iter(zones))
        self.assertNotIn(self.grid.cell(zones[0][0]), safe)
        self.assertNotIn(self.grid.cell(zones[1][0]), safe)
        costs, _ = self.grid.routes((0.0, 1.5), (zone for zone in zones))
        self.assertNotIn(self.grid.cell(zones[0][0]), costs)
        self.assertNotIn(self.grid.cell(zones[1][0]), costs)
        self.assertIn(self.grid.cell((0.0, -1.5)), costs)
