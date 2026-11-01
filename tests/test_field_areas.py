import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.field_areas import distance_to_area, validate_areas  # noqa: E402
from pai_motion.field_revision import validate_field_revision  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import ObservedMap, execute_search, sample_search  # noqa: E402
from tests.test_correction import SensorFixture  # noqa: E402
from tests.test_dialogue import messages  # noqa: E402

AREAS = [{"id": "area1", "bounds": [2.5, 1.0, 3.0, 1.5]}, {"id": "area2", "bounds": [2.5, -1.5, 3.0, -1.0]}]


class MovingFixture(SensorFixture):
    def observe(self):
        x = 0.0 if self.time < 1 or self.time >= 9 else 1.0 if self.time < 2 else 2.0
        return {**super().observe(), "position": [x, 0.0, 0.35]}

    def sense(self):
        result = self.observe()
        rays = [[i * math.tau / 180, 1.8, False, None] for i in range(180)]
        if 1 <= self.time < 2 or self.time >= 8:
            rays[0], rays[1] = [0.0, 1.7, True, "green"], [0.03, 1.7, True, "green"]
        if self.time >= 10:
            rays[50], rays[51] = [1.7, 1.0, True, "blue"], [1.73, 1.0, True, "blue"]
        return {**result, "scan": rays}


class AreaTests(unittest.TestCase):
    def test_bounds_and_distance(self):
        self.assertEqual(validate_areas(AREAS), AREAS)
        self.assertEqual(distance_to_area((2.7, 1.2), AREAS[0]["bounds"]), 0)
        self.assertAlmostEqual(distance_to_area((2.4, 1.2), AREAS[0]["bounds"]), 0.1)
        for bad in (
            [],
            [AREAS[0], AREAS[0]],
            [{"id": "a", "bounds": [0, 0, 0, 1]}],
            [{"id": "a", "bounds": [0, 0, float("nan"), 1]}],
        ):
            with self.assertRaises(ValueError):
                validate_areas(bad)

    def test_area_filters_grid_and_clearance(self):
        grid = ObservedMap()
        grid.free = {(x, y) for x in range(-20, 51) for y in range(-30, 31)}
        safe = grid.traversable([AREAS[0]["bounds"]])
        self.assertNotIn(grid.cell((2.7, 1.2)), safe)
        self.assertIn(grid.cell((2.7, -1.2)), safe)
        with self.assertRaisesRegex(RuntimeError, "inside"):
            grid.routes((2.7, 1.2), [AREAS[0]["bounds"]])

    def test_path_records_route_in_travel_order(self):
        grid = ObservedMap()
        previous = {(0, 0): None, (1, 0): (0, 0), (1, 1): (1, 0)}
        self.assertEqual(grid.path((1, 1), previous), [(0, 0), (0.1, 0), (0.1, 0.1)])

    def test_revision_validation(self):
        context = {"field_areas": AREAS}
        prompt = "エリア1は通っていいが、エリア2は避けて"
        value = {
            "status": "accept",
            "question": "",
            "changes": [
                {"id": "area1", "permission": "allow", "instruction": prompt},
                {"id": "area2", "permission": "avoid", "instruction": prompt},
            ],
        }
        self.assertEqual(validate_field_revision(value, context=context, prompt=prompt), value)
        for change in (
            {"id": "area3", "permission": "avoid", "instruction": prompt},
            {"id": "area1", "permission": "avoid", "instruction": "でっち上げ"},
        ):
            with self.assertRaises(InvalidPlan):
                validate_field_revision({**value, "changes": [change]}, context=context, prompt=prompt)

    def test_allow_and_avoid_are_independent_and_keep_tasks(self):
        class Fixture(MovingFixture):
            def __init__(self):
                super().__init__()
                self.field_states = []

            def set_field_permissions(self, blocked):
                self.field_states.append(set(blocked))

        sink, revisions = Fixture(), []
        texts = ["エリア2を避けて", "エリア1は通っていい、エリア2は避けて", "エリア2も通っていい"]

        def planner(message, plan, context):
            changes = (
                [{"id": "area2", "permission": "avoid", "instruction": message}]
                if message == texts[0]
                else [
                    {"id": "area1", "permission": "allow", "instruction": message},
                    {"id": "area2", "permission": "avoid", "instruction": message},
                ]
                if message == texts[1]
                else [{"id": "area2", "permission": "allow", "instruction": message}]
            )
            return {"status": "accept", "question": "", "changes": changes}

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(2, texts[0]), (4, texts[1]), (6, texts[2])]),
                correction_planner=planner,
                field_areas=AREAS,
            )
        self.assertEqual(sink.field_states, [{"area2"}, {"area2"}, set()])
        self.assertEqual([r["status"] for r in revisions], ["accepted"] * 3)
        self.assertTrue(revisions[0]["old_route"])
        self.assertTrue(revisions[0]["new_route"])
        self.assertEqual(result["plan"]["seconds"], 180)
        self.assertEqual(result["tasks"]["completed"][-1]["step"]["id"], "home")

    def test_forbidding_current_position_asks_without_changing_state(self):
        class Fixture(MovingFixture):
            def set_field_permissions(self, blocked):
                raise AssertionError("An unsafe area must not be applied")

        wording = "エリア1を避けて"
        areas = [{"id": "area1", "bounds": [1.8, -0.2, 2.2, 0.2]}]
        revisions = []

        def planner(message, plan, context):
            return {
                "status": "accept",
                "question": "",
                "changes": [{"id": "area1", "permission": "avoid", "instruction": message}],
            }

        with patch("pai_motion.search.time.sleep"):
            execute_search(
                sample_search(),
                Fixture(),
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(2, wording)]),
                correction_planner=planner,
                field_areas=areas,
            )
        self.assertEqual(revisions[0]["status"], "clarification")
        self.assertEqual(revisions[0]["clarification_source"], "feasibility_check")


if __name__ == "__main__":
    unittest.main()
