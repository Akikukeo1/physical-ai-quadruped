import sys
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.correction import validate_correction  # noqa: E402
from pai_motion.custom_map import map_scene, validate_map  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import execute_search, sample_search  # noqa: E402
from tests.test_dialogue import messages  # noqa: E402
from tests.test_field_areas import MovingFixture  # noqa: E402

MAP = {
    "start": [0, 0],
    "obstacles": [[1.5, 1, 2, 2]],
    "targets": {"blue": [3, 0]},
    "areas": [{"id": "area1", "bounds": [2, -2, 3, -1]}],
    "points": [{"id": "P1", "position": [1, 0]}],
}


class CustomMapTests(unittest.TestCase):
    def test_scene_and_validation(self):
        self.assertEqual(validate_map(deepcopy(MAP)), MAP)
        self.assertEqual(len(map_scene(MAP)), 6)
        bad = deepcopy(MAP)
        bad["start"] = [1.7, 1.5]
        with self.assertRaises(ValueError):
            validate_map(bad)
        bad = deepcopy(MAP)
        bad["points"].append(deepcopy(bad["points"][0]))
        with self.assertRaises(ValueError):
            validate_map(bad)

    def test_unknown_point_rejected(self):
        home = {"id": "home", "action": "return_home", "target": None, "observation_id": None}
        value = {
            "intent": "移動",
            "status": "accept",
            "question": "",
            "cancelled": [],
            "steps": [{"id": "visit", "action": "visit_point", "target": "P2", "observation_id": None}, home],
        }
        with self.assertRaises(InvalidPlan):
            validate_correction(
                value, context={"pending": [home], "completed": [], "observations": [], "points": MAP["points"]}
            )

    def test_point_visit_and_return_without_search(self):
        revisions = []

        def planner(prompt, plan, context):
            self.assertEqual(context["points"], MAP["points"])
            return {
                "intent": "P1を経由",
                "status": "accept",
                "question": "",
                "changes": [],
                "reply": "了解です。",
                "cancelled": [],
                "steps": [
                    {"id": "visit", "action": "visit_point", "target": "P1", "observation_id": None},
                    *deepcopy(context["pending"]),
                ],
            }

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                MovingFixture(),
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(0, "P1を経由して戻って")]),
                correction_planner=planner,
                agent_revision=True,
                points=MAP["points"],
                initial_home_only=True,
            )
        self.assertEqual(result["outcome"], "returned")
        self.assertEqual(
            [entry["step"]["action"] for entry in result["tasks"]["completed"]], ["visit_point", "return_home"]
        )

    def test_unreached_point_is_not_reported_as_completed(self):
        points = [{"id": "P1", "position": [4, 2]}]

        def planner(prompt, plan, context):
            return {
                "intent": "P1を経由",
                "status": "accept",
                "question": "",
                "changes": [],
                "reply": "了解です。",
                "cancelled": [],
                "steps": [
                    {"id": "visit", "action": "visit_point", "target": "P1", "observation_id": None},
                    *deepcopy(context["pending"]),
                ],
            }

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                MovingFixture(),
                observations=[],
                decisions=[],
                get_correction=messages([(0, "P1を経由して戻って")]),
                correction_planner=planner,
                agent_revision=True,
                points=points,
                initial_home_only=True,
            )
        self.assertEqual(result["outcome"], "point_unreached")
        self.assertEqual(result["tasks"]["pending"][0]["target"], "P1")
