"""Unit kinematics for atomic constraint application and conflict handling."""

import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from main import main  # noqa: E402
from pai_motion.search import ObservedMap, execute_search, sample_search  # noqa: E402
from tests.test_correction import SensorFixture  # noqa: E402
from tests.test_dialogue import messages  # noqa: E402


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


class TestConstraintExecution(unittest.TestCase):
    def test_cli_requires_correction_input_before_starting_simulator(self):
        args = ["main", "--task", "search", "--planner", "codex", "--executor", "mujoco", "--constraint-revisions"]
        with patch.object(sys, "argv", args), patch("pai_motion.sim_adapter.MuJoCoGo2Sink") as factory:
            with self.assertRaises(SystemExit):
                main()
            factory.assert_not_called()

    def test_add_conflict_question_release_and_continue(self):
        sink, revisions = MovingFixture(), []
        wording = ("緑の地点を避ける", "緑の地点で再確認", "制限を解除")
        route_areas = []
        original_routes = ObservedMap.routes

        def routes(grid, position, restricted=()):
            route_areas.append(list(restricted))
            return original_routes(grid, position, restricted)

        def plan(message, original, context):
            record = next(o for o in context["observations"] if o["color"] == "green")
            area = {"id": "zone", "observation_id": record["id"], "radius": 0.2, "instruction": wording[0]}
            result = {
                "intent": "訂正",
                "status": "accept",
                "question": "",
                "steps": deepcopy(context["pending"]),
                "cancelled": [],
                "constraints": deepcopy(context["constraints"]),
                "released_constraints": [],
            }
            if message == wording[0]:
                result["constraints"] = [area]
            else:
                result["steps"].insert(
                    0, {"id": "green-check", "action": "reobserve", "target": "green", "observation_id": record["id"]}
                )
                if message == wording[2]:
                    result["constraints"] = []
                    result["released_constraints"] = [{"id": "zone", "instruction": wording[2]}]
            return result

        with patch("pai_motion.search.time.sleep"), patch.object(ObservedMap, "routes", routes):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(2, wording[0]), (4, wording[1]), (6, wording[2])]),
                correction_planner=plan,
                constraint_mode=True,
            )
        self.assertEqual([r["status"] for r in revisions], ["accepted", "clarification", "accepted"])
        self.assertEqual(revisions[1]["clarification_source"], "feasibility_check")
        self.assertEqual(revisions[2]["context"]["constraints"], revisions[0]["active_constraints"])
        self.assertEqual([r["constraint"]["id"] for r in result["constraints"]["released"]], ["zone"])
        self.assertEqual(result["constraints"]["active"], [])
        self.assertEqual(
            [r["step"]["action"] for r in result["tasks"]["completed"]], ["reobserve", "search", "return_home"]
        )
        self.assertEqual(result["plan"]["seconds"], 180)
        self.assertTrue(any(route_areas))
        self.assertEqual(route_areas[-1], [])

    def test_drift_into_stopping_margin_stops_the_robot(self):
        class DriftFixture(MovingFixture):
            def observe(self):
                state = super().observe()
                if self.time >= 3:
                    state["position"] = [1.22, 0.0, 0.35]
                return state

        sink = DriftFixture()
        message = "緑の地点を避ける"

        def plan(prompt, original, context):
            record = next(o for o in context["observations"] if o["color"] == "green")
            return {
                "intent": "制限",
                "status": "accept",
                "question": "",
                "steps": context["pending"],
                "cancelled": [],
                "constraints": [{"id": "zone", "observation_id": record["id"], "radius": 0.2, "instruction": message}],
                "released_constraints": [],
            }

        with patch("pai_motion.search.time.sleep"), self.assertRaisesRegex(RuntimeError, "stopping margin"):
            execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                get_correction=messages([(2, message)]),
                correction_planner=plan,
                constraint_mode=True,
            )
        self.assertTrue(sink.stopped)

    def test_current_or_home_inside_new_area_is_not_applied(self):
        sink, revisions = MovingFixture(), []
        wording = ("緑の地点から1.5mを避ける", "制限しない")

        def plan(message, original, context):
            result = {
                "intent": "訂正",
                "status": "accept",
                "question": "",
                "steps": deepcopy(context["pending"]),
                "cancelled": [],
                "constraints": [],
                "released_constraints": [],
            }
            if message == wording[0]:
                record = next(o for o in context["observations"] if o["color"] == "green")
                result["constraints"] = [
                    {"id": "zone", "observation_id": record["id"], "radius": 1.5, "instruction": wording[0]}
                ]
            return result

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(2, wording[0]), (4, wording[1])]),
                correction_planner=plan,
                constraint_mode=True,
            )
        self.assertEqual([r["status"] for r in revisions], ["clarification", "accepted"])
        self.assertEqual(revisions[0]["clarification_source"], "feasibility_check")
        self.assertEqual(revisions[1]["context"]["constraints"], [])
        self.assertEqual(result["constraints"]["released"], [])
        self.assertEqual(result["outcome"], "found")
