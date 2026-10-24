import math
import os
import sys
import unittest
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.planner import codex_search  # noqa: E402
from pai_motion.search import ObservedMap, execute_search, sample_search, validate_search  # noqa: E402
from pai_motion.search_world import search_scene  # noqa: E402


def empty_scan():
    return [[i * math.tau / 180, 1.8, False, None] for i in range(180)]


class TestSearch(unittest.TestCase):
    def test_schema_and_python_validator_reject_coordinates_code_and_unbounded_budgets(self):
        plan = asdict(sample_search())
        for change in (
            {"seconds": True},
            {"seconds": 241},
            {"seconds": float("nan")},
            {"target": "red"},
            {"position": [3, 1]},
            {"code": "print(1)"},
            {"on_found": "continue"},
        ):
            with self.subTest(change=change), self.assertRaises(InvalidPlan):
                validate_search({**plan, **change})

    def test_codex_receives_only_instruction_and_capabilities(self):
        plan = {**asdict(sample_search()), "target": "green"}
        with patch("pai_motion.planner._codex_json", return_value=plan) as call:
            result = codex_search("緑の対象を見つけて戻って")
        self.assertEqual(result.target, "green")
        self.assertEqual(call.call_args.args[1].name, "search-plan.schema.json")
        self.assertNotIn("north_gate", call.call_args.args[0])
        self.assertNotIn("WAYPOINTS", call.call_args.args[0])

    def test_ray_map_does_not_clear_behind_first_hit_and_blocks_unknown_footprint(self):
        grid = ObservedMap()
        grid.update({"position": [0.0, 0.0], "yaw": 0.0, "scan": [[0.0, 1.0, True, None]]})
        self.assertIn((5, 0), grid.occupied)
        self.assertNotIn((6, 0), grid.free)
        with self.assertRaisesRegex(RuntimeError, "footprint"):
            grid.routes((0, 0))
        grid = ObservedMap()
        grid.update({"position": [0.0, 0.0], "yaw": 0.0, "scan": empty_scan()})
        costs, _ = grid.routes((0, 0))
        self.assertIn((0, 0), costs)
        self.assertNotIn((9, 0), costs)
        self.assertIsNotNone(grid.frontier(costs))

    def test_scene_is_repeatable_and_missing_blue_keeps_green(self):
        self.assertEqual(search_scene(0), search_scene(0))
        self.assertNotEqual(search_scene(0), search_scene(1))
        scene = search_scene(0, True)
        self.assertFalse(any(item["rgba"][2] == 1 for item in scene))
        self.assertTrue(any(item["rgba"][1] == 1 for item in scene))

    def test_budget_returns_not_found_without_hallucinating_target(self):
        class StalledSensorSink:
            time = 0.0
            stopped = False

            def observe(self):
                return {"time": self.time, "position": [0.0, 0.0, 0.35], "yaw": 0.0}

            def sense(self):
                return {**self.observe(), "scan": empty_scan()}

            def run_velocity(self, cmd, seconds):
                assert 0 <= cmd.vx <= 0.25 and abs(cmd.yaw) <= 0.6 and seconds == 0.2
                self.time += seconds

            def stop(self):
                self.stopped = True

        sink = StalledSensorSink()
        plan = validate_search({**asdict(sample_search()), "seconds": 60})
        result = execute_search(plan, sink, observations=[], decisions=[])
        self.assertEqual(result["outcome"], "not_found")
        self.assertIsNone(result["found"])
        self.assertTrue(sink.stopped)
        self.assertLessEqual(result["elapsed"], 60)


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for real MuJoCo physics tests")
class TestSearchPhysics(unittest.TestCase):
    def test_sensor_uses_first_surface_color_and_never_exposes_hidden_map(self):
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        scene = [
            {"box": (1.0, 0.0, 0.1, 0.5), "rgba": [0.65, 0.65, 0.65, 1]},
            {"box": (1.4, 0.0, 0.14, 0.14), "rgba": [0.1, 0.25, 1, 1]},
            {"box": (0.0, 1.2, 0.14, 0.14), "rgba": [0.1, 1, 0.25, 1]},
        ]
        with patch("pai_motion.search_world.search_scene", return_value=scene):
            sink = MuJoCoGo2Sink(headless=True, search_seed=0)
        try:
            observation = sink.sense()
            self.assertEqual(set(observation), {"time", "position", "yaw", "body_velocity", "yaw_rate", "scan"})
            self.assertLess(observation["scan"][0][1], 1.0)
            self.assertTrue(any(ray[3] == "green" for ray in observation["scan"]))
            self.assertFalse(any(ray[3] == "blue" for ray in observation["scan"]))
            self.assertTrue(all(0 <= ray[1] <= 1.8 for ray in observation["scan"]))
        finally:
            sink.close()

    def test_unknown_layout_find_and_return(self):
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        sink = MuJoCoGo2Sink(headless=True, search_seed=0)
        observations, decisions = [], []
        try:
            self.assertFalse(any(ray[3] == "blue" for ray in sink.sense()["scan"]))
            result = execute_search(sample_search(), sink, observations=observations, decisions=decisions)
            self.assertEqual(result["outcome"], "found")
            self.assertGreater(result["distance"], 2.0)
            self.assertLess(result["home_error"], 0.15)
            self.assertLessEqual(result["elapsed"], 180)
            self.assertFalse(any(state["obstacle_contact"] for state in sink.trace))
        finally:
            sink.close()
