import math
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.motion import InvalidPlan
from pai_motion.navigation import (
    GOAL_TOLERANCE,
    WAYPOINTS,
    execute_navigation,
    sample_navigation,
    segment_clear,
    validate_navigation,
)
from pai_motion.planner import codex_navigation


class TestNavigation(unittest.TestCase):
    def test_whole_segment_and_corner_checks(self):
        self.assertFalse(segment_clear((0, 0), (1.85, 0)))
        self.assertFalse(segment_clear((0.9, -2), (0.9, 2)))
        self.assertFalse(segment_clear((0.9, 0), (0.9, 0)))
        self.assertTrue(segment_clear((0, 0), WAYPOINTS["north_gate"]))
        self.assertTrue(segment_clear(WAYPOINTS["north_gate"], WAYPOINTS["blue"]))
        self.assertTrue(segment_clear((0, 1.05), (1.85, 1.05)))

    def test_reject_unsafe_and_unknown_routes(self):
        for value in (
            {"intent": "unsafe", "waypoints": ["blue"]},
            {"intent": "unknown", "waypoints": ["run_shell"]},
            {"intent": "extra", "waypoints": ["home"], "code": "print(1)"},
            {"intent": "many", "waypoints": ["home"] * 9},
        ):
            with self.subTest(value=value), self.assertRaises(InvalidPlan):
                validate_navigation(value)
        self.assertEqual(sample_navigation().waypoints, ("north_gate", "blue", "green"))

    def test_codex_uses_navigation_schema_and_validates_output(self):
        with patch("pai_motion.planner._codex_json", return_value={"intent": "visit", "waypoints": ["blue"]}) as call:
            with self.assertRaises(InvalidPlan):
                codex_navigation("青へ")
            self.assertEqual(call.call_args.args[1].name, "navigation-plan.schema.json")

    def test_stalled_robot_times_out_and_stops(self):
        class StalledSink:
            time = 0.0
            stopped = False

            def observe(self):
                return {"position": [0, 0, 0.35], "yaw": 0, "time": self.time}

            def run_velocity(self, cmd, seconds):
                self.time += seconds
                self.assert_bounds(cmd, seconds)

            @staticmethod
            def assert_bounds(cmd, seconds):
                assert 0 <= cmd.vx <= 0.25 and abs(cmd.yaw) <= 0.6 and seconds == 0.2

            def stop(self):
                self.stopped = True

        sink = StalledSink()
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            execute_navigation(sample_navigation(), sink)
        self.assertTrue(sink.stopped)
        self.assertLess(sink.time, 35.3)


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for real MuJoCo physics tests")
class TestNavigationPhysics(unittest.TestCase):
    def test_direct_drive_triggers_safety_stop(self):
        from pai_motion.executor import Velocity
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        sink = MuJoCoGo2Sink(headless=True, navigation=True)
        try:
            with self.assertRaisesRegex(RuntimeError, "obstacle safety stop"):
                sink.run_velocity(Velocity(0.25, 0, 0), 2.0)
            stopped_at = sink.observe()["time"]
            sink.stop()
            self.assertEqual(sink.observe()["time"], stopped_at)
            self.assertFalse(sink.observe()["obstacle_contact"])
            self.assertTrue((sink.data.ctrl == 0).all())
        finally:
            sink.close()

    def test_obstacle_detour_and_ordered_goals(self):
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        sink = MuJoCoGo2Sink(headless=True, navigation=True)
        try:
            result = execute_navigation(sample_navigation(), sink)
            self.assertEqual([item["waypoint"] for item in result["reached"]], ["north_gate", "blue", "green"])
            self.assertLessEqual(result["goal_error"], GOAL_TOLERANCE)
            self.assertLess(result["elapsed"], 90)
            self.assertFalse(any(state["obstacle_contact"] for state in sink.trace))
            self.assertGreater(min(state["obstacle_clearance"] for state in sink.trace), 0.05)
            self.assertGreater(min(state["upright"] for state in sink.trace), 0.9)
            self.assertLess(math.hypot(*result["final"]["body_velocity"][:2]), 0.05)
        finally:
            sink.close()
