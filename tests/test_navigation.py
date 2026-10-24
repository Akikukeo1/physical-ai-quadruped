import math
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.motion import InvalidPlan
from pai_motion.navigation import (
    BLOCKER,
    GOAL_TOLERANCE,
    GOALS,
    OBSTACLE,
    WAYPOINTS,
    NavigationPlan,
    execute_navigation,
    rules_navigation,
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

    def test_replans_validate_all_obstacles_and_preserve_goal_order(self):
        obstacles = [OBSTACLE, BLOCKER]
        start = (0.0, 0.5)
        with self.assertRaises(InvalidPlan):
            validate_navigation({"intent": "blocked", "waypoints": ["north_gate", "blue", "green"]}, start, obstacles)
        plan = rules_navigation({"position": [*start, 0.35], "obstacles": obstacles}, ("blue", "green"))
        self.assertEqual(tuple(name for name in plan.waypoints if name in GOALS), ("blue", "green"))
        self.assertIn("south_exit", plan.waypoints)
        with self.assertRaisesRegex(InvalidPlan, "preserve"):
            validate_navigation(
                {"intent": "wrong order", "waypoints": ["south_gate", "green", "blue"]},
                start,
                obstacles,
                ("blue", "green"),
            )
        with self.assertRaisesRegex(InvalidPlan, "finish"):
            validate_navigation(
                {"intent": "leaves goal again", "waypoints": ["blue", "green", "south_exit"]},
                WAYPOINTS["blue"],
                obstacles,
                ("blue", "green"),
            )

    def test_codex_receives_current_observations_and_remaining_goals(self):
        state = {"position": [0.0, 0.5, 0.35], "obstacles": [OBSTACLE, BLOCKER]}
        route = rules_navigation(state, ("blue", "green"))
        with patch(
            "pai_motion.planner._codex_json", return_value={"intent": route.intent, "waypoints": list(route.waypoints)}
        ) as call:
            result = codex_navigation(
                "青の後で緑へ", start=(0.0, 0.5), obstacles=state["obstacles"], required=("blue", "green")
            )
            self.assertEqual(result, route)
            instructions = call.call_args.args[0]
            self.assertIn("(0.0, 0.5)", instructions)
            self.assertIn(repr(state["obstacles"]), instructions)
            self.assertIn("('blue', 'green')", instructions)


class ChangingMapSink:
    """Unit-test fixture only: no physical simulation or movement claim."""

    def __init__(self, *, already_at_blue=False):
        self.time = 0.0
        self.position = [1.85, 1.05, 0.35] if already_at_blue else [0.0, 0.0, 0.35]
        self.blocker = (1.85, 0.0, 0.12, 0.24) if already_at_blue else BLOCKER
        self.commands = []
        self.stopped = False

    def observe(self):
        return {
            "time": self.time,
            "position": self.position,
            "yaw": 0.0,
            "body_velocity": [0.0, 0.0, 0.0],
            "yaw_rate": 0.0,
            "obstacles": [OBSTACLE] + ([self.blocker] if self.time else []),
        }

    def run_velocity(self, cmd, seconds):
        self.commands.append(cmd)
        self.time += seconds

    def stop(self):
        self.stopped = True


class TestReplanningFailures(unittest.TestCase):
    def test_failed_planner_never_resumes_movement(self):
        sink = ChangingMapSink()
        events = []
        with patch("builtins.print"), self.assertRaisesRegex(RuntimeError, "model unavailable"):

            def unavailable(state, remaining):
                self.assertEqual(remaining, ("blue", "green"))
                self.assertEqual(sink.commands[-1].vx, 0)
                raise RuntimeError("model unavailable")

            execute_navigation(sample_navigation(), sink, replanner=unavailable, replan_events=events)
        self.assertTrue(sink.stopped)
        self.assertEqual(len(sink.commands), 2)
        self.assertEqual(events[-1]["status"], "failed")

    def test_completed_goal_cannot_be_added_back_by_replanner(self):
        sink = ChangingMapSink(already_at_blue=True)
        initial = validate_navigation({"intent": "blue then green", "waypoints": ["blue", "green"]}, start=(1.85, 1.05))

        def wrong_order(state, remaining):
            self.assertEqual(remaining, ("green",))
            # Safe transit is allowed, but repeating the already visited goal is not.
            safe = rules_navigation(state, remaining)
            return NavigationPlan("repeat goal", ("blue", *safe.waypoints))

        with self.assertRaisesRegex(InvalidPlan, "preserve"):
            execute_navigation(initial, sink, replanner=wrong_order)
        self.assertTrue(sink.stopped)
        self.assertEqual(len(sink.commands), 2)


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for real MuJoCo physics tests")
class TestNavigationPhysics(unittest.TestCase):
    def test_moving_blocker_causes_fixed_plan_to_stop(self):
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        sink = MuJoCoGo2Sink(headless=True, navigation=True, dynamic_obstacle=True)
        try:
            with self.assertRaisesRegex(RuntimeError, "without replanning"):
                execute_navigation(sample_navigation(), sink)
            self.assertTrue(sink.stopped)
            self.assertLess(sink.observe()["time"], 10)
            self.assertFalse(any(state["obstacle_contact"] for state in sink.trace))
        finally:
            sink.close()

    def test_moving_blocker_replans_safely_with_rules(self):
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        sink = MuJoCoGo2Sink(headless=True, navigation=True, dynamic_obstacle=True)
        try:
            result = execute_navigation(sample_navigation(), sink, replanner=rules_navigation)
            self.assertEqual(len(result["replans"]), 1)
            event = result["replans"][0]
            self.assertEqual(event["status"], "accepted")
            self.assertEqual(event["remaining_goals"], ["blue", "green"])
            self.assertLess(math.hypot(*event["stopped"]["body_velocity"][:2]), 0.03)
            self.assertEqual(event["planning_sim_seconds"], 0.0)
            self.assertEqual([r["waypoint"] for r in result["reached"] if r["waypoint"] in GOALS], ["blue", "green"])
            self.assertLessEqual(result["goal_error"], GOAL_TOLERANCE)
            self.assertLess(result["elapsed"], 90)
            self.assertEqual(len(sink.world_events), 2)
            self.assertFalse(any(state["obstacle_contact"] for state in sink.trace))
            self.assertGreater(min(state["obstacle_clearance"] for state in sink.trace), 0.05)
        finally:
            sink.close()

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
