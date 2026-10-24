import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import unittest

from pai_motion.executor import DryRunSink, execute_plan
from pai_motion.motion import Action, InvalidPlan, MotionPlan, validate_plan
from pai_motion.planner import sample_plan


class TestPlans(unittest.TestCase):
    def test_sample_plan(self) -> None:
        plan = sample_plan()
        self.assertEqual(len(plan.actions), 4)
        self.assertEqual(plan.actions[1].type, "turn_right")

    def test_invalid_action(self) -> None:
        with self.assertRaises(InvalidPlan):
            validate_plan({"intent": "test", "actions": [{"type": "run_shell", "duration": 1.0}]})

    def test_reject_infinity(self) -> None:
        with self.assertRaises(InvalidPlan):
            validate_plan({"intent": "test", "actions": [{"type": "forward", "duration": float("inf")}]})

    def test_reject_boolean_duration(self) -> None:
        with self.assertRaises(InvalidPlan):
            validate_plan({"intent": "test", "actions": [{"type": "wait", "duration": True}]})

    def test_reject_extra_fields(self) -> None:
        with self.assertRaises(InvalidPlan):
            validate_plan({"intent": "test", "actions": [{"type": "wait", "duration": 1, "shell": "rm -rf /"}]})

    def test_reject_too_long(self) -> None:
        with self.assertRaises(InvalidPlan):
            validate_plan({"intent": "test", "actions": [{"type": "forward", "duration": 2.0}] * 8})

    def test_executor_stops_after_error(self) -> None:
        class FailingSink:
            stopped = False

            def run_velocity(self, _cmd, _duration):
                raise RuntimeError("sim failed")

            def stop(self):
                self.stopped = True

        sink = FailingSink()
        with self.assertRaises(RuntimeError):
            execute_plan(sample_plan(), sink)
        self.assertTrue(sink.stopped)

    def test_dry_run(self) -> None:
        sink = DryRunSink()
        execute_plan(sample_plan(), sink)
        self.assertEqual(len(sink.events), 4)
        self.assertLess(sink.events[1]["yaw"], 0)

    def test_executor_revalidates_constructed_plans_before_motion(self) -> None:
        sink = DryRunSink()
        with self.assertRaises(InvalidPlan):
            execute_plan(MotionPlan("invalid", (Action("forward", 100),)), sink)
        self.assertEqual(sink.events, [])


if __name__ == "__main__":
    unittest.main()
