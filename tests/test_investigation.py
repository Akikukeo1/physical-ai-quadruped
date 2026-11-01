import json
import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.investigation import (  # noqa: E402
    codex_investigation,
    execute_investigation,
    load_knowledge,
    validate_decision,
)
from pai_motion.investigation_world import warehouse_scene  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402


def decision(equipment="cooler", observations=(), cause=None):
    ids = [o["id"] for o in observations]
    return {
        "action": "report" if cause else "inspect",
        "equipment": None if cause else equipment,
        "hypotheses": [
            {"cause": c, "status": "supported" if c == cause else "possible", "evidence_ids": ids if c == cause else []}
            for c in ("power_loss", "cooler_fault", "door_open")
        ],
        "reason": "受電と運転状態を照合する",
        "expected_information": "受電が正常なら冷却機を疑う",
        "conclusion": cause or "pending",
        "evidence_ids": ids,
        "report": "観測された電圧と運転状態が根拠です。" if cause else "",
        "uncertainties": "複数故障は未評価",
        "knowledge_ids": ["MAN-POWER"],
    }


class KinematicSink:
    """Test double only; real physics is verified separately by scripts/investigate.py."""

    def __init__(self):
        self.time, self.x, self.y, self.yaw = 0.0, 0.0, 0.0, 0.0
        self.stopped = False
        self.commands = []

    def observe(self):
        return {
            "time": self.time,
            "position": [self.x, self.y, 0.35],
            "yaw": self.yaw,
            "body_velocity": [0, 0, 0],
            "yaw_rate": 0,
        }

    def sense(self):
        return {**self.observe(), "scan": [[i * math.tau / 180, 1.8, False, None] for i in range(180)]}

    def run_velocity(self, cmd, seconds):
        self.commands.append(cmd)
        assert 0 <= cmd.vx <= 0.25 and abs(cmd.yaw) <= 0.6
        self.x += cmd.vx * math.cos(self.yaw) * seconds
        self.y += cmd.vx * math.sin(self.yaw) * seconds
        self.yaw += cmd.yaw * seconds
        self.time += seconds

    def stop(self):
        self.time += 2
        self.stopped = True


class InvestigationTests(unittest.TestCase):
    def context(self):
        return {
            "inventory": warehouse_scene().inventory(),
            "observations": [],
            "unavailable": [],
            "knowledge": load_knowledge(),
        }

    def test_inspection_proximity_stationary_and_no_truth_in_inventory(self):
        world = warehouse_scene()
        sink = KinematicSink()
        with self.assertRaises(ValueError):
            world.inspect("supply", sink.observe())
        sink.x, sink.y = world.geometry["points"][0]["position"]
        state = sink.observe()
        reading = world.inspect("supply", state)
        self.assertEqual(reading["readings"]["output_voltage_v"], 0)
        self.assertEqual(reading["interface"], "simulated_local_inspection")
        with self.assertRaises(ValueError):
            world.inspect("supply", {**state, "body_velocity": [0.1, 0, 0]})
        self.assertEqual(world.inventory(), warehouse_scene(fault="cooler_fault").inventory())
        self.assertNotIn("readings", json.dumps(world.inventory()))
        reading["readings"]["output_voltage_v"] = 999
        self.assertEqual(world.inspect("supply", state)["readings"]["output_voltage_v"], 0)

    def test_validator_rejects_invented_evidence_code_revisits_and_unsupported_diagnosis(self):
        context = self.context()
        value = decision()
        self.assertEqual(validate_decision(value, context), value)
        for change in (
            {"equipment": "unknown"},
            {"code": "print(1)"},
            {"evidence_ids": ["invented"]},
            {"action": "run_velocity"},
            {"conclusion": "power_loss"},
            {"knowledge_ids": ["invented-manual"]},
            {"knowledge_ids": ["inspection-1"]},
        ):
            with self.subTest(change=change), self.assertRaises(InvalidPlan):
                validate_decision({**value, **change}, context)
        with self.assertRaises(InvalidPlan):
            validate_decision(decision(cause="power_loss"), context)
        context["unavailable"] = ["cooler"]
        with self.assertRaises(InvalidPlan):
            validate_decision(value, context)
        context["unavailable"] = []
        context["observations"] = [{"id": "i1", "equipment": "cooler"}]
        with self.assertRaises(InvalidPlan):
            validate_decision(value, context)
        with self.assertRaises(InvalidPlan):
            validate_decision(decision(observations=context["observations"], cause="power_loss"), context)

    def test_codex_prompt_receives_only_supplied_context_and_uses_existing_cli(self):
        context = self.context()
        with patch("pai_motion.investigation._codex_json", return_value=decision()) as call:
            codex_investigation(context)
        instructions, schema, timeout = call.call_args.args
        self.assertIn(json.dumps(context, ensure_ascii=False), instructions)
        self.assertNotIn("output_voltage_v", instructions)
        self.assertNotIn("seed", instructions)
        self.assertNotIn("obstacles", instructions)
        self.assertEqual(schema.name, "investigation-decision.schema.json")
        self.assertEqual(timeout, 45)

    def test_loop_preserves_observed_history_reports_then_returns(self):
        world = warehouse_scene()
        inventory = world.inventory()
        # Put stations close together for this unit test. No B layout is inspected during development.
        for i, point in enumerate(world.geometry["points"]):
            point["position"] = [0.6, (i - 1) * 0.3]
        inventory = world.inventory()
        contexts = []

        def planner(context):
            contexts.append(deepcopy(context))
            observations = context["observations"]
            if not observations:
                return decision("cooler")
            if len(observations) == 1:
                return decision("supply", observations)
            return decision(observations=observations, cause="power_loss")

        sink = KinematicSink()
        log = execute_investigation("原因を調べて帰還", sink, inventory, world.inspect, planner=planner)
        self.assertTrue(log["returned"], log["errors"])
        self.assertTrue(log["within_deadline"])
        self.assertEqual([o["equipment"] for o in log["observations"]], ["cooler", "supply"])
        self.assertEqual([len(c["observations"]) for c in contexts], [0, 1, 2])
        self.assertEqual(log["report"]["conclusion"], "power_loss")
        self.assertLessEqual(log["reported_at"], log["return_started"])
        self.assertTrue(world.evaluate(log["report"], log["observations"])["evidence_sufficient"])
        self.assertTrue(sink.stopped)

    def test_planner_error_reports_inconclusive_and_returns(self):
        sink, world = KinematicSink(), warehouse_scene()

        def fail(context):
            raise TimeoutError("test timeout")

        log = execute_investigation("調査", sink, world.inventory(), world.inspect, planner=fail)
        self.assertEqual(log["report"]["conclusion"], "inconclusive")
        self.assertEqual(log["report"]["source"], "executor_failure_report")
        self.assertTrue(log["returned"])
        self.assertIn("test timeout", log["errors"][0])

    def test_evaluator_does_not_accept_correct_label_without_two_supporting_readings(self):
        world = warehouse_scene()
        report = {"conclusion": "power_loss", "evidence_ids": []}
        evaluation = world.evaluate(report, [])
        self.assertTrue(evaluation["correct_cause"])
        self.assertFalse(evaluation["evidence_sufficient"])

    def test_physics_failure_stops_without_attempting_more_motion(self):
        class FailedSink(KinematicSink):
            def run_velocity(self, cmd, seconds):
                if cmd.vx:
                    raise RuntimeError("simulated safety stop")
                super().run_velocity(cmd, seconds)

        sink, world = FailedSink(), warehouse_scene()
        log = execute_investigation("調査", sink, world.inventory(), world.inspect, planner=lambda context: decision())
        self.assertFalse(log["returned"])
        self.assertEqual(log["report"]["source"], "executor_safety_report")
        self.assertEqual(log["errors"], ["execution: simulated safety stop"])
        self.assertTrue(sink.stopped)

    def test_unreachable_station_is_not_an_equipment_reading(self):
        from pai_motion.investigation import InvestigationMotion

        world, sink = warehouse_scene(), KinematicSink()
        contexts = []

        def planner(context):
            contexts.append(context)
            return decision(cause="inconclusive") if context["unavailable"] else decision()

        def go(motion, target, *, returning=False):
            return "reached" if returning else "unreachable"

        with patch.object(InvestigationMotion, "go", go):
            log = execute_investigation("調査", sink, world.inventory(), world.inspect, planner=planner)
        self.assertEqual(contexts[1]["unavailable"], ["cooler"])
        self.assertEqual(contexts[1]["observations"], [])
        self.assertEqual(log["report"]["conclusion"], "inconclusive")
        self.assertTrue(log["returned"])


if __name__ == "__main__":
    unittest.main()
