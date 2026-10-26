"""Repeated corrections, clarification holding and expiry."""

import json
import math
import sys
import tempfile
import unittest
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from main import main  # noqa: E402
from pai_motion.correction import SearchCorrection, initial_tasks, validate_correction  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import execute_search, sample_search, validate_search  # noqa: E402
from tests.test_task_revision import RecheckingSensor, recheck_then_search  # noqa: E402


def messages(events):
    queue = list(enumerate(events))

    def receive(state):
        if queue and state["time"] >= queue[0][1][0] - 1e-9:
            index, (_, text) = queue.pop(0)
            return {"id": str(index), "message": text}
        return None

    return receive


def ask(prompt, plan, context):
    return validate_correction(
        {
            "intent": "対象色を確認",
            "status": "clarify",
            "steps": context["pending"],
            "cancelled": [],
            "question": "青と緑のどちらを再確認しますか？",
        },
        context=context,
    )


class TestDialogue(unittest.TestCase):
    def test_cli_script_bounds_and_consumption_before_creating_physics(self):
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "input.json"
            args = [
                "main",
                "--task",
                "search",
                "--executor",
                "mujoco",
                "--correction-script",
                str(script),
                "--out-dir",
                directory,
            ]
            for events in (
                [],
                [{"at": True, "message": "緑"}],
                [{"at": 180, "message": "緑"}],
                [{"at": 3, "message": "緑"}, {"at": 2, "message": "青"}],
            ):
                script.write_text(json.dumps(events), encoding="utf-8")
                with (
                    self.subTest(events=events),
                    patch.object(sys, "argv", args),
                    patch("pai_motion.sim_adapter.MuJoCoGo2Sink") as factory,
                ):
                    self.assertEqual(main(), 1)
                    factory.assert_not_called()
            script.write_text(json.dumps([{"at": 2, "message": "緑"}, {"at": 3, "message": "青"}]), encoding="utf-8")

            def execute(plan, sink, **kwargs):
                receive = kwargs["get_correction"]
                self.assertIsNone(receive({"time": 2}))
                first = receive({"time": 3})
                second = receive({"time": 4})
                self.assertEqual((first["message"], second["message"]), ("緑", "青"))
                self.assertNotEqual(first["id"], second["id"])
                self.assertIsNone(receive({"time": 10}))
                return {"outcome": "unit fixture"}

            with (
                patch.object(sys, "argv", args),
                patch("pai_motion.sim_adapter.MuJoCoGo2Sink") as factory,
                patch("main.execute_search", side_effect=execute),
            ):
                sink = factory.return_value
                sink.events, sink.trace, sink.world_events = [], [], []
                sink.observe.return_value = {"time": 1.0}
                self.assertEqual(main(), 0)

    def test_reobservation_approaches_actual_viewpoint_before_declaring_missing_target(self):
        class ViewpointFixture(RecheckingSensor):
            # Unit kinematics only: sensor footprint is narrower than the old 15 cm arrival threshold.
            x, drive = 0.0, False

            def observe(self):
                if self.time >= 1 and not self.drive:
                    self.x = 0.14
                return {**super().observe(), "position": [self.x, 0.0, 0.35], "yaw": math.pi}

            def sense(self):
                observation = super().sense()
                if self.time >= 1 and self.x > 0.04:
                    for ray in observation["scan"]:
                        if ray[3] == "green":
                            ray[3] = None
                return observation

            def run_velocity(self, cmd, seconds):
                super().run_velocity(cmd, seconds)
                if self.drive:
                    self.x = max(0, self.x - cmd.vx * seconds)

        sink = ViewpointFixture()

        def interpret(prompt, plan, context):
            sink.drive = True
            return recheck_then_search(prompt, plan, context)

        result = execute_search(
            sample_search(),
            sink,
            observations=[],
            decisions=[],
            get_correction=messages([(2, "再確認")]),
            correction_planner=interpret,
        )
        self.assertEqual(result["tasks"]["completed"][0]["step"]["action"], "reobserve")
        self.assertLessEqual(result["tasks"]["completed"][0]["evidence"]["position"][0], 0.04)

    def test_question_holds_then_answer_resumes_with_original_blue_and_deadline(self):
        def interpret(prompt, plan, context):
            if prompt == "あっちを確認":
                return ask(prompt, plan, context)
            self.assertEqual(context["dialogue"][-2]["role"], "assistant")
            return recheck_then_search(prompt, plan, context)

        sink, revisions = RecheckingSensor(), []
        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                get_correction=messages([(2, "あっちを確認"), (4, "緑です")]),
                correction_planner=interpret,
                corrections=revisions,
            )
        self.assertEqual([r["status"] for r in revisions], ["clarification", "accepted"])
        self.assertEqual([e["step"]["id"] for e in result["tasks"]["completed"]], ["recheck-green", "search-1", "home"])
        self.assertEqual([r["deadline"] for r in revisions], [180, 180])
        self.assertTrue(
            all(
                c["cmd"].vx == c["cmd"].yaw == 0
                for c in sink.commands
                if revisions[0]["received"]["time"] <= c["time"] < revisions[1]["applied_time"]
            )
        )

    def test_second_revision_cancels_blue_but_retains_active_reobservation_reference(self):
        seen_reference = []

        def interpret(prompt, plan, context):
            if prompt == "再確認":
                correction = recheck_then_search(prompt, plan, context)
                seen_reference.append(correction.steps[0]["observation_id"])
                return correction
            self.assertIn(seen_reference[0], [r["id"] for r in context["observations"]])
            return validate_correction(
                {
                    "intent": "青を取り消す",
                    "status": "accept",
                    "steps": [s for s in context["pending"] if s["target"] != "blue"],
                    "cancelled": [{"id": "search-1", "instruction": prompt}],
                },
                context=context,
                prompt=prompt,
            )

        revisions = []
        result = execute_search(
            sample_search(),
            RecheckingSensor(),
            observations=[],
            decisions=[],
            get_correction=messages([(2, "再確認"), (2.6, "青を中止")]),
            correction_planner=interpret,
            corrections=revisions,
        )
        self.assertEqual(len(revisions), 2)
        self.assertEqual(result["tasks"]["cancelled"][0]["step"]["id"], "search-1")
        self.assertEqual([e["step"]["action"] for e in result["tasks"]["completed"]], ["reobserve", "return_home"])
        self.assertIsNone(result["found"])

    def test_no_answer_returns_on_original_budget_without_pretending_task_completed(self):
        plan = validate_search({**asdict(sample_search()), "seconds": 60})
        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                plan,
                RecheckingSensor(),
                observations=[],
                decisions=[],
                get_correction=messages([(2, "あっち")]),
                correction_planner=ask,
            )
        self.assertEqual(result["outcome"], "not_found")
        self.assertEqual(result["tasks"]["pending"], initial_tasks("blue")[:-1])
        self.assertLess(result["elapsed"], 60)

    def test_clarification_cannot_change_tasks_or_cancel_goals(self):
        context = {"pending": initial_tasks("blue"), "completed": [], "observations": []}
        value = asdict(ask("", sample_search(), context))
        for change in ({"steps": context["pending"][-1:]}, {"question": ""}, {"question": "x" * 201}):
            with self.subTest(change=change), self.assertRaises(InvalidPlan):
                validate_correction({**value, **change}, context=deepcopy(context))
        context["cancelled"] = [{"step": {"id": "old"}}]
        value = asdict(
            SearchCorrection(
                "再利用",
                "accept",
                [{"id": "old", "action": "search", "target": "green", "observation_id": None}, *context["pending"]],
                [],
            )
        )
        with self.assertRaises(InvalidPlan):
            validate_correction(value, context=context)

    def test_event_ids_allow_same_text_twice_without_replaying_same_event(self):
        calls = []

        def interpret(prompt, plan, context):
            calls.append(prompt)
            return validate_correction(
                {"intent": "維持", "status": "accept", "steps": context["pending"], "cancelled": []}, context=context
            )

        result = execute_search(
            sample_search(),
            RecheckingSensor(),
            observations=[],
            decisions=[],
            get_correction=messages([(1, "そのまま"), (2, "そのまま")]),
            correction_planner=interpret,
        )
        self.assertEqual(calls, ["そのまま", "そのまま"])
        self.assertEqual(result["outcome"], "found")
