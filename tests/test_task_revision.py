"""Task composition, grounded references and fresh-evidence execution."""

import json
import os
import sys
import unittest
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.correction import SearchCorrection, initial_tasks, validate_correction  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.planner import codex_correction  # noqa: E402
from pai_motion.search import execute_search, sample_search, validate_search  # noqa: E402
from tests.test_correction import SensorFixture, after  # noqa: E402


def recheck_then_search(prompt, plan, context):
    record = next(record for record in context["observations"] if record["color"] == "green")
    return validate_correction(
        {
            "intent": "過去に見た緑を再確認してから、青の探索と帰還を続ける",
            "status": "accept",
            "steps": [
                {"id": "recheck-green", "action": "reobserve", "target": "green", "observation_id": record["id"]},
                *context["pending"],
            ],
            "cancelled": [],
        },
        context=context,
        prompt=prompt,
    )


class RecheckingSensor(SensorFixture):
    def sense(self):
        observation = super().sense()
        for ray in observation["scan"]:
            ray[3] = None
        for index in (0, 1):
            if self.time < 1 or self.time >= 3:
                observation["scan"][index] = [index * 0.03, 1.2, True, "green"]
            if self.time >= 6:
                observation["scan"][index + 60] = [2 + index * 0.03, 1.2, True, "blue"]
        return observation


class TestTaskRevision(unittest.TestCase):
    def setUp(self):
        self.context = {
            "pending": initial_tasks("blue"),
            "completed": [],
            "observations": [{"id": "obs-1-green", "color": "green", "time": 0, "position": [0, 0], "range": 1.2}],
        }
        self.valid = asdict(recheck_then_search("緑をもう一度確認してから青を探して", sample_search(), self.context))

    def test_rejects_silent_loss_fabricated_evidence_and_changed_existing_ids(self):
        proposals = []
        omitted = deepcopy(self.valid)
        omitted["steps"].pop(1)
        proposals.append(omitted)
        for field, value in (("observation_id", "invented"), ("target", "blue"), ("position", [1, 2])):
            proposal = deepcopy(self.valid)
            proposal["steps"][0][field] = value
            proposals.append(proposal)
        changed = deepcopy(self.valid)
        changed["steps"][1]["target"] = "green"
        proposals.append(changed)
        duplicate = deepcopy(self.valid)
        duplicate["steps"].insert(0, duplicate["steps"][0])
        proposals.append(duplicate)
        for proposal in proposals:
            with self.subTest(proposal=proposal), self.assertRaises(InvalidPlan):
                validate_correction(proposal, context=self.context, prompt="緑をもう一度確認してから青を探して")
        completed = deepcopy(self.context)
        completed["completed"] = [{"step": self.valid["steps"][0]}]
        with self.assertRaisesRegex(InvalidPlan, "replayed"):
            validate_correction(self.valid, context=completed)

    def test_cancellation_requires_real_task_and_literal_user_quote_and_keeps_home(self):
        proposal = {
            "intent": "探索を中止して帰還",
            "status": "accept",
            "steps": [initial_tasks("blue")[-1]],
            "cancelled": [{"id": "search-1", "instruction": "青の探索を中止"}],
        }
        validate_correction(proposal, context=self.context, prompt="青の探索を中止して帰って")
        for entry in (
            {"id": "home", "instruction": "青の探索を中止"},
            {"id": "unknown", "instruction": "青の探索を中止"},
            {"id": "search-1", "instruction": "存在しない指示"},
        ):
            with self.subTest(entry=entry), self.assertRaises(InvalidPlan):
                validate_correction(
                    {**proposal, "cancelled": [entry]}, context=self.context, prompt="青の探索を中止して帰って"
                )
        with self.assertRaises(InvalidPlan):
            validate_correction({**self.valid, "steps": self.valid["steps"][:-1]}, context=self.context)

    def test_model_receives_observation_references_and_pending_obligations(self):
        with patch("pai_motion.planner._codex_json", return_value=self.valid) as model:
            result = codex_correction("緑をもう一度確認してから青を探して", sample_search(), self.context)
        self.assertEqual(result.steps[1]["target"], "blue")
        instructions = model.call_args.args[0]
        self.assertIn(json.dumps(self.context, ensure_ascii=False), instructions)
        self.assertIn("obs-1-green", instructions)
        self.assertIn("one static target per color", instructions)
        self.assertIn("observation with the greatest time", instructions)
        self.assertIn("Respect explicit earlier/latest sighting", instructions)
        self.assertNotIn("WAYPOINTS", instructions)

    def test_pending_tasks_can_be_reordered_without_cancelling_them(self):
        context = deepcopy(self.context)
        green = {"id": "search-green", "action": "search", "target": "green", "observation_id": None}
        context["pending"].insert(1, green)
        proposal = {
            "intent": "緑を先にする",
            "status": "accept",
            "cancelled": [],
            "steps": [green, *self.context["pending"]],
        }
        result = validate_correction(proposal, context=context)
        self.assertEqual([step["id"] for step in result.steps], ["search-green", "search-1", "home"])

    def test_immediate_return_records_cancellation_without_claiming_discovery(self):
        message = "探索を中止して帰って"

        def return_now(prompt, plan, context):
            return validate_correction(
                {
                    "intent": "探索を中止して帰還",
                    "status": "accept",
                    "steps": [context["pending"][-1]],
                    "cancelled": [{"id": "search-1", "instruction": message}],
                },
                context=context,
                prompt=prompt,
            )

        result = execute_search(
            sample_search(),
            SensorFixture(),
            observations=[],
            decisions=[],
            get_correction=lambda state: message if state["time"] >= 2 else None,
            correction_planner=return_now,
        )
        self.assertEqual(result["outcome"], "returned")
        self.assertIsNone(result["found"])
        self.assertEqual(result["tasks"]["cancelled"][0]["step"]["id"], "search-1")
        self.assertEqual([entry["step"]["id"] for entry in result["tasks"]["completed"]], ["home"])

    def test_planner_cannot_mutate_snapshot_to_authorize_dropping_a_goal(self):
        def mutate_context(prompt, plan, context):
            context["pending"].pop(0)
            return SearchCorrection("帰還", "accept", context["pending"], [])

        sink, revisions = SensorFixture(), []
        with self.assertRaisesRegex(InvalidPlan, "silently disappear"):
            execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                get_correction=after(2),
                correction_planner=mutate_context,
                corrections=revisions,
            )
        self.assertEqual(revisions[0]["context"]["pending"], initial_tasks("blue"))
        self.assertEqual(revisions[0]["status"], "failed")
        self.assertTrue(sink.stopped)

    def test_fresh_reobservation_precedes_original_search_and_context_has_no_hidden_map(self):
        sink, revisions, decisions = RecheckingSensor(), [], []
        result = execute_search(
            sample_search(),
            sink,
            observations=[],
            decisions=decisions,
            get_correction=after(2),
            correction_planner=recheck_then_search,
            corrections=revisions,
        )
        done = result["tasks"]["completed"]
        self.assertEqual([entry["step"]["id"] for entry in done], ["recheck-green", "search-1", "home"])
        self.assertGreater(done[0]["evidence"]["time"], revisions[0]["applied_time"])
        self.assertEqual(result["detections"]["green"]["time"], 0)
        self.assertEqual(result["tasks"]["pending"], [])
        self.assertEqual(result["tasks"]["cancelled"], [])
        context = revisions[0]["context"]
        self.assertEqual(context["deadline"], 180)
        self.assertEqual(context["observations"][0]["position"], [0.0, 0.0])
        self.assertEqual(context["latest_scan"]["visible_colors"], [])
        self.assertNotIn("obstacles", json.dumps(context))
        self.assertNotIn("seed", json.dumps(context))
        self.assertEqual(context["pending"], initial_tasks("blue"))

    def test_old_evidence_alone_never_completes_reobservation(self):
        sink, decisions = SensorFixture(remembered=True), []
        with self.assertRaisesRegex(RuntimeError, "could not be reobserved"):
            execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=decisions,
                get_correction=after(2),
                correction_planner=recheck_then_search,
            )
        self.assertTrue(sink.stopped)
        self.assertFalse(any(d.get("reason") == "fresh target observation" for d in decisions))

    def test_budget_keeps_unfinished_tasks_and_returns_without_claiming_completion(self):
        sink = SensorFixture()
        plan = validate_search({**asdict(sample_search()), "seconds": 60})

        def add_green(prompt, plan, context):
            return validate_correction(
                {
                    "intent": "緑を先に探索",
                    "status": "accept",
                    "cancelled": [],
                    "steps": [
                        {"id": "extra-green", "action": "search", "target": "green", "observation_id": None},
                        *context["pending"],
                    ],
                },
                context=context,
            )

        result = execute_search(
            plan, sink, observations=[], decisions=[], get_correction=after(2), correction_planner=add_green
        )
        self.assertEqual(result["outcome"], "not_found")
        self.assertEqual([step["id"] for step in result["tasks"]["pending"]], ["search-1"])
        self.assertEqual([entry["step"]["id"] for entry in result["tasks"]["completed"]], ["extra-green", "home"])
        self.assertLess(result["elapsed"], 60)


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for real MuJoCo physics tests")
class TestTaskRevisionPhysics(unittest.TestCase):
    def test_reobserve_remembered_green_then_resume_blue_and_return(self):
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        sink = MuJoCoGo2Sink(headless=True, search_seed=305)
        revisions, observations = [], []
        start = sink.observe()["time"]
        try:
            result = execute_search(
                sample_search(),
                sink,
                observations=observations,
                decisions=[],
                get_correction=after(start + 55),
                correction_planner=recheck_then_search,
                corrections=revisions,
            )
            done = result["tasks"]["completed"]
            self.assertEqual([entry["step"]["id"] for entry in done], ["recheck-green", "search-1", "home"])
            self.assertGreater(done[0]["evidence"]["time"], revisions[0]["applied_time"])
            self.assertEqual(result["outcome"], "found")
            self.assertEqual(result["tasks"]["pending"], [])
            self.assertLess(result["elapsed"], 180)
            self.assertLess(result["home_error"], 0.15)
            self.assertFalse(any(state["obstacle_contact"] for state in sink.trace))
        finally:
            sink.close()
