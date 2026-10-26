"""Evaluation must not turn safe fallback, stale sensing, or excluded opportunities into success."""

import sys
import unittest
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scripts.evaluate_corrections import message_source, observed_context, score  # noqa: E402


def scan(time, color=None):
    return {"time": time, "scan": [[0, 1, True, color], [0.1, 1, True, color]]}


class TestCorrectionEvaluation(unittest.TestCase):
    def test_ablation_removes_only_stale_unreferenced_observations_without_mutation(self):
        context = {
            "pending": [{"action": "reobserve", "observation_id": "active"}],
            "latest_scan": {"time": 10},
            "dialogue": [{"role": "user", "text": "緑です"}],
            "observations": [{"id": "old", "time": 2}, {"id": "active", "time": 3}, {"id": "now", "time": 10}],
        }
        view = observed_context(context, "current_only")
        self.assertEqual([o["id"] for o in view["observations"]], ["active", "now"])
        self.assertEqual(view["dialogue"], context["dialogue"])
        view["pending"].clear()
        self.assertEqual(len(context["pending"]), 1)
        self.assertEqual(len(context["observations"]), 3)
        self.assertEqual(observed_context(context, "history"), context)

    def test_trigger_uses_past_sensing_and_second_message_follows_either_outcome(self):
        for status, field in (("accepted", "applied_time"), ("clarification", "question_time")):
            observations, turns = [scan(0, "green")], []
            receive = message_source(observations, turns, ["first", "second"], 0)
            self.assertIsNone(receive({"time": 1}))
            observations.append(scan(2))
            self.assertEqual(receive({"time": 2})["message"], "first")
            turns.append({"status": status, field: 3})
            self.assertIsNone(receive({"time": 4}))
            self.assertEqual(receive({"time": 5})["message"], "second")
            self.assertIsNone(receive({"time": 6}))
        for observations, time in (
            ([scan(0, "green"), scan(1, "blue"), scan(2)], 2),
            ([scan(0, "green"), scan(2)], 91),
            ([scan(0)], 2),
        ):
            self.assertIsNone(message_source(observations, [], ["first", "second"], 0)({"time": time}))

    def test_score_requires_fresh_sensing_cancellation_home_and_safe_waits(self):
        step = {"id": "recheck", "action": "reobserve", "target": "green"}
        log = {
            "started": 0,
            "home": [0, 0],
            "observations": [scan(8, "green")],
            "decisions": [{"phase": "task_started", "step": step, "time": 6}],
            "corrections": [
                {
                    "message": "先に緑",
                    "status": "accepted",
                    "received": {"time": 2},
                    "applied_time": 3,
                    "deadline": 180,
                    "home": [0, 0],
                },
                {
                    "message": "青は取消し",
                    "status": "accepted",
                    "received": {"time": 4},
                    "applied_time": 5,
                    "deadline": 180,
                    "home": [0, 0],
                },
            ],
            "events": [{"start": {"time": t}, "vx": 0, "vy": 0, "yaw": 0} for t in (2, 4)],
            "trace": [{"finite": True, "upright": 1, "obstacle_contact": False}],
            "result": {
                "outcome": "returned",
                "elapsed": 10,
                "home_error": 0.1,
                "tasks": {
                    "pending": [],
                    "completed": [
                        {"step": step, "evidence": {"time": 8}},
                        {"step": {"id": "home", "action": "return_home", "target": None}},
                    ],
                    "cancelled": [{"step": {"id": "search-1"}, "instruction": "青は取消し"}],
                },
            },
        }
        self.assertTrue(score(log)["mission_success"])
        bad = deepcopy(log)
        bad["result"]["tasks"]["completed"][0]["evidence"]["time"] = 1
        self.assertFalse(score(bad)["mission_success"])
        for field, value in (("pending", [{"id": "search-1"}]), ("cancelled", [])):
            bad = deepcopy(log)
            bad["result"]["tasks"][field] = value
            self.assertFalse(score(bad)["mission_success"])
        bad = deepcopy(log)
        bad["events"][0]["vx"] = 0.1
        self.assertFalse(score(bad)["mission_success"])
        bad = deepcopy(log)
        bad["trace"][0]["obstacle_contact"] = True
        self.assertFalse(score(bad)["safe_return"])
        bad = deepcopy(log)
        bad["corrections"] = []
        self.assertIsNone(score(bad)["mission_success"])
        self.assertFalse(score(bad)["triggered"])
