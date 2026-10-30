import sys
import threading
import unittest
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.correction import SearchCorrection  # noqa: E402
from pai_motion.planner import CodexTimeoutError  # noqa: E402
from pai_motion.search import execute_search, sample_search, validate_search  # noqa: E402
from tests.test_dialogue import messages  # noqa: E402
from tests.test_field_areas import AREAS, MovingFixture  # noqa: E402


class TimeoutReturnTests(unittest.TestCase):
    def test_correction_timeout_returns_with_original_tasks(self):
        sink, revisions, decisions = MovingFixture(), [], []

        def timeout(prompt, plan, context):
            raise CodexTimeoutError("Codex timed out after 30 seconds")

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=decisions,
                corrections=revisions,
                get_correction=messages([(2, "緑に変更")]),
                correction_planner=timeout,
            )
        self.assertEqual(result["outcome"], "planner_timeout")
        self.assertLessEqual(result["home_error"], 0.15)
        self.assertEqual(revisions[0]["status"], "timed_out")
        self.assertEqual(result["tasks"]["pending"][0]["target"], "blue")
        self.assertTrue(any(d.get("reason") == "correction planner timeout" for d in decisions))

    def test_frontier_timeout_returns_home(self):
        def timeout(context, candidates):
            raise CodexTimeoutError("Codex timed out after 30 seconds")

        decisions = []
        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(), MovingFixture(), observations=[], decisions=decisions, frontier_planner=timeout
            )
        self.assertEqual(result["outcome"], "planner_timeout")
        self.assertLessEqual(result["home_error"], 0.15)
        self.assertTrue(any(d.get("reason") == "frontier planner timeout" for d in decisions))

    def test_timeout_keeps_existing_area_constraint(self):
        class Fixture(MovingFixture):
            def set_field_permissions(self, blocked):
                self.blocked = set(blocked)

        def planner(prompt, plan, context):
            if prompt == "エリア2を避けて":
                return {
                    "intent": "エリア2を避ける",
                    "status": "accept",
                    "question": "",
                    "reply": "了解です。",
                    "steps": context["pending"],
                    "cancelled": [],
                    "changes": [{"id": "area2", "permission": "avoid", "instruction": prompt}],
                }
            raise CodexTimeoutError("Codex timed out after 30 seconds")

        revisions, sink = [], Fixture()
        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(0, "エリア2を避けて"), (2, "緑に変更")]),
                correction_planner=planner,
                agent_revision=True,
                field_areas=AREAS,
            )
        self.assertEqual(result["outcome"], "planner_timeout")
        self.assertEqual(result["blocked_areas"], ["area2"])
        self.assertEqual(sink.blocked, {"area2"})
        self.assertEqual([r["status"] for r in revisions], ["accepted", "timed_out"])

    def test_return_reserve_discards_late_correction(self):
        revisions = []
        release = threading.Event()

        class Fixture(MovingFixture):
            def stop(self):
                release.set()
                super().stop()

        def slow(prompt, plan, context):
            release.wait(3)
            return SearchCorrection("unchanged", "accept", context["pending"], [])

        plan = validate_search({**asdict(sample_search()), "seconds": 60})
        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                plan,
                Fixture(),
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(2, "変更")]),
                correction_planner=slow,
            )
        self.assertEqual(result["outcome"], "planner_return_reserve")
        self.assertEqual(revisions[0]["status"], "return_reserve")
        self.assertLessEqual(result["home_error"], 0.15)
