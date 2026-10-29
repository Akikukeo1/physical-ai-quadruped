import sys
import threading
import unittest
from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.agent_planner import validate_agent_revision, validate_frontier_choice  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import execute_search, sample_search  # noqa: E402
from tests.test_dialogue import messages  # noqa: E402
from tests.test_field_areas import AREAS, MovingFixture  # noqa: E402


class AgentTests(unittest.TestCase):
    def test_combined_revision_and_atomic_clarification(self):
        message = "エリア2を避けて、緑を再確認してから青を続けて"
        context = {
            "pending": [
                {"id": "search-1", "action": "search", "target": "blue", "observation_id": None},
                {"id": "home", "action": "return_home", "target": None, "observation_id": None},
            ],
            "completed": [],
            "cancelled": [],
            "dialogue": [],
            "field_areas": AREAS,
            "observations": [{"id": "green-1", "color": "green"}],
        }
        value = {
            "intent": "緑を再確認し、エリア2を避ける",
            "status": "accept",
            "question": "",
            "steps": [
                {"id": "green-check", "action": "reobserve", "target": "green", "observation_id": "green-1"},
                *context["pending"],
            ],
            "cancelled": [],
            "changes": [{"id": "area2", "permission": "avoid", "instruction": message}],
        }
        revision = validate_agent_revision(value, context=context, prompt=message)
        self.assertEqual(revision.steps[0]["action"], "reobserve")
        self.assertEqual(
            validate_agent_revision({**value, "reply": "了解です"}, context=context, prompt=message), revision
        )
        with self.assertRaises(InvalidPlan):
            validate_agent_revision({**value, "reply": "bad\nreply"}, context=context, prompt=message)
        for invalid in (
            {**value, "steps": value["steps"][:-1]},
            {**value, "changes": [{"id": "unknown", "permission": "avoid", "instruction": message}]},
            {**value, "status": "clarify", "question": "確認しますか？"},
        ):
            with self.assertRaises(InvalidPlan):
                validate_agent_revision(invalid, context=context, prompt=message)

    def test_frontier_choice_must_be_supplied(self):
        candidates = [{"id": "frontier-1", "position": [0.5, 0.5]}]
        self.assertEqual(
            validate_frontier_choice({"id": "frontier-1", "reason": "探索"}, candidates)["id"], "frontier-1"
        )
        with self.assertRaises(InvalidPlan):
            validate_frontier_choice({"id": "frontier-2", "reason": "探索"}, candidates)

    def test_conversation_reply_keeps_tasks(self):
        sink, revisions = MovingFixture(), []

        def planner(prompt, plan, context):
            return {
                "intent": "会話",
                "status": "accept",
                "question": "",
                "reply": "会話できます。",
                "steps": deepcopy(context["pending"]),
                "cancelled": [],
                "changes": [],
            }

        output = StringIO()
        with patch("pai_motion.search.time.sleep"), redirect_stdout(output):
            execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(2, "会話できますか？")]),
                correction_planner=planner,
                agent_revision=True,
            )
        self.assertIn("REPLY: 会話できます。", output.getvalue())
        self.assertIn("UNCHANGED: tasks=['search:blue', 'return_home:None']", output.getvalue())
        self.assertEqual(revisions[0]["status"], "accepted")

    def test_unchanged_initial_reply_starts_frontier_exploration(self):
        sink, revisions, decisions = MovingFixture(), [], []

        def planner(prompt, plan, context):
            return {
                "intent": "探索を続ける",
                "status": "accept",
                "question": "",
                "reply": "了解です。",
                "steps": deepcopy(context["pending"]),
                "cancelled": [],
                "changes": [],
            }

        def frontier(context, candidates):
            return {"id": candidates[0]["id"], "reason": "探索"}

        with patch("pai_motion.search.time.sleep"):
            execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=decisions,
                corrections=revisions,
                get_correction=messages([(0, "青を探して")]),
                correction_planner=planner,
                agent_revision=True,
                frontier_planner=frontier,
            )
        self.assertTrue(any(item["phase"] == "frontier_selected" for item in decisions))

    def test_same_correction_changes_tasks_and_area(self):
        class Fixture(MovingFixture):
            def __init__(self):
                super().__init__()
                self.field_states = []

            def set_field_permissions(self, blocked):
                self.field_states.append(set(blocked))

        sink, revisions = Fixture(), []
        message = "エリア2を避けて、緑を再確認してから青を続けて"

        def planner(prompt, plan, context):
            green = next(o for o in context["observations"] if o["color"] == "green")
            return {
                "intent": "緑を再確認",
                "status": "accept",
                "question": "",
                "steps": [
                    {"id": "green-check", "action": "reobserve", "target": "green", "observation_id": green["id"]},
                    *deepcopy(context["pending"]),
                ],
                "cancelled": [],
                "changes": [{"id": "area2", "permission": "avoid", "instruction": prompt}],
            }

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=[],
                corrections=revisions,
                get_correction=messages([(2, message)]),
                correction_planner=planner,
                field_areas=AREAS,
                agent_revision=True,
            )
        self.assertEqual(revisions[0]["status"], "accepted")
        self.assertEqual(sink.field_states, [{"area2"}])
        self.assertEqual(
            [x["step"]["action"] for x in result["tasks"]["completed"]], ["reobserve", "search", "return_home"]
        )

    def test_llm_selects_only_safe_frontier(self):
        sink, decisions, calls = MovingFixture(), [], []

        def frontier(context, candidates):
            calls.append((deepcopy(context), deepcopy(candidates)))
            return {"id": candidates[-1]["id"], "reason": "別の候補を確認"}

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(), sink, observations=[], decisions=decisions, frontier_planner=frontier
            )
        self.assertTrue(calls)
        selections = [d for d in decisions if d["phase"] == "frontier_selected"]
        self.assertTrue(selections)
        for event in selections:
            self.assertIn(event["choice"]["id"], [candidate["id"] for candidate in event["candidates"]])
        self.assertEqual(result["outcome"], "found")
        self.assertNotIn("scene", calls[0][0])

    def test_user_correction_supersedes_pending_frontier_choice(self):
        sink, decisions, revisions = MovingFixture(), [], []
        release = threading.Event()
        sent = False

        def frontier(context, candidates):
            release.wait(timeout=2)
            return {"id": candidates[0]["id"], "reason": "探索"}

        def correction_source(state):
            nonlocal sent
            if not sent and state["time"] >= 1:
                sent = True
                release.set()
                return "青の探索を続けて"
            return None

        def correction(message, plan, context):
            return {
                "intent": "青の探索を継続",
                "status": "accept",
                "question": "",
                "steps": deepcopy(context["pending"]),
                "cancelled": [],
                "changes": [],
            }

        with patch("pai_motion.search.time.sleep"):
            result = execute_search(
                sample_search(),
                sink,
                observations=[],
                decisions=decisions,
                get_correction=correction_source,
                correction_planner=correction,
                corrections=revisions,
                agent_revision=True,
                frontier_planner=frontier,
            )
        self.assertTrue(any(d["phase"] == "frontier_cancelled" for d in decisions))
        self.assertEqual(revisions[0]["status"], "accepted")
        self.assertEqual(result["outcome"], "found")


if __name__ == "__main__":
    unittest.main()
