"""The disclosed rule baseline and the observation ablation use the intended inputs."""

import json
import sys
import unittest
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.constraint_revision import validate_constraint_revision  # noqa: E402
from pai_motion.correction import initial_tasks  # noqa: E402
from pai_motion.search import sample_search  # noqa: E402
from scripts.compare_constraint_baselines import current_only, rule_revision  # noqa: E402


class TestConstraintBaselines(unittest.TestCase):
    def setUp(self):
        self.context = {
            "pending": initial_tasks("blue"),
            "completed": [],
            "cancelled": [],
            "dialogue": [],
            "constraints": [],
            "released_constraints": [],
            "latest_scan": {"time": 10},
            "observations": [
                {"id": "green-1", "color": "green", "time": 2},
                {"id": "green-2", "color": "green", "time": 5},
            ],
        }
        path = Path(__file__).resolve().parents[1] / "examples/constraint-dialogue.json"
        self.messages = [e["message"] for e in json.loads(path.read_text(encoding="utf-8"))]

    def test_rules_add_question_and_release_the_original_observation(self):
        context = deepcopy(self.context)
        first = rule_revision(self.messages[0], sample_search(), context)
        validate_constraint_revision(first, context=context, prompt=self.messages[0])
        self.assertEqual(first["constraints"][0]["observation_id"], "green-2")
        context["constraints"] = deepcopy(first["constraints"])
        question = rule_revision(self.messages[1], sample_search(), context)
        validate_constraint_revision(question, context=context, prompt=self.messages[1])
        self.assertEqual(question["status"], "clarify")
        context["observations"].append({"id": "new-green", "color": "green", "time": 9})
        final = rule_revision(self.messages[2], sample_search(), context)
        validate_constraint_revision(final, context=context, prompt=self.messages[2])
        self.assertEqual(final["steps"][0]["observation_id"], "green-2")
        self.assertEqual(final["steps"][1:], context["pending"])
        self.assertEqual(final["constraints"], [])

    def test_no_history_preserves_active_references_and_does_not_mutate_context(self):
        view = current_only(self.context)
        self.assertEqual(view["observations"], [])
        self.assertEqual(len(self.context["observations"]), 2)
        self.context["constraints"] = [
            {"id": "z", "observation_id": "green-1", "radius": 0.5, "instruction": self.messages[0]}
        ]
        view = current_only(self.context)
        self.assertEqual([o["id"] for o in view["observations"]], ["green-1"])
        self.assertEqual(view["pending"], self.context["pending"])
