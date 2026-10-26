"""The offline constraint protocol must preserve obligations and reject invented evidence."""

import json
import sys
import tempfile
import unittest
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.constraint_revision import codex_constraint_revision, validate_constraint_revision  # noqa: E402
from pai_motion.correction import initial_tasks  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import sample_search  # noqa: E402
from scripts import probe_constraint_revision  # noqa: E402


class TestConstraintRevision(unittest.TestCase):
    def setUp(self):
        self.message = "緑を見た地点の半径0.5mは避けて"
        self.area = {"id": "avoid-green", "observation_id": "obs-green", "radius": 0.5, "instruction": self.message}
        self.context = {
            "pending": initial_tasks("blue"),
            "completed": [],
            "cancelled": [],
            "observations": [{"id": "obs-green", "color": "green", "position": [1, 2]}],
            "constraints": [],
            "released_constraints": [],
            "dialogue": [],
        }
        self.value = {
            "intent": "緑の観測地点を避ける",
            "status": "accept",
            "steps": initial_tasks("blue"),
            "cancelled": [],
            "question": "",
            "constraints": [self.area],
            "released_constraints": [],
        }

    def test_add_keep_and_explicit_release_preserve_home_and_old_constraint_id(self):
        self.assertEqual(
            validate_constraint_revision(self.value, context=self.context, prompt=self.message), self.value
        )
        context = {**self.context, "constraints": [self.area]}
        keep = {**self.value, "intent": "青を探索", "constraints": [self.area]}
        validate_constraint_revision(keep, context=context, prompt="青の探索は続けて")
        with self.assertRaisesRegex(InvalidPlan, "silently disappear"):
            validate_constraint_revision({**keep, "constraints": []}, context=context, prompt="青の探索は続けて")
        message = "緑の地点の立入制限を解除して"
        release = {**keep, "constraints": [], "released_constraints": [{"id": "avoid-green", "instruction": message}]}
        validate_constraint_revision(release, context=context, prompt=message)
        retired = {**self.context, "released_constraints": [{"constraint": self.area}]}
        with self.assertRaisesRegex(InvalidPlan, "reused"):
            validate_constraint_revision(self.value, context=retired, prompt=self.message)

    def test_reject_fabricated_reference_quote_radius_and_unapproved_edits(self):
        for field, value in (
            ("observation_id", "missing"),
            ("radius", float("nan")),
            ("radius", True),
            ("radius", 10**400),
            ("radius", 1.6),
            ("instruction", "捏造された指示"),
        ):
            invalid = deepcopy(self.value)
            invalid["constraints"][0][field] = value
            with self.subTest(field=field, value=str(value)[:30]), self.assertRaises(InvalidPlan):
                validate_constraint_revision(invalid, context=self.context, prompt=self.message)
        context = {**self.context, "constraints": [self.area]}
        invalid = deepcopy(self.value)
        invalid["constraints"][0]["radius"] = 1.0
        with self.assertRaisesRegex(InvalidPlan, "change meaning"):
            validate_constraint_revision(invalid, context=context, prompt=self.message)
        release = {
            **self.value,
            "constraints": [],
            "released_constraints": [{"id": "avoid-green", "instruction": "解除して"}],
        }
        with self.assertRaisesRegex(InvalidPlan, "quote"):
            validate_constraint_revision(release, context=context, prompt="青を続けて")

    def test_clarification_cannot_apply_or_release_a_constraint(self):
        context = {**self.context, "constraints": [self.area]}
        question = {**self.value, "status": "clarify", "question": "制限を解除しますか？"}
        validate_constraint_revision(question, context=context, prompt="緑を再確認して")
        invalid = {
            **question,
            "constraints": [],
            "released_constraints": [{"id": "avoid-green", "instruction": "解除して"}],
        }
        with self.assertRaisesRegex(InvalidPlan, "Clarification cannot change"):
            validate_constraint_revision(invalid, context=context, prompt="解除して")

    def test_model_receives_context_and_only_experimental_schema(self):
        with patch("pai_motion.planner._codex_json", return_value=self.value) as model:
            result = codex_constraint_revision(self.message, sample_search(), self.context)
        self.assertEqual(result["constraints"][0]["observation_id"], "obs-green")
        self.assertEqual(model.call_args.args[1].name, "constraint-revision.schema.json")
        self.assertIn("not around the object's hidden position", model.call_args.args[0])
        self.assertIn("latest matching observation", model.call_args.args[0])

    def test_probe_sends_prior_constraint_and_question_then_explicit_release(self):
        messages = probe_constraint_revision.MESSAGES
        area = {**self.area, "instruction": messages[0]}
        added = {**self.value, "constraints": [area]}
        clarification = {**added, "status": "clarify", "question": "禁止を解除しますか？"}
        reobserve = {"id": "recheck", "action": "reobserve", "target": "green", "observation_id": "obs-green"}
        released = {
            **added,
            "steps": [reobserve, *self.context["pending"]],
            "constraints": [],
            "released_constraints": [{"id": "avoid-green", "instruction": "立入禁止を解除して"}],
        }
        parent = Path(__file__).resolve().parents[1] / "outputs/test-tmp"
        parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as directory:
            root = Path(directory)
            snapshot = root / "snapshot.json"
            snapshot.write_text(
                json.dumps({"corrections": [{"context": self.context, "original_plan": asdict(sample_search())}]}),
                encoding="utf-8",
            )
            with (
                patch.object(sys, "argv", ["probe", str(snapshot), "--out-dir", str(root / "results")]),
                patch("pai_motion.planner._codex_json", side_effect=[added, clarification, released]) as model,
            ):
                self.assertEqual(probe_constraint_revision.main(), 0)
                self.assertEqual(model.call_count, 3)
            report = json.loads(next((root / "results").glob("*.json")).read_text(encoding="utf-8"))
            self.assertEqual([row["correct"] for row in report["results"]], [True, True, True])
            self.assertEqual(report["results"][1]["context"]["constraints"], [area])
            self.assertEqual(report["results"][2]["context"]["dialogue"][-1]["role"], "assistant")
