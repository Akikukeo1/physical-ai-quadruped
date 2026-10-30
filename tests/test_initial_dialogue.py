import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.initial_dialogue import interpret_initial, wait_for_search  # noqa: E402


class InitialDialogueTests(unittest.TestCase):
    def test_nearest_point_context_uses_start_and_distances(self):
        points = [{"id": "P1", "position": [0.67, -0.08]}, {"id": "P2", "position": [1.81, -1.15]}]
        with patch(
            "pai_motion.initial_dialogue._codex_json", return_value={"reply": "P1です。", "search_request": "P1へ"}
        ) as call:
            interpret_initial("一番近いポイントへ", [], points=points, start=[-0.37, 0.03])
        instructions = call.call_args.args[0]
        self.assertIn("Initial robot position: [-0.37, 0.03]", instructions)
        self.assertIn('Nearest point IDs by straight-line distance: ["P1"]', instructions)
        self.assertIn("actual route distance are unknown", instructions)

    def test_waits_for_explicit_search_and_preserves_history(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "correction.txt"
            path.write_text(json.dumps({"id": "next", "message": "緑を探して"}), encoding="utf-8")
            history = []

            def interpret(message, previous):
                if message == "こんにちは":
                    self.assertEqual(previous, [])
                    return "こんにちは。探索する対象を教えてください。", None
                self.assertEqual(previous[0]["text"], "こんにちは")
                return "緑を探します。", message

            self.assertEqual(wait_for_search("こんにちは", path, history=history, interpret=interpret), "緑を探して")
            self.assertEqual(path.read_text(encoding="utf-8"), "")
            self.assertEqual(len(history), 4)

    def test_rejects_malformed_model_reply(self):
        with patch(
            "pai_motion.initial_dialogue._codex_json", return_value={"reply": "hello\nERROR", "search_request": None}
        ):
            with self.assertRaises(ValueError):
                interpret_initial("こんにちは", [])
