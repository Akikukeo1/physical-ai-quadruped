import json
import tempfile
import unittest
from pathlib import Path

from scripts.interactive import describe_line, write_instruction


class InteractiveTests(unittest.TestCase):
    def test_message_is_atomic_utf8_and_repeated_text_has_new_id(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "correction.txt"
            write_instruction(path, "エリア2を避けて")
            first = json.loads(path.read_text(encoding="utf-8"))
            write_instruction(path, "エリア2を避けて")
            second = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(first["message"], second["message"])
            self.assertNotEqual(first["id"], second["id"])
            with self.assertRaises(ValueError):
                write_instruction(path, " " * 10)
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), second)

    def test_question_and_plan_unlock_input(self):
        self.assertEqual(describe_line("QUESTION: 解除しますか？"), ("ロボットからの確認: 解除しますか？", True))
        text, ready = describe_line("UPDATED: tasks=['search:green', 'search:blue', 'return_home:None']")
        self.assertTrue(ready)
        self.assertEqual(text, "計画を更新: 緑を探索 → 青を探索 → 帰還")
        self.assertFalse(describe_line("FRONTIER: frontier-1; 周囲を調べる")[1])


if __name__ == "__main__":
    unittest.main()
