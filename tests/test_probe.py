"""Infrastructure failures must not masquerade as language errors or trigger a call cascade."""

import json
import sys
import tempfile
import unittest
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.correction import initial_tasks  # noqa: E402
from pai_motion.search import sample_search  # noqa: E402
from scripts import probe_corrections  # noqa: E402


class TestProbe(unittest.TestCase):
    def test_stops_after_infrastructure_failure_and_excludes_it_from_semantic_scoring(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            snapshot = root / "snapshot.json"
            snapshot.write_text(
                json.dumps(
                    {
                        "corrections": [
                            {
                                "original_plan": asdict(sample_search()),
                                "context": {
                                    "pending": initial_tasks("blue"),
                                    "completed": [],
                                    "observations": [{"id": "green", "color": "green"}],
                                    "latest_scan": {"visible_colors": []},
                                },
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            with (
                patch.object(sys, "argv", ["probe", str(snapshot), "--out-dir", str(root / "results")]),
                patch.object(
                    probe_corrections.planner, "codex_correction", side_effect=RuntimeError("usage limit")
                ) as model,
            ):
                with self.assertRaises(SystemExit) as stopped:
                    probe_corrections.main()
                self.assertEqual(stopped.exception.code, 1)
                self.assertEqual(model.call_count, 1)
            report = json.loads(next((root / "results").glob("*.json")).read_text(encoding="utf-8"))
            self.assertEqual(len(report["results"]), 1)
            self.assertTrue(report["results"][0]["infrastructure_error"])
            self.assertIsNone(report["results"][0]["correct"])
