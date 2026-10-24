import hashlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from main import main
from pai_motion import assets
from pai_motion.planner import DEMO_PLAN, codex_plan


class TestAssets(unittest.TestCase):
    def test_download_cache_and_checksum_rejection(self):
        content = b"pinned asset"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps(
                    {
                        "files": [
                            {
                                "path": "model.xml",
                                "url": "https://example.invalid",
                                "sha256": hashlib.sha256(content).hexdigest(),
                            }
                        ]
                    }
                )
            )
            with (
                patch.object(assets, "MANIFEST", manifest),
                patch.object(assets, "ASSET_DIR", root / "cache"),
                patch.object(assets, "urlopen", return_value=io.BytesIO(content)) as fetch,
            ):
                result = assets.ensure_assets()
                self.assertEqual((result / "model.xml").read_bytes(), content)
                assets.ensure_assets()
                fetch.assert_called_once()
                (result / "model.xml").write_bytes(b"corrupt")
                with self.assertRaisesRegex(RuntimeError, "checksum mismatch"):
                    assets.ensure_assets()


class TestCodexPlanner(unittest.TestCase):
    def test_real_cli_contract_without_invoking_model(self):
        def fake_run(command, **kwargs):
            self.assertIn("--output-schema", command)
            self.assertEqual(command[command.index("--sandbox") + 1], "read-only")
            self.assertEqual(command[command.index("--disable") + 1], "shell_tool")
            self.assertIn("--ignore-user-config", command)
            self.assertEqual(kwargs["encoding"], "utf-8")
            output = Path(command[command.index("--output-last-message") + 1])
            self.assertEqual(Path(kwargs["cwd"]), output.parent)
            output.write_text(json.dumps(DEMO_PLAN), encoding="utf-8")
            return subprocess.CompletedProcess(command, 0, "", "")

        with (
            patch("pai_motion.planner.shutil.which", return_value="codex"),
            patch("pai_motion.planner.subprocess.run", side_effect=fake_run),
        ):
            self.assertEqual(codex_plan("右へ").actions[1].type, "turn_right")

    def test_timeout_and_invalid_json(self):
        with (
            patch("pai_motion.planner.shutil.which", return_value="codex"),
            patch("pai_motion.planner.subprocess.run", side_effect=subprocess.TimeoutExpired("codex", 1)),
        ):
            with self.assertRaisesRegex(RuntimeError, "timed out"):
                codex_plan("test", timeout=1)
        with (
            patch("pai_motion.planner.shutil.which", return_value="codex"),
            patch("pai_motion.planner.subprocess.run", return_value=subprocess.CompletedProcess([], 0, "", "")),
        ):
            with self.assertRaisesRegex(RuntimeError, "valid JSON"):
                codex_plan("test")


class TestCLI(unittest.TestCase):
    def test_failure_keeps_plan_and_closes_simulator(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(sys, "argv", ["main", "--executor", "mujoco", "--out-dir", directory]),
            patch("pai_motion.sim_adapter.MuJoCoGo2Sink") as factory,
        ):
            sink = factory.return_value
            sink.events, sink.trace, sink.world_events = [], [], []
            sink.run_velocity.side_effect = RuntimeError("sim failed")
            self.assertEqual(main(), 1)
            sink.stop.assert_called_once()
            sink.close.assert_called_once()
            log = json.loads(next(Path(directory).glob("plan-*.json")).read_text(encoding="utf-8"))
            self.assertEqual(log["status"], "failed")
            self.assertEqual(log["error"], "sim failed")
            self.assertEqual(log["actions"], DEMO_PLAN["actions"])


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for real MuJoCo physics tests")
class TestGo2Physics(unittest.TestCase):
    def test_forward_backward_both_turns_and_stop(self):
        import math

        from pai_motion.executor import Velocity
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        sink = MuJoCoGo2Sink(headless=True)
        try:
            for cmd in (Velocity(0.18, 0, 0), Velocity(-0.12, 0, 0), Velocity(0, 0, 0.35), Velocity(0, 0, -0.35)):
                start = sink._state()
                sink.run_velocity(cmd, 2.0)
                end = sink._state()
                if cmd.vx:
                    distance = (end["position"][0] - start["position"][0]) * math.cos(start["yaw"]) + (
                        end["position"][1] - start["position"][1]
                    ) * math.sin(start["yaw"])
                    self.assertGreater(distance * (1 if cmd.vx > 0 else -1), 0.15)
                else:
                    yaw_change = math.atan2(math.sin(end["yaw"] - start["yaw"]), math.cos(end["yaw"] - start["yaw"]))
                    self.assertGreater(yaw_change * (1 if cmd.yaw > 0 else -1), 0.3)
            for cmd, duration in ((Velocity(0.31, 0, 0), 1), (Velocity(0, 0, 0), float("nan"))):
                with self.assertRaises(ValueError):
                    sink.run_velocity(cmd, duration)
            sink.stop()
            end = sink._state()
            self.assertLess(math.hypot(*end["body_velocity"][:2]), 0.05)
            self.assertLess(abs(end["yaw_rate"]), 0.05)
            self.assertGreater(min(state["upright"] for state in sink.trace), 0.9)
            self.assertGreater(end["contacts"], 0)
        finally:
            sink.close()
