"""Unit fixtures are stationary; physical tests below use the real Go2 controller."""

import io
import json
import math
import os
import sys
import tempfile
import threading
import unittest
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from main import main  # noqa: E402
from pai_motion.correction import SearchCorrection, sample_correction, validate_correction  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.planner import codex_correction  # noqa: E402
from pai_motion.search import execute_search, sample_search, validate_search  # noqa: E402


class SensorFixture:
    def __init__(self, *, remembered=False):
        self.time = 0.0
        self.remembered = remembered
        self.commands = []
        self.stopped = False
        self.release = threading.Event()

    def observe(self):
        return {
            "time": self.time,
            "position": [0.0, 0.0, 0.35],
            "yaw": 0.0,
            "body_velocity": [0.0, 0.0, 0.0],
            "yaw_rate": 0.0,
        }

    def sense(self):
        rays = [[i * math.tau / 180, 1.8, False, None] for i in range(180)]
        if (self.remembered and self.time < 1.0) or (not self.remembered and self.time >= 4.0):
            for index in (0, 1):
                rays[index] = [rays[index][0], 1.2, True, "green"]
        return {**self.observe(), "scan": rays}

    def run_velocity(self, cmd, seconds):
        self.commands.append({"time": self.time, "cmd": cmd})
        self.time += seconds
        if sum(c["cmd"].vx == 0 and c["cmd"].yaw == 0 for c in self.commands) >= 3:
            self.release.set()

    def stop(self):
        self.stopped = True


def after(seconds):
    return lambda state: "やっぱり緑を探して" if state["time"] >= seconds - 1e-9 else None


class TestCorrection(unittest.TestCase):
    def test_boundary_rejects_other_fields_and_unsupported_responses(self):
        valid = asdict(sample_correction("青じゃなくて緑に変更して", sample_search()))
        for change in (
            {"seconds": 240},
            {"on_found": "continue"},
            {"code": "print(1)"},
            {"status": "reject"},
            {"target": "red"},
            {"intent": ""},
        ):
            with self.subTest(change=change), self.assertRaises(InvalidPlan):
                validate_correction({**valid, **change})
        plan = sample_search()
        with patch("pai_motion.planner._codex_json", return_value=valid) as model:
            result = codex_correction("青じゃなくて緑に変更して", plan)
        self.assertEqual(result.steps[0]["target"], "green")
        self.assertEqual(model.call_args.args[1].name, "search-correction.schema.json")
        self.assertIn('"on_found": "return_home"', model.call_args.args[0])
        self.assertEqual(plan.target, "blue")

    def test_remembered_target_returns_without_fabricating_current_observation(self):
        sink = SensorFixture(remembered=True)
        observations, decisions, revisions = [], [], []
        result = execute_search(
            sample_search(),
            sink,
            observations=observations,
            decisions=decisions,
            get_correction=after(2),
            correction_planner=sample_correction,
            corrections=revisions,
        )
        self.assertEqual(result["plan"]["target"], "green")
        self.assertEqual(result["plan"]["seconds"], 180)
        self.assertEqual(result["plan"]["on_found"], "return_home")
        self.assertEqual(result["found"]["time"], 0.0)
        self.assertEqual(result["detections"]["green"], result["found"])
        self.assertFalse(any(ray[3] == "green" for ray in observations[-1]["scan"]))
        self.assertIn("target in observation history", [d.get("reason") for d in decisions])
        event = revisions[0]
        self.assertEqual(event["known_colors_before"], ["green"])
        self.assertGreaterEqual(event["observations_after"], event["observations_before"])
        self.assertEqual(event["first_zero_command_time"], event["received"]["time"])
        self.assertTrue(sink.stopped)

    def test_unseen_target_keeps_exploring_and_physics_advances_while_waiting(self):
        sink = SensorFixture()

        def delayed(prompt, plan, context):
            if not sink.release.wait(2.0):
                raise RuntimeError("Physics did not advance while planner waited")
            return sample_correction(prompt, plan, context)

        revisions = []
        result = execute_search(
            sample_search(),
            sink,
            observations=[],
            decisions=[],
            get_correction=after(2),
            correction_planner=delayed,
            corrections=revisions,
        )
        event = revisions[0]
        self.assertEqual(event["known_colors_before"], [])
        self.assertGreaterEqual(event["planning_sim_seconds"], 0.6 - 1e-6)
        self.assertGreater(result["found"]["time"], event["applied_time"])
        self.assertTrue(
            any(
                (c["cmd"].vx or c["cmd"].yaw)
                for c in sink.commands
                if event["applied_time"] <= c["time"] < result["found"]["time"]
            )
        )
        self.assertTrue(
            all(
                c["cmd"].vx == 0 and c["cmd"].yaw == 0
                for c in sink.commands
                if event["received"]["time"] <= c["time"] < event["applied_time"]
            )
        )

    def test_failed_or_invalid_correction_never_resumes_old_task(self):
        def unavailable(prompt, plan, context):
            raise RuntimeError("model unavailable")

        for planner in (unavailable, lambda prompt, plan, context: SearchCorrection("wrong", "accept", [], [])):
            with self.subTest(planner=planner):
                sink, revisions = SensorFixture(remembered=True), []
                with self.assertRaises((RuntimeError, InvalidPlan)):
                    execute_search(
                        sample_search(),
                        sink,
                        observations=[],
                        decisions=[],
                        get_correction=after(2),
                        correction_planner=planner,
                        corrections=revisions,
                    )
                self.assertTrue(sink.stopped)
                self.assertEqual(revisions[0]["status"], "failed")
                self.assertTrue(
                    all(
                        c["cmd"].vx == 0 and c["cmd"].yaw == 0
                        for c in sink.commands
                        if c["time"] >= revisions[0]["received"]["time"]
                    )
                )

    def test_correction_does_not_restart_original_deadline(self):
        sink = SensorFixture(remembered=True)
        plan = validate_search({**asdict(sample_search()), "seconds": 60})
        revisions = []
        result = execute_search(
            plan,
            sink,
            observations=[],
            decisions=[],
            get_correction=after(30),
            correction_planner=sample_correction,
            corrections=revisions,
        )
        self.assertEqual(revisions[0]["deadline"], 60.0)
        self.assertEqual(result["plan"]["seconds"], 60)
        self.assertLess(result["elapsed"], 60.0)

    def test_cli_reads_a_correction_written_after_startup(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "correction.txt"

            def execute(plan, sink, **kwargs):
                self.assertIsNone(kwargs["get_correction"]({"time": 2.0}))
                path.write_text("", encoding="utf-8")
                self.assertIsNone(kwargs["get_correction"]({"time": 2.0}))
                path.write_text("x" * 5000, encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "4096"):
                    kwargs["get_correction"]({"time": 2.0})
                path.write_text("やっぱり緑へ変更して", encoding="utf-8")
                self.assertEqual(kwargs["get_correction"]({"time": 3.0}), "やっぱり緑へ変更して")
                return {"outcome": "unit fixture only"}

            with (
                patch.object(
                    sys,
                    "argv",
                    [
                        "main",
                        "--task",
                        "search",
                        "--executor",
                        "mujoco",
                        "--correction-file",
                        str(path),
                        "--out-dir",
                        directory,
                    ],
                ),
                patch("pai_motion.sim_adapter.MuJoCoGo2Sink") as factory,
                patch("main.execute_search", side_effect=execute),
            ):
                sink = factory.return_value
                sink.events, sink.trace, sink.world_events = [], [], []
                sink.observe.return_value = {"time": 1.0}
                self.assertEqual(main(), 0)
                sink.close.assert_called_once()
            log = json.loads(next(Path(directory).glob("plan-*.json")).read_text(encoding="utf-8"))
            self.assertEqual(log["correction_input"]["file"], str(path))

    def test_cli_rejects_invalid_correction_options_before_creating_simulator(self):
        for options in (
            ["--correction", "緑へ"],
            ["--correct-at", "8"],
            ["--correction", "緑へ", "--correct-at", "nan"],
            ["--correction", "緑へ", "--correct-at", "-1"],
            ["--correction", "", "--correct-at", "8"],
        ):
            with (
                self.subTest(options=options),
                patch.object(sys, "argv", ["main", "--task", "search", "--executor", "mujoco", *options]),
                patch("sys.stderr", new_callable=io.StringIO),
                patch("pai_motion.sim_adapter.MuJoCoGo2Sink") as factory,
            ):
                with self.assertRaises(SystemExit) as result:
                    main()
                self.assertEqual(result.exception.code, 2)
                factory.assert_not_called()


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for real MuJoCo physics tests")
class TestCorrectionPhysics(unittest.TestCase):
    def test_observed_and_unseen_green_both_keep_return_obligation(self):
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        for seed, seconds, remembered in ((0, 8.0, False), (305, 55.0, True)):
            with self.subTest(seed=seed):
                sink = MuJoCoGo2Sink(headless=True, search_seed=seed)
                started = sink.observe()["time"]
                revisions = []
                observations = []
                try:
                    result = execute_search(
                        sample_search(),
                        sink,
                        observations=observations,
                        decisions=[],
                        get_correction=after(started + seconds),
                        correction_planner=sample_correction,
                        corrections=revisions,
                    )
                    self.assertEqual(len(revisions), 1)
                    event = revisions[0]
                    self.assertEqual("green" in event["known_colors_before"], remembered)
                    self.assertEqual(result["plan"]["target"], "green")
                    self.assertEqual(result["plan"]["on_found"], "return_home")
                    self.assertEqual(result["plan"]["seconds"], 180)
                    self.assertEqual(result["outcome"], "found")
                    self.assertLessEqual(result["elapsed"], 180)
                    self.assertLessEqual(result["home_error"], 0.15)
                    self.assertLess(math.hypot(*event["settled"]["body_velocity"][:2]), 0.03)
                    self.assertFalse(any(state["obstacle_contact"] for state in sink.trace))
                    if remembered:
                        self.assertLess(result["found"]["time"], event["received"]["time"])
                        self.assertFalse(
                            any(
                                ray[3] == "green"
                                for obs in observations
                                if event["received"]["time"] <= obs["time"] <= event["applied_time"]
                                for ray in obs["scan"]
                            )
                        )
                    else:
                        self.assertGreaterEqual(result["found"]["time"], event["applied_time"])
                finally:
                    sink.close()
