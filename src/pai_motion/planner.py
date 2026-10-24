"""Codex CLI planner: fixed schema, no runtime code execution."""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from pathlib import Path

from pai_motion.motion import MotionPlan, validate_plan

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "schema" / "motion-plan.schema.json"

DEMO_PLAN = {
    "intent": "右方向を動きで示す",
    "actions": [
        {"type": "turn_right", "duration": 1.0},
        {"type": "forward", "duration": 1.0},
        {"type": "wait", "duration": 0.5},
    ],
}


def sample_plan() -> MotionPlan:
    """No-AI baseline for debugging the transport and validator."""
    return validate_plan(DEMO_PLAN)


def codex_plan(prompt: str, timeout: int = 120) -> MotionPlan:
    """Generate one plan via installed Codex CLI with strict output validation."""
    if not prompt.strip():
        raise ValueError("prompt is required")
    codex = shutil.which("codex")
    if not codex:
        raise RuntimeError("Codex CLI not found in PATH; install it and run 'codex login' first")

    instructions = (
        "You are a motion planner for a simulated quadruped robot. "
        "Produce ONLY the schema-compliant JSON requested by the output schema. "
        "Choose a short, expressive sequence from forward/backward/turn_left/turn_right/wait. "
        "Each duration is 0.2 to 2.0 seconds; max 12 actions and 15 seconds total. "
        "Do not write executable code, call tools, or claim that the plan has been executed. "
        "There is no visual perception or collision avoidance at this stage. "
        f"The user request is: {prompt!r}"
    )
    with tempfile.TemporaryDirectory(prefix="pai-motion-") as directory:
        dest = Path(directory) / "motion.json"
        command = [
            codex,
            "exec",
            "--sandbox",
            "read-only",
            "--skip-git-repo-check",
            "--output-schema",
            str(SCHEMA),
            "--output-last-message",
            str(dest),
            instructions,
        ]
        try:
            proc = subprocess.run(
                command,
                cwd=ROOT,
                text=True,
                capture_output=True,
                check=False,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"Codex timed out after {timeout} seconds") from exc
        if proc.returncode != 0:
            tail = proc.stderr[-1200:]
            raise RuntimeError(f"Codex failed (exit {proc.returncode}): {tail}")
        try:
            obj = json.loads(dest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError("Codex did not produce a valid JSON motion plan") from exc
    return validate_plan(obj)
