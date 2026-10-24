"""Codex CLI planner: fixed schema, no runtime code execution."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from pai_motion.motion import MotionPlan, validate_plan
from pai_motion.navigation import OBSTACLE, WAYPOINTS, NavigationPlan, validate_navigation

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "schema" / "motion-plan.schema.json"

DEMO_PLAN = {
    "intent": "右方向を動きで示す",
    "actions": [
        {"type": "forward", "duration": 2.0},
        {"type": "turn_right", "duration": 2.0},
        {"type": "forward", "duration": 2.0},
        {"type": "wait", "duration": 2.0},
    ],
}


def sample_plan() -> MotionPlan:
    """No-AI baseline for debugging the transport and validator."""
    return validate_plan(DEMO_PLAN)


def codex_plan(prompt: str, timeout: int = 120) -> MotionPlan:
    """Generate one plan via installed Codex CLI with strict output validation."""
    if not prompt.strip():
        raise ValueError("prompt is required")
    instructions = (
        "You are a motion planner for a simulated quadruped robot. "
        "Produce ONLY the schema-compliant JSON requested by the output schema. "
        "Choose a short, expressive sequence from forward/backward/turn_left/turn_right/wait. "
        "Each duration is 0.2 to 2.0 seconds; max 12 actions and 15 seconds total. "
        "Do not write executable code, call tools, or claim that the plan has been executed. "
        "There is no visual perception or collision avoidance at this stage. "
        f"The user request is: {prompt!r}"
    )
    return validate_plan(_codex_json(instructions, SCHEMA, timeout))


def codex_navigation(prompt: str, timeout: int = 120) -> NavigationPlan:
    if not prompt.strip():
        raise ValueError("prompt is required")
    instructions = (
        "Plan a waypoint route for a simulated Go2 using ONLY schema-compliant JSON. No tools or executable code. "
        "The map and current start position (0,0) are simulator ground truth, not camera observations. "
        f"Named points (x,y metres): {WAYPOINTS!r}. Red rectangular obstacle (cx,cy,hx,hy): {OBSTACLE!r}. "
        "Every entire straight segment must avoid the obstacle expanded by 0.55 metres on all sides. "
        "Use north_gate to go around the north/positive-y side; south_gate for the south/negative-y side. "
        "Blue and green are on the east side. home and gates are on the west side. "
        "Direct home-to-blue/green is unsafe. Use the same-side gate before crossing between west and east. "
        "At most 8 waypoints. Include the requested destinations in the requested order, plus necessary gates. "
        f"User request: {prompt!r}"
    )
    return validate_navigation(_codex_json(instructions, ROOT / "schema/navigation-plan.schema.json", timeout))


def _codex_json(instructions: str, schema: Path, timeout: int) -> object:
    codex = shutil.which("codex") or (shutil.which("codex.exe") if sys.platform == "linux" else None)
    if not codex:
        raise RuntimeError("Codex CLI not found in PATH; install it and run 'codex login' first")
    with tempfile.TemporaryDirectory(prefix="pai-motion-") as directory:
        dest = Path(directory) / "motion.json"

        # WSL can reuse the authenticated Windows CLI without copying credentials.
        def cli_path(path: Path) -> str:
            if sys.platform == "linux" and codex.lower().endswith(".exe"):
                return subprocess.check_output(["wslpath", "-w", str(path)], text=True).strip()
            return str(path)

        command = [
            codex,
            "exec",
            "--ephemeral",
            "--ignore-user-config",
            "--disable",
            "shell_tool",
            "--disable",
            "apps",
            "--disable",
            "plugins",
            "--sandbox",
            "read-only",
            "--skip-git-repo-check",
            "--output-schema",
            cli_path(schema),
            "--output-last-message",
            cli_path(dest),
            instructions,
        ]
        if sys.platform == "linux" and codex.lower().endswith(".exe") and Path("/init").exists():
            # WSL's P-flag interpreter expects both executable path and original argv[0].
            command = ["/init", codex, *command]
        try:
            proc = subprocess.run(
                command,
                cwd=directory,
                text=True,
                encoding="utf-8",
                capture_output=True,
                check=False,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"Codex timed out after {timeout} seconds") from exc
        except OSError as exc:
            raise RuntimeError(f"Cannot start Codex CLI: {exc}") from exc
        if proc.returncode != 0:
            tail = proc.stderr[-1200:]
            raise RuntimeError(f"Codex failed (exit {proc.returncode}): {tail}")
        try:
            obj = json.loads(dest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError("Codex did not produce a valid JSON motion plan") from exc
    return obj
