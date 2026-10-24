"""Codex CLI planner: fixed schema, no runtime code execution."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from pai_motion.motion import MotionPlan, validate_plan
from pai_motion.navigation import OBSTACLE, ROBOT_RADIUS, ROUTE_MARGIN, WAYPOINTS, NavigationPlan, validate_navigation

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


def codex_navigation(
    prompt: str, timeout: int = 120, *, start=(0.0, 0.0), obstacles=(OBSTACLE,), required=None
) -> NavigationPlan:
    if not prompt.strip():
        raise ValueError("prompt is required")
    instructions = (
        "Plan a waypoint route for a simulated Go2 using ONLY schema-compliant JSON. No tools or executable code. "
        f"The map and current position {start!r} are simulator ground truth, not camera observations. "
        f"Named points (x,y metres): {WAYPOINTS!r}. Obstacles (cx,cy,hx,hy): {obstacles!r}. "
        f"Every straight segment must avoid every obstacle expanded by {ROBOT_RADIUS + ROUTE_MARGIN} metres. "
        "Use north_gate to go around the north/positive-y side; south_gate for the south/negative-y side. "
        "Blue, green and exit points are on the east side. home and gates are on the west side. "
        "Direct home-to-blue/green is unsafe. Use the same-side gate before crossing between west and east. "
        "At most 8 waypoints. Include the requested destinations in the requested order, plus necessary gates. "
        f"User request: {prompt!r}"
    )
    if required is not None:
        instructions += (
            f" This is replanning after an observed route blockage. Remaining goals, in mandatory order: {required!r}. "
            "Include exactly these goal visits (home/blue/green), no extra goal visits. "
            "Gates/exits are transit points. "
            "Previously reached goals must not be requested again. An original route-side preference may be abandoned "
            "when blocked. Use south_exit/north_exit to go around without using an unrequested goal as transit. "
            "Explain why this replacement route is needed in intent. No hidden future obstacle positions are available."
        )
    return validate_navigation(
        _codex_json(instructions, ROOT / "schema/navigation-plan.schema.json", timeout), start, obstacles, required
    )


def codex_search(prompt: str, timeout: int = 120):
    from pai_motion.search import validate_search

    if not prompt.strip():
        raise ValueError("prompt is required")
    instructions = (
        "Translate the user's search-and-return task to schema-compliant JSON. No tools or code. "
        "The robot explores an UNKNOWN layout with range/color observations and builds its own map. "
        "No map, scene seed, obstacle positions, target positions or predefined detours are available to you. "
        "Targets are blue or green. Set target to the REQUESTED color, seconds to a total task budget "
        "in 60–240 seconds (default 180), and on_found to return_home. "
        "Return home after observation, or report not_found and return if exploration or time runs out. "
        f"User request: {prompt!r}"
    )
    return validate_search(_codex_json(instructions, ROOT / "schema/search-plan.schema.json", timeout))


def codex_correction(prompt: str, plan, timeout: int = 30):
    from dataclasses import asdict

    from pai_motion.correction import validate_correction

    if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= 500:
        raise ValueError("Correction must contain 1–500 characters")
    instructions = (
        "Interpret ONE in-flight correction to a robot's search task. Output schema-compliant JSON; no tools/code. "
        f"Current task: {json.dumps(asdict(plan), ensure_ascii=False)}. "
        "Only changing the search target between blue and green is supported. "
        "The original return_home obligation, original overall deadline and all observation history are retained "
        "by the controller, and cannot be changed. Do not reset the time budget. "
        "If the user clearly requests a target color replacement only, status=accept and target=the NEW color. "
        "For ambiguity, other actions, changes to time/return constraints or unsupported colors, status=reject "
        "and target=the current color. Do not guess. Explain briefly in intent (1–120 characters). "
        f"Correction: {prompt!r}"
    )
    return validate_correction(_codex_json(instructions, ROOT / "schema/search-correction.schema.json", timeout))


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
