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
CODEX_MODEL = "gpt-6-luna"
CODEX_REASONING_EFFORT = "xhigh"

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


def codex_correction(prompt: str, plan, context=None, timeout: int = 30):
    from dataclasses import asdict

    from pai_motion.correction import initial_tasks, validate_correction

    if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= 500:
        raise ValueError("Correction must contain 1–500 characters")
    context = context or {"pending": initial_tasks(plan.target), "completed": [], "observations": []}
    instructions = (
        "Interpret the latest in-flight instruction or clarification answer. "
        "Output schema-compliant JSON; no tools/code. "
        f"Current task: {json.dumps(asdict(plan), ensure_ascii=False)}. "
        f"Observed execution context (no hidden scene information): {json.dumps(context, ensure_ascii=False)}. "
        "Output the complete ORDERED remaining steps, at most 8. Capabilities: "
        "search(target) explores until that color is observed; past detection history may satisfy it. "
        "reobserve(target, observation_id) requires a NEW observation after that step starts; it can navigate "
        "to the robot's remembered sensing position in the referenced observation, NOT a hidden object position. "
        "Use reobserve for 'again', 'recheck' or 'さっき見た対象をもう一度確認'. "
        "return_home ends at the original home. Preserve the supplied home step exactly, last. "
        "You may insert searches/reobservations and reorder pending tasks. Retain unchanged task IDs and fields. "
        "Never silently drop pending tasks. Cancel a task ONLY if the instruction explicitly cancels/replaces it "
        "or requests immediate return: list its ID in cancelled with an exact quote from the user's instruction. "
        "A color replacement cancels the old search and adds a new search with a new ID. "
        "For 'green before blue', INSERT green before the retained blue task; do NOT cancel blue. "
        "New task IDs must be unique and must not reuse completed IDs. Reobserve must reference an ID supplied "
        "in context.observations with the matching color. "
        "These records are prior observations, not current visibility. "
        "The original deadline, home obligation, physical limits and factual observation history cannot be edited. "
        "No coordinates, executable code, arbitrary conditions or unsupported colors/actions. "
        "Use context.dialogue to resolve short answers to the most recent clarification question. "
        "For ambiguity or missing observation evidence, status=clarify with a concise question to the user; "
        "retain pending steps EXACTLY with cancelled=[] and do not guess a target or observation ID. "
        "For unsupported changes to physical limits, the deadline or home obligation, status=reject. "
        "For accept/reject set question to the empty string; for clarify supply a nonempty question. "
        "Explain the revision briefly in intent (1–120 characters). "
        f"Correction: {prompt!r}"
    )
    return validate_correction(
        _codex_json(instructions, ROOT / "schema/search-correction.schema.json", timeout),
        context=context,
        prompt=prompt,
    )


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
            "--model",
            CODEX_MODEL,
            "--config",
            f'model_reasoning_effort="{CODEX_REASONING_EFFORT}"',
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
