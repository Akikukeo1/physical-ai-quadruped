"""LLM decisions over observed tasks, named areas, and bounded frontier choices."""

import json
from dataclasses import asdict
from pathlib import Path

from pai_motion.correction import validate_correction
from pai_motion.field_revision import validate_field_revision
from pai_motion.motion import InvalidPlan

ROOT = Path(__file__).resolve().parents[2]


def validate_agent_revision(value, *, context, prompt):
    required = {"intent", "status", "question", "steps", "cancelled", "changes"}
    if not isinstance(value, dict) or not required <= set(value) or set(value) - required - {"reply"}:
        raise InvalidPlan("Agent revision requires task and area changes")
    if "reply" in value and (
        not isinstance(value["reply"], str) or len(value["reply"]) > 300 or "\n" in value["reply"]
    ):
        raise InvalidPlan("Invalid dialogue reply")
    task = {key: value[key] for key in ("intent", "status", "question", "steps", "cancelled")}
    areas = {key: value[key] for key in ("status", "question", "changes")}
    correction = validate_correction(task, context=context, prompt=prompt)
    validate_field_revision(areas, context=context, prompt=prompt)
    return correction


def codex_agent_revision(prompt, plan, context, timeout=30, *, dialogue=False):
    from pai_motion.planner import _codex_json

    instructions = (
        "Interpret the user's robot instruction as one atomic revision of tasks AND named area permissions. "
        "Return JSON only, no code or tools. Actions: search blue/green; reobserve a supplied observation ID "
        "of that color; return_home. Reobserve needs a fresh sensor detection, not a hidden object location. "
        "Return the complete ordered remaining steps, retain unchanged IDs and the original home step last. "
        "Never silently remove an existing pending task; explicitly cancelled tasks require an exact user quote. "
        "Use the latest supplied observation of the requested color unless a different one is specified. "
        "Area changes use existing IDs and allow/avoid; unmentioned permissions stay unchanged. Allow is not a visit. "
        "Copy the entire current user instruction verbatim into each area change's instruction field. "
        "If ambiguous, unsupported, or missing required evidence, ask one Japanese clarification and leave "
        "steps and area permissions unchanged. Retain return-home and deadline even when tasks change. "
        f"Original plan: {json.dumps(asdict(plan), ensure_ascii=False)}. "
        f"Observed context: {json.dumps(context, ensure_ascii=False)}. Current message: {prompt!r}"
    )
    if dialogue:
        instructions += (
            " Also provide reply: a concise Japanese response to the user, including answers to ordinary questions. "
            "Do not claim actions were executed. For conversation without a task change retain all steps "
            "and permissions. "
            "If a clarification question is needed, leave reply empty; the question will be shown separately. "
            "You are a simulated robot using Codex CLI, range/color sensing and simulator ground-truth odometry. "
            "You can search blue or green, return, reobserve known sightings and control named area permissions."
            " You cannot inspect a named area as a task, use a camera, or control a physical robot."
        )
        if context.get("points"):
            instructions += (
                " Named points are supplied in context.points. You may add visit_point steps using the point ID "
                "as target and null observation_id. Visit means the robot must reach that point, not just allow it. "
                "If there is no search task, do not invent one. Retain home as the final step."
            )
        if context.get("starting_mission"):
            instructions += (
                " This is the initial mission: pending contains only the required home step by design. "
                "Add all explicitly requested search and visit steps before home; do not ask confirmation "
                "just because they are absent from pending. The original plan's color may be a placeholder."
            )
    schema = "agent-dialogue.schema.json" if dialogue else "agent-revision.schema.json"
    return _codex_json(instructions, ROOT / "schema" / schema, timeout)


def codex_frontier_choice(context, candidates, timeout=30):
    from pai_motion.planner import _codex_json

    instructions = (
        "Choose the next frontier for a simulated search robot. Return JSON only, no code. "
        "Each candidate is a reachable observed-safe cell next to unknown space, with path distance and "
        "estimated information gain. Coordinates are simulator odometry, not hidden target locations. "
        "Use the current task, observations, remaining time, user constraints and previous choices. "
        "Choose exactly one supplied candidate ID; explain briefly in Japanese. Never invent a location. "
        f"Observed context: {json.dumps(context, ensure_ascii=False)}. "
        f"Candidates: {json.dumps(candidates, ensure_ascii=False)}"
    )
    return _codex_json(instructions, ROOT / "schema/frontier-choice.schema.json", timeout)


def validate_frontier_choice(value, candidates):
    if not isinstance(value, dict) or set(value) != {"id", "reason"}:
        raise InvalidPlan("Frontier choice requires id and reason")
    if not isinstance(value["id"], str) or value["id"] not in {c["id"] for c in candidates}:
        raise InvalidPlan("Frontier must be a supplied candidate")
    if not isinstance(value["reason"], str) or not 1 <= len(value["reason"].strip()) <= 200:
        raise InvalidPlan("Frontier choice requires a bounded reason")
    return value
