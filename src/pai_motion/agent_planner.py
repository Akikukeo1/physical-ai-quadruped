"""LLM decisions over observed tasks, named areas, and bounded frontier choices."""

import json
from dataclasses import asdict
from pathlib import Path

from pai_motion.correction import validate_correction
from pai_motion.field_revision import validate_field_revision
from pai_motion.motion import InvalidPlan

ROOT = Path(__file__).resolve().parents[2]


def validate_agent_revision(value, *, context, prompt):
    if not isinstance(value, dict) or set(value) != {"intent", "status", "question", "steps", "cancelled", "changes"}:
        raise InvalidPlan("Agent revision requires task and area changes")
    task = {key: value[key] for key in ("intent", "status", "question", "steps", "cancelled")}
    areas = {key: value[key] for key in ("status", "question", "changes")}
    correction = validate_correction(task, context=context, prompt=prompt)
    validate_field_revision(areas, context=context, prompt=prompt)
    return correction


def codex_agent_revision(prompt, plan, context, timeout=30):
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
    return _codex_json(instructions, ROOT / "schema/agent-revision.schema.json", timeout)


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
