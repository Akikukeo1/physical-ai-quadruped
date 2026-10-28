"""Revisions with observation-grounded area constraints."""

import math
import re
from pathlib import Path

from pai_motion.correction import validate_correction
from pai_motion.motion import InvalidPlan

SCHEMA = Path(__file__).resolve().parents[2] / "schema" / "constraint-revision.schema.json"


def validate_constraint_revision(value: object, *, context: dict, prompt: str) -> dict:
    fields = {"intent", "status", "steps", "cancelled", "question", "constraints", "released_constraints"}
    if not isinstance(value, dict) or set(value) != fields:
        raise InvalidPlan("Constraint revision requires the complete experimental schema")
    validate_correction(
        {k: value[k] for k in ("intent", "status", "steps", "cancelled", "question")}, context=context, prompt=prompt
    )
    constraints, released = value["constraints"], value["released_constraints"]
    if not isinstance(constraints, list) or len(constraints) > 4:
        raise InvalidPlan("At most four active area constraints are supported")
    if not isinstance(released, list) or len(released) > 4:
        raise InvalidPlan("At most four releases are supported")
    previous = {item["id"]: item for item in context.get("constraints", [])}
    retired = {item["constraint"]["id"] for item in context.get("released_constraints", [])}
    observations = {item["id"]: item for item in context["observations"]}
    user_text = "\n".join([item["text"] for item in context.get("dialogue", []) if item["role"] == "user"] + [prompt])
    active = {}
    for item in constraints:
        if not isinstance(item, dict) or set(item) != {"id", "observation_id", "radius", "instruction"}:
            raise InvalidPlan("Area constraint requires id, observation_id, radius and instruction")
        identifier, ref, radius, quote = (item[k] for k in ("id", "observation_id", "radius", "instruction"))
        if (
            not isinstance(identifier, str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", identifier)
            or identifier in active
        ):
            raise InvalidPlan("Constraint IDs must be unique bounded identifiers")
        if identifier in retired or (identifier in previous and item != previous[identifier]):
            raise InvalidPlan("Existing or released constraint IDs cannot change meaning or be reused")
        if not isinstance(ref, str) or ref not in observations:
            raise InvalidPlan("Area constraint must reference a supplied observation")
        if type(radius) not in (int, float) or not 0.2 <= radius <= 1.5 or not math.isfinite(radius):
            raise InvalidPlan("Area radius must be finite and between 0.2 and 1.5 metres")
        if not isinstance(quote, str) or not 1 <= len(quote.strip()) <= 500:
            raise InvalidPlan("Area constraint must quote the user's instruction")
        if identifier not in previous and quote not in user_text:
            raise InvalidPlan("New area constraint quote is not in the user's instructions")
        active[identifier] = item
    removed = set()
    for item in released:
        if not isinstance(item, dict) or set(item) != {"id", "instruction"}:
            raise InvalidPlan("Release requires id and a quoted instruction")
        identifier, quote = item["id"], item["instruction"]
        if (
            not isinstance(identifier, str)
            or identifier not in previous
            or identifier in active
            or identifier in removed
        ):
            raise InvalidPlan("Release must reference one active constraint exactly once")
        if not isinstance(quote, str) or not 1 <= len(quote.strip()) <= 500 or quote not in user_text:
            raise InvalidPlan("Release must quote the user's instruction")
        removed.add(identifier)
    if (previous.keys() - removed) - active.keys():
        raise InvalidPlan("Active constraints cannot silently disappear")
    if value["status"] == "clarify" and (constraints != context.get("constraints", []) or released):
        raise InvalidPlan("Clarification cannot change constraints")
    return value


def codex_constraint_revision(prompt: str, plan, context: dict, timeout: int = 30) -> dict:
    """Interpret a revision; the executor must validate feasibility before applying it."""
    import json
    from dataclasses import asdict

    from pai_motion.planner import _codex_json

    if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= 500:
        raise ValueError("Instruction must contain 1–500 characters")
    instructions = (
        "Interpret a simulated robot's in-flight instruction. Produce schema-compliant JSON only; no code or tools. "
        f"Initial task: {json.dumps(asdict(plan), ensure_ascii=False)}. "
        f"Execution context: {json.dumps(context, ensure_ascii=False)}. "
        "Return complete ordered remaining steps and complete active constraints. "
        "Actions: search(color), reobserve(color, observation_id), return_home. "
        "Reobserve requires fresh sensing; it currently navigates toward the robot's recorded sensing position. "
        "Preserve all pending tasks unless explicitly cancelled with an exact quote from the user's text; "
        "retain existing IDs and keep the exact home step last. "
        "An area constraint forbids the robot CENTER inside a circle around a supplied observation's robot position, "
        "not around the object's hidden position. Select only a supplied observation_id and radius 0.2–1.5 metres. "
        "Unless the user specifies an earlier observation or multiple places, use the latest matching observation "
        "and create one constraint for it, not one per past observation. "
        "Each new constraint must include an exact user quote, not a paraphrase. "
        "Preserve existing constraints exactly; "
        "only an explicit user request may release one, with its ID and exact user quote in released_constraints. "
        "A released ID cannot be reused. No arbitrary coordinates, unknown observations, or executable code. "
        "If a user explicitly asks to enter an area that is still forbidden, ask before making any changes. "
        "Questions for the user must describe the color and place in ordinary language, not internal observation IDs. "
        "Do not offer finding an alternative observation viewpoint: this controller cannot plan such a viewpoint. "
        "If a request is ambiguous, unsupported, lacks evidence, or the current reobserve method cannot reach its "
        "referenced sensing position due to an active area, ask a concise question with status clarify. "
        "Clarification must leave the pending steps and active constraints exactly unchanged, with no cancellations "
        "or releases. This limitation does not imply the target cannot be observed from another position. "
        "Use dialogue to interpret short answers; preserve deadline and home obligation. "
        "For accepted plans use status accept and an empty question. "
        f"Latest user message: {prompt!r}"
    )
    return validate_constraint_revision(_codex_json(instructions, SCHEMA, timeout), context=context, prompt=prompt)
