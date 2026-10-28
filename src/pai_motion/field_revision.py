"""Interpret named field-area permissions without generating executable code."""

import json
from dataclasses import asdict
from pathlib import Path

from pai_motion.motion import InvalidPlan

SCHEMA = Path(__file__).resolve().parents[2] / "schema" / "field-revision.schema.json"


def validate_field_revision(value, *, context, prompt):
    if not isinstance(value, dict) or set(value) != {"status", "question", "changes"}:
        raise InvalidPlan("Field revision requires status, question and changes")
    status, question, changes = value["status"], value["question"], value["changes"]
    if status not in ("accept", "clarify") or not isinstance(question, str) or len(question) > 200:
        raise InvalidPlan("Invalid field revision status or question")
    if (status == "clarify") != bool(question.strip()) or not isinstance(changes, list) or len(changes) > 8:
        raise InvalidPlan("Clarification requires a question; changes must be bounded")
    if status == "clarify" and changes:
        raise InvalidPlan("Clarification cannot alter area permissions")
    ids = {a["id"] for a in context["field_areas"]}
    mentioned = set()
    for change in changes:
        if not isinstance(change, dict) or set(change) != {"id", "permission", "instruction"}:
            raise InvalidPlan("Area change requires id, permission and instruction")
        ident, permission, quote = change["id"], change["permission"], change["instruction"]
        if not isinstance(ident, str) or ident not in ids or ident in mentioned or permission not in ("avoid", "allow"):
            raise InvalidPlan("Unknown, repeated, or invalid area change")
        if not isinstance(quote, str) or not quote.strip() or quote not in prompt:
            raise InvalidPlan("Area change must quote the current instruction")
        mentioned.add(ident)
    return value


def codex_field_revision(prompt, plan, context, timeout=30):
    from pai_motion.planner import _codex_json

    instructions = (
        "Interpret the user's instruction for a simulated robot. Return JSON only; do not execute code. "
        "The field_areas are named rectangles supplied by the user; each begins allowed. "
        "Return only permission changes for the named areas: avoid means do not enter; allow removes the restriction. "
        "Allowing an area does not force a visit: the existing navigation chooses its route. "
        "In 'エリア1は通るがエリア2は通るな', treat area1 as allowed and area2 as avoided; "
        "do not interpret this as an obligatory visit unless explicitly required. "
        "Unmentioned areas retain their existing permissions. Use the exact ID; unknown or ambiguous references "
        "require a clarification, not a guess. If the instruction demands an actual visit, another task change, "
        "or an unsupported operation, clarify instead of claiming it can be done. "
        "For each change, copy the entire current user instruction verbatim into its instruction field, "
        "including all punctuation. Do not translate, paraphrase or add quote marks. Questions must be in Japanese. "
        f"Mission: {json.dumps(asdict(plan), ensure_ascii=False)}. "
        f"Context: {json.dumps(context, ensure_ascii=False)}. User instruction: {prompt!r}"
    )
    return validate_field_revision(_codex_json(instructions, SCHEMA, timeout), context=context, prompt=prompt)
