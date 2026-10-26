"""Observation-grounded revisions of the remaining high-level tasks."""

import re
from dataclasses import dataclass

from pai_motion.motion import InvalidPlan


def initial_tasks(target):
    return [
        {"id": "search-1", "action": "search", "target": target, "observation_id": None},
        {"id": "home", "action": "return_home", "target": None, "observation_id": None},
    ]


@dataclass(frozen=True)
class SearchCorrection:
    intent: str
    status: str
    steps: list[dict]
    cancelled: list[dict]
    question: str = ""


def validate_correction(value: object, *, context=None, prompt=None) -> SearchCorrection:
    if prompt is not None and context is not None:
        prompt = "\n".join(
            [entry["text"] for entry in context.get("dialogue", []) if entry["role"] == "user"] + [prompt]
        )
    required = {"intent", "status", "steps", "cancelled"}
    if not isinstance(value, dict) or not required <= value.keys() or value.keys() - required - {"question"}:
        raise InvalidPlan("Correction requires intent, status, steps and cancelled only")
    if not isinstance(value["intent"], str) or not 1 <= len(value["intent"].strip()) <= 120:
        raise InvalidPlan("Correction intent must contain 1–120 characters")
    if value["status"] not in ("accept", "clarify"):
        raise InvalidPlan("Correction was not accepted")
    question = value.get("question", "")
    if not isinstance(question, str) or len(question) > 200:
        raise InvalidPlan("Question must be a bounded string")
    if (value["status"] == "clarify") != bool(question.strip()):
        raise InvalidPlan("Only clarify responses require a question")
    steps, cancelled = value["steps"], value["cancelled"]
    if not isinstance(steps, list) or not 1 <= len(steps) <= 8:
        raise InvalidPlan("Correction requires 1–8 remaining steps")
    if not isinstance(cancelled, list) or len(cancelled) > 7:
        raise InvalidPlan("Correction allows at most 7 explicit cancellations")
    ids = set()
    for step in steps:
        if not isinstance(step, dict) or set(step) != {"id", "action", "target", "observation_id"}:
            raise InvalidPlan("Task requires id, action, target and observation_id only")
        identifier = step["id"]
        if not isinstance(identifier, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", identifier) or identifier in ids:
            raise InvalidPlan("Task IDs must be unique bounded identifiers")
        ids.add(identifier)
        action, target, reference = step["action"], step["target"], step["observation_id"]
        if action == "return_home":
            if target is not None or reference is not None:
                raise InvalidPlan("Return home cannot specify a target or observation")
        elif action in ("search", "reobserve") and target in ("blue", "green"):
            if action == "search" and reference is not None:
                raise InvalidPlan("Search cannot claim an observation reference")
            if action == "reobserve" and (not isinstance(reference, str) or not 1 <= len(reference) <= 60):
                raise InvalidPlan("Reobserve requires an observation ID")
        else:
            raise InvalidPlan("Unsupported task action or target")
    home = {"id": "home", "action": "return_home", "target": None, "observation_id": None}
    if steps[-1] != home or any(s["action"] == "return_home" for s in steps[:-1]):
        raise InvalidPlan("The original home obligation must remain last, exactly once")
    removed = set()
    for entry in cancelled:
        if not isinstance(entry, dict) or set(entry) != {"id", "instruction"}:
            raise InvalidPlan("Cancellation requires id and a quoted instruction")
        identifier, quote = entry["id"], entry["instruction"]
        if (
            not isinstance(identifier, str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", identifier)
            or identifier == "home"
            or identifier in removed
            or identifier in ids
        ):
            raise InvalidPlan("Invalid or conflicting cancellation")
        if not isinstance(quote, str) or not 1 <= len(quote.strip()) <= 500:
            raise InvalidPlan("Cancellation must quote the user's instruction")
        if prompt is not None and quote not in prompt:
            raise InvalidPlan("Cancellation quote is not in the instruction")
        removed.add(identifier)
    if context is not None:
        old = {step["id"]: step for step in context["pending"]}
        completed = {entry["step"]["id"] for entry in context["completed"]}
        completed |= {entry["step"]["id"] for entry in context.get("cancelled", [])}
        observations = {entry["id"]: entry for entry in context["observations"]}
        if not removed <= old.keys() or (old.keys() - removed) - ids:
            raise InvalidPlan("Pending obligations cannot silently disappear")
        for step in steps:
            if step["id"] in completed or (step["id"] in old and step != old[step["id"]]):
                raise InvalidPlan("Existing task IDs cannot be replayed or change meaning")
            if step["action"] == "reobserve":
                evidence = observations.get(step["observation_id"])
                if evidence is None or evidence["color"] != step["target"]:
                    raise InvalidPlan("Reobserve must reference a supplied observation of that color")
        if value["status"] == "clarify" and (steps != context["pending"] or cancelled):
            raise InvalidPlan("Clarification cannot alter pending tasks")
    elif value["status"] == "clarify":
        raise InvalidPlan("Clarification requires an execution context")
    return SearchCorrection(**value)


def sample_correction(prompt, plan, context=None) -> SearchCorrection:
    """Transport baseline only: replace the initial search with green, without language interpretation."""
    context = context or {"pending": initial_tasks(plan.target), "completed": [], "observations": []}
    return validate_correction(
        {
            "intent": "探索対象を緑へ変更",
            "status": "accept",
            "steps": [
                {"id": "search-green", "action": "search", "target": "green", "observation_id": None},
                context["pending"][-1],
            ],
            "cancelled": [{"id": step["id"], "instruction": prompt or "sample"} for step in context["pending"][:-1]],
        },
        context=context,
        prompt=prompt or "sample",
    )
