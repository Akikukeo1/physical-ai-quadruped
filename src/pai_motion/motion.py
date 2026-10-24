"""Strict validation of LLM-proposed high-level motion plans."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

ACTION_TYPES = frozenset({"forward", "backward", "turn_left", "turn_right", "wait"})
MAX_ACTIONS = 12
MAX_TOTAL_SECONDS = 15.0
MIN_DURATION = 0.2
MAX_DURATION = 2.0


class InvalidPlan(ValueError):
    """Raised when a plan cannot be run safely."""


@dataclass(frozen=True)
class Action:
    type: str
    duration: float


@dataclass(frozen=True)
class MotionPlan:
    intent: str
    actions: tuple[Action, ...]


def validate_plan(value: Any) -> MotionPlan:
    if not isinstance(value, dict) or set(value) != {"intent", "actions"}:
        raise InvalidPlan("plan must have exactly: intent, actions")

    intent = value["intent"]
    actions = value["actions"]
    if not isinstance(intent, str) or not 1 <= len(intent.strip()) <= 120:
        raise InvalidPlan("intent must be a nonempty string of up to 120 characters")
    if not isinstance(actions, list) or not 1 <= len(actions) <= MAX_ACTIONS:
        raise InvalidPlan(f"actions must contain 1 to {MAX_ACTIONS} items")

    parsed: list[Action] = []
    for index, item in enumerate(actions):
        if not isinstance(item, dict) or set(item) != {"type", "duration"}:
            raise InvalidPlan(f"action {index}: expected exactly type, duration")
        kind, duration = item["type"], item["duration"]
        if not isinstance(kind, str) or kind not in ACTION_TYPES:
            raise InvalidPlan(f"action {index}: unknown type {kind!r}")
        if isinstance(duration, bool) or not isinstance(duration, (int, float)):
            raise InvalidPlan(f"action {index}: duration must be a number")
        duration = float(duration)
        if not math.isfinite(duration) or not MIN_DURATION <= duration <= MAX_DURATION:
            raise InvalidPlan(f"action {index}: duration outside [{MIN_DURATION}, {MAX_DURATION}]")
        parsed.append(Action(kind, duration))

    if sum(action.duration for action in parsed) > MAX_TOTAL_SECONDS + 1e-9:
        raise InvalidPlan(f"plan exceeds {MAX_TOTAL_SECONDS} seconds")
    return MotionPlan(intent=intent.strip(), actions=tuple(parsed))
