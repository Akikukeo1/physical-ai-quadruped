"""One target-only correction. Deadline and return obligation are not model-editable."""

from dataclasses import dataclass

from pai_motion.motion import InvalidPlan


@dataclass(frozen=True)
class SearchCorrection:
    intent: str
    target: str
    status: str


def validate_correction(value: object) -> SearchCorrection:
    if not isinstance(value, dict) or set(value) != {"intent", "target", "status"}:
        raise InvalidPlan("Correction requires intent, target and status only")
    if not isinstance(value["intent"], str) or not 1 <= len(value["intent"].strip()) <= 120:
        raise InvalidPlan("Correction intent must contain 1–120 characters")
    if value["status"] != "accept" or value["target"] not in ("blue", "green"):
        raise InvalidPlan("Correction must clearly specify a supported blue/green target change")
    return SearchCorrection(**value)


def sample_correction(prompt, plan) -> SearchCorrection:
    """Transport baseline only; does not interpret prompt or observation history."""
    return validate_correction({"intent": "探索対象を緑へ変更", "target": "green", "status": "accept"})
