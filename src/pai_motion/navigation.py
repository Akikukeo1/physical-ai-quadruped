"""Known-map waypoint task with bounded, measured-position feedback."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from pai_motion.executor import Velocity
from pai_motion.motion import InvalidPlan

WAYPOINTS = {
    "home": (0.0, 0.0),
    "north_gate": (0.0, 1.05),
    "south_gate": (0.0, -1.05),
    "blue": (1.85, 1.05),
    "green": (1.85, -1.05),
}
OBSTACLE = (0.9, 0.0, 0.18, 0.28)  # center x/y and box half sizes, metres
ROBOT_RADIUS = 0.45
ROUTE_MARGIN = 0.1
GOAL_TOLERANCE = 0.12
APPROACH_TOLERANCE = 0.08  # Leave room for the released policy's stopping transient.
MAX_TASK_SECONDS = 90.0
MAX_WAYPOINT_SECONDS = 35.0


@dataclass(frozen=True)
class NavigationPlan:
    intent: str
    waypoints: tuple[str, ...]


def segment_clear(start: tuple[float, float], end: tuple[float, float]) -> bool:
    """Intersect the entire segment with a box inflated by the robot footprint."""
    x, y, hx, hy = OBSTACLE
    padding = ROBOT_RADIUS + ROUTE_MARGIN
    low, high = 0.0, 1.0
    for a, b, minimum, maximum in zip(
        start, end, (x - hx - padding, y - hy - padding), (x + hx + padding, y + hy + padding), strict=True
    ):
        delta = b - a
        if abs(delta) < 1e-12:
            if a < minimum or a > maximum:
                return True
        else:
            t0, t1 = sorted(((minimum - a) / delta, (maximum - a) / delta))
            low, high = max(low, t0), min(high, t1)
            if low > high:
                return True
    return False


def validate_navigation(value: object, start: tuple[float, float] = (0.0, 0.0)) -> NavigationPlan:
    if not isinstance(value, dict) or set(value) != {"intent", "waypoints"}:
        raise InvalidPlan("navigation plan must have exactly intent, waypoints")
    intent, points = value["intent"], value["waypoints"]
    if not isinstance(intent, str) or not 1 <= len(intent.strip()) <= 120:
        raise InvalidPlan("intent must be a nonempty string of up to 120 characters")
    if not isinstance(points, list) or not 1 <= len(points) <= 8:
        raise InvalidPlan("waypoints must contain 1 to 8 named points")
    for point in points:
        if not isinstance(point, str) or point not in WAYPOINTS:
            raise InvalidPlan(f"unknown waypoint: {point!r}")
        if not segment_clear(start, WAYPOINTS[point]):
            raise InvalidPlan(f"route to {point} crosses the obstacle safety margin")
        start = WAYPOINTS[point]
    return NavigationPlan(intent.strip(), tuple(points))


def sample_navigation() -> NavigationPlan:
    return validate_navigation(
        {"intent": "障害物を避けて青、緑の順に訪問する", "waypoints": ["north_gate", "blue", "green"]}
    )


class NavigationSink(Protocol):
    def observe(self) -> dict: ...
    def run_velocity(self, cmd: Velocity, seconds: float) -> None: ...
    def stop(self) -> None: ...


def execute_navigation(plan: NavigationPlan, sink: NavigationSink) -> dict:
    """Track waypoints at 5 Hz; physics/controller still owns every joint."""
    first = sink.observe()
    plan = validate_navigation({"intent": plan.intent, "waypoints": list(plan.waypoints)}, tuple(first["position"][:2]))
    reached = []
    start_time = first["time"]
    try:
        for name in plan.waypoints:
            target = WAYPOINTS[name]
            waypoint_start = sink.observe()["time"]
            while True:
                state = sink.observe()
                x, y = state["position"][:2]
                distance = math.hypot(target[0] - x, target[1] - y)
                if distance <= APPROACH_TOLERANCE:
                    reached.append({"waypoint": name, "distance": distance, "state": state})
                    break
                if (
                    state["time"] - start_time >= MAX_TASK_SECONDS
                    or state["time"] - waypoint_start >= MAX_WAYPOINT_SECONDS
                ):
                    raise RuntimeError(f"Navigation timeout before reaching {name}")
                if not segment_clear((x, y), target):
                    raise RuntimeError(f"Measured route to {name} no longer clears obstacle; stopping")
                error = math.atan2(
                    math.sin(math.atan2(target[1] - y, target[0] - x) - state["yaw"]),
                    math.cos(math.atan2(target[1] - y, target[0] - x) - state["yaw"]),
                )
                yaw = max(-0.6, min(0.6, 1.8 * error))
                speed = min(0.25, 0.8 * distance) * max(0.0, math.cos(error)) if abs(error) < 0.45 else 0.0
                sink.run_velocity(Velocity(speed, 0.0, yaw), 0.2)
    finally:
        sink.stop()
    final = sink.observe()
    target = WAYPOINTS[plan.waypoints[-1]]
    error = math.hypot(target[0] - final["position"][0], target[1] - final["position"][1])
    if error > GOAL_TOLERANCE:
        raise RuntimeError(f"Stopped outside final goal tolerance: {error:.3f} m")
    return {"reached": reached, "elapsed": final["time"] - start_time, "final": final, "goal_error": error}
