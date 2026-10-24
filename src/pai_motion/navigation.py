"""Known-map waypoint task with bounded, measured-position feedback."""

from __future__ import annotations

import heapq
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol

from pai_motion.executor import Velocity
from pai_motion.motion import InvalidPlan

WAYPOINTS = {
    "home": (0.0, 0.0),
    "north_gate": (0.0, 1.05),
    "south_gate": (0.0, -1.05),
    "north_exit": (2.45, 1.35),
    "south_exit": (2.45, -1.35),
    "blue": (1.85, 1.05),
    "green": (1.85, -1.05),
}
OBSTACLE = (0.9, 0.0, 0.18, 0.28)  # center x/y and box half sizes, metres
BLOCKER = (0.9, 1.05, 0.24, 0.20)
GOALS = frozenset({"home", "blue", "green"})
ROBOT_RADIUS = 0.45
ROUTE_MARGIN = 0.1
GOAL_TOLERANCE = 0.12
APPROACH_TOLERANCE = 0.08  # Leave room for the released policy's stopping transient.
MAX_TASK_SECONDS = 90.0
MAX_WAYPOINT_SECONDS = 35.0
MAX_REPLANS = 2


@dataclass(frozen=True)
class NavigationPlan:
    intent: str
    waypoints: tuple[str, ...]


def _segment_clear_box(start: tuple[float, float], end: tuple[float, float], obstacle: Sequence[float]) -> bool:
    """Intersect the entire segment with a box inflated by the robot footprint."""
    x, y, hx, hy = obstacle
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


def segment_clear(start, end, obstacles=(OBSTACLE,)) -> bool:
    return all(_segment_clear_box(start, end, obstacle) for obstacle in obstacles)


def route_clear(start, points, obstacles) -> bool:
    for name in points:
        if not segment_clear(start, WAYPOINTS[name], obstacles):
            return False
        start = WAYPOINTS[name]
    return True


def validate_navigation(value: object, start=(0.0, 0.0), obstacles=(OBSTACLE,), required=None) -> NavigationPlan:
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
        if not segment_clear(start, WAYPOINTS[point], obstacles):
            raise InvalidPlan(f"route to {point} crosses the obstacle safety margin")
        start = WAYPOINTS[point]
    if required is not None:
        if tuple(point for point in points if point in GOALS) != tuple(required) or (
            required and points[-1] != required[-1]
        ):
            raise InvalidPlan("replanned route must preserve every remaining goal in order and finish at the last goal")
    return NavigationPlan(intent.strip(), tuple(points))


def sample_navigation() -> NavigationPlan:
    return validate_navigation(
        {"intent": "障害物を避けて青、緑の順に訪問する", "waypoints": ["north_gate", "blue", "green"]}
    )


def rules_navigation(state: dict, required: tuple[str, ...]) -> NavigationPlan:
    """Shortest visibility-graph route, using the same observations and validator as Codex."""
    start = tuple(state["position"][:2])
    route = []
    for goal in required:
        nodes = {name: point for name, point in WAYPOINTS.items() if name not in GOALS or name == goal}
        nodes["start"] = start
        queue = [(0.0, (), "start")]
        visited = set()
        while queue:
            cost, path, name = heapq.heappop(queue)
            if name in visited:
                continue
            visited.add(name)
            if name == goal:
                route.extend(path)
                start = WAYPOINTS[goal]
                break
            for neighbor, point in nodes.items():
                if neighbor not in visited and segment_clear(nodes[name], point, state["obstacles"]):
                    heapq.heappush(queue, (cost + math.dist(nodes[name], point), (*path, neighbor), neighbor))
        else:
            raise RuntimeError(f"No safe named-waypoint route to {goal}")
    return validate_navigation(
        {"intent": "更新地図に基づく最短の安全経路", "waypoints": route},
        tuple(state["position"][:2]),
        state["obstacles"],
        required,
    )


class NavigationSink(Protocol):
    def observe(self) -> dict: ...
    def run_velocity(self, cmd: Velocity, seconds: float) -> None: ...
    def stop(self) -> None: ...


def execute_navigation(
    plan: NavigationPlan,
    sink: NavigationSink,
    *,
    replanner: Callable[[dict, tuple[str, ...]], NavigationPlan] | None = None,
    replan_events: list[dict] | None = None,
) -> dict:
    """Track waypoints at 5 Hz; physics/controller still owns every joint."""
    first = sink.observe()
    plan = validate_navigation(
        {"intent": plan.intent, "waypoints": list(plan.waypoints)},
        tuple(first["position"][:2]),
        first.get("obstacles", (OBSTACLE,)),
    )
    reached = []
    replans = [] if replan_events is None else replan_events
    pending = list(plan.waypoints)
    remaining = [name for name in pending if name in GOALS]
    start_time = first["time"]
    waypoint_start = start_time
    try:
        while pending:
            name = pending[0]
            target = WAYPOINTS[name]
            state = sink.observe()
            x, y = state["position"][:2]
            distance = math.hypot(target[0] - x, target[1] - y)
            if distance <= APPROACH_TOLERANCE:
                reached.append({"waypoint": name, "distance": distance, "state": state})
                pending.pop(0)
                if name in GOALS:
                    remaining.pop(0)
                waypoint_start = state["time"]
                continue
            if (
                state["time"] - start_time >= MAX_TASK_SECONDS - 1e-9
                or state["time"] - waypoint_start >= MAX_WAYPOINT_SECONDS - 1e-9
            ):
                raise RuntimeError(f"Navigation timeout before reaching {name}")
            if not route_clear((x, y), pending, state.get("obstacles", (OBSTACLE,))):
                if len(replans) >= MAX_REPLANS:
                    raise RuntimeError("Navigation replan limit exceeded")
                event = {
                    "reason": "observed route blockage",
                    "detected": state,
                    "remaining_goals": list(remaining),
                    "old_route": list(pending),
                    "status": "stopped",
                }
                replans.append(event)
                if replanner is None:
                    raise RuntimeError("Measured route is blocked; stopping without replanning")
                for _ in range(3):
                    if sink.observe()["time"] - start_time + 1.0 > MAX_TASK_SECONDS + 1e-9:
                        raise RuntimeError("Navigation time budget exhausted while settling")
                    sink.run_velocity(Velocity(0.0, 0.0, 0.0), 1.0)
                    stopped = sink.observe()
                    if math.hypot(*stopped["body_velocity"][:2]) < 0.03 and abs(stopped["yaw_rate"]) < 0.05:
                        event["stopped"] = stopped
                        break
                else:
                    raise RuntimeError("Go2 did not settle before replanning")
                requested_at = time.monotonic()
                event["status"] = "planning"
                try:
                    replacement = replanner(stopped, tuple(remaining))
                    replacement = validate_navigation(
                        {"intent": replacement.intent, "waypoints": list(replacement.waypoints)},
                        tuple(stopped["position"][:2]),
                        stopped["obstacles"],
                        remaining,
                    )
                    event.update(status="accepted", intent=replacement.intent, new_route=list(replacement.waypoints))
                except Exception as exc:
                    event.update(status="failed", error=str(exc))
                    raise
                finally:
                    event["planning_wall_seconds"] = time.monotonic() - requested_at
                    event["planning_sim_seconds"] = sink.observe()["time"] - stopped["time"]
                print(f"REPLAN: {replacement.intent}; {' -> '.join(replacement.waypoints)}")
                pending = list(replacement.waypoints)
                waypoint_start = stopped["time"]
                continue
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
    return {
        "reached": reached,
        "replans": replans,
        "elapsed": final["time"] - start_time,
        "final": final,
        "goal_error": error,
    }
