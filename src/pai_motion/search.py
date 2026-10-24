"""Partial-map frontier exploration; consumes pose and range/color samples only."""

from __future__ import annotations

import heapq
import math
from dataclasses import asdict, dataclass

from pai_motion.executor import Velocity
from pai_motion.motion import InvalidPlan

RESOLUTION = 0.2
SENSOR_RANGE = 1.8
HOME_TOLERANCE = 0.15


@dataclass(frozen=True)
class SearchPlan:
    intent: str
    target: str
    seconds: int
    on_found: str


def validate_search(value: object) -> SearchPlan:
    if not isinstance(value, dict) or set(value) != {"intent", "target", "seconds", "on_found"}:
        raise InvalidPlan("Search requires intent, target, seconds and on_found only")
    if not isinstance(value["intent"], str) or not 1 <= len(value["intent"].strip()) <= 120:
        raise InvalidPlan("Search intent must contain 1–120 characters")
    if value["target"] not in ("blue", "green") or value["on_found"] != "return_home":
        raise InvalidPlan("Search supports blue/green and return_home only")
    if type(value["seconds"]) is not int or not 60 <= value["seconds"] <= 240:
        raise InvalidPlan("Search budget must be an integer in 60–240 seconds")
    return SearchPlan(**value)


def sample_search() -> SearchPlan:
    return validate_search(
        {"intent": "青を探して出発地点に戻る", "target": "blue", "seconds": 180, "on_found": "return_home"}
    )


class ObservedMap:
    def __init__(self):
        self.free: set[tuple[int, int]] = set()
        self.occupied: set[tuple[int, int]] = set()

    @staticmethod
    def cell(point):
        return tuple(round(v / RESOLUTION) for v in point)

    @staticmethod
    def point(cell):
        return tuple(v * RESOLUTION for v in cell)

    def update(self, observation):
        x, y = observation["position"][:2]
        for angle, distance, hit, _ in observation["scan"]:
            angle += observation["yaw"]
            dx, dy = math.cos(angle), math.sin(angle)
            for step in range(math.ceil(distance / 0.05)):
                d = min(step * 0.05, distance)
                self.free.add(self.cell((x + d * dx, y + d * dy)))
            if hit:
                self.occupied.add(self.cell((x + distance * dx, y + distance * dy)))
        self.free -= self.occupied

    def traversable(self):
        # shortcut: static 2-D world and exact odometry; upgrade for noise, slopes or moving objects.
        blocked = set()
        for x, y in self.occupied:
            for dx in range(-4, 5):
                for dy in range(-4, 5):
                    if math.hypot(dx, dy) * RESOLUTION <= 0.7:
                        blocked.add((x + dx, y + dy))
        safe = self.free - blocked
        # Unknown space also blocks the footprint, so exploration never plans through unseen areas.
        return {
            c
            for c in safe
            if all(
                (c[0] + dx, c[1] + dy) in self.free
                for dx in range(-2, 3)
                for dy in range(-2, 3)
                if math.hypot(dx, dy) <= 2.5
            )
        }

    def routes(self, position):
        safe = self.traversable()
        start = self.cell(position)
        if start not in safe:
            raise RuntimeError("Robot has no observed safe footprint")
        costs, previous = {start: 0.0}, {start: None}
        queue = [(0.0, start)]
        while queue:
            cost, cell = heapq.heappop(queue)
            if cost > costs[cell]:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = cell[0] + dx, cell[1] + dy
                if nxt not in safe or (cell[0] + dx, cell[1]) not in safe or (cell[0], cell[1] + dy) not in safe:
                    continue
                distance = cost + math.hypot(dx, dy) * RESOLUTION
                if distance < costs.get(nxt, math.inf):
                    costs[nxt], previous[nxt] = distance, cell
                    heapq.heappush(queue, (distance, nxt))
        return costs, previous

    def frontier(self, costs, visited=()):
        candidates = []
        observed = self.free | self.occupied
        for cell, distance in costs.items():
            if distance < 0.6:
                continue
            if any(math.dist(cell, old) * RESOLUTION < 0.6 for old in visited):
                continue
            # Score new sensing area; coordinates carry no target-specific or route-side preference.
            gain = sum(
                (cell[0] + dx, cell[1] + dy) not in observed
                for dx in range(-7, 8, 2)
                for dy in range(-7, 8, 2)
                if math.hypot(dx, dy) <= 7
            )
            if gain >= 3:
                candidates.append((gain / (distance + 1.0), cell))
        return max(candidates)[1] if candidates else None

    def lookahead(self, position, goal, previous):
        path = [goal]
        while previous[path[-1]] is not None:
            path.append(previous[path[-1]])
        safe = set(previous)
        target = self.point(path[-1])
        for cell in reversed(path[:-1]):
            point = self.point(cell)
            distance = math.dist(position, point)
            if distance > 0.8:
                break
            count = max(1, math.ceil(distance / 0.05))
            if not all(
                self.cell(tuple(a + (b - a) * i / count for a, b in zip(position, point, strict=True))) in safe
                for i in range(count + 1)
            ):
                break
            target = point
        return target


def execute_search(plan: SearchPlan, sink, *, observations: list, decisions: list) -> dict:
    plan = validate_search(asdict(plan))
    grid = ObservedMap()
    initial = sink.observe()
    home = tuple(initial["position"][:2])
    started = initial["time"]
    next_scan = started
    previous = {}
    goal = None
    outcome = None
    found = None
    distance_travelled = 0.0
    last = home
    visited = []
    try:
        while True:
            state = sink.observe()
            position = tuple(state["position"][:2])
            distance_travelled += math.dist(last, position)
            last = position
            elapsed = state["time"] - started
            if elapsed >= plan.seconds - 2.0:
                raise RuntimeError("Search total time budget exhausted before returning home")
            if state["time"] >= next_scan - 1e-9:
                observation = sink.sense()
                observations.append(observation)
                grid.update(observation)
                costs, previous = grid.routes(position)
                home_cell = grid.cell(home)
                if home_cell not in costs:
                    raise RuntimeError("No observed safe route home")
                hits = [ray for ray in observation["scan"] if ray[3] == plan.target]
                if found is None and len(hits) >= 2:
                    found = {"time": state["time"], "position": list(position), "range": min(ray[1] for ray in hits)}
                    outcome = "found"
                    goal = home_cell
                    decisions.append({"phase": "return", "reason": "target observed", **found})
                    print(f"FOUND: {plan.target} at t={state['time']:.1f}s; returning home")
                reserve = costs[home_cell] / 0.12 + 25.0
                if outcome is None and elapsed + reserve >= plan.seconds:
                    outcome, goal = "not_found", home_cell
                    decisions.append({"phase": "return", "reason": "return reserve", "time": state["time"]})
                if outcome is None and (goal not in costs or math.dist(position, grid.point(goal)) < 0.25):
                    if goal is not None:
                        visited.append(goal)
                    goal = grid.frontier(costs, visited)
                    if goal is None:
                        outcome, goal = "not_found", home_cell
                    decisions.append(
                        {"phase": "return" if outcome else "explore", "goal": grid.point(goal), "time": state["time"]}
                    )
                next_scan = state["time"] + 1.0
            if outcome and math.dist(position, home) < 0.10:
                break
            if goal not in previous:
                raise RuntimeError("Exploration goal left the observed safe map")
            target = home if outcome and math.dist(position, home) < 0.35 else grid.lookahead(position, goal, previous)
            heading = math.atan2(target[1] - position[1], target[0] - position[0])
            error = math.atan2(math.sin(heading - state["yaw"]), math.cos(heading - state["yaw"]))
            speed = min(0.25, 0.8 * math.dist(position, target)) if abs(error) < 0.45 else 0.0
            sink.run_velocity(Velocity(speed, 0.0, max(-0.6, min(0.6, 1.8 * error))), 0.2)
    finally:
        sink.stop()
    final = sink.observe()
    error = math.dist(home, final["position"][:2])
    if error > HOME_TOLERANCE:
        raise RuntimeError(f"Stopped outside home tolerance: {error:.3f} m")
    return {
        "outcome": outcome,
        "found": found,
        "elapsed": final["time"] - started,
        "home_error": error,
        "distance": distance_travelled,
        "final": final,
        "map": {"resolution": RESOLUTION, "free": sorted(grid.free), "occupied": sorted(grid.occupied)},
    }
