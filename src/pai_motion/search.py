"""Partial-map frontier exploration; consumes pose and range/color samples only."""

from __future__ import annotations

import heapq
import math
import time
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict, dataclass, replace

from pai_motion.correction import initial_tasks, validate_correction
from pai_motion.executor import Velocity
from pai_motion.motion import InvalidPlan

RESOLUTION = 0.1
SENSOR_RANGE = 1.8
HOME_TOLERANCE = 0.15
FOOTPRINT = [(dx, dy) for dx in range(-5, 6) for dy in range(-5, 6) if math.hypot(dx, dy) * RESOLUTION <= 0.45]
# .45 m body + .05 m reserve + .071 m worst-case cell quantization.
INFLATION = [(dx, dy) for dx in range(-6, 7) for dy in range(-6, 7) if math.hypot(dx, dy) * RESOLUTION <= 0.58]


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

    def traversable(self, restricted=()):
        restricted = tuple(restricted)
        # shortcut: static 2-D world and exact odometry; upgrade for noise, slopes or moving objects.
        blocked = set()
        for x, y in self.occupied:
            blocked.update((x + dx, y + dy) for dx, dy in INFLATION)
        safe = self.free - blocked
        # The radius constrains the robot center; the margin accounts for grid rounding and tracking error.
        if restricted:
            safe = {
                cell
                for cell in safe
                if all(math.dist(self.point(cell), center) > radius + 0.15 for center, radius in restricted)
            }
        # Unknown space also blocks the footprint, so exploration never plans through unseen areas.
        return {c for c in safe if all((c[0] + dx, c[1] + dy) in self.free for dx, dy in FOOTPRINT)}

    def routes(self, position, restricted=()):
        restricted = tuple(restricted)
        if any(math.dist(position, center) <= radius + 0.15 for center, radius in restricted):
            raise RuntimeError("Robot is inside or too close to a restricted area")
        safe = self.traversable(restricted)
        start = self.cell(position)
        if start not in safe:
            # Tracking drift can round a safe measured pose into an inflated cell.
            if (
                not safe
                or any((start[0] + dx, start[1] + dy) not in self.free for dx, dy in FOOTPRINT)
                or any(math.dist(position, self.point(c)) < 0.52 for c in self.occupied)
            ):
                raise RuntimeError("Robot has no observed safe footprint")
            start = min(safe, key=lambda c: math.dist(position, self.point(c)))
            if math.dist(position, self.point(start)) > 0.15:
                raise RuntimeError("No adjacent observed safe cell for tracking recovery")
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
            gain = 0
            for dx in range(-16, 17, 4):
                for dy in range(-16, 17, 4):
                    if math.hypot(dx, dy) > 16 or (cell[0] + dx, cell[1] + dy) in observed:
                        continue
                    steps = max(abs(dx), abs(dy))
                    # A known wall hides the unknown cells behind it; those are not information gain.
                    if not any(
                        (cell[0] + round(dx * i / steps), cell[1] + round(dy * i / steps)) in self.occupied
                        for i in range(1, steps + 1)
                    ):
                        gain += 1
            if gain >= 1:
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


def execute_search(
    plan: SearchPlan,
    sink,
    *,
    observations: list,
    decisions: list,
    get_correction=None,
    correction_planner=None,
    corrections: list | None = None,
) -> dict:
    plan = validate_search(asdict(plan))
    if (get_correction is None) != (correction_planner is None):
        raise ValueError("Correction source and planner must be provided together")
    corrections = [] if corrections is None else corrections
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
    detections = {}
    evidence_records = {}
    first_seen, last_seen = {}, {}
    tasks, completed, cancelled = initial_tasks(plan.target), [], []
    active_id, active_since = None, started
    worker = pending = revision = None
    seen_messages = set()
    dialogue = []
    awaiting_answer = False
    try:
        while True:
            state = sink.observe()
            position = tuple(state["position"][:2])
            distance_travelled += math.dist(last, position)
            last = position
            elapsed = state["time"] - started
            if elapsed >= plan.seconds - 2.0:
                raise RuntimeError("Search total time budget exhausted before returning home")
            scanned = state["time"] >= next_scan - 1e-9
            if scanned:
                observation = sink.sense()
                observations.append(observation)
                grid.update(observation)
                costs, previous = grid.routes(position)
                home_cell = grid.cell(home)
                if home_cell not in costs:
                    raise RuntimeError("No observed safe route home")
                for color in ("blue", "green"):
                    hits = [ray for ray in observation["scan"] if ray[3] == color]
                    if len(hits) >= 2:
                        detection = {
                            "time": state["time"],
                            "position": list(position),
                            "range": min(ray[1] for ray in hits),
                        }
                        detections.setdefault(color, detection)
                        record = {"id": f"obs-{len(observations)}-{color}", "color": color, **detection}
                        evidence_records[record["id"]] = record
                        first_seen.setdefault(color, record)
                        last_seen[color] = record
                next_scan = state["time"] + 1.0
            if get_correction and pending is None and outcome is None:
                message = get_correction(state)
                message_id = None
                if isinstance(message, dict):
                    if set(message) != {"id", "message"} or not isinstance(message["id"], str):
                        raise ValueError("Correction event requires a string id and message")
                    message_id, message = message["id"], message["message"]
                    if not 1 <= len(message_id) <= 80:
                        raise ValueError("Correction event ID must contain 1–80 characters")
                if message is not None:
                    if not isinstance(message, str) or not 1 <= len(message.strip()) <= 500:
                        raise ValueError("Correction must contain 1–500 characters")
                    token = ("event", message_id) if message_id is not None else ("text", message)
                    if token in seen_messages:
                        message = None
                if message is not None:
                    if len(corrections) >= 12:
                        raise ValueError("At most 12 correction messages per mission")
                    seen_messages.add(token)
                    dialogue.append({"role": "user", "text": message})
                    # A detached snapshot avoids sharing mutable task/history state with the planner thread.
                    context = deepcopy(
                        {
                            "time": state["time"],
                            "position": list(position),
                            "yaw": state["yaw"],
                            "home": list(home),
                            "deadline": started + plan.seconds,
                            "remaining_seconds": plan.seconds - elapsed,
                            "latest_scan": {
                                "time": observations[-1]["time"],
                                "visible_colors": [
                                    color
                                    for color in ("blue", "green")
                                    if sum(ray[3] == color for ray in observations[-1]["scan"]) >= 2
                                ],
                                "range_limit": SENSOR_RANGE,
                            },
                            "pending": tasks,
                            "completed": completed,
                            "cancelled": cancelled,
                            "dialogue": dialogue,
                            "observations": list(
                                {
                                    record["id"]: record
                                    for record in [
                                        *first_seen.values(),
                                        *last_seen.values(),
                                        *(
                                            evidence_records[step["observation_id"]]
                                            for step in tasks
                                            if step["action"] == "reobserve"
                                        ),
                                    ]
                                }.values()
                            ),
                            "map_summary": {
                                "resolution": RESOLUTION,
                                "free_cells": len(grid.free),
                                "occupied_cells": len(grid.occupied),
                                "visited_frontiers": [grid.point(cell) for cell in visited],
                            },
                        }
                    )
                    revision = {
                        "message": message,
                        "status": "planning",
                        "received": state,
                        "original_plan": asdict(plan),
                        "home": list(home),
                        "deadline": started + plan.seconds,
                        "observations_before": len(observations),
                        "known_colors_before": sorted(detections),
                        "wall_started": time.monotonic(),
                        "old_goal": grid.point(goal) if goal is not None else None,
                        "context": context,
                    }
                    corrections.append(revision)
                    if worker is None:
                        worker = ThreadPoolExecutor(max_workers=1)
                    pending = worker.submit(correction_planner, message, plan, deepcopy(context))
                    print(f"CORRECTION: {message}; holding position", flush=True)
            if pending is not None:
                tick = time.monotonic()
                revision.setdefault("first_zero_command_time", state["time"])
                sink.run_velocity(Velocity(0.0, 0.0, 0.0), 0.2)
                stopped = sink.observe()
                if "settled" not in revision:
                    if math.hypot(*stopped["body_velocity"][:2]) < 0.03 and abs(stopped["yaw_rate"]) < 0.05:
                        revision["settled"] = stopped
                    elif stopped["time"] - revision["received"]["time"] > 3.0:
                        raise RuntimeError("Robot did not settle during correction")
                if pending.done() and "settled" in revision:
                    correction = validate_correction(
                        asdict(pending.result()), context=revision["context"], prompt=revision["message"]
                    )
                    if correction.status == "clarify":
                        revision.update(
                            status="clarification",
                            correction=asdict(correction),
                            question_time=stopped["time"],
                            planning_wall_seconds=time.monotonic() - revision["wall_started"],
                        )
                        dialogue.append({"role": "assistant", "text": correction.question})
                        print(f"QUESTION: {correction.question}", flush=True)
                        awaiting_answer, pending = True, None
                        continue
                    removed = {entry["id"]: entry for entry in correction.cancelled}
                    cancelled.extend(
                        {"step": step, "time": stopped["time"], "instruction": removed[step["id"]]["instruction"]}
                        for step in tasks
                        if step["id"] in removed
                    )
                    tasks = deepcopy(correction.steps)
                    searches = [step for step in tasks if step["action"] == "search"]
                    plan = replace(
                        plan, target=searches[-1]["target"] if searches else plan.target, intent=correction.intent
                    )
                    revision.update(
                        status="accepted",
                        correction=asdict(correction),
                        updated_plan=asdict(plan),
                        applied_time=stopped["time"],
                        observations_after=len(observations),
                        known_colors_after=sorted(detections),
                        planning_wall_seconds=time.monotonic() - revision["wall_started"],
                        planning_sim_seconds=stopped["time"] - revision["received"]["time"],
                    )
                    pending = None
                    awaiting_answer = False
                    goal = None
                    active_id = None
                    next_scan = stopped["time"]
                    print(
                        f"UPDATED: tasks={[step['action'] + ':' + str(step['target']) for step in tasks]}", flush=True
                    )
                else:
                    # Keep physics running while the model thinks, including in fast headless runs.
                    time.sleep(max(0.0, 0.2 - (time.monotonic() - tick)))
                continue
            if awaiting_answer:
                tick = time.monotonic()
                reserve = costs[home_cell] / 0.12 + 25.0
                if elapsed + reserve >= plan.seconds:
                    awaiting_answer = False
                    outcome, goal = "not_found", home_cell
                    decisions.append(
                        {"phase": "return", "reason": "clarification return reserve", "time": state["time"]}
                    )
                else:
                    sink.run_velocity(Velocity(0.0, 0.0, 0.0), 0.2)
                    time.sleep(max(0.0, 0.2 - (time.monotonic() - tick)))
                    continue
            if scanned:
                step = tasks[0]
                if active_id != step["id"]:
                    active_id, active_since, goal = step["id"], state["time"], home_cell if outcome else None
                    decisions.append({"phase": "task_started", "step": deepcopy(step), "time": active_since})
                if outcome is None and step["action"] != "return_home":
                    color = step["target"]
                    fresh = last_seen.get(color)
                    evidence = detections.get(color) if step["action"] == "search" else None
                    if step["action"] == "reobserve" and fresh and fresh["time"] > active_since + 1e-9:
                        evidence = fresh
                    if evidence is not None:
                        if step["action"] == "search":
                            found = evidence
                        completed.append({"step": tasks.pop(0), "time": state["time"], "evidence": deepcopy(evidence)})
                        reason = (
                            "fresh target observation"
                            if step["action"] == "reobserve"
                            else "target observed"
                            if abs(evidence["time"] - state["time"]) < 1e-6
                            else "target in observation history"
                        )
                        decisions.append(
                            {
                                "phase": "return" if tasks[0]["action"] == "return_home" else "task_completed",
                                "reason": reason,
                                "step": deepcopy(step),
                                "target": color,
                                "time": state["time"],
                                "evidence": deepcopy(evidence),
                            }
                        )
                        print(f"TASK DONE: {step['id']}; {reason}; evidence t={evidence['time']:.1f}s", flush=True)
                        active_id, goal = None, None
                        step = tasks[0]
                        if step["action"] != "return_home":
                            # A later task cannot reuse this scan as a new observation after its activation.
                            active_id, active_since = step["id"], state["time"]
                            decisions.append({"phase": "task_started", "step": deepcopy(step), "time": active_since})
                if outcome is None and step["action"] == "return_home":
                    outcome, goal = "found" if found is not None else "returned", home_cell
                    if not decisions or decisions[-1]["phase"] != "return":
                        decisions.append({"phase": "return", "reason": "remaining task is home", "time": state["time"]})
                reserve = costs[home_cell] / 0.12 + 25.0
                if outcome is None and elapsed + reserve >= plan.seconds:
                    outcome, goal = "not_found", home_cell
                    decisions.append({"phase": "return", "reason": "return reserve", "time": state["time"]})
                if outcome is None and step["action"] == "reobserve":
                    reference = evidence_records[step["observation_id"]]
                    goal = grid.cell(reference["position"])
                    if goal not in costs:
                        raise RuntimeError("Remembered sensing position is not reachable on the observed safe map")
                    if math.dist(position, reference["position"]) < 0.04 and state["time"] > active_since + 1e-9:
                        raise RuntimeError("Target could not be reobserved from its remembered sensing position")
                elif outcome is None and (goal not in costs or math.dist(position, grid.point(goal)) < 0.25):
                    if goal is not None:
                        visited.append(goal)
                    goal = grid.frontier(costs, visited)
                    if goal is None:
                        outcome, goal = "not_found", home_cell
                    decisions.append(
                        {"phase": "return" if outcome else "explore", "goal": grid.point(goal), "time": state["time"]}
                    )
            if outcome and math.dist(position, home) < 0.10:
                completed.append({"step": tasks[-1], "time": state["time"], "evidence": None})
                tasks = tasks[:-1]
                break
            if goal not in previous:
                raise RuntimeError("Exploration goal left the observed safe map")
            target = home if outcome and math.dist(position, home) < 0.35 else grid.lookahead(position, goal, previous)
            if outcome is None and tasks[0]["action"] == "reobserve":
                viewpoint = evidence_records[tasks[0]["observation_id"]]["position"]
                if math.dist(position, viewpoint) < 0.35:
                    target = viewpoint
            heading = math.atan2(target[1] - position[1], target[0] - position[0])
            error = math.atan2(math.sin(heading - state["yaw"]), math.cos(heading - state["yaw"]))
            limit = 0.15 if any(math.dist(position, grid.point(c)) < 0.8 for c in grid.occupied) else 0.25
            speed = min(limit, 0.8 * math.dist(position, target)) if abs(error) < 0.3 else 0.0
            sink.run_velocity(Velocity(speed, 0.0, max(-0.6, min(0.6, 1.8 * error))), 0.2)
    except Exception as exc:
        if revision is not None and revision["status"] == "planning":
            revision.update(status="failed", error=str(exc))
        raise
    finally:
        try:
            sink.stop()
        finally:
            if worker is not None:
                worker.shutdown(wait=True, cancel_futures=True)
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
        "plan": asdict(plan),
        "detections": detections,
        "corrections": corrections,
        "tasks": {"pending": tasks, "completed": completed, "cancelled": cancelled},
        "map": {"resolution": RESOLUTION, "free": sorted(grid.free), "occupied": sorted(grid.occupied)},
    }
