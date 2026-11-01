"""Evidence-driven investigation; only local observations cross the LLM boundary."""

import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy

from pai_motion.executor import Velocity
from pai_motion.motion import InvalidPlan
from pai_motion.planner import ROOT, _codex_json
from pai_motion.search import HOME_TOLERANCE, ObservedMap

CAUSES = ("power_loss", "cooler_fault", "door_open")
ZERO = Velocity(0.0, 0.0, 0.0)


def load_knowledge(probe=False):
    name = "warehouse-probe-manual.json" if probe else "warehouse-manual.json"
    return json.loads((ROOT / "docs" / name).read_text(encoding="utf-8"))


def validate_decision(value, context):
    fields = {
        "action",
        "equipment",
        "hypotheses",
        "reason",
        "expected_information",
        "conclusion",
        "evidence_ids",
        "report",
        "uncertainties",
        "knowledge_ids",
    }
    probe = context.get("probe", False)
    causes = (*CAUSES, "airflow_fault") if probe else CAUSES
    if probe:
        fields.add("measurement")
    if not isinstance(value, dict) or set(value) != fields:
        raise InvalidPlan("Investigation decision has unexpected fields")
    for key in ("reason", "expected_information", "report", "uncertainties"):
        if not isinstance(value[key], str) or len(value[key]) > 1800:
            raise InvalidPlan(f"Invalid text: {key}")
    if not value["reason"].strip() or not value["uncertainties"].strip():
        raise InvalidPlan("Decision needs a reason and explicit uncertainty")
    available = {o["id"] for o in context["observations"] if o.get("valid", True)}

    def evidence(ids):
        if (
            not isinstance(ids, list)
            or any(not isinstance(i, str) for i in ids)
            or len(ids) != len(set(ids))
            or not set(ids) <= available
        ):
            raise InvalidPlan("Evidence must reference supplied observation IDs only")

    evidence(value["evidence_ids"])
    knowledge_ids = value["knowledge_ids"]
    known_sections = {s["id"] for s in context["knowledge"]["sections"]}
    if (
        not isinstance(knowledge_ids, list)
        or not knowledge_ids
        or any(not isinstance(i, str) for i in knowledge_ids)
        or len(knowledge_ids) != len(set(knowledge_ids))
        or not set(knowledge_ids) <= known_sections
    ):
        raise InvalidPlan("Cite supplied manual section IDs separately from observation evidence")
    hypotheses = value["hypotheses"]
    if not isinstance(hypotheses, list) or len(hypotheses) != len(causes):
        raise InvalidPlan("Track all supported hypotheses")
    seen = set()
    for hypothesis in hypotheses:
        if not isinstance(hypothesis, dict) or set(hypothesis) != {"cause", "status", "evidence_ids"}:
            raise InvalidPlan("Invalid hypothesis")
        if hypothesis["cause"] not in causes or hypothesis["cause"] in seen:
            raise InvalidPlan("Invalid/duplicate cause")
        seen.add(hypothesis["cause"])
        if hypothesis["status"] not in ("possible", "supported", "ruled_out"):
            raise InvalidPlan("Invalid hypothesis status")
        evidence(hypothesis["evidence_ids"])
        if hypothesis["status"] != "possible" and not hypothesis["evidence_ids"]:
            raise InvalidPlan("Changed hypotheses require evidence")
    if value["action"] == "inspect":
        if probe and value["measurement"] is not None:
            raise InvalidPlan("Inspection cannot request a measurement")
        known = {p["id"] for p in context["inventory"]}
        inspected = {o["equipment"] for o in context["observations"] if "item" not in o}
        unavailable = set(context["unavailable"])
        if value["equipment"] not in known - inspected - unavailable:
            raise InvalidPlan("Inspect a supplied, uninspected, available equipment ID")
        if not value["expected_information"].strip() or value["conclusion"] != "pending" or value["report"]:
            raise InvalidPlan("Inspection needs an information goal, no final report")
    elif probe and value["action"] == "measure":
        from pai_motion.sensor_probe import ITEMS

        item = value["measurement"]
        if value["equipment"] != "cooler" or item not in ITEMS:
            raise InvalidPlan("Measure only supplied cooler air quantities")
        if any(o.get("item") == item for o in context["observations"]):
            raise InvalidPlan("Measurement already attempted")
        if value["conclusion"] != "pending" or value["report"] or not value["expected_information"].strip():
            raise InvalidPlan("Measurement needs an information goal, no final diagnosis")
    elif value["action"] == "report":
        if probe and value["measurement"] is not None:
            raise InvalidPlan("Report cannot request a measurement")
        if value["equipment"] is not None or value["conclusion"] not in (*causes, "inconclusive"):
            raise InvalidPlan("Invalid report conclusion")
        if not value["report"].strip():
            raise InvalidPlan("A final report is required")
        if value["conclusion"] != "inconclusive":
            stations = {
                (o["equipment"], o.get("item") if probe else None)
                for o in context["observations"]
                if o["id"] in value["evidence_ids"]
            }
            if len(stations) < 2:
                raise InvalidPlan("A diagnosis needs evidence from at least two equipment stations")
            chosen = next(h for h in hypotheses if h["cause"] == value["conclusion"])
            if chosen["status"] != "supported" or not set(chosen["evidence_ids"]) <= set(value["evidence_ids"]):
                raise InvalidPlan("Conclusion must agree with the supported hypothesis and cited evidence")
    else:
        raise InvalidPlan("Only inspect/report actions are executable")
    return deepcopy(value)


def codex_investigation(context):
    instructions = (
        "You investigate rising temperature in a refrigerated warehouse. No tools, code or hidden world access. "
        "Use only the supplied local inspection observations, not equipment order or ID as evidence. "
        "Equipment coordinates are a known maintenance registry, not measured equipment states. "
        "Track power_loss, cooler_fault, door_open as hypotheses, initially possible. "
        "Read context.knowledge: a synthetic equipment manual containing general causal relationships, "
        "not current site facts. Combine its diagnostic conditions with context.observations. "
        "Cite relevant manual section IDs in knowledge_ids and actual observation IDs in evidence_ids; "
        "never treat a manual condition as an observed reading. "
        "Choose the next uninspected equipment that best distinguishes remaining hypotheses. "
        "Explain briefly what different readings would mean in expected_information, and update hypotheses "
        "after each new observation. Do not follow a fixed tour. Stop when sufficient evidence is obtained, "
        "or report inconclusive if the remaining checks cannot resolve it. Equipment marked unavailable "
        "could not be reached; that is NOT a fault reading. Cite only supplied observation IDs. "
        "A final diagnosis needs at least two distinct stations as evidence. Keep uncertainty about untested "
        "alternatives and limits explicit. Report before returning home. "
        "For action=inspect, conclusion MUST be pending and report MUST be the empty string. "
        "For action=report, equipment MUST be null; conclusion is a cause or inconclusive, report is nonempty. "
        "Use concise Japanese for explanations and report. Respond ONLY with schema-compliant JSON. "
        f"Observed context: {json.dumps(context, ensure_ascii=False)}"
    )
    if context.get("probe"):
        instructions += (
            " PROBE MODE: also track airflow_fault, four hypotheses total. inspect cooler exposes only power and "
            "compressor status; air data MUST be obtained using action=measure, equipment=cooler, measurement="
            "air_temperature or airflow. Each is a separate physical measurement requiring a different sampling "
            "position and probe alignment. Select the item using the manual and past evidence, not a fixed order. "
            "For inspect/report set measurement=null. For measure set conclusion=pending and report=''. "
            "Final cooler_fault/airflow_fault/door_open diagnoses need BOTH air temperature and airflow evidence. "
            "Normal compressor status does not prove cooling or airflow. An invalid measurement has no value "
            "and cannot support a diagnosis. No repeated measurements or repeated equipment inspections. "
            "knowledge_ids must contain unique IDs from context.knowledge.sections, with no duplicates. "
            "In probe mode, two independent measurement channels at the same equipment count as distinct evidence."
        )
    schema = (
        "investigation-probe-decision.schema.json" if context.get("probe") else "investigation-decision.schema.json"
    )
    return _codex_json(instructions, ROOT / "schema" / schema, 45)


class InvestigationMotion:
    """Reuse search mapping, safe routing and frontier exploration for named equipment."""

    def __init__(self, sink, seconds, log):
        self.sink, self.log = sink, log
        self.grid = ObservedMap()
        self.home = tuple(sink.observe()["position"][:2])
        self.started = sink.observe()["time"]
        self.deadline = self.started + seconds

    def sample(self):
        observation = self.sink.sense()
        self.log["scans"].append(observation)
        self.grid.update(observation)
        return observation

    def reserve(self, state):
        costs, _ = self.grid.routes(state["position"][:2])
        cost = costs.get(self.grid.cell(self.home))
        if cost is None:
            raise RuntimeError("No observed return path")
        return max(20.0, cost / 0.12 + 15.0)

    def remaining(self):
        return self.deadline - self.sink.observe()["time"]

    def settle(self):
        for _ in range(5):
            self.sink.run_velocity(ZERO, 0.4)
            state = self.sink.observe()
            if math.hypot(*state["body_velocity"][:2]) < 0.03 and abs(state["yaw_rate"]) < 0.05:
                return state
        raise RuntimeError("Robot did not settle for inspection/planning")

    def go(self, destination, *, returning=False):
        visited, goal = [], None
        started = self.sink.observe()["time"]
        while True:
            state = self.sample()
            position = state["position"][:2]
            if self.remaining() <= (2.5 if returning else self.reserve(state) + 3):
                return "deadline" if returning else "return_reserve"
            if math.dist(position, destination) < 0.10:
                state = self.settle()
                if math.dist(state["position"][:2], destination) <= (0.14 if returning else 0.22):
                    return "reached"
            if not returning and state["time"] - started > 100:
                return "unreachable"
            costs, previous = self.grid.routes(position)
            destination_cell = self.grid.cell(destination)
            if destination_cell in previous:
                goal = destination_cell
            elif returning:
                raise RuntimeError("Home left the observed safe map")
            elif goal not in previous or math.dist(position, self.grid.point(goal)) < 0.2:
                if goal is not None:
                    visited.append(goal)
                goal = self.grid.frontier(costs, visited)
                if goal is None:
                    return "unreachable"
            target = self.grid.lookahead(position, goal, previous)
            if goal == destination_cell and math.dist(position, destination) < 0.3:
                target = destination
            heading = math.atan2(target[1] - position[1], target[0] - position[0])
            error = math.atan2(math.sin(heading - state["yaw"]), math.cos(heading - state["yaw"]))
            limit = 0.15 if any(math.dist(position, self.grid.point(c)) < 0.8 for c in self.grid.occupied) else 0.25
            speed = min(limit, 0.8 * math.dist(position, target)) if abs(error) < 0.3 else 0.0
            self.sink.run_velocity(Velocity(speed, 0.0, max(-0.6, min(0.6, 1.8 * error))), 0.2)


def execute_investigation(
    task, sink, inventory, inspect, *, planner=codex_investigation, seconds=360, log=None, measure=None
):
    """No scene ID, seed, unobserved readings or cause is accepted by the planner interface."""
    if not isinstance(task, str) or not 1 <= len(task.strip()) <= 1800:
        raise ValueError("Task must contain 1–1800 characters")
    if type(seconds) is not int or not 60 <= seconds <= 600:
        raise ValueError("Investigation budget must be 60–600 simulated seconds")
    log = {} if log is None else log
    log.update(observations=[], decisions=[], actions=[], scans=[], report=None, errors=[], returned=False)
    knowledge = load_knowledge(probe=measure is not None)
    motion = InvestigationMotion(sink, seconds, log)
    unavailable, report = [], None
    worker = ThreadPoolExecutor(max_workers=1)
    try:
        for _ in range(8 if measure is not None else 5):
            state = motion.sample()
            if motion.remaining() <= motion.reserve(state) + 5:
                log["errors"].append("return_reserve")
                break
            motion.settle()
            context = {
                "task": task,
                "symptom": "倉庫温度8°C、設定-18°C。温度上昇警報。",
                "inventory": deepcopy(inventory),
                "knowledge": deepcopy(knowledge),
                "observations": deepcopy(log["observations"]),
                "unavailable": list(unavailable),
                "previous_decisions": [d["decision"] for d in log["decisions"] if "decision" in d],
                "remaining_seconds": motion.remaining(),
                "home_obligation": "report_then_return",
            }
            if measure is not None:
                context["probe"] = True
            entry = {"context": deepcopy(context), "started": sink.observe()["time"]}
            log["decisions"].append(entry)
            if hasattr(sink, "set_demo_status"):
                sink.set_demo_status(phase="LLM_WAIT")
            request = worker.submit(planner, deepcopy(context))
            wall_start = time.monotonic()
            timeout = None
            while not request.done():
                if motion.remaining() <= motion.reserve(sink.observe()) + 3:
                    timeout = "Return reserve reached while awaiting planner"
                    break
                if time.monotonic() - wall_start > 50:
                    timeout = "Planner wall timeout"
                    break
                tick = time.monotonic()
                sink.run_velocity(ZERO, 0.2)
                time.sleep(max(0, 0.2 - (time.monotonic() - tick)))
            try:
                if timeout:
                    raise TimeoutError(timeout)
                raw = request.result()
                entry["raw"] = deepcopy(raw)
                decision = validate_decision(raw, context)
                entry["decision"] = decision
                entry["finished"] = sink.observe()["time"]
                entry["wall_seconds"] = time.monotonic() - wall_start
            except Exception as exc:
                entry["error"] = str(exc)
                log["errors"].append(f"planner: {exc}")
                break
            motion.settle()
            print(f"INVESTIGATE: {decision['action']} {decision['equipment']}; {decision['reason']}", flush=True)
            if hasattr(sink, "set_demo_status"):
                sink.set_demo_status(
                    phase=decision["action"].upper(),
                    equipment=decision["equipment"],
                    measurement=decision.get("measurement"),
                    conclusion=decision["conclusion"],
                    hypotheses=deepcopy(decision["hypotheses"]),
                )
            if decision["action"] == "report":
                report = decision
                log["report"] = deepcopy(report)
                log["reported_at"] = sink.observe()["time"]
                print(f"REPORT: {report['report']} 不確実性: {report['uncertainties']}", flush=True)
                break
            point = next(p for p in inventory if p["id"] == decision["equipment"])
            measuring = decision["action"] == "measure"
            if measuring:
                from pai_motion.sensor_probe import execute_measurement

                port = next(p for p in point["measurements"] if p["item"] == decision["measurement"])
            action = {"equipment": point["id"], "started": sink.observe()["time"]}
            if measuring:
                action["measurement"] = decision["measurement"]
            log["actions"].append(action)
            action["outcome"] = motion.go(port["position"] if measuring else point["position"])
            action["finished"] = sink.observe()["time"]
            if action["outcome"] == "reached":
                try:
                    if measuring:
                        action["arrived"] = action["finished"]
                    observation = (
                        execute_measurement(motion, port, measure, action)
                        if measuring
                        else inspect(point["id"], sink.observe())
                    )
                    action["finished"] = sink.observe()["time"]
                except Exception as exc:
                    log["errors"].append(f"inspection: {exc}")
                    break
                observation["id"] = f"inspection-{len(log['observations']) + 1}"
                log["observations"].append(observation)
                if hasattr(sink, "set_demo_status"):
                    sink.set_demo_status(last_observation=deepcopy(observation))
                print(f"OBSERVATION: {json.dumps(observation, ensure_ascii=False)}", flush=True)
            elif action["outcome"] == "return_reserve":
                log["errors"].append("return_reserve")
                break
            else:
                unavailable.append(point["id"])
        if report is None:
            log["report"] = {
                "action": "report",
                "conclusion": "inconclusive",
                "evidence_ids": [],
                "report": "調査未完了。原因を確定できません。",
                "uncertainties": "; ".join(log["errors"]) or "判断回数上限",
                "source": "executor_failure_report",
            }
            log["reported_at"] = sink.observe()["time"]
            print(f"REPORT: {log['report']['report']} {log['report']['uncertainties']}", flush=True)
        log["return_started"] = sink.observe()["time"]
        if hasattr(sink, "set_demo_status"):
            sink.set_demo_status(phase="RETURN_HOME", equipment="home", measurement=None)
        log["return_outcome"] = motion.go(motion.home, returning=True)
    except Exception as exc:
        log["errors"].append(f"execution: {exc}")
        # Physical safety failures must stop rather than continuing a dangerous return.
        if log["report"] is None:
            log["report"] = {
                "conclusion": "inconclusive",
                "evidence_ids": [],
                "report": "物理実行を中断しました。原因を確定できず、帰還も未完了です。",
                "uncertainties": str(exc),
                "source": "executor_safety_report",
            }
            log["reported_at"] = sink.observe()["time"]
            print(f"REPORT: {log['report']['report']} {exc}", flush=True)
    finally:
        try:
            sink.stop()
        finally:
            worker.shutdown(wait=True, cancel_futures=True)
            final = sink.observe()
            log["home_error"] = math.dist(motion.home, final["position"][:2])
            log["returned"] = log.get("return_outcome") == "reached" and log["home_error"] <= HOME_TOLERANCE
            log["elapsed"] = final["time"] - motion.started
            log["within_deadline"] = final["time"] <= motion.deadline + 1e-6
            log["map"] = {"free": sorted(motion.grid.free), "occupied": sorted(motion.grid.occupied)}
    return log
