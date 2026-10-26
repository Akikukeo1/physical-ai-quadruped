"""Frozen paired MuJoCo evaluation of observation history in in-flight corrections."""

import argparse
import hashlib
import importlib.metadata
import json
import sys
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pai_motion import planner  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import execute_search, validate_search  # noqa: E402
from pai_motion.sim_adapter import MuJoCoGo2Sink  # noqa: E402


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")


def observed(scan, color):
    return sum(ray[3] == color for ray in scan["scan"]) >= 2


def observed_context(context, condition):
    view = deepcopy(context)
    if condition == "current_only":
        references = {s["observation_id"] for s in view["pending"] if s["action"] == "reobserve"}
        view["observations"] = [
            o for o in view["observations"] if o["time"] >= view["latest_scan"]["time"] - 1e-9 or o["id"] in references
        ]
    return view


def message_source(observations, corrections, messages, started):
    sent = 0

    def receive(state):
        nonlocal sent
        if sent == 0 and observations:
            if (
                state["time"] - started <= 90
                and any(observed(o, "green") for o in observations)
                and not observed(observations[-1], "green")
                and not any(observed(o, "blue") for o in observations)
            ):
                sent = 1
                return {"id": "revision-1", "message": messages[0]}
        if sent == 1 and corrections and corrections[0]["status"] in ("accepted", "clarification"):
            finished = corrections[0].get("applied_time", corrections[0].get("question_time"))
            if state["time"] >= finished + 2 - 1e-9:
                sent = 2
                return {"id": "revision-2", "message": messages[1]}
        return None

    return receive


def score(log):
    result = log.get("result", {})
    turns = log["corrections"]
    trace = log.get("trace", [])
    tasks = result.get("tasks", {"pending": [], "completed": [], "cancelled": []})
    safe = bool(trace) and all(s["finite"] and s["upright"] >= 0.65 and not s["obstacle_contact"] for s in trace)
    fresh = False
    for task in tasks["completed"]:
        step = task["step"]
        if step["action"] == "reobserve" and step["target"] == "green":
            activations = [
                d["time"] for d in log["decisions"] if d["phase"] == "task_started" and d["step"]["id"] == step["id"]
            ]
            evidence = task["evidence"]
            fresh = (
                bool(activations)
                and evidence["time"] > max(activations)
                and any(abs(o["time"] - evidence["time"]) < 1e-6 and observed(o, "green") for o in log["observations"])
            )
    cancelled = any(
        t["step"]["id"] == "search-1" and any(t["instruction"] in r["message"] for r in turns)
        for t in tasks["cancelled"]
    )
    home = any(t["step"]["id"] == "home" for t in tasks["completed"])
    returned = bool(result) and safe and result["home_error"] <= 0.15 and result["elapsed"] <= 180 and home
    invariants = all(abs(t["deadline"] - log["started"] - 180) < 1e-6 and t["home"] == log["home"] for t in turns)
    zero_waits = True
    for index, turn in enumerate(turns):
        end = turn.get("applied_time")
        if turn["status"] == "clarification":
            end = (
                turns[index + 1]["received"]["time"]
                if index + 1 < len(turns)
                else next(
                    (d["time"] for d in log["decisions"] if d.get("reason") == "clarification return reserve"), None
                )
            )
        if end is None:
            zero_waits = False
            continue
        wait = [e for e in log["events"] if turn["received"]["time"] <= e["start"]["time"] < end - 1e-9]
        zero_waits &= bool(wait) and all(e["vx"] == e["vy"] == e["yaw"] == 0 for e in wait)
    success = (
        returned
        and result["outcome"] == "returned"
        and len(turns) == 2
        and fresh
        and cancelled
        and not tasks["pending"]
        and invariants
        and zero_waits
    )
    return {
        "triggered": bool(turns),
        "mission_success": success if turns else None,
        "safe_return": returned,
        "fresh_green": fresh,
        "blue_cancelled": cancelled,
        "home_deadline_retained": invariants,
        "zero_wait_commands": zero_waits,
        "clarifications": sum(t["status"] == "clarification" for t in turns),
        "model_wait_wall_seconds": sum(t.get("planning_wall_seconds", 0) for t in turns),
        "elapsed": result.get("elapsed"),
        "home_error": result.get("home_error"),
        "distance": result.get("distance"),
        "error_kind": log.get("error_kind"),
        "error": log.get("error"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs/correction-evaluation-v1")
    args = parser.parse_args()
    protocol_path = ROOT / "docs/correction-evaluation.json"
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    if (planner.CODEX_MODEL, planner.CODEX_REASONING_EFFORT) != (protocol["model"], protocol["reasoning_effort"]):
        raise RuntimeError("Requested model differs from frozen protocol")
    files = [
        *ROOT.glob("src/**/*.py"),
        *ROOT.glob("schema/*.json"),
        Path(__file__).resolve(),
        protocol_path,
        ROOT / "assets/go2-assets.json",
    ]
    frozen = {
        "protocol": protocol,
        "source_sha256": {
            p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)
        },
        "versions": {p: importlib.metadata.version(p) for p in ("mujoco", "numpy", "onnxruntime")},
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    freeze_path, report_path = args.out_dir / "frozen-inputs.json", args.out_dir / "evaluation.json"
    if freeze_path.exists():
        if json.loads(freeze_path.read_text(encoding="utf-8")) != frozen:
            raise RuntimeError("Frozen inputs changed; do not resume or tune held-out evaluation")
    else:
        save(freeze_path, frozen)
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else {"results": []}
    original_json = planner._codex_json
    for index, seed in enumerate(protocol["seeds"]):
        conditions = protocol["conditions"] if index % 2 == 0 else list(reversed(protocol["conditions"]))
        for condition in conditions:
            previous = [r for r in report["results"] if (r["seed"], r["condition"]) == (seed, condition)]
            if previous and previous[-1]["error_kind"] != "infrastructure":
                continue
            log = {
                "seed": seed,
                "condition": condition,
                "observations": [],
                "decisions": [],
                "corrections": [],
                "planner_inputs": [],
            }
            sink = None

            def interpret(message, plan, context, *, condition=condition, log=log):
                view = observed_context(context, condition)
                entry = {"message": message, "context": view}
                log["planner_inputs"].append(entry)

                def capture(*arguments):
                    raw = original_json(*arguments)
                    entry["raw"] = raw
                    return raw

                try:
                    with patch.object(planner, "_codex_json", side_effect=capture):
                        return planner.codex_correction(message, plan, view)
                except InvalidPlan:
                    log["error_kind"] = "language"
                    raise
                except (RuntimeError, OSError):
                    log["error_kind"] = "infrastructure"
                    raise

            try:
                sink = MuJoCoGo2Sink(headless=True, search_seed=seed)
                initial = sink.observe()
                log.update(started=initial["time"], home=list(initial["position"][:2]))
                log["result"] = execute_search(
                    validate_search(protocol["plan"]),
                    sink,
                    observations=log["observations"],
                    decisions=log["decisions"],
                    corrections=log["corrections"],
                    get_correction=message_source(
                        log["observations"], log["corrections"], protocol["messages"][index], initial["time"]
                    ),
                    correction_planner=interpret,
                )
            except (RuntimeError, ValueError, OSError) as exc:
                log["error"] = str(exc)
                log.setdefault("error_kind", "physics")
            finally:
                if sink:
                    sink.close()
                    log["trace"], log["events"] = sink.trace, sink.events
            item = {"seed": seed, "condition": condition, **score(log)}
            path = args.out_dir / f"seed-{seed}-{condition}-attempt-{len(previous) + 1}.json"
            save(path, log)
            item.update(log=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            report["results"].append(item)
            save(report_path, report)
            print(json.dumps(item, ensure_ascii=False), flush=True)
            if item["error_kind"] == "infrastructure":
                return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
