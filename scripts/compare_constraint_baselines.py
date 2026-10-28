"""Run development baselines on the same recorded initial plan and timed inputs."""

import argparse
import hashlib
import json
import re
import sys
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pai_motion import planner  # noqa: E402
from pai_motion.constraint_revision import codex_constraint_revision  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import execute_search, validate_search  # noqa: E402
from pai_motion.sim_adapter import MuJoCoGo2Sink  # noqa: E402


def rule_revision(message, plan, context):
    """Small disclosed grammar: 入らない/避ける, 戻る+確認, and 解除."""
    pending, active = deepcopy(context["pending"]), deepcopy(context["constraints"])
    result = {
        "intent": "ルールによる訂正",
        "status": "accept",
        "steps": pending,
        "cancelled": [],
        "question": "",
        "constraints": active,
        "released_constraints": [],
    }
    observations = [o for o in context["observations"] if o["color"] == "green"]
    latest = max(observations, key=lambda o: o["time"]) if observations else None
    if "解除" in message and len(active) == 1 and any(o["id"] == active[0]["observation_id"] for o in observations):
        result["constraints"] = []
        result["released_constraints"] = [{"id": c["id"], "instruction": message} for c in active]
        result["steps"].insert(
            0,
            {
                "id": "rule-reobserve",
                "action": "reobserve",
                "target": "green",
                "observation_id": active[0]["observation_id"],
            },
        )
    elif ("戻" in message and "確認" in message) and active:
        result.update(status="clarify", question="立入制限を解除しますか？")
    elif ("入らない" in message or "避け" in message) and latest:
        match = re.search(r"半径\s*(\d+(?:\.\d+)?)\s*(?:m|メートル)", message)
        if match and 0.2 <= float(match.group(1)) <= 1.5:
            result["constraints"].append(
                {
                    "id": "rule-area",
                    "observation_id": latest["id"],
                    "radius": float(match.group(1)),
                    "instruction": message,
                }
            )
        else:
            result.update(status="clarify", question="範囲を指定してください。")
    else:
        result.update(status="clarify", question="対象や指示を確認してください。")
    return result


def current_only(context):
    view = deepcopy(context)
    references = {c["observation_id"] for c in view["constraints"]}
    references |= {s["observation_id"] for s in view["pending"] if s["action"] == "reobserve"}
    view["observations"] = [
        o for o in view["observations"] if o["time"] >= view["latest_scan"]["time"] - 1e-9 or o["id"] in references
    ]
    return view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", choices=("rules", "no-history"), required=True)
    parser.add_argument("--plan-log", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    source = json.loads(args.plan_log.read_text(encoding="utf-8"))
    plan = validate_search(source["search_plan"])
    input_path = ROOT / "examples/constraint-dialogue.json"
    events = json.loads(input_path.read_text(encoding="utf-8"))
    args.out_dir.mkdir(parents=True, exist_ok=False)
    log = {
        "baseline": args.baseline,
        "plan_log": str(args.plan_log),
        "plan_log_sha256": hashlib.sha256(args.plan_log.read_bytes()).hexdigest(),
        "input_sha256": hashlib.sha256(input_path.read_bytes()).hexdigest(),
        "plan": source["search_plan"],
        "seed": source["scene"]["seed"],
        "observations": [],
        "decisions": [],
        "corrections": [],
        "planner_inputs": [],
        "status": "failed",
    }
    original_json = planner._codex_json
    sink = None
    try:
        sink = MuJoCoGo2Sink(headless=True, search_seed=log["seed"])
        started = sink.observe()["time"]
        queue = list(enumerate(events))

        def receive(state):
            if queue and state["time"] - started >= queue[0][1]["at"] - 1e-9:
                index, event = queue.pop(0)
                return {"id": f"baseline-{index}", "message": event["message"]}
            return None

        def interpret(message, original, context):
            view = current_only(context) if args.baseline == "no-history" else deepcopy(context)
            entry = {"message": message, "context": deepcopy(view)}
            log["planner_inputs"].append(entry)
            if args.baseline == "rules":
                return rule_revision(message, original, view)

            def capture(*arguments):
                raw = original_json(*arguments)
                entry["raw"] = raw
                return raw

            try:
                with patch.object(planner, "_codex_json", side_effect=capture):
                    return codex_constraint_revision(message, original, view)
            except InvalidPlan:
                log["error_kind"] = "language_or_validation"
                raise
            except (RuntimeError, OSError):
                log["error_kind"] = "infrastructure"
                raise

        log["result"] = execute_search(
            plan,
            sink,
            observations=log["observations"],
            decisions=log["decisions"],
            corrections=log["corrections"],
            get_correction=receive,
            correction_planner=interpret,
            constraint_mode=True,
        )
        log["status"] = "completed"
    except (RuntimeError, ValueError, OSError) as exc:
        log["error"] = str(exc)
        log.setdefault("error_kind", "validation" if isinstance(exc, ValueError) else "execution")
    finally:
        if sink:
            sink.close()
            log["events"], log["trace"] = sink.events, sink.trace
        output = args.out_dir / "run.json"
        output.write_text(json.dumps(log, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(
        json.dumps(
            {
                "baseline": args.baseline,
                "status": log["status"],
                "error": log.get("error"),
                "outcome": log.get("result", {}).get("outcome"),
                "log": str(output),
            },
            ensure_ascii=False,
        )
    )
    return 0 if log["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
