"""Independently audit saved successful Codex/MuJoCo investigation traces."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pai_motion.investigation import validate_decision  # noqa: E402
from pai_motion.investigation_world import INSPECTION_RADIUS, InspectionWorld, ProbeInspectionWorld  # noqa: E402


def verify(log):
    assert log["status"] == "success", "Mission did not succeed"
    assert log["returned"] and log["home_error"] <= 0.15 and log["within_deadline"]
    assert not log["errors"]
    assert log["reported_at"] <= log["return_started"]
    geometry = log["evaluation_world"]["geometry"]
    probe = log.get("probe_enabled", False)
    world = (ProbeInspectionWorld if probe else InspectionWorld)(geometry, log["evaluation_world"]["cause"])
    observations = log["observations"]
    expected_decisions = []
    for entry in log["decisions"]:
        context = entry["context"]
        expected_fields = {
            "task",
            "symptom",
            "inventory",
            "knowledge",
            "observations",
            "unavailable",
            "previous_decisions",
            "remaining_seconds",
            "home_obligation",
        }
        if probe:
            expected_fields.add("probe")
        assert set(context) == expected_fields
        assert context["inventory"] == world.inventory(), "Inventory must contain locations, not equipment states"
        assert context["observations"] == [o for o in observations if o["time"] <= entry["started"]]
        assert context["previous_decisions"] == expected_decisions
        assert entry["raw"] == entry["decision"] == validate_decision(entry["raw"], context)
        expected_decisions.append(entry["decision"])
        waiting = [
            e for e in log["events"] if e["start"]["time"] >= entry["started"] and e["end"]["time"] <= entry["finished"]
        ]
        assert all(e["vx"] == e["vy"] == e["yaw"] == 0 for e in waiting)
    for observation in observations:
        point = next(p for p in geometry["points"] if p["id"] == observation["equipment"])
        if "item" not in observation:
            assert math.dist(point["position"], observation["position"]) <= INSPECTION_RADIUS
        state = min(log["trace"], key=lambda s: abs(s["time"] - observation["time"]))
        assert abs(state["time"] - observation["time"]) < 0.051
        assert math.dist(state["position"][:2], observation["position"]) < 0.01
        assert math.hypot(*state["body_velocity"][:2]) <= 0.05 and abs(state["yaw_rate"]) <= 0.08
        if "item" in observation:
            from types import SimpleNamespace

            assert observation["valid"] and observation["value"] is not None
            assert math.dist(state["probe"]["tip"], observation["pose"]["tip"]) < 0.01
            fixture = SimpleNamespace(
                observe=lambda state=state: state,
                probe_pose=lambda observation=observation: observation["pose"],
                probe_line_clear=lambda source, observation=observation: observation["geometry"]["line_of_sight"],
            )
            recalculated = world.measure(observation["item"], fixture)
            assert recalculated["valid"] and recalculated["value"] == observation["value"]
        else:
            assert world.inspect(observation["equipment"], state)["readings"] == observation["readings"]
    if probe:
        for event in log["events"]:
            if any(abs(event[key]) > 1e-6 for key in ("vx", "vy", "yaw")):
                assert abs(event["start"]["probe"]["angle"]) <= 0.05
                assert abs(event["end"]["probe"]["angle"]) <= 0.05
        for action in log["actions"]:
            if "measurement" in action:
                assert action["started"] <= action["sampled_at"] <= action["finished"]
                assert abs(action["deployed"]["angle"]) > 0.2 and abs(action["stowed"]["angle"]) < 0.02
    assert all(
        not s["obstacle_contact"] and s["finite"] and s["upright"] >= 0.65 and s["obstacle_clearance"] >= 0.05
        for s in log["trace"]
    )
    assert all(
        abs(e["vx"]) <= 0.3 and abs(e["vy"]) <= 0.2 and abs(e["yaw"]) <= 0.6 and 0.2 <= e["duration"] <= 2
        for e in log["events"]
    )
    evaluation = world.evaluate(log["report"], observations)
    assert evaluation["correct_cause"] and evaluation["evidence_sufficient"]
    return {
        "scene": log["scene"],
        "seed": log["seed"],
        "conclusion": log["report"]["conclusion"],
        "probe_enabled": probe,
        "inspection_order": [o["equipment"] for o in observations],
        "elapsed": log["elapsed"],
        "home_error": log["home_error"],
        "min_clearance": min(s["obstacle_clearance"] for s in log["trace"]),
        "decisions": [
            {
                "action": d["decision"]["action"],
                "equipment": d["decision"]["equipment"],
                "reason": d["decision"]["reason"],
                "knowledge_ids": d["decision"]["knowledge_ids"],
                "evidence_ids": d["decision"]["evidence_ids"],
            }
            for d in log["decisions"]
        ],
        "report": log["report"],
        "observations": observations,
        "evaluation": evaluation,
        "checks": "context isolation, full observed history, manual/observation references, proximity, "
        "settled reading, stationary inference, bounded controls, safe trace, report-before-return, "
        "deadline, home tolerance, evidence sufficiency",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    results = []
    for path in args.logs:
        raw = path.read_bytes()
        result = verify(json.loads(raw))
        result.update(log=str(path), log_sha256=hashlib.sha256(raw).hexdigest())
        results.append(result)
        print(f"PASS: {path}: {result['conclusion']}; {result['elapsed']:.1f}s; home={result['home_error']:.3f}m")
    if args.output:
        args.output.write_text(json.dumps({"results": results}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
