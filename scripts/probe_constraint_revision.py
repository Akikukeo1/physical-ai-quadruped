"""Probe a three-turn constraint dialogue against one saved development observation; no robot execution."""

import argparse
import hashlib
import json
import sys
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pai_motion import planner  # noqa: E402
from pai_motion.constraint_revision import SCHEMA, codex_constraint_revision  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import validate_search  # noqa: E402

MESSAGES = (
    "緑を見た地点から半径0.5m以内へは立ち入らず、青を引き続き探索して",
    "その緑を見た地点に戻ってもう一度緑を確認して。青の探索は続けて",
    "緑を見た地点の立入禁止を解除して、そこで緑を再確認して。青の探索と帰還は維持して",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs/constraint-language-probe")
    args = parser.parse_args()
    saved = json.loads(args.snapshot.read_text(encoding="utf-8"))
    run = saved["corrections"][0]
    context = deepcopy(run["context"])
    context.update(constraints=[], released_constraints=[], dialogue=[])
    plan = validate_search(run["original_plan"])
    assert plan.target == "blue" and any(o["color"] == "green" for o in context["observations"])
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output = args.out_dir / (datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    report = {
        "scope": "Three-turn development language probe; historical observation, no simulator execution.",
        "snapshot": str(args.snapshot),
        "snapshot_sha256": hashlib.sha256(args.snapshot.read_bytes()).hexdigest(),
        "schema_sha256": hashlib.sha256(SCHEMA.read_bytes()).hexdigest(),
        "requested_model": planner.CODEX_MODEL,
        "requested_reasoning_effort": planner.CODEX_REASONING_EFFORT,
        "results": [],
    }
    original = planner._codex_json
    for index, message in enumerate(MESSAGES):
        raw = None

        def capture(*arguments):
            nonlocal raw
            raw = original(*arguments)
            return raw

        error, infrastructure = None, False
        try:
            with patch.object(planner, "_codex_json", side_effect=capture):
                answer = codex_constraint_revision(message, plan, deepcopy(context))
        except (InvalidPlan, RuntimeError, OSError) as exc:
            error, answer = str(exc), None
            infrastructure = raw is None
        correct = False
        if answer and index == 0:
            areas = answer["constraints"]
            correct = (
                answer["status"] == "accept"
                and len(areas) == 1
                and areas[0]["radius"] == 0.5
                and any(
                    o["id"] == areas[0]["observation_id"] and o["color"] == "green" for o in context["observations"]
                )
                and answer["steps"] == context["pending"]
                and not answer["released_constraints"]
            )
        elif answer and index == 1:
            correct = answer["status"] == "clarify" and bool(answer["question"])
        elif answer:
            correct = (
                answer["status"] == "accept"
                and not answer["constraints"]
                and {x["id"] for x in answer["released_constraints"]} == {x["id"] for x in context["constraints"]}
                and any(s["action"] == "reobserve" and s["target"] == "green" for s in answer["steps"])
                and context["pending"][0] in answer["steps"]
                and answer["steps"][-1] == context["pending"][-1]
            )
        report["results"].append(
            {
                "turn": index + 1,
                "message": message,
                "context": deepcopy(context),
                "raw": raw,
                "correct": None if infrastructure else correct,
                "error": error,
                "infrastructure_error": infrastructure,
            }
        )
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
        print(
            f"turn {index + 1}: {'ERROR' if infrastructure else 'PASS' if correct else 'FAIL'} {error or ''}",
            flush=True,
        )
        if not correct:
            print(f"Stopped; saved: {output}", flush=True)
            return 2 if infrastructure else 1
        context["dialogue"].append({"role": "user", "text": message})
        if answer["status"] == "clarify":
            context["dialogue"].append({"role": "assistant", "text": answer["question"]})
        else:
            context["pending"], context["constraints"] = deepcopy(answer["steps"]), deepcopy(answer["constraints"])
    print(f"Saved: {output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
