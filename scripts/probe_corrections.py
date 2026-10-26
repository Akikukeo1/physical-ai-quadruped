"""Real CLI language probes against a frozen observed snapshot; no physics is run here."""

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
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.search import validate_search  # noqa: E402

CASES = [
    ("replace", "青はやめて緑にして。最後は戻って", [["search", "green"], ["return_home", None]], ["search-1"]),
    (
        "insert",
        "青をやめるわけではないけど、その前に緑も探して",
        [["search", "green"], ["search", "blue"], ["return_home", None]],
        [],
    ),
    (
        "reobserve",
        "緑をもう一度確認してから青の探索を再開して",
        [["reobserve", "green"], ["search", "blue"], ["return_home", None]],
        [],
    ),
    (
        "reference",
        "さっき見えた対象をもう一度確認してから、青の探索を続けて",
        [["reobserve", "green"], ["search", "blue"], ["return_home", None]],
        [],
    ),
    ("cancel", "青の探索は取り消して、今すぐ出発地点へ帰って", [["return_home", None]], ["search-1"]),
    ("ambiguous", "やっぱりあっちを先にして", None, []),
    ("negation", "緑に変更しないで。青の探索を続けて帰って", [["search", "blue"], ["return_home", None]], []),
    ("unsupported", "帰還は不要にして、時間制限もなくして", None, []),
]


def matches(raw, accepted, steps, cancelled, status="reject"):
    if steps is None:
        return isinstance(raw, dict) and raw.get("status") == status and (status == "reject" or accepted)
    return (
        accepted
        and [[s["action"], s["target"]] for s in raw["steps"]] == steps
        and sorted(c["id"] for c in raw["cancelled"]) == sorted(cancelled)
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path, help="Completed real run containing corrections[0].context")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs/language-probes")
    args = parser.parse_args()
    data = json.loads(args.snapshot.read_text(encoding="utf-8"))
    context = data["corrections"][0]["context"]
    plan = validate_search(data["corrections"][0]["original_plan"])
    assert {entry["color"] for entry in context["observations"]} == {"green"}
    assert context["latest_scan"]["visible_colors"] == []
    assert matches({"status": "reject"}, False, None, [])
    assert not matches([], False, None, [])
    assert not matches({"status": "accept", "steps": [], "cancelled": []}, True, [["search", "blue"]], [])
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output = args.out_dir / (datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    report = {
        "scope": "Development language probes; frozen snapshot, not robot execution or held-out evaluation.",
        "snapshot": str(args.snapshot),
        "snapshot_sha256": hashlib.sha256(args.snapshot.read_bytes()).hexdigest(),
        "cases": CASES,
        "results": [],
    }
    original = planner._codex_json
    for condition in ("history", "no_history"):
        observed = deepcopy(context)
        if condition == "no_history":
            observed["observations"] = []
        for name, message, expected, cancelled in CASES:
            raw, error, accepted = None, None, False

            def capture(*arguments):
                nonlocal raw
                raw = original(*arguments)
                return raw

            try:
                with patch.object(planner, "_codex_json", side_effect=capture):
                    planner.codex_correction(message, plan, deepcopy(observed))
                accepted = True
            except (InvalidPlan, RuntimeError, OSError) as exc:
                error = str(exc)
            target = None if condition == "no_history" and name in ("reobserve", "reference") else expected
            expected_status = "accept" if target is not None else "clarify" if name != "unsupported" else "reject"
            infrastructure_error = raw is None and error is not None
            correct = None if infrastructure_error else matches(raw, accepted, target, cancelled, expected_status)
            report["results"].append(
                {
                    "case": name,
                    "condition": condition,
                    "message": message,
                    "expected_steps": target,
                    "expected_status": expected_status,
                    "expected_cancelled": cancelled,
                    "raw": raw,
                    "accepted": accepted,
                    "error": error,
                    "correct": correct,
                    "infrastructure_error": infrastructure_error,
                }
            )
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
            print(f"{condition}/{name}: {'PASS' if correct else 'FAIL'} accepted={accepted}; {error or ''}", flush=True)
            if infrastructure_error:
                print(f"Stopped on infrastructure error; saved: {output}", flush=True)
                raise SystemExit(1)
    print(f"Saved: {output}", flush=True)


if __name__ == "__main__":
    main()
