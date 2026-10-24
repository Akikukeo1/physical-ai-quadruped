"""Run the predeclared held-out matrix using one frozen plan per target color."""

import argparse
import hashlib
import importlib.metadata
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pai_motion.planner import codex_search  # noqa: E402
from pai_motion.search import execute_search, sample_search, validate_search  # noqa: E402
from pai_motion.sim_adapter import MuJoCoGo2Sink  # noqa: E402


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", choices=["sample", "codex"], default="sample")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs/search-evaluation")
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    protocol = json.loads((ROOT / "docs/search-evaluation.json").read_text(encoding="utf-8"))
    files = sorted([*ROOT.glob("src/**/*.py"), *ROOT.glob("schema/*.json"), ROOT / "assets/go2-assets.json"])
    digests = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    report = {
        "protocol": protocol,
        "planner": args.planner,
        "source_sha256": digests,
        "versions": {name: importlib.metadata.version(name) for name in ("mujoco", "numpy", "onnxruntime")},
        "plans": {},
        "results": [],
    }
    plans = {}
    for color, prompt in (
        ("blue", "青い対象を探して、発見したら出発地点へ戻って。制限は180秒"),
        ("green", "青ではなく緑を探してください。見つけたらスタート地点へ帰還。全体で180秒以内"),
    ):
        started = time.monotonic()
        plan = (
            codex_search(prompt)
            if args.planner == "codex"
            else validate_search({**asdict(sample_search()), "target": color})
        )
        if plan.target != color or plan.seconds != 180:
            raise RuntimeError(f"Task interpretation mismatch: {plan}")
        plans[color] = plan
        report["plans"][color] = {
            "prompt": prompt,
            "plan": asdict(plan),
            "planning_wall_seconds": time.monotonic() - started,
        }
    # Save the source hashes and plans before running ANY held-out scene.
    save(args.out_dir / "frozen-inputs.json", report)
    cases = [(seed, color, False) for seed in protocol["held_out_seeds"] for color in ("blue", "green")]
    cases += [(seed, "blue", True) for seed in protocol["missing_blue_seeds"]]
    for seed, color, missing in cases:
        path = args.out_dir / f"seed-{seed}-{color}-{'missing' if missing else 'present'}.json"
        log = {"seed": seed, "target": color, "missing_blue": missing, "observations": [], "decisions": []}
        sink = None
        started = time.monotonic()
        try:
            sink = MuJoCoGo2Sink(headless=True, search_seed=seed, missing_target=missing)
            result = execute_search(plans[color], sink, observations=log["observations"], decisions=log["decisions"])
            log["result"] = result
            expected = "not_found" if missing else "found"
            success = result["outcome"] == expected and result["elapsed"] <= plans[color].seconds
        except (RuntimeError, ValueError, OSError) as exc:
            log["error"] = str(exc)
            success = False
        finally:
            if sink:
                sink.close()
                log["events"], log["trace"] = sink.events, sink.trace
            log["success"] = success
            log["wall_seconds"] = time.monotonic() - started
            save(path, log)
        result = log.get("result", {})
        trace = log.get("trace", [])
        item = {
            "seed": seed,
            "target": color,
            "missing_blue": missing,
            "success": success,
            "outcome": result.get("outcome"),
            "error": log.get("error"),
            "elapsed": result.get("elapsed"),
            "home_error": result.get("home_error"),
            "distance": result.get("distance"),
            "found": result.get("found"),
            "contact": any(state["obstacle_contact"] for state in trace),
            "min_clearance": min((state["obstacle_clearance"] for state in trace), default=None),
            "log": path.name,
        }
        report["results"].append(item)
        save(args.out_dir / "evaluation.json", report)
        print(json.dumps(item, ensure_ascii=False), flush=True)
    return 0 if all(item["success"] for item in report["results"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
