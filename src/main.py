"""CLI entrypoint. Use 'uv run python src/main.py ...' from project root."""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

from pai_motion.executor import DryRunSink, execute_plan
from pai_motion.planner import codex_plan, sample_plan

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="Quadruped natural-language motion planner")
    parser.add_argument("--planner", choices=["sample", "codex"], default="sample")
    parser.add_argument("--executor", choices=["dry-run", "mujoco"], default="dry-run")
    parser.add_argument("--prompt", default="右の方向を動作で案内して")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs")
    args = parser.parse_args()

    try:
        plan = codex_plan(args.prompt) if args.planner == "codex" else sample_plan()
        if args.executor == "mujoco":
            from pai_motion.sim_adapter import MuJoCoGo2Sink

            sink = MuJoCoGo2Sink()
        else:
            sink = DryRunSink()
        execute_plan(plan, sink)
        args.out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = args.out_dir / f"plan-{stamp}.json"
        data = {
            "planner": args.planner,
            "executor": args.executor,
            "intent": plan.intent,
            "actions": [{"type": a.type, "duration": a.duration} for a in plan.actions],
            "events": getattr(sink, "events", []),
            "disclaimer": "dry-run only, not a physical simulation" if args.executor == "dry-run" else None,
        }
        output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Saved: {output}")
        return 0
    except (RuntimeError, ValueError, NotImplementedError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
