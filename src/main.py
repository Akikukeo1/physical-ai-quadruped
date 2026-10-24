"""CLI entrypoint. Use 'uv run python src/main.py ...' from project root."""

import argparse
import json
import sys
from datetime import UTC, datetime
from pathlib import Path

from pai_motion.executor import DryRunSink, execute_plan
from pai_motion.navigation import (
    MAX_TASK_SECONDS,
    OBSTACLE,
    WAYPOINTS,
    execute_navigation,
    rules_navigation,
    sample_navigation,
)
from pai_motion.planner import codex_navigation, codex_plan, sample_plan

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="Quadruped natural-language motion planner")
    parser.add_argument("--planner", choices=["sample", "codex"], default="sample")
    parser.add_argument("--executor", choices=["dry-run", "mujoco"], default="dry-run")
    parser.add_argument("--task", choices=["motion", "navigation", "replanning"], default="motion")
    parser.add_argument(
        "--replanner",
        choices=["stop", "rules", "codex"],
        help="Blocked-route strategy for --task replanning (default: codex)",
    )
    parser.add_argument("--prompt")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs")
    parser.add_argument("--headless", action="store_true", help="Run MuJoCo without the interactive viewer")
    parser.add_argument("--video", type=Path, help="Save a 25 fps MuJoCo recording (.mp4)")
    args = parser.parse_args()
    navigation = args.task != "motion"
    dynamic = args.task == "replanning"
    if args.replanner and not dynamic:
        parser.error("--replanner requires --task replanning")
    args.replanner = args.replanner or ("codex" if dynamic else "stop")
    args.prompt = args.prompt or (
        "青い地点へ行ってから緑へ。最初は北側を使い、塞がったら安全な別経路へ切り替えて"
        if dynamic
        else "障害物を避けて青い地点へ行き、その後で緑の地点へ移動して"
        if navigation
        else "右の方向を動作で案内して"
    )
    if navigation and args.executor != "mujoco":
        parser.error("navigation requires --executor mujoco; it uses measured physics state")
    if args.executor != "mujoco" and (args.headless or args.video):
        parser.error("--headless and --video require --executor mujoco")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    output = args.out_dir / f"plan-{stamp}.json"
    sink = None
    execution_completed = False
    data = {
        "planner": args.planner,
        "executor": args.executor,
        "task": args.task,
        "replanner": args.replanner,
        "replans": [],
        "prompt": args.prompt,
        "status": "failed",
        "events": [],
        "disclaimer": "dry-run only, not a physical simulation" if args.executor == "dry-run" else None,
    }
    try:
        if navigation:
            plan = codex_navigation(args.prompt) if args.planner == "codex" else sample_navigation()
            data.update(intent=plan.intent, waypoints=list(plan.waypoints))
            data["world"] = {
                "state_source": "simulator ground truth",
                "waypoints": WAYPOINTS,
                "obstacle": OBSTACLE,
                "max_task_seconds": MAX_TASK_SECONDS,
            }
        else:
            plan = codex_plan(args.prompt) if args.planner == "codex" else sample_plan()
            data.update(intent=plan.intent, actions=[{"type": a.type, "duration": a.duration} for a in plan.actions])
        output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if args.executor == "mujoco":
            from pai_motion.sim_adapter import MuJoCoGo2Sink

            sink = MuJoCoGo2Sink(
                headless=args.headless, video=args.video, navigation=navigation, dynamic_obstacle=dynamic
            )
            from pai_motion.assets import MANIFEST

            data["assets"] = json.loads(MANIFEST.read_text(encoding="utf-8"))
            data["video"] = str(args.video) if args.video else None
        else:
            sink = DryRunSink()
        if navigation:

            def replan(state, remaining):
                return codex_navigation(
                    args.prompt,
                    timeout=60,
                    start=tuple(state["position"][:2]),
                    obstacles=state["obstacles"],
                    required=remaining,
                )

            replanner = rules_navigation if args.replanner == "rules" else replan if args.replanner == "codex" else None
            data["result"] = execute_navigation(plan, sink, replanner=replanner, replan_events=data["replans"])
        else:
            execute_plan(plan, sink)
        execution_completed = True
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        data["error"] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            if sink is not None and hasattr(sink, "close"):
                sink.close()
            if execution_completed:
                data["status"] = "completed"
        finally:
            data["events"] = getattr(sink, "events", [])
            data["trace"] = getattr(sink, "trace", [])
            data["world_events"] = getattr(sink, "world_events", [])
            output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"Saved: {output}")


if __name__ == "__main__":
    raise SystemExit(main())
