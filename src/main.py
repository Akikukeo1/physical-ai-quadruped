"""CLI entrypoint. Use 'uv run python src/main.py ...' from project root."""

import argparse
import json
import math
import sys
from dataclasses import asdict
from datetime import UTC, datetime
from functools import partial
from pathlib import Path

from pai_motion.agent_planner import codex_agent_revision, codex_frontier_choice
from pai_motion.correction import sample_correction
from pai_motion.executor import DryRunSink, execute_plan
from pai_motion.field_areas import validate_areas
from pai_motion.field_revision import codex_field_revision
from pai_motion.navigation import (
    MAX_TASK_SECONDS,
    OBSTACLE,
    WAYPOINTS,
    execute_navigation,
    rules_navigation,
    sample_navigation,
)
from pai_motion.planner import (
    CODEX_MODEL,
    CODEX_REASONING_EFFORT,
    codex_correction,
    codex_navigation,
    codex_plan,
    codex_search,
    sample_plan,
)
from pai_motion.search import execute_search, sample_search

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="Quadruped natural-language motion planner")
    parser.add_argument("--planner", choices=["sample", "codex"], default="sample")
    parser.add_argument("--executor", choices=["dry-run", "mujoco"], default="dry-run")
    parser.add_argument("--task", choices=["motion", "navigation", "replanning", "search"], default="motion")
    parser.add_argument("--seed", type=int, default=0, help="Simulator-only search layout seed; never sent to planner")
    parser.add_argument("--missing-target", action="store_true", help="Search scene with no blue target")
    correction_input = parser.add_mutually_exclusive_group()
    correction_input.add_argument(
        "--correction-file", type=Path, help="Read one natural-language correction from this UTF-8 file during search"
    )
    correction_input.add_argument(
        "--correction", help="Inject one natural-language correction at --correct-at for replay"
    )
    correction_input.add_argument("--correction-script", type=Path, help="JSON list of timed correction messages")
    parser.add_argument("--field-areas", type=Path, help="JSON file of named rectangles drawn on the field")
    parser.add_argument("--map", type=Path, help="User-created search map JSON")
    parser.add_argument("--agent", action="store_true", help="Experimental LLM task and frontier decisions")
    parser.add_argument("--interactive-dialogue", action="store_true", help="Wait for a search task and answer chat")
    parser.add_argument("--correct-at", type=float, help="Seconds after search start to inject --correction")
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
    search = args.task == "search"
    navigation = args.task in ("navigation", "replanning")
    dynamic = args.task == "replanning"
    if args.replanner and not dynamic:
        parser.error("--replanner requires --task replanning")
    args.replanner = args.replanner or ("codex" if dynamic else "stop")
    args.prompt = args.prompt or (
        "青い対象を探して、見つけたら出発地点へ戻って。180秒以内で、見つからなくても戻って"
        if search
        else "青い地点へ行ってから緑へ。最初は北側を使い、塞がったら安全な別経路へ切り替えて"
        if dynamic
        else "障害物を避けて青い地点へ行き、その後で緑の地点へ移動して"
        if navigation
        else "右の方向を動作で案内して"
    )
    if (navigation or search) and args.executor != "mujoco":
        parser.error("navigation/search requires --executor mujoco; it uses measured physics state")
    if args.missing_target and not search:
        parser.error("--missing-target requires --task search")
    if args.agent and (not search or args.planner != "codex"):
        parser.error("--agent requires --task search --planner codex")
    if args.interactive_dialogue and (not args.agent or args.correction_file is None):
        parser.error("--interactive-dialogue requires --agent and --correction-file")
    if args.map and (not search or not args.agent or not args.interactive_dialogue):
        parser.error("--map requires interactive search agent mode")
    if args.field_areas and (not search or args.planner != "codex"):
        parser.error("--field-areas requires --task search --planner codex")
    if args.field_areas and not (
        args.agent or args.correction_file or args.correction_script or args.correction is not None
    ):
        parser.error("--field-areas requires a correction input")
    if (
        args.correction_file or args.correction_script or args.correction is not None or args.correct_at is not None
    ) and not search:
        parser.error("correction input requires --task search")
    if (args.correction is None) != (args.correct_at is None):
        parser.error("--correction and --correct-at must be used together")
    if args.correction is not None and not 1 <= len(args.correction.strip()) <= 500:
        parser.error("--correction must contain 1–500 characters")
    if args.correct_at is not None and (not math.isfinite(args.correct_at) or not 0 <= args.correct_at < 238):
        parser.error("--correct-at must be finite and in 0–238 seconds")
    if args.executor != "mujoco" and (args.headless or args.video):
        parser.error("--headless and --video require --executor mujoco")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    output = args.out_dir / f"plan-{stamp}.json"
    sink = None
    execution_completed = False
    data = {
        "planner": args.planner,
        "requested_codex_settings": {"model": CODEX_MODEL, "reasoning_effort": CODEX_REASONING_EFFORT}
        if args.planner == "codex" or args.replanner == "codex"
        else None,
        "executor": args.executor,
        "task": args.task,
        "replanner": args.replanner,
        "replans": [],
        "prompt": args.prompt,
        "status": "failed",
        "events": [],
        "observations": [],
        "decisions": [],
        "corrections": [],
        "disclaimer": "dry-run only, not a physical simulation" if args.executor == "dry-run" else None,
    }
    try:
        field_areas = None
        custom_map = None
        if args.map:
            from pai_motion.custom_map import validate_map

            if args.map.stat().st_size > 32768:
                raise ValueError("Map file exceeds 32768 bytes")
            custom_map = validate_map(json.loads(args.map.read_text(encoding="utf-8")))
            field_areas = custom_map["areas"] or None
            data["custom_map"] = custom_map
            print("POINTS: " + ", ".join(f"{i + 1}={p['id']}" for i, p in enumerate(custom_map["points"])), flush=True)
        elif args.field_areas:
            if args.field_areas.stat().st_size > 16384:
                raise ValueError("Area file exceeds 16384 bytes")
            field_areas = validate_areas(json.loads(args.field_areas.read_text(encoding="utf-8")))
        if field_areas:
            data["field_areas"] = field_areas
            print("FIELD AREAS: " + ", ".join(f"{i + 1}={a['id']}" for i, a in enumerate(field_areas)), flush=True)
        if search:
            search_prompt = args.prompt
            if args.interactive_dialogue:
                from pai_motion.initial_dialogue import interpret_initial, wait_for_search

                data["initial_dialogue"] = []
                search_prompt = wait_for_search(
                    args.prompt,
                    args.correction_file,
                    history=data["initial_dialogue"],
                    interpret=partial(interpret_initial, points=custom_map["points"] if custom_map else None),
                )
            plan = codex_search(search_prompt) if args.planner == "codex" else sample_search()
            if args.correct_at is not None and args.correct_at >= plan.seconds - 2:
                raise ValueError("--correct-at must precede the original task deadline")
            data["search_plan"] = asdict(plan)
            if custom_map:
                data["search_plan_target_role"] = (
                    "The initial target is a protocol placeholder; only the agent's validated tasks drive the mission."
                )
            if args.agent:
                data["agent"] = True
            data["scene"] = (
                {"source": "custom_map"} if custom_map else {"seed": args.seed, "missing_blue": args.missing_target}
            )
            data["correction_input"] = {
                "file": str(args.correction_file) if args.correction_file else None,
                "message": args.correction,
                "at": args.correct_at,
                "script": str(args.correction_script) if args.correction_script else None,
            }
            scheduled = []
            if args.correction_script:
                if args.correction_script.stat().st_size > 16384:
                    raise ValueError("Correction script exceeds 16384 bytes")
                scheduled = json.loads(args.correction_script.read_text(encoding="utf-8"))
                if not isinstance(scheduled, list) or not 1 <= len(scheduled) <= 12:
                    raise ValueError("Correction script requires 1–12 entries")
                for event in scheduled:
                    if (
                        not isinstance(event, dict)
                        or set(event) != {"at", "message"}
                        or type(event["at"]) not in (int, float)
                        or not math.isfinite(event["at"])
                        or not 0 <= event["at"] < plan.seconds - 2
                        or not isinstance(event["message"], str)
                        or not 1 <= len(event["message"].strip()) <= 500
                    ):
                        raise ValueError("Invalid timed correction entry")
                if [e["at"] for e in scheduled] != sorted(e["at"] for e in scheduled):
                    raise ValueError("Correction script times must be ordered")
        elif navigation:
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
                headless=args.headless,
                video=args.video,
                navigation=navigation,
                dynamic_obstacle=dynamic,
                search_seed=args.seed if search else None,
                missing_target=args.missing_target,
                field_areas=field_areas,
                custom_map=custom_map,
            )
            from pai_motion.assets import MANIFEST

            data["assets"] = json.loads(MANIFEST.read_text(encoding="utf-8"))
            data["video"] = str(args.video) if args.video else None
        else:
            sink = DryRunSink()
        if search:
            search_started = sink.observe()["time"]
            initial_agent_request = args.agent

            def get_correction(state):
                nonlocal initial_agent_request
                if initial_agent_request:
                    initial_agent_request = False
                    return {"id": "agent-initial", "message": search_prompt}
                if scheduled and state["time"] - search_started >= scheduled[0]["at"] - 1e-9:
                    event = scheduled.pop(0)
                    return {"id": f"scheduled-{len(scheduled)}", "message": event["message"]}
                if args.correction_file and args.correction_file.exists():
                    if args.correction_file.stat().st_size > 4096:
                        raise ValueError("Correction file exceeds 4096 bytes")
                    message = args.correction_file.read_text(encoding="utf-8").strip()
                    if message.startswith("{"):
                        return json.loads(message)
                    return message or None
                if args.correction is not None and state["time"] - search_started >= args.correct_at - 1e-9:
                    return args.correction
                return None

            correction_enabled = (
                args.agent
                or args.correction_file is not None
                or args.correction is not None
                or args.correction_script is not None
            )
            if args.correction_file:
                print(
                    f"CORRECTION INPUT: {args.correction_file}; write one UTF-8 instruction during search", flush=True
                )
            data["result"] = execute_search(
                plan,
                sink,
                observations=data["observations"],
                decisions=data["decisions"],
                get_correction=get_correction if correction_enabled else None,
                correction_planner=(
                    partial(codex_agent_revision, dialogue=args.interactive_dialogue)
                    if args.agent
                    else codex_field_revision
                    if args.field_areas
                    else codex_correction
                    if args.planner == "codex"
                    else sample_correction
                )
                if correction_enabled
                else None,
                corrections=data["corrections"],
                field_areas=field_areas,
                agent_revision=args.agent,
                frontier_planner=codex_frontier_choice if args.agent else None,
                points=custom_map["points"] if custom_map else None,
                initial_home_only=custom_map is not None,
            )
        elif navigation:

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
