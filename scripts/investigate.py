"""Run the warehouse investigation with real Go2 physics and Codex CLI."""

import argparse
import hashlib
import json
import sys
from datetime import UTC, datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pai_motion.investigation import execute_investigation  # noqa: E402
from pai_motion.investigation_world import warehouse_scene  # noqa: E402
from pai_motion.planner import CODEX_MODEL, CODEX_REASONING_EFFORT  # noqa: E402
from pai_motion.sim_adapter import MuJoCoGo2Sink  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=["A", "B"], default="A")
    parser.add_argument("--seed", type=int, default=1102, help="B generator seed; never sent to Codex")
    parser.add_argument("--development-fault", choices=["power_loss", "cooler_fault", "door_open"])
    parser.add_argument("--seconds", type=int, default=360)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs" / "investigation")
    parser.add_argument(
        "--prompt",
        default="冷蔵倉庫の温度が上昇しています。原因を調査し、確認できた証拠とともに報告してから帰還してください。",
    )
    args = parser.parse_args()
    if args.scene == "B" and args.development_fault:
        parser.error("B is held out; fault overrides are development-only")
    if not 60 <= args.seconds <= 600:
        parser.error("--seconds must be 60–600")
    world = warehouse_scene(args.seed, development=args.scene == "A", fault=args.development_fault)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output = args.out_dir / f"investigation-{args.scene}-{stamp}.json"
    log = {
        "scene": args.scene,
        "seed": args.seed if args.scene == "B" else None,
        "task": args.prompt,
        "requested_codex_settings": {"model": CODEX_MODEL, "reasoning_effort": CODEX_REASONING_EFFORT},
        "source_sha256": {
            p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                *sorted((ROOT / "src/pai_motion").glob("*.py")),
                ROOT / "schema/investigation-decision.schema.json",
                ROOT / "docs/warehouse-manual.json",
                ROOT / "pyproject.toml",
                ROOT / "uv.lock",
                ROOT / "assets/go2-assets.json",
                Path(__file__).resolve(),
            ]
        },
        "status": "failed",
        "limits": "simulated local inspection, oracle odometry; no camera recognition",
    }
    sink = None
    try:
        sink = MuJoCoGo2Sink(headless=args.headless, search_seed=0, custom_map=world.geometry)
        execute_investigation(args.prompt, sink, world.inventory(), world.inspect, seconds=args.seconds, log=log)
        log["evaluation"] = world.evaluate(log["report"] or {}, log["observations"])
        success = (
            log["returned"]
            and log["within_deadline"]
            and not log["errors"]
            and log["evaluation"]["correct_cause"]
            and log["evaluation"]["evidence_sufficient"]
        )
        log["status"] = "success" if success else "incomplete"
    except Exception as exc:
        log["error"] = str(exc)
        print(f"FAILED: {exc}", file=sys.stderr)
    finally:
        if sink is not None:
            log["events"], log["trace"] = sink.events, sink.trace
            sink.close()
        # Evaluator-only data is intentionally stored separately from every planner context.
        log["evaluation_world"] = {"cause": world.cause, "geometry": world.geometry}
        output.write_text(json.dumps(log, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Saved: {output}; status={log['status']}", flush=True)
    return 0 if log["status"] == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
