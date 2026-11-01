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
    parser.add_argument("--development-fault", choices=["power_loss", "cooler_fault", "door_open", "airflow_fault"])
    parser.add_argument("--probe", action="store_true", help="Use kinematic yaw probe for air temperature and airflow")
    parser.add_argument("--seconds", type=int, default=360)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--demo", action="store_true", help="Show distinguishable 3D warehouse equipment")
    parser.add_argument(
        "--record-dir", type=Path, help="Save synchronized overview/follow videos and timestamp indices"
    )
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs" / "investigation")
    parser.add_argument(
        "--prompt",
        default="冷蔵倉庫の温度が上昇しています。原因を調査し、確認できた証拠とともに報告してから帰還してください。",
    )
    args = parser.parse_args()
    if args.scene == "B" and args.development_fault:
        parser.error("B is held out; fault overrides are development-only")
    if args.development_fault == "airflow_fault" and not args.probe:
        parser.error("airflow_fault requires --probe")
    if args.probe and args.scene == "B":
        parser.error("Probe development uses A; the earlier B evaluation belongs to the original implementation")
    if not 60 <= args.seconds <= 600:
        parser.error("--seconds must be 60–600")
    world = warehouse_scene(args.seed, development=args.scene == "A", fault=args.development_fault, probe=args.probe)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output = args.out_dir / f"investigation-{args.scene}-{stamp}.json"
    recording = args.record_dir / stamp if args.record_dir else None
    if recording is not None:
        recording.mkdir(parents=True, exist_ok=False)
        output = recording / "execution.json"
    log = {
        "scene": args.scene,
        "probe_enabled": args.probe,
        "warehouse_display": args.demo or recording is not None,
        "seed": args.seed if args.scene == "B" else None,
        "task": args.prompt,
        "requested_codex_settings": {"model": CODEX_MODEL, "reasoning_effort": CODEX_REASONING_EFFORT},
        "source_sha256": {
            p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                *sorted((ROOT / "src/pai_motion").glob("*.py")),
                ROOT / "schema/investigation-decision.schema.json",
                ROOT / "docs/warehouse-manual.json",
                ROOT / "schema/investigation-probe-decision.schema.json",
                ROOT / "docs/warehouse-probe-manual.json",
                ROOT / "pyproject.toml",
                ROOT / "uv.lock",
                ROOT / "assets/go2-assets.json",
                Path(__file__).resolve(),
            ]
        },
        "status": "failed",
        "limits": "kinematic probe, analytic air field, no payload dynamics; oracle odometry, no camera recognition"
        if args.probe
        else "simulated local inspection, oracle odometry; no camera recognition",
    }
    sink = None
    try:
        sink = MuJoCoGo2Sink(
            headless=args.headless,
            search_seed=0,
            custom_map=world.geometry,
            probe=args.probe,
            warehouse_display=args.demo,
            record_dir=recording,
            door_open=world.cause == "door_open",
        )
        execute_investigation(
            args.prompt,
            sink,
            world.inventory(),
            world.inspect,
            seconds=args.seconds,
            log=log,
            measure=world.measure if args.probe else None,
        )
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
            try:
                sink.close()
                if sink.recorder is not None:
                    log["recording"] = sink.recorder.finalize(log)
            except Exception as exc:
                log.update(status="failed", recording_error=str(exc))
        # Evaluator-only data is intentionally stored separately from every planner context.
        log["evaluation_world"] = {"cause": world.cause, "geometry": world.geometry}
        output.write_text(json.dumps(log, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Saved: {output}; status={log['status']}", flush=True)
    return 0 if log["status"] == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
