"""Step-level motion executor; real robot/controller implementation is injected."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from pai_motion.motion import MotionPlan, validate_plan


@dataclass(frozen=True)
class Velocity:
    vx: float
    vy: float
    yaw: float


# Low-speed commands verified with the pinned MoE Go2 controller on flat ground.
COMMAND_VELOCITIES = {
    "forward": Velocity(0.18, 0.0, 0.0),
    "backward": Velocity(-0.12, 0.0, 0.0),
    "turn_left": Velocity(0.0, 0.0, 0.35),
    "turn_right": Velocity(0.0, 0.0, -0.35),
    "wait": Velocity(0.0, 0.0, 0.0),
}


class VelocitySink(Protocol):
    def run_velocity(self, cmd: Velocity, seconds: float) -> None: ...
    def stop(self) -> None: ...


@dataclass
class DryRunSink:
    """Print commands only. This is *not* a physical simulation."""

    events: list[dict[str, float]] = field(default_factory=list)

    def run_velocity(self, cmd: Velocity, seconds: float) -> None:
        event = {"vx": cmd.vx, "vy": cmd.vy, "yaw": cmd.yaw, "duration": seconds}
        self.events.append(event)
        print(f"DRY RUN  vx={cmd.vx:+.2f}, vy={cmd.vy:+.2f}, yaw={cmd.yaw:+.2f}, t={seconds:.1f}s")

    def stop(self) -> None:
        print("DRY RUN  stop")


def execute_plan(plan: MotionPlan, sink: VelocitySink) -> None:
    """Run validated commands and always request a stop."""
    plan = validate_plan(
        {"intent": plan.intent, "actions": [{"type": a.type, "duration": a.duration} for a in plan.actions]}
    )
    try:
        for action in plan.actions:
            sink.run_velocity(COMMAND_VELOCITIES[action.type], action.duration)
    finally:
        sink.stop()
