"""Attach a *real* MuJoCo Go2 locomotion controller here in Phase 1b."""

from pai_motion.executor import Velocity


class MuJoCoGo2Sink:
    """Explicitly incomplete until a compatible control backend is integrated."""

    def __init__(self) -> None:
        raise NotImplementedError(
            "Real Go2 locomotion is not integrated yet. "
            "See docs/NEXT_CODEX_TASK.md. Do not treat dry-run as a robot video."
        )

    def run_velocity(self, cmd: Velocity, seconds: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError
