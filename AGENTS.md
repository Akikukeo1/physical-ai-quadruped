# Agent instructions

Project: Physical AI final assignment, quadruped natural-language motion planning.

## Current milestone
- Phase 1a (included): validated motion protocol, Codex CLI planner, dry-run executor, tests.
- Phase 1b: MuJoCo Go2 locomotion with a pinned MoE ONNX policy. Sample and real Codex plans verified on Ubuntu 24.04 / WSL2; see README and docs/PHASE1_RESULTS.md for evidence and limits.
- Phase 2: task-specific motion vocabulary and constraint adaptation.
- Phase 3: comparative experiments and a 1–3 minute video.

## Development rules
- Do not present dry-run output or moving a suspended 3D model as a completed robot simulation.
- Use only documented, version-pinned simulator/control integrations; verify upstream model/controller licenses.
- Keep LLM outputs at action-plan level. Never execute LLM-generated Python or shell commands.
- Validate actions and bound speed, duration, and total plan time prior to execution.
- The simulator/controller must own physics and joint-level control; high-level planner must never drive joints at 50 Hz.
- Do not create or push a GitHub repository without an explicit request. Do not add AI co-author attribution without an explicit request.
- Keep README honest about tested vs untested functionality.
- Preserve meaningful logs and metrics for future experiments.
