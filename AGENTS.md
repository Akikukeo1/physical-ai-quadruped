# Agent instructions

Project: Physical AI final assignment, quadruped natural-language motion planning.

## Current milestone
- Phase 1a (included): validated motion protocol, Codex CLI planner, dry-run executor, tests.
- Phase 1b: MuJoCo Go2 locomotion with a pinned MoE ONNX policy. Sample and real Codex plans verified on Ubuntu 24.04 / WSL2; see README and docs/PHASE1_RESULTS.md for evidence and limits.
- Phase 2: known-map waypoint navigation and event-driven replanning after a scripted moving blocker are implemented and verified on WSL2. See docs/NAVIGATION_RESULTS.md and docs/REPLANNING_RESULTS.md. Camera perception and real-time continuously changing environments are not implemented.
- Unknown-layout search: range/first-hit surface-color sensing, partial occupancy mapping, frontier exploration and return home. Oracle odometry remains; this is not camera perception or SLAM. See docs/SEARCH_RESULTS.md and docs/search-evaluation.json. Preserve frozen evaluation records and never tune on held-out layouts without relabeling them as development.
- Correction MVP: one in-flight blue/green target replacement via scheduled text or UTF-8 file. Preserve observation history, original home obligation and deadline. Physics continues under zero-velocity control during asynchronous Codex interpretation. See docs/CORRECTION_MVP.md; its trials are development cases, not a new held-out benchmark.
- Task revisions: corrections now receive observed context and generate ordered search/reobserve/return_home steps with explicit cancellations. Retain task IDs and obligations, ground reobservations in supplied observation IDs, and require fresh sensor evidence after reobserve activation. See docs/TASK_REVISION_RESULTS.md. The old correction JSON is historical; initial search JSON and CLI flags are unchanged.
- Dialogue: up to 12 in-flight messages, clarification questions, short answers and repeated corrections. Preserve active observation references, retired IDs, original deadline/home. Zero-velocity physics while awaiting answers; unanswered clarification returns under the original reserve. See docs/DIALOGUE_RESULTS.md. Frozen earlier language probes used rejection instead of clarification.
- Luna development trials: CLI is pinned to gpt-6-luna / xhigh and requested settings are logged. For supported single-static-target-per-color scenes, repeated sightings default to the latest supplied observation unless the user specifies one. See docs/LUNA6_RESULTS.md for failed and successful dialogue runs; these remain development cases. Keep pre-policy language results separate.
- Correction-memory evaluation: docs/correction-evaluation.json and code were frozen at ca0dbd5 before seeds 701–704. Eight real runs completed, but only seed 703 triggered a correction: history succeeded, current-only safely returned without completing the revision. See docs/CORRECTION_EVALUATION.md; do not call non-triggered cases successes or infer a general success rate from one paired opportunity. Preserve frozen records.
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
