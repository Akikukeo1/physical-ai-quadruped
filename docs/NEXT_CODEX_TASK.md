# Codexに次に実装させるタスク — Phase 1b

> 2026-10-24: このタスクの実装結果と検証記録は [PHASE1_RESULTS.md](PHASE1_RESULTS.md) を参照してください。以下は実装時の要件です。

You are implementing the next milestone of this repository on the user's Windows 11 / WSL2 Ubuntu 24.04 machine. Read AGENTS.md, README.md, and tests first.

**Goal:** Replace the intentionally missing MuJoCoGo2Sink backend so that `--planner sample --executor mujoco` makes the Unitree Go2 model physically walk/turn/stop with a reproducible demonstration video. Then confirm that `--planner codex --executor mujoco` uses the exact same motion interface. Keep dry-run intact.

Constraints:

1. Use `uv` and a project-local Python runtime (prefer 3.12); pin runtime dependencies and commit uv.lock.
2. Prefer a credible Go2 locomotion controller with an existing, documented velocity-command interface. Do not train an RL policy from scratch.
3. MuJoCo Menagerie supplies robot models, *not* pretrained locomotion; inspect and verify the chosen model and controller compatibility and license before integrating.
4. Research implementation options before coding. Example sources to examine (not automatically trusted):
   - https://github.com/google-deepmind/mujoco_menagerie
   - https://github.com/darshmenon/quadruped-robotics-stack
   - https://github.com/syswonder/robot-unitree-go2-mujoco
5. Do not copy entire upstream repositories or commit large binaries without justification and license documentation. Pin commits and write precise setup instructions.
6. Prefer using a controller that accepts `(vx, vy, yaw_rate)` and advances simulation at its expected control frequency.
7. Never call Codex at simulation frame rate; only for bounded motion plans.
8. The robot must remain upright and physically move on a flat ground plane. Verify forward/turn/stop in simulation, not merely a viewer showing the static Go2 mesh.
9. Save a video or a tested capture recipe. Capture and inspect evidence of actual robot movement.
10. Include reproducibility commands, tests, control limits, observed results, and any known limitations in README.
11. Do not claim success until a true end-to-end run in this environment was observed. If the first backend fails, document why and try another; do not silently replace a real sim with a mock.
12. Keep a single command or script to run the complete demo.
13. Ask before pushing to GitHub or modifying a remote repository. The final course deliverable needs a standalone **public** repository, but do not publish secrets or copied assets.

**Acceptance:**

- `uv run python src/main.py --planner sample --executor dry-run` works.
- `uv run python -m unittest discover -s tests -v` passes.
- `uv run python src/main.py --planner sample --executor mujoco` works on WSL2 and a captured video clearly shows forward movement and turning.
- Real Codex plan is validated and run by the exact same simulator adapter (after logging in to Codex CLI).
- No shell commands or executable code from the LLM are passed to the simulator.

After this milestone, do not start Phase 2 (novel communication task) without reporting Phase 1 results and known limitations.
