#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export UV_PROJECT_ENVIRONMENT="$PWD/.venv-wsl"
export UV_PYTHON_INSTALL_DIR="$PWD/.python-wsl"
export UV_CACHE_DIR="$PWD/.uv-cache"
uv_bin="$(command -v uv || true)"
if [[ -z "$uv_bin" ]]; then
  uv_bin="$PWD/.tools/uv-x86_64-unknown-linux-gnu/uv"
fi
if [[ ! -x "$uv_bin" ]]; then
  echo "Install uv in WSL: https://docs.astral.sh/uv/getting-started/installation/" >&2
  exit 1
fi
exec "$uv_bin" run --locked --python 3.12.12 --extra sim python src/main.py --executor mujoco "$@"
