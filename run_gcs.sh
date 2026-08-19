#!/usr/bin/env bash
# Launch the pysitl ground station (browser dashboard) with a Python that
# actually has the required dependencies installed.
#
# Usage:
#   bash run_gcs.sh [port]        # default port 8765
#
# Why this exists: this Mac has several Python interpreters and not all of
# them have numpy/pandas/matplotlib (the system /usr/bin/python3 and the
# 3.12 framework build don't). Running `python -m pysitl.run --gcs` fails
# with "command not found" (no `python` alias on macOS) and picking a bare
# `python3` can fail with "ModuleNotFoundError: No module named 'numpy'".
set -euo pipefail
cd "$(dirname "$0")"

PORT="${1:-8765}"

# Fail loudly instead of crashing later if something already holds the port.
if lsof -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "Port $PORT is already in use (an old pysitl server still running?)."
  echo "Stop it first with:  lsof -ti tcp:$PORT | xargs kill"
  exit 1
fi

# First interpreter that can import the project's dependencies wins.
CANDIDATES=(
  "$(command -v python3 || true)"
  /opt/homebrew/bin/python3
  /usr/local/bin/python3
  /Library/Frameworks/Python.framework/Versions/3.14/bin/python3
  /Library/Frameworks/Python.framework/Versions/3.13/bin/python3
)
for PY in "${CANDIDATES[@]}"; do
  [ -n "$PY" ] && [ -x "$PY" ] || continue
  if "$PY" -c "import numpy, pandas, matplotlib" >/dev/null 2>&1; then
    echo "Using $("$PY" --version 2>&1) -> $PY"
    command -v open >/dev/null 2>&1 && open "http://127.0.0.1:$PORT" || true
    exec "$PY" -m pysitl.run --gcs --port "$PORT"
  fi
done

echo "No python3 with numpy/pandas/matplotlib was found."
echo "Install them into Homebrew Python with:"
echo "  /opt/homebrew/bin/python3 -m pip install numpy pandas matplotlib"
exit 1
