#!/usr/bin/env bash
set -e

# pick a python
PYTHON=${PYTHON:-python}
command -v "$PYTHON" >/dev/null 2>&1 || PYTHON=py
command -v "$PYTHON" >/dev/null 2>&1 || PYTHON=".venv/Scripts/python.exe"

# create venv if missing
if [ ! -d ".venv" ]; then
  "$PYTHON" -m venv .venv
fi

# activate venv (Windows or Unix)
if [ -f ".venv/Scripts/activate" ]; then
  # Git Bash on Windows
  # shellcheck disable=SC1091
  source .venv/Scripts/activate
else
  # Linux/macOS
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

# use python -m pip on Windows
"$PYTHON" -m pip install -U pip
"$PYTHON" -m pip install -r requirements.txt

# Phase 1: simulate data
"$PYTHON" bin/simulate.py --out out

# Phase 2: calibrate from synthetic detections
"$PYTHON" bin/calibrate.py \
  --cams out/cams.json \
  --detections out/detections.csv \
  --tags out/tags.json \
  --out out

echo "Done. See out/results.json"
