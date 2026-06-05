#!/usr/bin/env bash
set -e

echo "========================================="
echo "  OpenVINS + OAK-D  —  Logged Session"
echo "========================================="
echo ""

# Ask for a session name
read -rp "Session name (e.g. lab_run_1): " SESSION_NAME

# Sanitise: replace spaces/slashes with underscores
SESSION_NAME=$(echo "$SESSION_NAME" | tr ' /' '_')

if [[ -z "$SESSION_NAME" ]]; then
  echo "No name given — aborting."
  exit 1
fi

export RUN_TIMESTAMP="${SESSION_NAME}_$(date +%Y%m%d_%H%M%S)"

echo ""
echo "Logs will be written to:"
echo "  bags         → logs/bags/$RUN_TIMESTAMP/"
echo "  trajectory   → logs/trajectories/$RUN_TIMESTAMP/"
echo "  timing       → logs/timing/$RUN_TIMESTAMP/"
echo ""
read -rp "Start? [Y/n] " CONFIRM
CONFIRM=${CONFIRM:-Y}
if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
  echo "Aborted."
  exit 0
fi

echo ""
echo "Starting services..."
docker compose up --build
