#!/usr/bin/env bash
# Pulls the latest bahr-pilot code on the Pi and restarts the service.
# Run this ON THE PI (e.g. over SSH), from anywhere:
#   ssh bahr-pi 'bash ~/bahr-pilot/deploy/update.sh'
# Not yet tried against the real Pi (no hardware this session) — the repo
# layout and venv path assume the clone lives at ~/bahr-pilot, matching
# the (also not-yet-created) bahr-pilot.service unit in this same folder.
set -euo pipefail

REPO_DIR="${BAHR_PILOT_DIR:-$HOME/bahr-pilot}"
SERVICE_NAME="bahr-pilot"

cd "$REPO_DIR"
echo "Pulling latest bahr-pilot..."
git pull --ff-only

echo "Updating venv dependencies..."
"$REPO_DIR/.venv/bin/pip" install -r requirements.txt --quiet

echo "Restarting $SERVICE_NAME.service..."
sudo systemctl restart "$SERVICE_NAME.service"
sudo systemctl --no-pager status "$SERVICE_NAME.service"
