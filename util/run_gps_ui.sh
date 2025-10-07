#!/usr/bin/env bash
set -euo pipefail

LAUNCH_CMD=(ros2 launch ublox_gps ublox_gps_node-launch.py)

ZENOH_LOG=/tmp/zenoh.log
UBLOX_LOG=/tmp/ublox_launch.log
# Start a command in its own process group and record PID & PGID
start_pg() {
  # usage: start_pg <name> <cmd...>
  local name="$1"; shift
  echo "[start] $name" | tee -a "$ZENOH_LOG" "$UBLOX_LOG" >/dev/null
  setsid "$@" >>"${UBLOX_LOG}" 2>&1 &
  echo $!
}
# Kill a whole process group by PID
kill_pg() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  local pgid
  pgid=$(ps -o pgid= "$pid" 2>/dev/null | tr -d ' ' || true)
  if [[ -n "$pgid" ]]; then
    kill -TERM -"$pgid" 2>/dev/null || true
    sleep 0.5
    kill -KILL -"$pgid" 2>/dev/null || true
  else
    kill "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  echo "[cleanup] stopping TUI, u-blox launch, and zenoh…"
  kill_pg "${LAUNCH_PID:-}"
  kill_pg "${ZENOH_PID:-}"
  # make extra sure zenoh is gone
  pkill -9 -f 'rmw_zenoh_cpp|rmw_zenohd' 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM


: > "$ZENOH_LOG"
: > "$UBLOX_LOG"

echo "[start] zenoh daemon -> $ZENOH_LOG"
setsid ros2 run rmw_zenoh_cpp rmw_zenohd >>"$ZENOH_LOG" 2>&1 &
ZENOH_PID=$!

echo "[start] ${LAUNCH_CMD[*]} -> $UBLOX_LOG"
setsid "${LAUNCH_CMD[@]}" >>"$UBLOX_LOG" 2>&1 &
LAUNCH_PID=$!

./NeuROAM/util/cgps_ros.py

cleanup
