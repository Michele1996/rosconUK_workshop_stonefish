#!/usr/bin/env bash
# Run with: bash scripts/debug_blueboat_thruster_abort.sh [scenario filename]
# Defaults to the known-failing thruster scene. On SIGABRT, prints a backtrace.
set -euo pipefail
source /opt/ros/humble/setup.bash
if [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
  source "${HOME}/ros2_ws/install/setup.bash"
fi
SCENE="${1:-blueboat_no_sensors.scn}"
DATA="$(ros2 pkg prefix roscon_stonefish_workshop)/share/roscon_stonefish_workshop/data"
SIM="$(ros2 pkg prefix stonefish_ros2)/lib/stonefish_ros2/stonefish_simulator"
if [ ! -f "$DATA/scenarios/$SCENE" ]; then echo "Missing scenario $DATA/scenarios/$SCENE" >&2; exit 2; fi
if ! command -v gdb >/dev/null 2>&1; then echo 'Install debugger: sudo apt install gdb' >&2; exit 2; fi
mkdir -p /tmp/roscon_stonefish_debug
LOG="/tmp/roscon_stonefish_debug/${SCENE%.scn}_gdb.txt"
printf 'GDB output: %s\n' "$LOG"
# On a crash, capture the actual failing C++ call stack; if it starts,
# close the simulator window to end the run.
gdb -q -batch \
  -ex 'set pagination off' \
  -ex 'set print thread-events off' \
  -ex run \
  -ex 'thread apply all bt 12' \
  --args "$SIM" "$DATA" "$DATA/scenarios/$SCENE" 100.0 1280 720 high 2>&1 | tee "$LOG"
