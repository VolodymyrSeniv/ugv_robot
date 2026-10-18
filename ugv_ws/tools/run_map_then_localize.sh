#!/bin/bash
# Saved-map experiment on the standard near goals (same world/goals/guard as series_03):
#   1) mapping mode: run the 16-trial series, then stop RTAB-Map gracefully so it writes its database
#   2) localization mode on that database: run the same 16-trial series again
# usage: tools/run_map_then_localize.sh NAME [WORLD]
set -u
command -v caffeinate >/dev/null && caffeinate -i -s -w $$ &   # macOS only: keep the machine awake
export PATH="/Applications/OrbStack.app/Contents/MacOS/xbin:$PATH"
NAME=$1; WORLD=${2:-test_world}
W="$(cd "$(dirname "$0")/.." && pwd)"
DB_HOST="$W/data/maps/$NAME.db"; DB=/ws/data/maps/$NAME.db
mkdir -p "$W/data/maps"

start_stack() {   # $1 mode
  docker rm -f ugv-sim >/dev/null 2>&1
  docker run -d --name ugv-sim \
    -v "$W/src:/ws/src" -v "$W/data:/ws/data" -v "$W/tools:/ws/tools" \
    -v ugv_build:/ws/build -v ugv_install:/ws/install -v ugv_log:/ws/log ugv-ros2:humble \
    bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install >/dev/null 2>&1; \
             source install/setup.bash && ros2 launch ugv_bringup autonomy.launch.py headless:=true world:=$WORLD mode:=$1 database_path:=$DB" >/dev/null
  sleep 90
}
run() { docker exec ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; $1"; }

rm -f "$DB_HOST"
start_stack mapping
run "python3 /ws/tools/run_trials.py --out /ws/data/${NAME}_mapping --repeats 2" > "$W/data/${NAME}_mapping.log" 2>&1
# SIGINT to ros2 launch (PID 1 is a bash that would not forward it) so RTAB-Map saves the database
docker exec ugv-sim pkill -INT -f "ros2 launch" 2>/dev/null
timeout 180 docker wait ugv-sim >/dev/null 2>&1
docker logs ugv-sim > "$W/data/${NAME}_mapping.stack.log" 2>&1
docker rm -f ugv-sim >/dev/null 2>&1
ls -la "$DB_HOST" >> "$W/data/${NAME}_mapping.log" 2>&1
echo "done mapping" >> "$W/data/${NAME}_mapping.log"

start_stack localization
run "python3 /ws/tools/run_trials.py --out /ws/data/${NAME}_localization --repeats 2" > "$W/data/${NAME}_localization.log" 2>&1
docker logs ugv-sim > "$W/data/${NAME}_localization.stack.log" 2>&1
docker rm -f ugv-sim >/dev/null 2>&1
echo "done localization" >> "$W/data/${NAME}_localization.log"
