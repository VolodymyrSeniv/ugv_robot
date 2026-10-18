#!/bin/bash
# Trial series with injected map->odom jumps (validates the localization-jump guard).
# usage: tools/run_injection_series.sh NAME GUARD [REPEATS] [MAX_TRIALS]     GUARD: true | false
# Odd trials get a ~100 deg jump of the map frame: idx%4==1 during GOTO, idx%4==3 during RETURN.
set -u
command -v caffeinate >/dev/null && caffeinate -i -s -w $$ &   # macOS only: keep the machine awake
export PATH="/Applications/OrbStack.app/Contents/MacOS/xbin:$PATH"
NAME=$1; GUARD=$2; REPEATS=${3:-1}; MAXT=${4:-0}
W="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$W/data"
docker rm -f ugv-sim >/dev/null 2>&1
rm -f "$W/data/rtabmap.db"
docker run -d --name ugv-sim \
  -v "$W/src:/ws/src" -v "$W/data:/ws/data" -v "$W/tools:/ws/tools" \
  -v ugv_build:/ws/build -v ugv_install:/ws/install -v ugv_log:/ws/log ugv-ros2:humble \
  bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install >/dev/null 2>&1; \
           source install/setup.bash && ros2 launch ugv_bringup autonomy.launch.py headless:=true world:=test_world jump_guard:=$GUARD publish_map_tf:=false" >/dev/null
docker exec -d ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/tools/fault_injector.py > /ws/data/$NAME.injector.log 2>&1"
sleep 90
docker exec ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \
  python3 /ws/tools/run_trials.py --out /ws/data/$NAME --repeats $REPEATS --max-trials $MAXT --inject" > "$W/data/$NAME.log" 2>&1
docker logs ugv-sim > "$W/data/$NAME.stack.log" 2>&1
docker stop ugv-sim >/dev/null
echo "done $NAME" >> "$W/data/$NAME.log"
