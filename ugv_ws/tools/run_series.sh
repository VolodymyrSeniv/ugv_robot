#!/bin/bash
# Run one trial series from scratch (fresh container, fresh map) and stop the stack afterwards.
# usage: tools/run_series.sh NAME WORLD GUARD [REPEATS] [MAX_TRIALS]
#   WORLD: test_world | test_world_aliased      GUARD: true | false
set -u
# A sleeping Mac makes the container clock jump, which makes Nav2 declare its servers dead.
command -v caffeinate >/dev/null && caffeinate -i -s -w $$ &   # macOS only: keep the machine awake
export PATH="/Applications/OrbStack.app/Contents/MacOS/xbin:$PATH"
NAME=$1; WORLD=$2; GUARD=$3; REPEATS=${4:-2}; MAXT=${5:-0}
W="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$W/data"
docker rm -f ugv-sim >/dev/null 2>&1
rm -f "$W/data/rtabmap.db"
docker run -d --name ugv-sim \
  -v "$W/src:/ws/src" -v "$W/data:/ws/data" -v "$W/tools:/ws/tools" \
  -v ugv_build:/ws/build -v ugv_install:/ws/install -v ugv_log:/ws/log ugv-ros2:humble \
  bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install >/dev/null 2>&1; \
           source install/setup.bash && ros2 launch ugv_bringup autonomy.launch.py headless:=true world:=$WORLD jump_guard:=$GUARD" >/dev/null
sleep 90
docker exec ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \
  python3 /ws/tools/run_trials.py --out /ws/data/$NAME --repeats $REPEATS --max-trials $MAXT" > "$W/data/$NAME.log" 2>&1
docker logs ugv-sim > "$W/data/$NAME.stack.log" 2>&1
docker stop ugv-sim >/dev/null
echo "done $NAME" >> "$W/data/$NAME.log"
