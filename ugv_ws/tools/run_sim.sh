#!/bin/bash
# Start the simulation with the operator UI. Open http://localhost:8080 afterwards.
# usage: tools/run_sim.sh [mode] [database_path]
#   mode: mapping (default) | localization (reuse a saved map: pass its path inside the container)
set -u
export PATH="/Applications/OrbStack.app/Contents/MacOS/xbin:$PATH"
MODE=${1:-mapping}; DB=${2:-/ws/data/rtabmap.db}
W="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$W/data"
docker rm -f ugv-sim >/dev/null 2>&1
docker run -d --name ugv-sim -p 8080:8080 -p 9090:9090 \
  -v "$W/src:/ws/src" -v "$W/data:/ws/data" -v "$W/tools:/ws/tools" \
  -v ugv_build:/ws/build -v ugv_install:/ws/install -v ugv_log:/ws/log ugv-ros2:humble \
  bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install >/dev/null 2>&1; \
           source install/setup.bash && ros2 launch ugv_bringup autonomy.launch.py headless:=true ui:=true mode:=$MODE database_path:=$DB"
echo "stack starting; UI at http://localhost:8080 in ~90 s. Stop: docker stop ugv-sim (use 'docker kill -s INT ugv-sim' first to save the map)"
