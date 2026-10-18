#!/bin/bash
# Ubuntu + NVIDIA: simulation WITH the Gazebo window and the operator UI.
# Needs: docker, nvidia-container-toolkit, an X11 session (see docs/UBUNTU.md).
# usage: tools/run_sim_gui.sh [mapping|localization] [database_path_in_container]
set -u
MODE=${1:-mapping}; DB=${2:-/ws/data/rtabmap.db}
W="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$W/data"
xhost +local:docker >/dev/null 2>&1 || echo "warning: xhost not available, the Gazebo window may not open"
docker rm -f ugv-sim >/dev/null 2>&1
docker run -d --name ugv-sim --gpus all --network host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$W/src:/ws/src" -v "$W/data:/ws/data" -v "$W/tools:/ws/tools" \
  -v ugv_build:/ws/build -v ugv_install:/ws/install -v ugv_log:/ws/log ugv-ros2:humble \
  bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install >/dev/null 2>&1; \
           source install/setup.bash && ros2 launch ugv_bringup autonomy.launch.py headless:=false ui:=true mode:=$MODE database_path:=$DB"
echo "starting: the Gazebo window opens in ~1-2 min; operator UI at http://localhost:8080 ; logs: docker logs -f ugv-sim"
