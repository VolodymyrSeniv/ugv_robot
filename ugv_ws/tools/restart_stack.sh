#!/bin/bash
# Headless stack in the background for experiments (no Gazebo window, no UI needed).
# usage: tools/restart_stack.sh   -> waits until map->base_link exists
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$W/data"
docker rm -f ugv-sim >/dev/null 2>&1
rm -f "$W/data/rtabmap.db"
docker run -d --name ugv-sim --runtime=nvidia --network host -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v "$W/src:/ws/src" -v "$W/data:/ws/data" -v "$W/tools:/ws/tools" \
  -v ugv_build:/ws/build -v ugv_install:/ws/install -v ugv_log:/ws/log ugv-ros2:humble \
  bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install >/ws/data/colcon.log 2>&1; \
           source install/setup.bash && ros2 launch ugv_bringup autonomy.launch.py headless:=true ui:=true $*" >/dev/null
for i in $(seq 1 60); do
  sleep 5
  docker exec ugv-sim bash -c 'source /opt/ros/humble/setup.bash; timeout 4 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q Translation' && { echo "stack ready"; exit 0; }
done
echo "stack NOT ready"; exit 1
