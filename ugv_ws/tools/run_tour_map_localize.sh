#!/bin/bash
# Saved-map test on Linux/NVIDIA: 1) mapping mode, full survey tour, stop RTAB-Map cleanly so it writes the db
# 2) localization mode on that db, far goals (run_trials.py --goals far).
# SKIP_MAPPING=1 reuses an existing data/maps/NAME.db and only runs the localization phase.
# BAG=1 records /vo/odom, /odometry/filtered, /cmd_odom, /cmd_vel, /imu, /ground_truth/odom, /tf to data/NAME/bag.
# TRACE=1 also logs a 10 Hz trace (data/NAME/trace.csv); MAX_TRIALS=N stops after N trials.
# usage: tools/run_tour_map_localize.sh NAME [REPEATS]
set -u
NAME=$1; REPEATS=${2:-1}
W="$(cd "$(dirname "$0")/.." && pwd)"
DB=/ws/data/maps/$NAME.db
mkdir -p "$W/data/maps" "$W/data/$NAME"
export DOCKER_CONTEXT=default

start_stack() {   # $1 mode
  docker rm -f ugv-sim >/dev/null 2>&1
  docker run -d --name ugv-sim --runtime=nvidia --network host -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$W/src:/ws/src" -v "$W/data:/ws/data" -v "$W/tools:/ws/tools" \
    -v ugv_build:/ws/build -v ugv_install:/ws/install -v ugv_log:/ws/log ugv-ros2:humble \
    bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install >/ws/data/colcon.log 2>&1; \
             source install/setup.bash && ros2 launch ugv_bringup autonomy.launch.py headless:=true mode:=$1 database_path:=$DB" >/dev/null
  for i in $(seq 1 60); do
    sleep 5
    docker exec ugv-sim bash -c 'source /opt/ros/humble/setup.bash; timeout 4 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q Translation' && return 0
  done
  echo "stack NOT ready"; return 1
}
run() { docker exec ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; $1"; }

if [ "${SKIP_MAPPING:-0}" != 1 ]; then
rm -f "$W/data/maps/$NAME.db"
start_stack mapping || exit 1
run "python3 /ws/tools/survey.py 10" > "$W/data/$NAME/survey.log" 2>&1
docker exec ugv-sim pkill -INT -f "ros2 launch" 2>/dev/null
timeout 180 docker wait ugv-sim >/dev/null 2>&1
docker logs ugv-sim > "$W/data/$NAME/mapping.stack.log" 2>&1
docker rm -f ugv-sim >/dev/null 2>&1
ls -la "$W/data/maps/$NAME.db" > "$W/data/$NAME/db.txt" 2>&1
fi

start_stack localization || exit 1
[ "${TRACE:-0}" = 1 ] && docker exec -d ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/tools/trace_run.py /ws/data/$NAME/trace.csv 0.1 > /dev/null 2>&1"
[ "${BAG:-0}" = 1 ] && docker exec -d ugv-sim bash -c "source /opt/ros/humble/setup.bash; ros2 bag record -o /ws/data/$NAME/bag /vo/odom /odometry/filtered /cmd_odom /cmd_vel /imu /ground_truth/odom /tf > /dev/null 2>&1"
run "python3 /ws/tools/run_trials.py --out /ws/data/$NAME/loc --repeats $REPEATS --goals far --max-trials ${MAX_TRIALS:-0}" > "$W/data/$NAME/loc.log" 2>&1
docker exec ugv-sim pkill -INT -f "ros2 bag record" 2>/dev/null; sleep 4
docker logs ugv-sim > "$W/data/$NAME/loc.stack.log" 2>&1
docker rm -f ugv-sim >/dev/null 2>&1
echo "done $NAME"
