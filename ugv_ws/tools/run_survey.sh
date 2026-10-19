#!/bin/bash
# Run survey.py (first N waypoints) with a trace, wait for it, copy logs to data/NAME/.
# usage: tools/run_survey.sh NAME [N]
set -u
NAME=$1; N=${2:-10}
W="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$W/data/$NAME"
docker exec -d ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/tools/trace_run.py /ws/data/$NAME/trace.csv > /dev/null 2>&1"
docker exec ugv-sim bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/tools/survey.py $N" > "$W/data/$NAME/survey.log" 2>&1
docker exec ugv-sim pkill -f trace_run.py
docker logs ugv-sim > "$W/data/$NAME/stack.log" 2>&1
cat "$W/data/$NAME/survey.log"
