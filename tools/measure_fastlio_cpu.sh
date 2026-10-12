#!/bin/bash
# Measure FAST-LIO's CPU/RSS footprint over a full replay of the bag.
# Run: tools/measure_fastlio_cpu.sh
# Takes ~16 min (bag length at rate 1.0). Writes logs/cpu_fastlio.log
# (FAST-LIO's own STATS lines) and tools/fastlio_cpu{.csv,_summary.txt}.
#
# Deliberately does NOT call /map_save and does NOT run the /Odometry
# recorder: this measures the *odometry* footprint only, which is what has to
# run onboard during flight. The map export and R+T are post-flight steps.
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/sayooj/livox_ws/install/setup.bash
source /home/sayooj/fastlio_ws/install/setup.bash

DIR=/home/sayooj/Documents/livox_imu_gnss
LOGDIR=$DIR/logs

echo "[$(date +%T)] launching FAST-LIO"
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false \
    > "$LOGDIR/cpu_fastlio.log" 2>&1 &
LAUNCH_PID=$!

UP=0
for i in $(seq 1 60); do
  if ros2 node list 2>/dev/null | grep -q '^/laser_mapping$'; then
    echo "[$(date +%T)] /laser_mapping up after ${i}s"; UP=1; break
  fi
  sleep 1
done
if [ "$UP" != "1" ]; then
  echo "NODE NEVER CAME UP"; cat "$LOGDIR/cpu_fastlio.log"; exit 1
fi
sleep 5   # let QoS discovery settle before sampling

echo "[$(date +%T)] starting sampler + playback"
python3 "$DIR/tools/measure_fastlio_cpu.py"
RC=$?

echo "[$(date +%T)] shutting down FAST-LIO"
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 3
pkill -x fastlio_mapping 2>/dev/null      # -x: exact comm, no self-match
sleep 2

echo "[$(date +%T)] FAST-LIO STATS lines from this run:"
grep -o 'STATS.*' "$LOGDIR/cpu_fastlio.log" | tail -20
echo "CPU_RUN_DONE rc=$RC"
