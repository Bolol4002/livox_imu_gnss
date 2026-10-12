#!/bin/bash
# Re-run the stadtgarten_seq2 bag through FAST-LIO while recording the
# absolute-time trajectory (/Odometry) needed for GNSS alignment.
#
# Why a re-run instead of reusing the previous run:
#   laserMapping's mat_pre.txt only has RELATIVE time + Euler angles, so its
#   rows cannot be matched against the GNSS fix stamps. /Odometry carries the
#   absolute header.stamp (same epoch as the bag's /gnss/fix) and the
#   quaternion. The run is deterministic given the same bag/code/QoS, and we
#   re-save the map at the end so map and trajectory come from the SAME run.
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/sayooj/livox_ws/install/setup.bash
source /home/sayooj/fastlio_ws/install/setup.bash

DIR=/home/sayooj/Documents/livox_imu_gnss
BAG=$DIR/stadtgarten_seq2.db3
LOGDIR=$DIR/logs
OUT=$DIR/stadtgarten_seq2_odom.csv

rm -f "$OUT"

echo "[$(date +%T)] starting FAST-LIO"
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false \
    > "$LOGDIR/traj_fastlio.log" 2>&1 &
LAUNCH_PID=$!

for i in $(seq 1 60); do
  if ros2 node list 2>/dev/null | grep -q '^/laser_mapping$'; then
    echo "[$(date +%T)] /laser_mapping up after ${i}s"; break
  fi
  sleep 1
done

echo "[$(date +%T)] starting /Odometry recorder -> $OUT"
python3 "$DIR/tools/record_odom.py" "$OUT" > "$LOGDIR/traj_recorder.log" 2>&1 &
REC_PID=$!
sleep 3
kill -0 "$REC_PID" 2>/dev/null || { echo "RECORDER FAILED TO START"; cat "$LOGDIR/traj_recorder.log"; exit 1; }

echo "[$(date +%T)] starting playback"
ros2 bag play "$BAG" \
    --topics /livox/points /livox/imu /gnss/fix \
    --rate 1.0 --read-ahead-queue-size 400 \
    > "$LOGDIR/traj_bagplay.log" 2>&1
echo "[$(date +%T)] playback finished"

sleep 3                      # let the last scans settle through the EKF
kill -INT "$REC_PID" 2>/dev/null
wait "$REC_PID" 2>/dev/null
sleep 2
echo "[$(date +%T)] recorder log:"; cat "$LOGDIR/traj_recorder.log"

echo "[$(date +%T)] saving map (keeps map and trajectory from the same run)"
ros2 service call /map_save std_srvs/srv/Trigger > "$LOGDIR/traj_mapsave.log" 2>&1
cat "$LOGDIR/traj_mapsave.log"

echo "[$(date +%T)] stopping FAST-LIO"
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 3
pkill -f 'fastlio[_]mapping' 2>/dev/null
sleep 2

echo "[$(date +%T)] trajectory rows: $(($(wc -l < "$OUT") - 1))"
echo "TRAJ_RUN_DONE"
