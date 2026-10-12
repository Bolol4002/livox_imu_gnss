#!/usr/bin/env python3
"""Two pre-flight checks before trajectory/GNSS matching:

1. Do /livox/points, /livox/imu and /gnss/fix share ONE time epoch?
   FAST-LIO stamps are LiDAR/IMU time; GNSS fixes are receiver time (UTC or
   GPS).  If they differ by 18 s (leap seconds) or by years, matching by
   timestamp silently produces garbage.

2. What is the actual distribution of NavSatFix.status and the fix quality?
"""
import sys
from collections import Counter

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]

reader = SequentialReader()
reader.open(
    StorageOptions(uri=bag, storage_id="sqlite3"),
    ConverterOptions(input_serialization_format="cdr",
                     output_serialization_format="cdr"),
)
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

WANT = ("/livox/points", "/livox/imu", "/gnss/fix")
first_stamp = {}
first_recv = {}
status = Counter()
# All three topics fire within the first ~0.2 s of bag time, so stop right
# after all three are seen instead of streaming all 16 GB of point clouds.
while reader.has_next() and len(first_stamp) < 3:
    topic, data, t = reader.read_next()
    if topic not in WANT or topic in first_stamp:
        continue
    m = deserialize_message(data, get_message(types[topic]))
    s = m.header.stamp
    first_stamp[topic] = s.sec + s.nanosec * 1e-9
    first_recv[topic] = t * 1e-9
    if topic == "/gnss/fix":
        status[int(m.status.status)] += 1

print("=== first header.stamp per topic (bag time epoch) ===")
for k, v in sorted(first_stamp.items(), key=lambda x: x[1]):
    print(f"  {k:18s} {v:.6f}")
print("=== first received (record) time ===")
for k, v in sorted(first_recv.items(), key=lambda x: x[1]):
    print(f"  {k:18s} {v:.6f}")

base = min(first_stamp.values())
print("=== offsets relative to earliest stamp ===")
for k, v in sorted(first_stamp.items(), key=lambda x: x[1]):
    print(f"  {k:18s} {(v - base) * 1000:+10.2f} ms")

print("=== NavSatFix.status distribution (all 8768 fixes, from gnss_fixes.csv) ===")
import csv as _csv
NAMES = {-1: "NO_FIX", 0: "FIX", 1: "SBAS_FIX", 2: "GBAS_FIX"}
status = Counter()
with open(sys.argv[2]) as fh:
    for row in _csv.DictReader(fh):
        status[int(row["status"])] += 1
for k, v in sorted(status.items()):
    print(f"  {k} {NAMES.get(k, '?'):9s} {v:6d}  ({100.0 * v / sum(status.values()):.1f}%)")
print(f"  total {sum(status.values())}")
