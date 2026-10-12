#!/usr/bin/env python3
"""Independent check on the fitted R: what does the sensor actually measure?

Reads the first ~2 s of /livox/imu and reports the mean specific force.
For a stationary IMU the accelerometer reads -g in the sensor frame, so:

    mean_acc ~ (0, 0, +9.81)  -> sensor +z points UP
    mean_acc ~ (0, 0, -9.81)  -> sensor +z points DOWN  (mounted inverted)
    tilted   -> x/y components give the mounting tilt

This tells us whether the roll ~177 deg / pitch ~20 deg that the R+T fit
returned is a real mounting orientation or a fitting artefact.
"""
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
WINDOW = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id="sqlite3"),
            ConverterOptions(input_serialization_format="cdr",
                             output_serialization_format="cdr"))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msg_type = get_message(types["/livox/imu"])

acc, gyr, stamps = [], [], []
t_ref = None
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic != "/livox/imu":
        continue
    m = deserialize_message(data, msg_type)
    s = m.header.stamp
    ts = s.sec + s.nanosec * 1e-9
    if t_ref is None:
        t_ref = ts
    if ts - t_ref > WINDOW:
        break
    acc.append([m.linear_acceleration.x, m.linear_acceleration.y,
                m.linear_acceleration.z])
    gyr.append([m.angular_velocity.x, m.angular_velocity.y,
                m.angular_velocity.z])
    stamps.append(ts)

acc = np.array(acc)
gyr = np.array(gyr)
G = 9.80665

mean_acc = acc.mean(0)
norm = np.linalg.norm(mean_acc)
print(f"samples in {WINDOW:g} s window : {len(acc)}")
print(f"mean specific force          : "
      f"[{mean_acc[0]:+.4f} {mean_acc[1]:+.4f} {mean_acc[2]:+.4f}] m/s^2")
print(f"|mean|                        : {norm:.4f}  (gravity = {G})")
print(f"acc std                      : {np.std(acc, axis=0)}")
print(f"mean gyro                    : "
      f"[{gyr[0][0]:+.5f} {gyr[0][1]:+.5f} {gyr[0][2]:+.5f}] rad/s "
      f"(|.| {np.linalg.norm(gyr.mean(0)):.5f})")

u = mean_acc / norm
print()
print(f"gravity direction in sensor frame (unit): [{u[0]:+.4f} {u[1]:+.4f} {u[2]:+.4f}]")
# up-direction of the sensor = direction of measured specific force
pitch = math.degrees(math.asin(max(-1, min(1, u[0]))))
roll_like = math.degrees(math.atan2(u[1], u[2]))
print(f"  -> sensor 'up' is tilted: "
      f"x-component {math.degrees(math.asin(max(-1, min(1, u[0])))):+.2f} deg, "
      f"y-component {math.degrees(math.asin(max(-1, min(1, u[1])))):+.2f} deg")
if u[2] > 0.5:
    print("  -> sensor +z points UP   (nominal mounting)")
elif u[2] < -0.5:
    print("  -> sensor +z points DOWN (INVERTED mounting)")
else:
    print("  -> sensor +z is roughly horizontal (sideways mounting)")
