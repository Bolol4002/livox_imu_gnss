#!/usr/bin/env python3
"""Full-bag validation: /livox/points frames + /livox/imu timing."""
import sys, struct, math, statistics
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
DT = {5: ('i', 4), 6: ('I', 4), 7: ('f', 4), 8: ('d', 8)}

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'),
            ConverterOptions(input_serialization_format='cdr',
                             output_serialization_format='cdr'))
topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
PC = '/livox/points'
IM = '/livox/imu'

pc_stamps, imu_stamps, empty = [], [], []
pc_stats = []
imu_acc = []
while reader.has_next():
    topic, data, bagt = reader.read_next()
    if topic == PC:
        m = deserialize_message(data, get_message(topics[PC]))
        n = m.width
        tf = [f for f in m.fields if f.name == 'offset_time']
        hdr = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        pc_stamps.append(hdr)
        if tf and n > 0:
            f = tf[0]
            fmt = DT[f.datatype][0]
            idx = list(range(0, n, 97))       # subsample for speed
            ts = [struct.unpack_from('<' + fmt, m.data, i * m.point_step + f.offset)[0]
                  for i in idx]
            zeros = 0
            for i in idx:
                x, y, z = struct.unpack_from('<fff', m.data, i * m.point_step)
                if x == 0.0 and y == 0.0 and z == 0.0:
                    zeros += 1
            pc_stats.append(dict(n=n, tmin=min(ts), tmax=max(ts),
                                 nz=zeros, nsamp=len(idx)))
        else:
            empty.append(n)
    elif topic == IM:
        m = deserialize_message(data, get_message(topics[IM]))
        imu_stamps.append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
        if len(imu_acc) < 2000:
            a = m.linear_acceleration
            imu_acc.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))

print('--- LIDAR ---')
print('frames:', len(pc_stamps))
print('frames without offset_time field / empty:', len(empty), sorted(set(empty))[:5])
print('widths:', sorted(set(s['n'] for s in pc_stats))[:10])
print('offset_time tmax ms: min=%.3f max=%.3f mean=%.3f' % (
    min(s['tmax'] for s in pc_stats) / 1e6, max(s['tmax'] for s in pc_stats) / 1e6,
    statistics.mean(s['tmax'] for s in pc_stats) / 1e6))
print('offset_time tmin ms: min=%d max=%d' % (
    min(s['tmin'] for s in pc_stats), max(s['tmin'] for s in pc_stats)))
print('zero-xyz fraction: mean=%.4f max=%.4f' % (
    statistics.mean(s['nz'] / s['nsamp'] for s in pc_stats),
    max(s['nz'] / s['nsamp'] for s in pc_stats)))
gaps = [pc_stamps[i + 1] - pc_stamps[i] for i in range(len(pc_stamps) - 1)]
print('frame dt ns: min=%d max=%d median=%.0f  non-monotonic=%d' % (
    min(gaps), max(gaps), statistics.median(gaps),
    sum(1 for g in gaps if g <= 0)))
print('big gaps >150ms:', sum(1 for g in gaps if g > 1.5e8))
print('span s: %.3f' % ((pc_stamps[-1] - pc_stamps[0]) / 1e9))

print('--- IMU ---')
print('msgs:', len(imu_stamps))
d = [imu_stamps[i + 1] - imu_stamps[i] for i in range(len(imu_stamps) - 1)]
print('dt ns: min=%d max=%d median=%.0f' % (min(d), max(d), statistics.median(d)))
print('rate Hz (median): %.2f' % (1e9 / statistics.median(d)))
print('non-monotonic:', sum(1 for x in d if x <= 0), ' gaps >10ms:', sum(1 for x in d if x > 1e7))
print('span s: %.3f' % ((imu_stamps[-1] - imu_stamps[0]) / 1e9))
print('|acc| mean=%.4f std=%.4f min=%.3f max=%.3f' % (
    statistics.mean(imu_acc), statistics.pstdev(imu_acc), min(imu_acc), max(imu_acc)))
print('--- ALIGNMENT ---')
print('lidar first/last :', pc_stamps[0], pc_stamps[-1])
print('imu   first/last :', imu_stamps[0], imu_stamps[-1])
print('offset start s: %.3f  end s: %.3f' % (
    (imu_stamps[0] - pc_stamps[0]) / 1e9, (imu_stamps[-1] - pc_stamps[-1]) / 1e9))
