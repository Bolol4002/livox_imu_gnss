#!/usr/bin/env python3
"""Inspect selected topics of a rosbag2 without deserializing unavailable types."""
import sys, struct
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
TOPICS = sys.argv[2].split(',')
MAXN = int(sys.argv[3]) if len(sys.argv) > 3 else 3

DT = {1: ('b', 1), 2: ('B', 1), 3: ('h', 2), 4: ('H', 2),
      5: ('i', 4), 6: ('I', 4), 7: ('f', 4), 8: ('d', 8)}

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'),
            ConverterOptions(input_serialization_format='cdr',
                             output_serialization_format='cdr'))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
wanted = {t: types[t] for t in types if t in TOPICS}
print('available:', types)

stored = {t: [] for t in wanted}
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic in wanted and len(stored[topic]) < MAXN:
        stored[topic].append((t, data))
    if all(len(v) >= MAXN for v in stored.values()):
        break


def read_field(msg_bytes, base, f):
    fmt, sz = DT[f.datatype]
    return struct.unpack_from('<' + fmt, msg_bytes, base + f.offset)[0]


for topic, msgs in stored.items():
    print('=' * 70)
    print(topic, wanted[topic], 'n=', len(msgs))
    for t, data in msgs:
        try:
            m = deserialize_message(data, get_message(wanted[topic]))
        except Exception as e:
            print('  deserialize failed:', e)
            continue
        print('  bag time:', t)
        if topic.endswith('points'):
            print('  header.stamp:', m.header.stamp.sec, m.header.stamp.nanosec,
                  'frame:', repr(m.header.frame_id))
            print('  height,width:', m.height, m.width, 'is_dense:', m.is_dense,
                  'point_step,row_step:', m.point_step, m.row_step)
            for f in m.fields:
                print('   field', f.name, 'offset', f.offset, 'datatype', f.datatype, 'count', f.count)
            buf = bytes(m.data)
            n = m.width
            for i in list(range(min(n, 4))) + ([n - 1] if n > 4 else []):
                base = i * m.point_step
                vals = {f.name: round(read_field(buf, base, f), 6) for f in m.fields}
                print('   pt', i, vals)
            # stats over timestamp field if present
            tf = [f for f in m.fields if f.name in ('timestamp', 'time', 't')]
            if tf:
                tf = tf[0]
                ts = [read_field(buf, i * m.point_step, tf) for i in range(n)]
                print('   ts min/max:', min(ts), max(ts), 'span', max(ts) - min(ts),
                      'monotonic:', all(ts[i] <= ts[i + 1] for i in range(n - 1)))
            lf = [f for f in m.fields if f.name == 'line']
            if lf:
                lines = set(read_field(buf, i * m.point_step, lf[0]) for i in range(n))
                print('   lines:', sorted(lines))
        elif topic.endswith('imu'):
            print('  stamp', m.header.stamp.sec, m.header.stamp.nanosec,
                  'frame', repr(m.header.frame_id))
            print('  ang_vel', (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z),
                  'lin_acc', (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
            print('  cov_ang', m.angular_velocity_covariance[:3],
                  'cov_acc', m.linear_acceleration_covariance[:3])
        elif topic.endswith('fix'):
            print('  stamp', m.header.stamp.sec, m.header.stamp.nanosec,
                  'frame', repr(m.header.frame_id))
            print('  status', m.status.status, 'lat,lon,alt', m.latitude, m.longitude, m.altitude)
            print('  pos_cov', m.position_covariance[:3], 'type', m.position_covariance_type)
