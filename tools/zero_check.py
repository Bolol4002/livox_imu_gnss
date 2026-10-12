#!/usr/bin/env python3
"""Check zero-point pattern + ranges in /livox/points, and empty-frame timeline."""
import sys, struct, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'),
            ConverterOptions(input_serialization_format='cdr',
                             output_serialization_format='cdr'))
topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
PC = '/livox/points'
t0 = None
frames = []
empties = []
shown = 0
while reader.has_next():
    topic, data, bagt = reader.read_next()
    if topic != PC:
        continue
    m = deserialize_message(data, get_message(topics[PC]))
    hdr = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
    if t0 is None:
        t0 = hdr
    n = m.width
    frames.append((hdr - t0) / 1e9)
    if n == 0:
        empties.append((hdr - t0) / 1e9)
        continue
    if shown >= 3:
        continue
    shown += 1
    buf = bytes(m.data)
    pts = [struct.unpack_from('<ffff', buf, i * m.point_step) for i in range(n)]
    zero = [(x == 0 and y == 0 and z == 0) for x, y, z, _ in pts]
    print('frame %d  n=%d  zeros=%d (%.1f%%)' % (shown, n, sum(zero), 100.0 * sum(zero) / n))
    print('  first 60:', ''.join('0' if v else '.' for v in zero[:60]))
    print('  around 500:', ''.join('0' if v else '.' for v in zero[500:560]))
    runs, cur, cnt = [], zero[0], 1
    for v in zero[1:]:
        if v == cur:
            cnt += 1
        else:
            runs.append((int(cur), cnt)); cur, cnt = v, 1
    runs.append((int(cur), cnt))
    print('  first 15 runs:', runs[:15])
    rr, ii = [], []
    for i in range(0, n, 7):
        x, y, z, it = struct.unpack_from('<ffff', buf, i * m.point_step)
        r = math.sqrt(x * x + y * y + z * z)
        if r > 0:
            rr.append(r)
        ii.append(it)
    rr.sort()
    print('  nonzero range: n=%d min=%.2f p50=%.2f p95=%.2f max=%.2f'
          % (len(rr), rr[0], rr[len(rr) // 2], rr[int(len(rr) * .95)], rr[-1]))
    print('  intensity sample: min=%.2f max=%.2f' % (min(ii), max(ii)))

print('total frames:', len(frames), 'empty:', len(empties))
if empties:
    print('empty frames at t(s):', ['%.1f' % e for e in empties[:40]],
          '...' if len(empties) > 40 else '')
    # bucket by minute
    import collections
    b = collections.Counter(int(e // 60) for e in empties)
    print('empty per minute-of-bag:', dict(sorted(b.items())))
