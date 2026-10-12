#!/usr/bin/env python3
"""Extract /gnss/fix from the bag and report the track statistics."""
import sys, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id='sqlite3'),
       ConverterOptions(input_serialization_format='cdr',
                        output_serialization_format='cdr'))
topics = {t.name: t.type for t in r.get_all_topics_and_types()}
lat0 = lon0 = None
rows = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic != '/gnss/fix':
        continue
    m = deserialize_message(data, get_message(topics[topic]))
    rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                 m.latitude, m.longitude, m.altitude,
                 m.status.status, m.position_covariance[0],
                 m.position_covariance[4], m.position_covariance[8]))

# reference = first fix
t0, la0, lo0, al0 = rows[0][0], rows[0][1], rows[0][2], rows[0][3]
R = 6378137.0
def enu(lat, lon, alt):
    dlat = math.radians(lat - la0)
    dlon = math.radians(lon - lo0)
    e = dlon * R * math.cos(math.radians(la0))
    n = dlat * R
    u = alt - al0
    return e, n, u

good = [r for r in rows if r[4] == 0 and math.sqrt(r[5]) < 1.0 and math.sqrt(r[6]) < 1.0]
print('all fixes:', len(rows), ' status0:', sum(1 for r in rows if r[4] == 0),
      ' high-quality (status0 & sigma<1m):', len(good))
rows = good if len(good) > 100 else rows
print('using', len(rows), 'fixes')
print('status values:', sorted(set(r[4] for r in rows)))
e = [enu(*r[1:4])[0] for r in rows]
n = [enu(*r[1:4])[1] for r in rows]
u = [enu(*r[1:4])[2] for r in rows]
alt = [r[3] for r in rows]
print('first fix lat/lon/alt: %.9f %.9f %.3f' % (la0, lo0, al0))
print('E range %.1f .. %.1f m (span %.1f)' % (min(e), max(e), max(e) - min(e)))
print('N range %.1f .. %.1f m (span %.1f)' % (min(n), max(n), max(n) - min(n)))
print('altitude: min %.2f  max %.2f  span %.2f m  first %.2f  last %.2f'
      % (min(alt), max(alt), max(alt) - min(alt), alt[0], alt[-1]))
print('ENU up: min %.2f max %.2f span %.2f' % (min(u), max(u), max(u) - min(u)))
cov = [math.sqrt(r[5]) for r in rows]
covn = [math.sqrt(r[6]) for r in rows]
covu = [math.sqrt(r[7]) for r in rows]
print('sigma_E mean %.3f max %.3f | sigma_N mean %.3f | sigma_U mean %.3f m'
      % (sum(cov) / len(cov), max(cov), sum(covn) / len(covn), sum(covu) / len(covu)))
# path length
L = 0.0
for i in range(1, len(e)):
    L += math.hypot(e[i] - e[i - 1], n[i] - n[i - 1])
print('2D path length: %.1f m' % L)
# start/end gap (loop closure error indicator)
print('start->end displacement: %.1f m' % math.hypot(e[-1] - e[0], n[-1] - n[0]))
# per-second altitude samples
step = max(1, len(rows) // 12)
print('altitude profile (t, alt):')
for i in range(0, len(rows), step):
    print('   %6.1fs  %8.2f m   E=%8.1f N=%8.1f' % (rows[i][0] - t0, rows[i][3], e[i], n[i]))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(12, 8))
sc = ax.scatter(e, n, s=6, c=[r[3] for r in rows], cmap='viridis')
ax.plot([e[0]], [n[0]], 'r^', ms=14, label='first fix')
ax.plot([e[-1]], [n[-1]], 'ks', ms=10, label='last fix')
ax.set_aspect('equal'); ax.grid(alpha=.3)
ax.set_xlabel('East [m]'); ax.set_ylabel('North [m]')
ax.set_title('GNSS track (Stadtgarten seq2), colour = altitude')
plt.colorbar(sc, label='altitude [m]')
ax.legend()
plt.tight_layout()
plt.savefig('/home/sayooj/Documents/livox_imu_gnss/gnss_track.png', dpi=110)
print('wrote /home/sayooj/Documents/livox_imu_gnss/gnss_track.png')
