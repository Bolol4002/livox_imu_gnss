#!/usr/bin/env python3
"""Render a 2D preview (top-down + side view) from a binary PCD map."""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PCD = sys.argv[1]
OUT = sys.argv[2]

# --- parse header ---------------------------------------------------------
fields, sizes, types, counts, header_len, points, data_off = [], [], [], [], 0, 0, 0
with open(PCD, 'rb') as f:
    while True:
        line = f.readline().decode('ascii', 'replace').strip()
        header_len += len(line) + 1
        if line.startswith('FIELDS'):
            fields = line.split()[1:]
        elif line.startswith('SIZE'):
            sizes = [int(v) for v in line.split()[1:]]
        elif line.startswith('POINTS'):
            points = int(line.split()[1])
        elif line.startswith('DATA'):
            assert line.split()[1] == 'binary', 'only binary PCD supported'
            data_off = header_len
            break
print('fields', fields, 'points', points, 'sizes', sizes)

fmt = {'F': 'f4', 'U': 'u1', 'I': 'i1'}[types[0]] if types else None
dtype = []
for name, size in zip(fields, sizes):
    if size == 4:
        dtype.append((name, '<f4'))
    elif size == 8:
        dtype.append((name, '<f8'))
    elif size == 1:
        dtype.append((name, 'u1'))
    else:
        dtype.append((name, 'V%d' % size))
arr = np.fromfile(PCD, dtype=np.dtype(dtype), count=points, offset=data_off)
x, y, z = arr['x'].astype(np.float64), arr['y'].astype(np.float64), arr['z'].astype(np.float64)
ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
x, y, z = x[ok], y[ok], z[ok]
print('loaded %d points' % len(x))
print('X %.1f .. %.1f   Y %.1f .. %.1f   Z %.1f .. %.1f' % (x.min(), x.max(), y.min(), y.max(), z.min(), z.max()))

fig, ax = plt.subplots(1, 2, figsize=(20, 10))
sc = ax[0].scatter(x, y, s=0.15, c=z, cmap='terrain', linewidths=0)
ax[0].set_aspect('equal')
ax[0].set_title('Top view (XY), colour = elevation Z [m]')
ax[0].set_xlabel('X [m]'); ax[0].set_ylabel('Y [m]')
plt.colorbar(sc, ax=ax[0], shrink=0.7, label='Z [m]')

sc2 = ax[1].scatter(x, z, s=0.15, c=y, cmap='viridis', linewidths=0)
ax[1].set_aspect('equal')
ax[1].set_title('Side view (XZ), colour = Y')
ax[1].set_xlabel('X [m]'); ax[1].set_ylabel('Z [m]')
plt.colorbar(sc2, ax=ax[1], shrink=0.7, label='Y [m]')
plt.tight_layout()
plt.savefig(OUT, dpi=110)
print('wrote', OUT)
