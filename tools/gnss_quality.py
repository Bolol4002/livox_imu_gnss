#!/usr/bin/env python3
"""Characterise the /gnss/fix track quality before we trust it for R+T.

Answers:
  - Is NO_FIX contiguous (a stretch of sky occlusion) or interleaved?
  - During NO_FIX, are coordinates still smooth, or frozen/jumping?
  - What is the ENU extent of the usable fixes vs. all fixes?
  - What is the horizontal noise floor of the FIX periods?

Writes gnss_quality.png.
"""
import csv
import math
import sys
from collections import Counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PATH = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else "gnss_quality.png"

ts, lat, lon, alt, st, ce, cn, cu = [], [], [], [], [], [], [], []
with open(PATH) as fh:
    for r in csv.DictReader(fh):
        ts.append(float(r["stamp_sec"]))
        lat.append(float(r["latitude"]))
        lon.append(float(r["longitude"]))
        alt.append(float(r["altitude"]))
        st.append(int(r["status"]))
        ce.append(float(r["cov_e"]))
        cn.append(float(r["cov_n"]))
        cu.append(float(r["cov_u"]))

ts = np.array(ts)
lat = np.array(lat)
lon = np.array(lon)
alt = np.array(alt)
st = np.array(st)
sig_h = np.hypot(np.array(ce), np.array(cn))
sig_v = np.array(cu)
good = st >= 0

t0 = ts[0]
R = 6378137.0
lat0 = lat[good][0]
E = np.radians(lon - lon[0]) * R * math.cos(math.radians(lat0))
N = np.radians(lat - lat[0]) * R
U = alt - alt[0]

print(f"fixes              : {len(ts)}  span {ts[-1] - ts[0]:.1f} s")
print(f"FIX (status>=0)    : {good.sum()} ({100 * good.mean():.1f}%)")
print(f"NO_FIX (status=-1) : {(~good).sum()} ({100 * (~good).mean():.1f}%)")

# --- contiguity: runs of equal status ---
runs, cur, n = [], st[0], 1
for v in st[1:]:
    if v == cur:
        n += 1
    else:
        runs.append((cur, n))
        cur, n = v, 1
runs.append((cur, n))
print(f"status transitions : {len(runs) - 1}  "
      f"(longest NO_FIX run {max((n for s, n in runs if s < 0), default=0)} samples)")

# --- per 60 s window ---
print("\n window |  FIX  NOFIX | sig_h med/max | sig_v med |  E range   N range   U range")
print("--------+-------------+---------------+-----------+--------------------------------")
W = 60.0
edges = np.arange(ts[0], ts[-1] + W, W)
for i in range(len(edges) - 1):
    m = (ts >= edges[i]) & (ts < edges[i + 1])
    if not m.any():
        continue
    g = m & good
    f, nf = int(g.sum()), int((m & ~good).sum())
    if g.sum() >= 2:
        sh = f"{np.median(sig_h[g]):8.3f}/{np.max(sig_h[g]):5.2f}"
        sv = f"{np.median(sig_v[g]):8.3f}"
        er = f"{E[g].max() - E[g].min():6.1f}m"
        nr = f"{N[g].max() - N[g].min():6.1f}m"
        ur = f"{U[g].max() - U[g].min():6.1f}m"
    else:
        sh = sv = er = nr = ur = "     --"
    print(f" {int((edges[i] - t0) // 60):4d}-{int((edges[i + 1] - t0) // 60):3d}m "
          f"| {f:5d} {nf:6d} | {sh} | {sv} | {er} {nr} {ur}")

# --- noise floor of FIX period: high-pass residual ---
g = np.where(good)[0]
res_e, res_n = [], []
for k in g[2:-2]:
    if not (good[k - 2:k + 3]).all():
        continue
    res_e.append(E[k] - 0.5 * (E[k - 2] + E[k + 2]))
    res_n.append(N[k] - 0.5 * (N[k - 2] + N[k + 2]))
if res_e:
    print(f"\nFIX-period jitter (2-sample predictor residual): "
          f"RMS E {np.std(res_e) * 100:.1f} cm, RMS N {np.std(res_n) * 100:.1f} cm")

# --- are NO_FIX coordinates still moving? ---
d = np.hypot(np.diff(E), np.diff(N))
bad = (st[1:] < 0) & (st[:-1] < 0)
print(f"median step all    : {np.median(d) * 100:.1f} cm/sample")
if bad.sum():
    print(f"median step NO_FIX : {np.median(d[bad]) * 100:.1f} cm/sample "
          f"(freeze -> ~0, smooth -> still tracking)")

# --- figure ---
fig, ax = plt.subplots(2, 2, figsize=(15, 9))
a = ax[0, 0]
a.plot(E, N, lw=0.4, color="0.7", label="all")
a.plot(E[good], N[good], ".", ms=2, color="tab:green", label="FIX")
a.plot(E[~good], N[~good], ".", ms=2, color="tab:red", alpha=.5, label="NO_FIX")
a.set_aspect("equal"); a.set_xlabel("E (m)"); a.set_ylabel("N (m)")
a.set_title("GNSS track in local ENU"); a.legend(markerscale=6); a.grid(alpha=.3)

a = ax[0, 1]
a.plot(ts - t0, st, lw=0.6)
a.set_yticks([-1, 0]); a.set_yticklabels(["NO_FIX", "FIX"])
a.set_xlabel("t (s)"); a.set_title("NavSatFix.status over time"); a.grid(alpha=.3)

a = ax[1, 0]
a.plot(ts - t0, sig_h, lw=0.6, label="horizontal")
a.plot(ts - t0, sig_v, lw=0.6, label="vertical")
a.set_yscale("log"); a.set_xlabel("t (s)"); a.set_ylabel("1-sigma (m)")
a.set_title("Reported covariance"); a.legend(); a.grid(alpha=.3)

a = ax[1, 1]
a.plot(ts - t0, U, lw=0.7)
a.set_xlabel("t (s)"); a.set_ylabel("U (m)")
a.set_title("GNSS altitude relative to first fix "
            f"({U[good].min():.1f} .. {U[good].max():.1f} m over FIX samples)")
a.grid(alpha=.3)

fig.suptitle("stadtgarten_seq2 /gnss/fix quality", fontsize=13)
fig.tight_layout()
fig.savefig(OUT, dpi=110)
print(f"\nwrote {OUT}")
