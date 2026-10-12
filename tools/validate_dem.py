#!/usr/bin/env python3
"""Validate the DEM against GNSS.

The GNSS fixes are an INDEPENDENT check: they set the R+T fit (3481 pairs)
but they do not constrain the ground surface at all.  The DEM should be the
ground, and the GNSS antenna rides a fixed height above it, so

    DEM(lat, lon)  -  GNSS_altitude  ~=  -sensor_height   (constant)

A constant negative offset with a small spread means:
  * the vertical axis is not mirrored,
  * the absolute elevation datum is right,
  * the grid really is bare earth rather than canopy/rooftops.

A drifting offset means the R+T is absorbing LIO drift incorrectly.
"""
import csv
import math
import sys

import numpy as np
from osgeo import gdal

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import geo

tif, gnss_csv = sys.argv[1], sys.argv[2]

ds = gdal.Open(tif)
gt = ds.GetGeoTransform()
band = ds.GetRasterBand(1)
nodata = band.GetNoDataValue()
W, H = ds.RasterXSize, ds.RasterYSize
arr = band.ReadAsArray()
inv_ok = ds.GetSpatialRef() is not None
print(f"DEM {W} x {H} @ {gt[1]:g} m, nodata={nodata}, "
      f"EPSG={ds.GetSpatialRef().GetAuthorityCode(None) if inv_ok else '?'}")

rows = []
with open(gnss_csv) as fh:
    for r in csv.DictReader(fh):
        if int(r["status"]) < 0:
            continue
        rows.append((float(r["stamp_sec"]), float(r["latitude"]),
                     float(r["longitude"]), float(r["altitude"])))
rows.sort()
t = np.array([r[0] for r in rows])
lat = np.array([r[1] for r in rows])
lon = np.array([r[2] for r in rows])
alt = np.array([r[3] for r in rows])
print(f"GNSS FIX samples used: {len(rows)}")

utm, epsg = geo.llh_to_utm(lat, lon, alt)
col = np.floor((utm[:, 0] - gt[0]) / gt[1]).astype(int)
row = np.floor((utm[:, 1] - gt[3]) / gt[5]).astype(int)
inside = (col >= 0) & (col < W) & (row >= 0) & (row < H)
print(f"inside DEM extent: {int(inside.sum())} / {len(rows)}")

vals = np.full(len(rows), np.nan)
ok = inside.copy()
ri = np.clip(row, 0, H - 1)
ci = np.clip(col, 0, W - 1)
v = arr[ri, ci]
hit = (v != nodata) & ~np.isnan(v)
vals[hit & inside] = v[hit & inside]
print(f"sampled a real (non-nodata) cell: {int(np.isfinite(vals).sum())} / "
      f"{len(rows)}")

d = vals - alt
good = np.isfinite(d)
if good.sum() < 10:
    raise SystemExit("too few overlapping samples to validate")
resid = d[good]

# The antenna is at a FIXED height above ground, so the offset must be
# constant along the track.  Look for drift as well as bias.
print()
print(f"samples with DEM data : {good.sum()}")
print(f"DEM - GNSS altitude   : mean {resid.mean():+.3f} m, "
      f"median {np.median(resid):+.3f} m")
print(f"                        std  {resid.std():.3f} m, "
      f"min {resid.min():+.3f}, max {resid.max():+.3f}")
print(f"=> implied sensor height above ground: "
      f"{-resid.mean():+.3f} m  (std {resid.std():.3f} m)")

# drift check: split into 5 time windows
print()
print("window            n   mean offset   std")
for k in range(5):
    lo, hi = t.min() + k * (t.max() - t.min()) / 5, t.min() + (k + 1) * (t.max() - t.min()) / 5
    m = good & (t >= lo) & (t < hi)
    if m.sum() >= 5:
        print(f"  {k + 1}  {lo - t.min():6.0f}-{hi - t.min():.0f}s "
              f"{m.sum():5d}   {d[m].mean():+.3f} m   {d[m].std():.3f}")

print()
off = -resid.mean()
print(f"verdict: {'PASS' if resid.std() < 1.0 else 'SUSPECT'} "
      f"({'offset is constant -> heights are correct' if resid.std() < 1.0 else 'offset varies -> drift/mirroring'})")
