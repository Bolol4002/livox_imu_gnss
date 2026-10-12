#!/usr/bin/env python3
"""Compare two DEM GeoTIFFs cell-by-cell.

Used to answer: how much does the 74%-of-runtime `filters.outlier` stage
actually change the delivered DEM? If the answer is "almost nothing", the
onboard incremental loop can drop it or run it rarely.

Reports, over cells valid in BOTH rasters:
  mean / stddev / p95 / max of (test - ref)
plus how many cells change validity (valid in one, nodata in the other),
since that is the other way a stage can matter.

Usage: tools/compare_dem.py REF.tif TEST.tif
"""
import os
import sys

import numpy as np
from osgeo import gdal

gdal.UseExceptions()


def load(path):
    ds = gdal.Open(path)
    if ds is None:
        sys.exit(f"cannot open {path}")
    band = ds.GetRasterBand(1)
    nd = band.GetNoDataValue()
    arr = band.ReadAsArray().astype(np.float64)
    gt = ds.GetGeoTransform()
    ds = None
    return arr, nd, gt


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    ref_p, test_p = sys.argv[1], sys.argv[2]

    ref, ref_nd, ref_gt = load(ref_p)
    test, test_nd, test_gt = load(test_p)

    if ref.shape != test.shape:
        sys.exit(f"shape mismatch: {ref.shape} vs {test.shape}")
    if ref_gt != test_gt:
        print(f"WARNING: geotransforms differ\n  ref  {ref_gt}\n  test {test_gt}")

    def valid(a, nd):
        m = np.isfinite(a)
        if nd is not None:
            m &= (a != nd)
        return m

    vr, vt = valid(ref, ref_nd), valid(test, test_nd)
    both = vr & vt

    print(f"ref   : {ref_p}   {ref.shape[1]}x{ref.shape[0]}  valid {vr.sum():,} "
          f"({100.0 * vr.sum() / vr.size:.2f}%)")
    print(f"test  : {test_p}   {test.shape[1]}x{test.shape[0]}  valid {vt.sum():,} "
          f"({100.0 * vt.sum() / vt.size:.2f}%)")
    print(f"both  : {both.sum():,} cells")
    print(f"only-ref  : {(vr & ~vt).sum():,}")
    print(f"only-test : {(vt & ~vr).sum():,}")

    if both.sum() == 0:
        print("no overlapping valid cells -- cannot compare")
        return 1

    d = test[both] - ref[both]
    ad = np.abs(d)
    print()
    print(f"elevation difference  (test - ref)  over {both.sum():,} common cells")
    print(f"  mean   {d.mean():+.4f} m")
    print(f"  stddev {d.std():.4f} m")
    print(f"  p50    {np.percentile(ad, 50):.4f} m")
    print(f"  p95    {np.percentile(ad, 95):.4f} m")
    print(f"  p99    {np.percentile(ad, 99):.4f} m")
    print(f"  max    {ad.max():.4f} m")
    for thr in (0.05, 0.10, 0.25, 0.50):
        n = int((ad > thr).sum())
        print(f"  |diff| > {thr:.2f} m : {n:8,} cells "
              f"({100.0 * n / ad.size:5.2f} %)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
