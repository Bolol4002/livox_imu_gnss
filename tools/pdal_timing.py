#!/usr/bin/env python3
"""Stage-by-stage timing of the DEM PDAL pipeline, plus its scaling with point count.

Answers the question "what does one incremental DEM refresh cost?" -- which is
the design-critical number for running the DEM loop onboard a Jetson.

Two things are measured:

  1. BREAKDOWN   -- cumulative wall time of read / +outlier / +smrf / +gdal
                    on the full cloud. Differences give per-stage cost.

  2. SCALING     -- total time on uniformly-strided subsets (25/50/75/100%).
                    Stride sampling -- not truncation -- so the *spatial extent*
                    is preserved. This matters: filters.smrf's morphology window
                    and writers.gdal's grid both scale with extent, not with
                    point count, so truncating the file would understate cost.

Output: tools/pdal_timing.csv
"""
import csv
import json
import os
import subprocess
import sys
import time

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(DIR, "stadtgarten_seq2_cloud_utm.csv")
TMPDIR = os.path.join(DIR, "logs", "pdal_timing")
OUTCSV = os.path.join(DIR, "tools", "pdal_timing.csv")

READER = {
    "type": "readers.text",
    "filename": None,          # filled per-run
    "override_srs": "EPSG:32632",
}
OUTLIER = {
    "type": "filters.outlier",
    "method": "statistical",
    "mean_k": 10,
    "multiplier": 2.0,
    "class": 7,
}
SMRF = {
    "type": "filters.smrf",
    "cell": 1.0,
    "slope": 0.3,
    "scalar": 1.0,
    "threshold": 0.5,
    "where": "Classification != 7",
}
GOLD = {
    "type": "writers.gdal",
    "filename": None,          # filled per-run
    "resolution": 0.5,
    "radius": 1.5,
    "power": 1.0,
    "output_type": "idw",
    "where": "Classification == 2",
    "data_type": "float",
    "nodata": -9999.0,
}


def stage_list(stages, src, dst):
    """Build the stage list for a given cutoff: 'read', 'outlier', 'smrf', 'gdal'."""
    r = dict(READER)
    r["filename"] = src
    out = [r]
    if stages == "read":
        return out
    out.append(dict(OUTLIER))
    if stages == "outlier":
        return out
    out.append(dict(SMRF))
    if stages == "smrf":
        return out
    g = dict(GOLD)
    g["filename"] = dst
    out.append(g)
    return out


def make_subset(src, dst, keep_every):
    """Uniform stride copy preserving header. keep_every=1 -> full copy."""
    n = 0
    with open(src, "r") as fi, open(dst, "w") as fo:
        header = fi.readline()
        fo.write(header)
        for i, line in enumerate(fi):
            if i % keep_every == 0:
                fo.write(line)
                n += 1
    return n


def run(stages, src, dst):
    pipeline = {"pipeline": stage_list(stages, src, dst)}
    path = os.path.join(TMPDIR, "cur.json")
    with open(path, "w") as f:
        json.dump(pipeline, f, indent=2)
    t0 = time.perf_counter()
    p = subprocess.run(["pdal", "pipeline", path],
                       capture_output=True, text=True, cwd=DIR)
    dt = time.perf_counter() - t0
    if p.returncode != 0:
        print(f"  FAILED [{stages}]: {p.stderr.strip()[:600]}", file=sys.stderr)
        return None, p.stderr
    return dt, p.stderr


def main():
    os.makedirs(TMPDIR, exist_ok=True)
    for f in os.listdir(TMPDIR):
        if f.endswith((".json", ".tif")):
            os.remove(os.path.join(TMPDIR, f))

    rows = []

    # ---- 1. stage breakdown on the full cloud -----------------------------
    print("=== stage breakdown, full cloud ===")
    full = os.path.join(TMPDIR, "full.csv")
    n_full = make_subset(SRC, full, 1)
    print(f"  points: {n_full}")
    prev = 0.0
    for stages in ("read", "outlier", "smrf", "gdal"):
        dt, _ = run(stages, full, os.path.join(TMPDIR, "break.tif"))
        if dt is None:
            return 1
        inc = dt - prev
        print(f"  {'through ' + stages:<16} total {dt:7.3f}s   stage {inc:7.3f}s")
        rows.append(("breakdown", n_full, stages, round(dt, 3), round(inc, 3)))
        prev = dt

    # ---- 2. scaling with point count (extent-preserving stride) ----------
    print("=== scaling (uniform stride, extent preserved) ===")
    for keep in (4, 2, 1):
        sub = os.path.join(TMPDIR, f"sub_{keep}.csv")
        n = make_subset(SRC, sub, keep)
        # full pipeline incl. writer
        dt, _ = run("gdal", sub, os.path.join(TMPDIR, f"scale_{keep}.tif"))
        if dt is None:
            return 1
        print(f"  {keep:>2}x stride -> {n:>8,} pts   {dt:7.3f}s   "
              f"({n / dt / 1000:6.1f} k pts/s)")
        rows.append(("scaling", n, "gdal", round(dt, 3), ""))

    os.makedirs(os.path.dirname(OUTCSV), exist_ok=True)
    with open(OUTCSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["group", "points", "stage", "total_s", "stage_s"])
        w.writerows(rows)
    print(f"\nwrote {OUTCSV}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
