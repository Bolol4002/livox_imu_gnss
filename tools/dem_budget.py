#!/usr/bin/env python3
"""Can an incremental DEM loop keep up onboard? Cost model from measured timings.

Reads the timings measured by tools/pdal_timing.py and answers: if the DEM
pipeline is re-run every T seconds while the map grows, for how long does it
stay within budget as a function of (a) how much slower the target CPU is than
this machine and (b) the refresh cadence.

Model
-----
Measured full-pipeline wall time is close to linear in point count:

    cost(N) = c0 + b * N          (least-squares fit to the scaling rows)

The map grows at a roughly constant rate r (points per second of flight) for a
survey covering new ground -- revisit passes add few points because the voxel
grid is already populated, so r from this run is an UPPER bound.

At flight time t the map holds N(t) = r*t, so a full rebuild costs
cost(r*t), and it keeps up iff  cost(r*t) < cadence.

Two variants:

  full     -- re-run the whole pipeline (read + outlier + smrf + gdal)
  incremental -- only NEW points go through filters.outlier (the 74% term,
              which is a per-point kNN cost), while the cheap stages
              (read/smrf/gdal) re-run over everything.

Usage:
  tools/dem_budget.py [--slowdown 3.0] [--cadence 60,30,10]
"""
import argparse
import csv
import os
import sys

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TIMING = os.path.join(DIR, "tools", "pdal_timing.csv")
TRAJ_T = 875.5      # s of flight that actually produced poses (IMU span, see §3)


def load():
    n_full = stage = None
    breakdown = {}
    scaling = []
    with open(TIMING) as f:
        for row in csv.DictReader(f):
            if row["group"] == "breakdown":
                if row["stage"] == "gdal":
                    n_full = int(row["points"])
                breakdown[row["stage"]] = float(row["total_s"])
            elif row["group"] == "scaling":
                scaling.append((int(row["points"]), float(row["total_s"])))
    if not scaling or n_full is None:
        sys.exit(f"bad or missing {TIMING}; run tools/pdal_timing.py first")
    return n_full, breakdown, sorted(scaling)


def linfit(pts):
    """Least squares cost = c0 + b*N over (N, seconds) pairs."""
    n = len(pts)
    sx = sum(p for p, _ in pts)
    sy = sum(s for _, s in pts)
    sxx = sum(p * p for p, _ in pts)
    sxy = sum(p * s for p, s in pts)
    den = n * sxx - sx * sx
    b = (n * sxy - sx * sy) / den
    c0 = (sy - b * sx) / n
    return c0, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slowdown", type=float, default=3.0,
                    help="target CPU / this CPU time ratio (A78 vs i5 estimate)")
    ap.add_argument("--cadence", default="60,30,10",
                    help="comma-separated refresh cadences in seconds")
    ap.add_argument("--incremental", action="store_true",
                    help="count filters.outlier only over NEW points")
    args = ap.parse_args()

    n_full, breakdown, scaling = load()
    c0, b = linfit(scaling)
    rate = n_full / TRAJ_T
    # breakdown[] holds CUMULATIVE totals per cutoff, so the real total is the
    # last entry -- summing them would double-count every earlier stage.
    total = breakdown.get("gdal", max(breakdown.values()))
    parts = {
        "read": breakdown.get("read", 0.0),
        "outlier": breakdown.get("outlier", 0.0) - breakdown.get("read", 0.0),
        "smrf": breakdown.get("smrf", 0.0) - breakdown.get("outlier", 0.0),
        "gdal": breakdown.get("gdal", 0.0) - breakdown.get("smrf", 0.0),
    }

    out = []
    out.append("DEM keep-up budget (from measured timings)")
    out.append("=" * 62)
    out.append(f"this machine          : i5-1155G7 (4C/8T)")
    out.append(f"full pipeline @ {n_full:,} pts : {total:.2f} s"
               f"   (read {parts['read']:.2f} / "
               f"outlier {parts['outlier']:.2f} / "
               f"smrf {parts['smrf']:.2f} / "
               f"gdal {parts['gdal']:.2f})")
    out.append(f"linear fit            : cost = {c0:.3f} + {b * 1e6:.3f}e-6 * N")
    out.append(f"map growth rate       : {rate:.0f} pts/s "
               f"({n_full:,} pts over {TRAJ_T:.0f} s)")
    out.append(f"assumed slowdown      : {args.slowdown:.1f}x  (ESTIMATE, not measured)")
    if args.incremental:
        frac = parts["outlier"]
        out.append("mode                  : INCREMENTAL "
                   f"(outlier over new points only, {frac:.2f}s/{n_full:,} pts)")
    else:
        out.append("mode                  : FULL rebuild each cycle")
    out.append("")

    # largest flight time t at which one cycle still fits inside the cadence
    def max_t(cad, sl):
        """Largest t for which cost(t) < cad."""
        if args.incremental:
            # recurring = read+smrf+gdal over full N, outlier over new pts only
            per_pt = (parts["read"] + parts["smrf"] + parts["gdal"]) / n_full
            out_pt = parts["outlier"] / n_full
            # cad = sl * (c0 + per_pt*N + out_pt*cad*rate)  -> solve for N
            # the new-point term is constant per cycle (= sl*out_pt*cad*rate)
            fixed = sl * out_pt * cad * rate
            N = (cad - fixed - sl * c0) / (sl * per_pt)
            return N / rate
        # full rebuild: sl*(c0 + b*r*t) = cad
        return (cad / sl - c0) / (b * rate)

    for cad in [float(x) for x in args.cadence.split(",")]:
        t = max_t(cad, args.slowdown)
        marker = ""
        if t < TRAJ_T:
            marker = "  <-- too short for this dataset"
        elif t < 2 * TRAJ_T:
            marker = "  <- tight"
        out.append(f"  cadence {cad:5.0f} s  ->  keeps up for {t / 60:6.1f} min{marker}")

    out.append("")
    out.append("note: t must exceed the planned flight time for the loop to hold.")
    out.append("      An incremental pass relaxes this substantially because")
    out.append("      filters.outlier is 74% of the cost and is a per-point kNN.")

    text = "\n".join(out)
    print(text)
    dest = os.path.join(DIR, "tools",
                        "dem_budget_inc.txt" if args.incremental else "dem_budget.txt")
    with open(dest, "w") as f:
        f.write(text + "\n")
    print(f"\nwrote {dest}")


if __name__ == "__main__":
    main()
