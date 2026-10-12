#!/usr/bin/env python3
"""Profile the dynamics actually exercised by this dataset.

Why this matters: the Jetson/real-time question has to be sized against the
WORST case (fast, aggressively yawing flight), but this bag was recorded on
foot. If the recorded motion only reaches a few deg/s and a few dm/s, then the
dataset contains no evidence that FAST-LIO behaves at survey speed -- and that
gap has to be stated rather than papered over.

Reads only the already-extracted trajectory CSV (no bag access, so it is safe
to run concurrently with other work.

Input : stadtgarten_seq2_odom.csv   header: stamp_ns,stamp_sec,x,y,z,qx,qy,qz,qw
Output: printed percentiles
"""
import csv
import math
import os
import sys

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRAJ = os.path.join(DIR, "stadtgarten_seq2_odom.csv")


def quat_angle_deg(qa, qb):
    """Signed-ish rotation angle between two quaternions, in degrees."""
    d = abs(sum(a * b for a, b in zip(qa, qb)))
    d = min(1.0, d)
    return math.degrees(2.0 * math.acos(d))


def pct(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return sorted_vals[lo]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def main():
    if not os.path.exists(TRAJ):
        print(f"missing {TRAJ}", file=sys.stderr)
        return 1

    rows = []
    with open(TRAJ) as f:
        r = csv.DictReader(f)
        fields = r.fieldnames or []
        # resolve columns by name so a header change can't silently shift them
        try:
            ix = {k: fields.index(k) for k in
                  ("stamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw")}
        except ValueError as e:
            print(f"unexpected header {fields}: {e}", file=sys.stderr)
            return 1
        for row in r:
            try:
                rows.append([float(row[k]) for k in
                             ("stamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw")])
            except (ValueError, KeyError):
                continue
    if len(rows) < 3:
        print("too few poses", file=sys.stderr)
        return 1

    # row layout after DictReader: stamp_sec, x, y, z, qx, qy, qz, qw
    t = [r[0] for r in rows]
    dt, speed, yawrate, path = [], [], [], 0.0

    for i in range(1, len(rows)):
        dts = t[i] - t[i - 1]
        if dts <= 0:
            continue
        dx = rows[i][1] - rows[i - 1][1]
        dy = rows[i][2] - rows[i - 1][2]
        dz = rows[i][3] - rows[i - 1][3]
        step = math.sqrt(dx * dx + dy * dy + dz * dz)
        path += step
        qa = rows[i - 1][4:8]
        qb = rows[i][4:8]
        ang = quat_angle_deg(qa, qb)
        dt.append(dts)
        speed.append(step / dts)
        yawrate.append(ang / dts)

    dur = t[-1] - t[0]
    s = sorted(speed)
    y = sorted(yawrate)

    out = f"""Dynamics profile of stadtgarten_seq2 (from {len(rows)} FAST-LIO poses)
=========================================================================
duration              : {dur:.1f} s
path length           : {path:.1f} m
mean speed            : {path / dur:.3f} m/s
pose interval         : {pct(sorted(dt), 50):.4f} s  (median)

Translation speed (m/s)
  median              : {pct(s, 50):.3f}
  p95                 : {pct(s, 95):.3f}
  p99                 : {pct(s, 99):.3f}
  max                 : {s[-1]:.3f}

Rotation rate (deg/s, from consecutive quaternions)
  median              : {pct(y, 50):.2f}
  p95                 : {pct(y, 95):.2f}
  p99                 : {pct(y, 99):.2f}
  max                 : {y[-1]:.2f}
"""
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
