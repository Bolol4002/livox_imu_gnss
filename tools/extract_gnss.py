#!/usr/bin/env python3
"""Extract /gnss/fix from the bag into a CSV.

Why this is done offline instead of live: the fixes are already recorded in the
bag, so replaying them through ROS adds nothing but risk (QoS / queue drops).
Reading only /gnss/fix with rosbag2_py skips the 16 GB of point clouds.

Output columns:
    stamp_ns, stamp_sec, latitude, longitude, altitude, status,
    cov_e, cov_n, cov_u
status: NavSatStatus.status  (0 = FIX, 1 = SBAS, 2 = NO_FIX ... -1 = no signal)
cov_*:  sqrt of position_covariance diagonal, i.e. 1-sigma metres in ENU.
"""
import csv
import math
import sys

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPIC = "/gnss/fix"


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: extract_gnss.py <bag_dir> <out.csv>", file=sys.stderr)
        return 2
    bag, out = sys.argv[1], sys.argv[2]

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TOPIC not in types:
        print(f"topic {TOPIC} not in bag: {sorted(types)}", file=sys.stderr)
        return 1

    msg_type = get_message(types[TOPIC])
    rows = []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != TOPIC:
            continue
        m = deserialize_message(data, msg_type)
        ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        c = m.position_covariance  # 3x3 row-major, ENU
        rows.append([
            ns,
            f"{ns / 1e9:.9f}",
            f"{m.latitude:.9f}",
            f"{m.longitude:.9f}",
            f"{m.altitude:.4f}",
            int(m.status.status),
            f"{math.sqrt(max(c[0], 0.0)):.4f}",
            f"{math.sqrt(max(c[4], 0.0)):.4f}",
            f"{math.sqrt(max(c[8], 0.0)):.4f}",
        ])

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp_ns", "stamp_sec", "latitude", "longitude",
                    "altitude", "status", "cov_e", "cov_n", "cov_u"])
        w.writerows(rows)

    ok = sum(1 for r in rows if r[5] in (0, 1))
    sig = [float(r[6]) for r in rows if r[5] in (0, 1)]
    print(f"wrote {len(rows)} fixes -> {out}")
    print(f"  status FIX/SBAS : {ok}")
    print(f"  span             : {(int(rows[-1][0]) - int(rows[0][0])) / 1e9:.2f} s"
          if rows else "  (empty)")
    if sig:
        sig.sort()
        print(f"  sigma_E median   : {sig[len(sig) // 2]:.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
