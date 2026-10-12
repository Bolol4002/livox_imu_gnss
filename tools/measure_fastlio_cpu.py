#!/usr/bin/env python3
"""Measure FAST-LIO's CPU and memory footprint over a full bag replay.

Produces the one number that decides Jetson Orin NX feasibility: how many
CPU-seconds laserMapping burns per second of flight. Orin NX has 6x Cortex-A78,
so if FAST-LIO needs, say, 1.4 cores of *this* x86 machine, it needs some
fraction of that on A78 cores -- and the ratio has to be estimated separately,
but the x86 number is the anchor.

Samples once per second:
  utime, stime  -> /proc/<pid>/stat fields 14,15 (USER_HZ ticks)
  rss           -> /proc/<pid>/status VmRSS

Run via tools/measure_fastlio_cpu.sh (ROS must be sourced first).

Output: tools/fastlio_cpu.csv  (one row per second)
"""
import csv
import os
import signal
import subprocess
import sys
import time

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BAG = os.path.join(DIR, "stadtgarten_seq2.db3")
OUT = os.path.join(DIR, "tools", "fastlio_cpu.csv")
OUT2 = os.path.join(DIR, "tools", "fastlio_cpu_summary.txt")

HZ = os.sysconf("SC_CLK_TCK")


def find_pid(comm="fastlio_mapping"):
    """Find the laserMapping process by its executable name (comm)."""
    try:
        out = subprocess.check_output(["pgrep", "-x", comm], text=True)
    except subprocess.CalledProcessError:
        return None
    pids = [int(p) for p in out.split()]
    return pids[0] if pids else None


def read_proc(pid):
    """Return (utime_s, stime_s, rss_bytes) or None if the process is gone."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().split()
        # comm can contain spaces/parens, but we match by comm so it's simple;
        # be safe and split on the last ')'.
        utime = int(fields[13]) / HZ
        stime = int(fields[14]) / HZ
        rss_pages = int(fields[21])
    except (OSError, IndexError, ValueError):
        return None
    try:
        with open(f"/proc/{pid}/status") as f:
            rss = next(int(l.split()[1]) * 1024 for l in f if l.startswith("VmRSS:"))
    except (OSError, StopIteration, IndexError, ValueError):
        rss = rss_pages * os.sysconf("SC_PAGE_SIZE")
    return utime, stime, rss


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--wait-only":
        # probe mode: just confirm the pid is findable
        pid = find_pid()
        print(pid if pid else "NOT_FOUND")
        return 0 if pid else 1

    pid = find_pid()
    if not pid:
        print("fastlio_mapping not found -- is FAST-LIO running?", file=sys.stderr)
        return 1
    print(f"[*] sampling fastlio_mapping pid={pid}", flush=True)

    cmd = [
        "ros2", "bag", "play", BAG,
        "--topics", "/livox/points", "/livox/imu", "/gnss/fix",
        "--rate", "1.0", "--read-ahead-queue-size", "400",
    ]
    play = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rows = []
    t_start = time.time()
    last = read_proc(pid)
    # settle: give the node a moment after playback is requested
    while play.poll() is None:
        cur = read_proc(pid)
        now = time.time() - t_start
        if cur and last:
            rows.append((round(now, 1),
                         round(cur[0] - last[0], 3),
                         round(cur[1] - last[1], 3),
                         cur[2]))
            last = cur
        elif cur:
            last = cur
        time.sleep(1.0)

    wall = time.time() - t_start
    print(f"[*] playback finished, wall={wall:.1f}s, samples={len(rows)}", flush=True)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "utime_s", "stime_s", "rss_bytes"])
        w.writerows(rows)

    if not rows:
        print("no samples", file=sys.stderr)
        return 1

    cpu_s = [u + s for _, u, s, _ in rows]
    rss = [r for *_, r in rows]
    total_cpu = sum(cpu_s)
    n = len(cpu_s)
    mean_pct = 100.0 * total_cpu / (n * 1.0)   # cores-equivalent x100
    # steady state = drop the first 15 s (map warm-up / QoS discovery)
    steady = cpu_s[15:] if n > 30 else cpu_s
    steady_pct = 100.0 * sum(steady) / len(steady)

    # thirds: does CPU grow with map size?
    third = n // 3
    def pct(seg):
        return 100.0 * sum(seg) / len(seg) if seg else float("nan")
    early, mid, late = pct(cpu_s[:third]), pct(cpu_s[third:2 * third]), pct(cpu_s[2 * third:])

    summary = f"""FAST-LIO CPU footprint over full bag replay
=================================================
bag                 : {os.path.basename(BAG)}
wall clock          : {wall:.1f}s
samples             : {n}
process             : fastlio_mapping pid={pid}

CPU (one core = 100%)
  mean all          : {mean_pct:6.1f} %
  steady (t>15s)    : {steady_pct:6.1f} %
  first third       : {early:6.1f} %
  second third      : {mid:6.1f} %
  last third        : {late:6.1f} %
  total CPU used    : {total_cpu:.1f} core-seconds
  core-sec / sec    : {total_cpu / wall:.3f}

Memory
  RSS start         : {rss[0] / 1e6:8.1f} MB
  RSS peak          : {max(rss) / 1e6:8.1f} MB
  RSS end           : {rss[-1] / 1e6:8.1f} MB
  growth over run   : {(rss[-1] - rss[0]) / 1e6:+8.1f} MB
"""
    with open(OUT2, "w") as f:
        f.write(summary)
    print(summary, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
