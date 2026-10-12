#!/usr/bin/env python3
"""Stage 2 of the georeferencing pipeline:

    FAST-LIO trajectory  +  GNSS fixes
             ->  common ENU frame
             ->  time matching (interpolate GNSS at LIO stamps)
             ->  rigid R + T   (Umeyama, no scale)
             ->  transform every map point
             ->  UTM CSV for PDAL

The document's Phase A calls for exactly one global rigid transform between
the LIO world frame and a local ENU frame built from the GNSS track; this
script implements that and reports how well it actually fits.

Because a single rigid transform cannot absorb LIO drift, three models are
evaluated and compared:

  rigid  full 6-DOF rotation + translation (document's Phase A, the default)
  yaw    4-DOF: rotation about the ENU up-axis only + translation
  --     per-fix-block residuals expose time-varying drift either way

Usage:
  align_r_t.py --odom ODOM.csv --gnss GNSS.csv --pcd IN.pcd \
               --out PREFIX [--model rigid|yaw] [--block-s 10]
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geo


# ---------------------------------------------------------------- loading ---
def read_pcd(path):
    """Read a binary PCD and return (x, y, z) as float64 arrays."""
    with open(path, "rb") as fh:
        hdr = {}
        while True:
            line = fh.readline().decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            hdr[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        if hdr["DATA"][0] != "binary":
            raise SystemExit(f"unsupported PCD DATA mode: {hdr['DATA']}")
        fields = hdr["FIELDS"]
        sizes = [int(v) for v in hdr["SIZE"]]
        types = hdr["TYPE"]
        counts = [int(v) for v in hdr.get("COUNT", ["1"] * len(fields))]
        npts = int(hdr["POINTS"][0])

        fmt = {("4", "F"): "<f4", ("8", "F"): "<f8", ("4", "U"): "<u4",
               ("4", "I"): "<i4", ("2", "U"): "<u2", ("2", "I"): "<i2",
               ("1", "U"): "<u1", ("1", "I"): "<i1", ("8", "U"): "<u8",
               ("8", "I"): "<i8"}
        dt = []
        for name, sz, ty, ct in zip(fields, sizes, types, counts):
            code = fmt.get((str(sz), ty))
            if code is None:
                raise SystemExit(f"unsupported PCD field {name} size={sz} type={ty}")
            dt.append((name, code) if ct == 1 else (name, code, (ct,)))
        data = np.fromfile(fh, dtype=np.dtype(dt), count=npts)
    if len(data) != npts:
        raise SystemExit(f"short read: {len(data)} of {npts} points")
    return (data["x"].astype(np.float64),
            data["y"].astype(np.float64),
            data["z"].astype(np.float64))


def load_odom(path):
    t, p, q = [], [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            t.append(float(r["stamp_sec"]))
            p.append([float(r["x"]), float(r["y"]), float(r["z"])])
            q.append([float(r["qx"]), float(r["qy"]), float(r["qz"]),
                      float(r["qw"])])
    order = np.argsort(t)
    t = np.asarray(t)[order]
    return t, np.asarray(p)[order], np.asarray(q)[order]


def load_gnss(path):
    t, lat, lon, alt, st, sh = [], [], [], [], [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            t.append(float(r["stamp_sec"]))
            lat.append(float(r["latitude"]))
            lon.append(float(r["longitude"]))
            alt.append(float(r["altitude"]))
            st.append(int(r["status"]))
            sh.append(math.hypot(float(r["cov_e"]), float(r["cov_n"])))
    order = np.argsort(t)
    idx = np.array(order)
    return (np.asarray(t)[idx], np.asarray(lat)[idx], np.asarray(lon)[idx],
            np.asarray(alt)[idx], np.asarray(st)[idx], np.asarray(sh)[idx])


def fix_blocks(t, st, gap=0.25, min_span=10.0):
    """Contiguous runs of valid fixes (status >= 0) long enough to trust."""
    good = st >= 0
    blocks, start = [], None
    for i, g in enumerate(good):
        if g and start is None:
            start = i
        elif not g and start is not None:
            blocks.append((start, i))
            start = None
    if start is not None:
        blocks.append((start, len(good)))
    # split blocks that contain a time gap larger than `gap`
    out = []
    for a, b in blocks:
        sub, s = [], a
        for i in range(a + 1, b):
            if t[i] - t[i - 1] > gap:
                sub.append((s, i))
                s = i
        sub.append((s, b))
        out.extend(sub)
    return [(a, b) for a, b in out if t[b - 1] - t[a] >= min_span]


# ------------------------------------------------------------------ models --
def umeyama(x, y):
    """Optimal rigid (no-scale) transform mapping x -> y.

    x, y: (n,3).  Returns R (3,3), t (3,).
    """
    mx, my = x.mean(0), y.mean(0)
    hx, hy = x - mx, y - my
    H = hx.T @ hy
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = my - R @ mx
    return R, t


def yaw_only(x, y):
    """Rotation restricted to the ENU up-axis + translation (4-DOF)."""
    mx, my = x.mean(0), y.mean(0)
    hx, hy = x - mx, y - my
    a = np.sum(hx[:, 0] * hy[:, 0] + hx[:, 1] * hy[:, 1])
    b = np.sum(hx[:, 0] * hy[:, 1] - hx[:, 1] * hy[:, 0])
    th = math.atan2(b, a)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return R, my - R @ mx, th


def euler_zyx(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    roll = math.atan2(R[2, 1], R[2, 2])
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def res_stats(res):
    """res: (n,3) in ENU metres -> dict of residual figures."""
    h = np.hypot(res[:, 0], res[:, 1])
    d = np.linalg.norm(res, axis=1)
    return dict(n=len(res),
                rms_e=float(np.sqrt(np.mean(res[:, 0] ** 2))),
                rms_n=float(np.sqrt(np.mean(res[:, 1] ** 2))),
                rms_u=float(np.sqrt(np.mean(res[:, 2] ** 2))),
                rms_h=float(np.sqrt(np.mean(h ** 2))),
                rms_3d=float(np.sqrt(np.mean(d ** 2))),
                max_h=float(h.max()) if len(h) else float("nan"),
                max_3d=float(d.max()) if len(d) else float("nan"),
                med_3d=float(np.median(d)) if len(d) else float("nan"))


# -------------------------------------------------------------------- main --
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom", required=True)
    ap.add_argument("--gnss", required=True)
    ap.add_argument("--pcd", required=True)
    ap.add_argument("--out", required=True, help="output prefix")
    ap.add_argument("--model", choices=["rigid", "yaw"], default="rigid")
    ap.add_argument("--block-s", type=float, default=10.0,
                    help="minimum contiguous fix-block length to use")
    ap.add_argument("--up", default=None,
                    help="world-frame up-vector as x,y,z (IMU-measured). "
                         "If given, the fit is checked against it: a "
                         "physically valid R must send it to +U.")
    args = ap.parse_args()

    md = []
    def say(s=""):
        print(s)
        md.append(s)

    say("## Stage 2 — trajectory/GNSS alignment (R + T)")
    say()

    # ---- load -------------------------------------------------------------
    t_lio, P_lio, Q_lio = load_odom(args.odom)
    t_g, lat, lon, alt, st, sig_h = load_gnss(args.gnss)
    x_pcd, y_pcd, z_pcd = read_pcd(args.pcd)
    say(f"- FAST-LIO poses: **{len(t_lio)}**  "
        f"({t_lio[0]:.3f} … {t_lio[-1]:.3f})")
    say(f"- GNSS fixes: **{len(t_g)}**, valid **{int((st >= 0).sum())}**")
    say(f"- Map points: **{len(x_pcd)}**  "
        f"(LIO extent {x_pcd.max()-x_pcd.min():.1f} × "
        f"{y_pcd.max()-y_pcd.min():.1f} × {z_pcd.max()-z_pcd.min():.1f} m)")
    say()

    # ---- ENU frame --------------------------------------------------------
    good = st >= 0
    if not good.any():
        raise SystemExit("no valid GNSS fixes")
    anchor_i = int(np.argmax(good))
    frame = geo.ENUFrame(lat[anchor_i], lon[anchor_i], alt[anchor_i])
    say(f"- ENU anchor: {frame}")
    say()

    blocks = fix_blocks(t_g, st, min_span=args.block_s)
    say(f"- usable fix blocks (>= {args.block_s:g} s, no >0.25 s gap): "
        f"{len(blocks)}")
    for k, (a, b) in enumerate(blocks):
        say(f"    {k + 1}. {t_g[a]:.1f} … {t_g[b - 1]:.1f} s "
            f"({t_g[b - 1] - t_g[a]:.1f} s, {b - a} fixes)")
    say()

    # ---- interpolate GNSS ENU at LIO stamps, block by block ---------------
    matched_t, matched_P, matched_E, matched_blk = [], [], [], []
    for k, (a, b) in enumerate(blocks):
        tg, eg = t_g[a:b], None
        enu = frame.to_enu(lat[a:b], lon[a:b], alt[a:b])
        m = (t_lio >= tg[0]) & (t_lio <= tg[-1])
        if not m.any():
            continue
        sel = t_lio[m]
        for axis in range(3):
            v = np.interp(sel, tg, enu[:, axis])
            if axis == 0:
                e = np.tile(np.nan, (len(sel), 3))
            e[:, axis] = v
        matched_t.append(sel)
        matched_P.append(P_lio[m])
        matched_E.append(e)
        matched_blk.append(np.full(len(sel), k))

    if not matched_t:
        raise SystemExit("no LIO timestamps fall inside a valid fix block")
    m_t = np.concatenate(matched_t)
    m_P = np.concatenate(matched_P)
    m_E = np.concatenate(matched_E)
    m_b = np.concatenate(matched_blk)
    order = np.argsort(m_t)
    m_t, m_P, m_E, m_b = m_t[order], m_P[order], m_E[order], m_b[order]

    say(f"- matched (LIO stamp inside a fix block): **{len(m_P)}** pairs "
        f"over {len(np.unique(m_b))} blocks")
    say(f"- matched span: {m_t[0]:.1f} … {m_t[-1]:.1f} s "
        f"({m_t[-1] - m_t[0]:.1f} s)")
    say()

    # ---- extents ----------------------------------------------------------
    enu_all = frame.to_enu(lat[good], lon[good], alt[good])
    say("| extent | LIO (map frame) | GNSS (ENU, valid fixes) |")
    say("|---|---|---|")
    lio_ext = [x_pcd.max() - x_pcd.min(),
               y_pcd.max() - y_pcd.min(),
               z_pcd.max() - z_pcd.min()]
    for i, nm in enumerate(["E ↔ x", "N ↔ y", "U ↔ z"]):
        say(f"| {nm} | {lio_ext[i]:.1f} m "
            f"| {enu_all[:, i].max() - enu_all[:, i].min():.1f} m |")
    say()

    # path-length comparison over the matched window (scale/drift sanity)
    li = np.linalg.norm(np.diff(m_P, axis=0), axis=1)
    gi = np.linalg.norm(np.diff(m_E, axis=0), axis=1)
    say(f"- path length over matched window — LIO **{li.sum():.1f} m** vs "
        f"GNSS **{gi.sum():.1f} m**  (ratio {li.sum() / max(gi.sum(), 1e-9):.3f})")
    say()

    # ---- fit --------------------------------------------------------------
    results = {}
    for name in ("rigid", "yaw"):
        if name == "yaw":
            R, t_vec, th = yaw_only(m_P, m_E)
        else:
            R, t_vec = umeyama(m_P, m_E)
        res = (m_P @ R.T + t_vec) - m_E
        results[name] = dict(R=R, t=t_vec, res=res, st=res_stats(res))

    say("### fit comparison (residual = estimate − GNSS, matched pairs)")
    say()
    say("| model | yaw | pitch | roll | RMS E | RMS N | RMS U | RMS horiz | RMS 3D | max 3D |")
    say("|---|---|---|---|---|---|---|---|---|---|")
    for name, r in results.items():
        y, p, ro = euler_zyx(r["R"])
        s = r["st"]
        say(f"| {name} | {y:+.3f}° | {p:+.3f}° | {ro:+.3f}° | "
            f"{s['rms_e']:.3f} | {s['rms_n']:.3f} | {s['rms_u']:.3f} | "
            f"{s['rms_h']:.3f} | {s['rms_3d']:.3f} | {s['max_3d']:.3f} |")
    say()
    say("(metres)")
    say()

    chosen = args.model
    R, t_vec = results[chosen]["R"], results[chosen]["t"]
    say(f"### chosen model: **{chosen}**")
    say()
    say("```")
    say(np.array2string(R, precision=8, suppress_small=True))
    say(f"t = {np.array2string(t_vec, precision=4)}")
    say("```")
    say()

    # Physical sanity: a rigid map between two frames must send "up" to "up".
    # Skipping this is how a mirrored cloud gets shipped (terrain relief
    # inverted, trees underground) while the trajectory still matches GNSS.
    if args.up:
        up = np.array([float(v) for v in args.up.split(",")])
        up = up / np.linalg.norm(up)
        mapped = R @ up
        ang = math.degrees(math.acos(max(-1.0, min(1.0, mapped[2]))))
        say("### physical sanity check (world-up -> ENU)")
        say()
        say(f"- world up-vector `{np.round(up, 5)}` -> `{np.round(mapped, 5)}`")
        say(f"- angle off ENU +U: **{ang:.2f}°**")
        if mapped[2] > 0:
            say("- **PASS** — up maps to up. Heights are not mirrored.")
        else:
            say("- **FAIL** — up maps to DOWN. The cloud would be mirrored "
                "vertically; do not use it.")
        say()
        if mapped[2] <= 0:
            raise SystemExit("REFUSING to export: R maps world-up to ENU down")

    # ---- per-block residuals (drift diagnosis) ----------------------------
    res = results[chosen]["res"]
    say("### residual per fix block (time-varying drift shows up here)")
    say()
    say("| block | t range (s) | n | RMS E | RMS N | RMS U | RMS 3D | mean U |")
    say("|---|---|---|---|---|---|---|---|")
    for k in sorted(np.unique(m_b)):
        m = m_b == k
        r = res[m]
        s = res_stats(r)
        say(f"| {k + 1} | {m_t[m][0]:.1f}–{m_t[m][-1]:.1f} | {s['n']} | "
            f"{s['rms_e']:.3f} | {s['rms_n']:.3f} | {s['rms_u']:.3f} | "
            f"{s['rms_3d']:.3f} | {np.mean(r[:, 2]):+.3f} |")
    say()

    # ---- write matched pairs ---------------------------------------------
    mp = f"{args.out}_matched.csv"
    with open(mp, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["stamp_sec", "block", "lio_x", "lio_y", "lio_z",
                    "enu_e", "enu_n", "enu_u", "res_e", "res_n", "res_u"])
        for i in range(len(m_t)):
            w.writerow([f"{m_t[i]:.6f}", int(m_b[i]),
                        *(f"{v:.4f}" for v in m_P[i]),
                        *(f"{v:.4f}" for v in m_E[i]),
                        *(f"{v:.4f}" for v in res[i])])
    say(f"- wrote `{mp}`")

    # ---- transform every map point ---------------------------------------
    pts = np.column_stack([x_pcd, y_pcd, z_pcd])
    enu_cloud = pts @ R.T + t_vec
    llh_cloud = frame.to_llh(enu_cloud)
    utm, epsg = geo.llh_to_utm(llh_cloud[:, 0], llh_cloud[:, 1],
                               llh_cloud[:, 2])
    say(f"- transformed {len(pts)} points -> ENU -> **EPSG:{epsg}**")
    say()

    cp = f"{args.out}_cloud_utm.csv"
    with open(cp, "w", newline="") as fh:
        fh.write("x,y,z\n")
        np.savetxt(fh, utm, fmt="%.3f,%.3f,%.3f", delimiter=",")
    say(f"- wrote `{cp}` ({os.path.getsize(cp) / 1e6:.1f} MB, {len(utm)} pts)")

    # ---- also emit the georeferenced PCD (self-contained product) ---------
    pp = f"{args.out}_cloud_geo.pcd"
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\n"
           "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
           "COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           "POINTS %d\nDATA binary\n" % (len(utm), len(utm)))
    with open(pp, "wb") as fh:
        fh.write(hdr.encode("ascii"))
        utm.astype("<f4").tofile(fh)
    say(f"- wrote `{pp}` ({os.path.getsize(pp) / 1e6:.1f} MB, "
        f"UTM {epsg}, float32)")

    # ---- trajectory for the record ---------------------------------------
    tp = f"{args.out}_trajectory_utm.csv"
    traj_enu = P_lio @ R.T + t_vec
    traj_llh = frame.to_llh(traj_enu)
    traj_utm, _ = geo.llh_to_utm(traj_llh[:, 0], traj_llh[:, 1],
                                 traj_llh[:, 2], epsg=epsg)
    with open(tp, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["stamp_sec", "easting", "northing", "height",
                    "qx", "qy", "qz", "qw"])
        for i in range(len(t_lio)):
            w.writerow([f"{t_lio[i]:.6f}",
                        *(f"{v:.4f}" for v in traj_utm[i]),
                        *(f"{v:.6f}" for v in Q_lio[i])])
    say(f"- wrote `{tp}` ({len(t_lio)} poses)")

    rep = f"{args.out}_align.md"
    with open(rep, "w") as fh:
        fh.write("\n".join(md) + "\n")
    print(f"\nreport -> {rep}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
