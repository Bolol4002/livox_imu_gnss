#!/usr/bin/env python3
"""Diagnose the planar degeneracy in the LIO->ENU rigid fit.

A trajectory driven on flat ground is essentially a PLANE, so the orthogonal
Procrustes/SVD solution is degenerate: mapping the trajectory's plane normal
to +U or to -U both fit the in-plane positions equally well.  SVD silently
returns one of them.  If it returns the -U branch, every height in the
georeferenced cloud is sign-inverted (terrain relief flips, trees point
underground) even though the trajectory itself still matches the GNSS track.

This script settles it three independent ways:

  1. What is "up" in the LIO world frame?  For a body-fixed up-vector u,
     R(q_i) @ u must be CONSTANT across the run (gravity is fixed in world).
     Compare constancy for u (IMU-measured) vs. plain [0,0,1].
  2. Fit A: unconstrained SVD (what align_r_t.py currently does).
  3. Fit B: CONSTRAINED so that world-up maps to +U, with only the in-plane
     yaw left free (solved in closed form).

If A and B have comparable residuals, the fit is degenerate and B must be
used because it is the physically correct branch.
"""
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geo
from align_r_t import fix_blocks, load_gnss, load_odom, res_stats, umeyama


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ])


def rot_between(a, b):
    """Minimal rotation taking unit vector `a` to unit vector `b`."""
    a = np.asarray(a, dtype=float); a = a / np.linalg.norm(a)
    b = np.asarray(b, dtype=float); b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def constrained_fit(P, E, up_world):
    """R, t with R @ up_world = e_U and the best in-plane yaw.

    Closed form: R0 maps up_world -> e_U, then R(psi) = Rz(psi) @ R0, which
    preserves that constraint because Rz fixes e_U.  With a = (R0 P)_E,
    b = (R0 P)_N and centred targets e, n the objective
        sum( (cos*a - sin*b - e)^2 + (sin*a + cos*b - n)^2 )
    reduces to const - 2[A cos + B sin], so
        psi* = atan2(B, A),  A = sum(e*a + n*b),  B = sum(n*a - e*b)
    """
    R0 = rot_between(up_world, np.array([0.0, 0.0, 1.0]))
    assert np.linalg.norm(R0 @ up_world - np.array([0, 0, 1.0])) < 1e-9
    Q = P @ R0.T
    # centre everything horizontally; vertical offset is handled by t
    a, b = Q[:, 0] - Q[:, 0].mean(), Q[:, 1] - Q[:, 1].mean()
    e, n = E[:, 0] - E[:, 0].mean(), E[:, 1] - E[:, 1].mean()
    A = float(np.sum(e * a + n * b))
    B = float(np.sum(n * a - e * b))
    psi = math.atan2(B, A)
    c, s = math.cos(psi), math.sin(psi)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    R = Rz @ R0
    t = E.mean(0) - R @ P.mean(0)
    return R, t, math.degrees(psi), R0


def main():
    odom_csv, gnss_csv = sys.argv[1], sys.argv[2]
    t, P, Q = load_odom(odom_csv)
    tg, lat, lon, alt, st, sig = load_gnss(gnss_csv)

    # ---- 1. what is "up" in the LIO world frame? -------------------------
    print("=" * 74)
    print("1. determining 'up' in the LIO world frame")
    print("=" * 74)
    u_sensor = np.array([0.3381, -0.0121, 0.9410])   # from IMU mean acc
    u_sensor /= np.linalg.norm(u_sensor)

    Rs = np.array([quat_to_R(*q) for q in Q])
    v_imu = np.einsum("nij,j->ni", Rs, u_sensor)      # body-up in world
    v_imu_T = np.einsum("nji,j->ni", Rs, u_sensor)    # if q were world->body
    v_z = np.einsum("nij,j->ni", Rs, np.array([0, 0, 1.0]))
    print(f"  poses: {len(P)}   (span {t[-1] - t[0]:.1f} s)")
    for nm, v in (("R(q) @ u_sensor   ", v_imu),
                  ("R(q)^T @ u_sensor ", v_imu_T),
                  ("R(q) @ [0,0,1]    ", v_z)):
        print(f"  {nm} mean {np.round(v.mean(0), 4)}  "
              f"std {np.round(v.std(0), 4)}  |std| {np.linalg.norm(v.std(0)):.4f}")
    # FAST-LIO initialises state.grav = -mean_acc/|mean_acc| * G in the WORLD
    # frame, and world == initial body frame, so gravity-up in world is exactly
    # the accelerometer direction measured while stationary at t=0.
    up_world = u_sensor.copy()
    print(f"\n  up_world (IMU, world == initial body frame) = {np.round(up_world, 5)}")
    print(f"  angle from LIO +z : {math.degrees(math.acos(np.clip(up_world[2], -1, 1))):.2f} deg")
    print(f"  (mean of R(q)@u over run = {np.round(v_imu.mean(0), 4)}; "
          f"its drift |std|={np.linalg.norm(v_imu.std(0)):.3f} is attitude drift)")

    # ---- matched pairs ---------------------------------------------------
    blocks = fix_blocks(tg, st, min_span=10.0)
    frame = geo.ENUFrame(lat[st >= 0][0], lon[st >= 0][0], alt[st >= 0][0])
    mP, mE = [], []
    for a, b in blocks:
        enu = frame.to_enu(lat[a:b], lon[a:b], alt[a:b])
        m = (t >= tg[a]) & (t <= tg[b - 1])
        if not m.any():
            continue
        sel = t[m]
        e = np.column_stack([np.interp(sel, tg[a:b], enu[:, k]) for k in range(3)])
        mP.append(P[m]); mE.append(e)
    mP, mE = np.vstack(mP), np.vstack(mE)
    print(f"\n  matched pairs: {len(mP)}")

    # ---- 2/3. the two candidate fits -------------------------------------
    print()
    print("=" * 74)
    print("2. comparing the two branches of the (degenerate) fit")
    print("=" * 74)
    rows = []
    for name in ("A: unconstrained SVD", "B: constrained up->+U"):
        if name.startswith("A"):
            R, t_vec = umeyama(mP, mE)
            psi = float("nan")
        else:
            R, t_vec, psi, _ = constrained_fit(mP, mE, up_world)
        res = (mP @ R.T + t_vec) - mE
        s = res_stats(res)
        map_up = R @ up_world
        rows.append((name, R, t_vec, s, psi, map_up))
        print(f"\n{name}")
        print(f"   R @ up_world        = {np.round(map_up, 4)}")
        print(f"   in-plane yaw offset = {psi:+.3f} deg")
        print(f"   RMS horiz {s['rms_h']:.4f} m | RMS U {s['rms_u']:.4f} m | "
              f"RMS 3D {s['rms_3d']:.4f} m | max3D {s['max_3d']:.3f} m")
        print(f"   det(R) = {np.linalg.det(R):+.6f}")

    d = abs(rows[0][3]['rms_3d'] - rows[1][3]['rms_3d'])
    a_ok = rows[0][5][2] > 0
    print()
    print("=" * 74)
    print("3. verdict")
    print("=" * 74)
    print(f"   residual gap between branches: {d:.4f} m")
    if d < 0.05:
        print("   -> branches are EQUIVALENT for positions => fit is degenerate")
        print("      (expected: a ground trajectory is a plane).")
    if not a_ok:
        print("   -> unconstrained branch maps world-up to ENU DOWN: WRONG.")
    else:
        print("   -> unconstrained branch maps world-up to +U: physically valid.")
    print(f"   -> residuals: unconstrained {rows[0][3]['rms_3d']:.3f} m vs "
          f"constrained {rows[1][3]['rms_3d']:.3f} m")
    if a_ok:
        print("   => use the UNCONSTRAINED fit: it is physically valid and fits")
        print("      best. (Forcing up to land on exactly e_U costs "
              f"{d:.2f} m,")
        print("      which is the ~1.8 deg residual tilt in the IMU up-vector.)")
    else:
        print("   => use the CONSTRAINED fit: it is the only physically valid one.")


if __name__ == "__main__":
    main()
