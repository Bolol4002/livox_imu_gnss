## Stage 2 — trajectory/GNSS alignment (R + T)

- FAST-LIO poses: **8632**  (1762269335.994 … 1762270211.522)
- GNSS fixes: **8768**, valid **3541**
- Map points: **995439**  (LIO extent 277.3 × 310.3 × 124.1 m)

- ENU anchor: ENUFrame(anchor lat=48.781994775 lon=9.172948618 alt=302.572)

- usable fix blocks (>= 10 s, no >0.25 s gap): 4
    1. 1762269335.6 … 1762269525.6 s (190.0 s, 1901 fixes)
    2. 1762269667.4 … 1762269751.0 s (83.6 s, 837 fixes)
    3. 1762269943.0 … 1762269990.0 s (47.0 s, 471 fixes)
    4. 1762269997.5 … 1762270030.5 s (33.0 s, 331 fixes)

- matched (LIO stamp inside a fix block): **3481** pairs over 4 blocks
- matched span: 1762269336.0 … 1762270030.5 s (694.5 s)

| extent | LIO (map frame) | GNSS (ENU, valid fixes) |
|---|---|---|
| E ↔ x | 277.3 m | 171.7 m |
| N ↔ y | 310.3 m | 91.4 m |
| U ↔ z | 124.1 m | 3.1 m |

- path length over matched window — LIO **337.8 m** vs GNSS **335.3 m**  (ratio 1.008)

### fit comparison (residual = estimate − GNSS, matched pairs)

| model | yaw | pitch | roll | RMS E | RMS N | RMS U | RMS horiz | RMS 3D | max 3D |
|---|---|---|---|---|---|---|---|---|---|
| rigid | -112.909° | -21.195° | -1.094° | 0.281 | 0.188 | 0.173 | 0.339 | 0.380 | 1.402 |
| yaw | -111.195° | -0.000° | +0.000° | 2.000 | 1.181 | 17.116 | 2.323 | 17.273 | 27.776 |

(metres)

### chosen model: **rigid**

```
[[-0.36293539  0.9182705   0.15829466]
 [-0.85881425 -0.39555353  0.32553877]
 [ 0.36154666 -0.01779617  0.93218416]]
t = [0.4221 0.0085 0.0208]
```

### physical sanity check (world-up -> ENU)

- world up-vector `[ 0.33811 -0.0121   0.94103]` -> `[0.01514 0.02075 0.99967]`
- angle off ENU +U: **1.47°**
- **PASS** — up maps to up. Heights are not mirrored.

### residual per fix block (time-varying drift shows up here)

| block | t range (s) | n | RMS E | RMS N | RMS U | RMS 3D | mean U |
|---|---|---|---|---|---|---|---|
| 1 | 1762269336.0–1762269525.5 | 1867 | 0.223 | 0.085 | 0.115 | 0.265 | -0.009 |
| 2 | 1762269667.4–1762269751.0 | 824 | 0.133 | 0.225 | 0.195 | 0.326 | +0.095 |
| 3 | 1762269943.0–1762269990.0 | 464 | 0.260 | 0.078 | 0.132 | 0.302 | +0.108 |
| 4 | 1762269997.6–1762270030.5 | 326 | 0.647 | 0.449 | 0.350 | 0.862 | -0.340 |

- wrote `stadtgarten_seq2_matched.csv`
- transformed 995439 points -> ENU -> **EPSG:32632**

- wrote `stadtgarten_seq2_cloud_utm.csv` (30.9 MB, 995439 pts)
- wrote `stadtgarten_seq2_cloud_geo.pcd` (11.9 MB, UTM 32632, float32)
- wrote `stadtgarten_seq2_trajectory_utm.csv` (8632 poses)
