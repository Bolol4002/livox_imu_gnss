# livox_imu_gnss — LiDAR/IMU/GNSS georeferencing pipeline (FAST-LIO → UTM → DEM)

Offline pipeline that turns a single 16 GB rosbag recorded with a **Livox Mid-360**
(+ built-in IMU) and an external **GNSS receiver** into:

1. a **georeferenced point cloud** in UTM (EPSG:32632, Stuttgart area),
2. a **georeferenced FAST-LIO trajectory** (UTM + quaternion, absolute GNSS time),
3. a **bare-earth DEM** (0.5 m GeoTIFF, SMRF-ground + IDW gridding),
4. plus a set of **feasibility measurements** (FAST-LIO CPU/RAM, PDAL DEM timing,
   keep-up budget) that were used to judge whether the same loop can run onboard a
   Jetson Orin NX.

Everything is replay-based: no live sensor is needed. The only heavy input is
`stadtgarten_seq2.db3` (16 GB). All other files in this directory are derived
products that can be regenerated from the bag with the scripts in `tools/`.

The dataset was recorded on **foot** (mean speed 0.57 m/s), not in flight — see
[Limitations](#limitations-read-this-before-trusting-any-number).

---

## Quick orientation for AI agents (read this first)

If you are an agent picking this task up cold, this section is the whole context.
Details follow further down.

* **Goal of the repo**: reproduce and verify the chain
  `bag → FAST-LIO odometry → rigid R+T alignment to GNSS/ENU → UTM products → PDAL DEM → validation`.
* **Canonical working directory**: `/home/sayooj/Documents/livox_imu_gnss` (this dir).
  Paths are hard-coded in `tools/run_traj.sh`, `tools/dynamics_profile.py`,
  `tools/pdal_timing.py`, `tools/dem_budget.py` and `tools/gnss_stats.py`.
  Change the `DIR=` line in the shell scripts if you relocate the data.
* **Everything is regenerable except the bag.** `stadtgarten_seq2.db3` (16 GB,
  recorded 2025-11-04) is the one irreplaceable file. Never delete or overwrite it.
  Products are cheap to rebuild (~20 min of wall clock for the whole chain).
* **The map must come from the same FAST-LIO run as the trajectory.**
  `stadtgarten_seq2_map.pcd` (995,439 pts) and `stadtgarten_seq2_odom.csv`
  (8,632 poses) are a matched pair produced by `tools/run_traj.sh`. The `.bak`
  map (992,440 pts) is from an earlier, *different* run — do not mix it with the
  current odom CSV.
* **Time base**: all three sensors share one epoch. Verified offsets at t=0 are
  LiDAR 0.00 ms / GNSS +5.07 ms / IMU +37.69 ms. No leap-second handling is
  needed for this bag. If you ever see a ~18 s or ~31 s offset between FAST-LIO
  stamps and GNSS stamps, stop and re-run `tools/timebase_check.py` — matching
  across epochs silently produces garbage.
* **The alignment is a single global rigid transform (Umeyama, no scale).**
  It cannot absorb LIO drift; drift is *diagnosed*, not corrected, via
  per-fix-block residuals (`stadtgarten_seq2_align.md`).
* **The one non-obvious physics trap** in this project: a trajectory driven on
  flat ground is a *plane*, so the SVD/Procrustes fit is degenerate — it can map
  world-up to ENU-up or to ENU-down with almost identical residuals. Always pass
  `--up <imu-measured gravity dir>` to `tools/align_r_t.py`; it refuses to export
  a vertically mirrored cloud. See `tools/align_diag.py`.
* **Python environment trap**: the distro GDAL bindings (`python3-gdal`,
  `/usr/lib/python3/dist-packages/osgeo`) are compiled against NumPy 1.x, but
  `~/.local` has NumPy 2.2.6, which shadows it and crashes every
  `ReadAsArray()` call. Prefix GDAL-touching commands with `PYTHONNOUSERSITE=1`
  (or install `numpy<2`). This is the single most common error in this repo.
* **ROS is not auto-sourced.** Every `rosbag2_py`/`rclpy` script fails on import
  until `source /opt/ros/humble/setup.bash` (plus the two workspace installs) is
  done. `tools/run_traj.sh` and `tools/measure_fastlio_cpu.sh` do it for you.
* **Never stream the whole bag through Python naively** — 16 GB, 219,926 messages.
  Every bag-reading tool filters by topic and (where possible) stops early.
* **Status of the pipeline**: complete and validated end-to-end. DEM vs GNSS
  check passes (constant −0.75 m offset ⇒ implied antenna height 0.75 m above
  ground, std 0.56 m, no drift). No known broken step.

### Canonical commands (copy-paste)

```bash
cd /home/sayooj/Documents/livox_imu_gnss
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/fastlio_ws/install/setup.bash

# 1) validate bag                       (offline, minutes)
python3 tools/bag_stats.py stadtgarten_seq2.db3            > logs/bag_stats.out
python3 tools/zero_check.py stadtgarten_seq2.db3           > logs/zero_check.out
python3 tools/timebase_check.py stadtgarten_seq2.db3 gnss_fixes.csv

# 2) GNSS extraction + quality          (offline, ~1 min)
python3 tools/extract_gnss.py stadtgarten_seq2.db3 gnss_fixes.csv
python3 tools/gnss_quality.py gnss_fixes.csv gnss_quality.png

# 3) FAST-LIO replay + trajectory       (~16 min, starts ROS nodes)
bash tools/run_traj.sh

# 4) alignment / georeferencing         (offline, ~2 min)
python3 tools/imu_mount_check.py stadtgarten_seq2.db3 2.0
python3 tools/align_r_t.py --odom stadtgarten_seq2_odom.csv --gnss gnss_fixes.csv \
    --pcd stadtgarten_seq2_map.pcd --out stadtgarten_seq2 --model rigid \
    --up 0.33811,-0.01210,0.94103
python3 tools/align_diag.py stadtgarten_seq2_odom.csv gnss_fixes.csv

# 5) DEM                                (offline, ~15 s)
pdal pipeline dem/make_dem.json
PYTHONNOUSERSITE=1 python3 tools/validate_dem.py dem/stadtgarten_seq2_dem.tif gnss_fixes.csv

# 6) performance / budget               (~16 min for the CPU run)
bash tools/measure_fastlio_cpu.sh
python3 tools/dynamics_profile.py
python3 tools/pdal_timing.py
python3 tools/dem_budget.py --incremental
```

---

## Table of contents

1. [What the project does](#what-the-project-does)
2. [Environment and dependencies](#environment-and-dependencies)
3. [The dataset](#the-dataset)
4. [Pipeline, stage by stage](#pipeline-stage-by-stage)
5. [Tool reference — every script in `tools/`](#tool-reference--every-script-in-tools)
6. [File inventory](#file-inventory)
7. [Measured results](#measured-results)
8. [Errors and fixes](#errors-and-fixes)
9. [Limitations](#limitations-read-this-before-trusting-any-number)
10. [Rules for AI agents](#rules-for-ai-agents)

---

## What the project does

The hardware recorded a walk through a city garden (Stadtgarten, Stuttgart) with:

* a Livox Mid-360 LiDAR publishing `sensor_msgs/PointCloud2` at ~10 Hz,
* its built-in IMU at ~193 Hz on `/livox/imu`,
* a GNSS receiver publishing `sensor_msgs/NavSatFix` at ~10 Hz on `/gnss/fix`,
* a camera (present in the bag, not used here).

The LiDAR+IMU data is post-processed by **FAST-LIO** (ROS 2 Humble port,
`~/fastlio_ws/src/FAST_LIO_ROS2`) into an odometry trajectory in its own
arbitrary "world" frame (`camera_init`). To make the map useful for GIS/DEM work
it must be placed in a real-world frame. That is what this repo does:

```
stadtgarten_seq2.db3                       (16 GB, raw sensor bag)
        │
        ├── tools/bag_stats.py            validation: rates, gaps, zero-points
        ├── tools/zero_check.py           zero-xyz pattern, empty-frame timeline
        ├── tools/timebase_check.py       one time epoch? GNSS status mix?
        │
        ├── tools/extract_gnss.py ──────► gnss_fixes.csv        (8768 fixes)
        ├── tools/gnss_quality.py ──────► gnss_quality.png      (fix quality figs)
        │
        ├── tools/run_traj.sh             FAST-LIO replay
        │     └─ tools/record_odom.py ─► stadtgarten_seq2_odom.csv  (8632 poses)
        │        + /map_save service ──► stadtgarten_seq2_map.pcd   (995439 pts)
        │
        ├── tools/align_r_t.py            Umeyama R+T, LIO world → ENU → UTM
        │     └─► stadtgarten_seq2_align.md              (fit report + sanity check)
        │     └─► stadtgarten_seq2_matched.csv           (3481 LIO↔GNSS pairs)
        │     └─► stadtgarten_seq2_cloud_utm.csv         (995439 pts, for PDAL)
        │     └─► stadtgarten_seq2_cloud_geo.pcd         (same, binary, UTM)
        │     └─► stadtgarten_seq2_trajectory_utm.csv    (8632 poses, UTM+quat)
        │
        ├── pdal pipeline dem/make_dem.json
        │     └─► dem/stadtgarten_seq2_dem.tif           (0.5 m bare-earth DEM)
        │
        ├── tools/validate_dem.py         DEM vs GNSS altitude → PASS
        ├── tools/compare_dem.py          outlier-stage ablation
        └── tools/measure_fastlio_cpu.py / pdal_timing.py / dem_budget.py
                                          onboard-feasibility numbers
```

The `--up` physical check and the `align_diag.py` degeneracy analysis exist
because a naive Procrustes fit of a *planar* trajectory can produce a
vertically-mirrored cloud that still fits the GNSS track well — the classic way
to ship terrain relief inverted with trees pointing underground.

---

## Environment and dependencies

Verified on this machine (Ubuntu 22.04, x86_64, i5-1155G7 4C/8T).

| Component | Version / path | Needed by |
|---|---|---|
| ROS 2 | Humble (`/opt/ros/humble`) | all bag readers, FAST-LIO replay |
| `rosbag2_py`, `rclpy`, `rosidl_runtime_py` | from ROS Humble | `inspect_bag`, `bag_stats`, `zero_check`, `timebase_check`, `extract_gnss`, `gnss_stats`, `imu_mount_check`, `record_odom`, `count_topics`, `lag` |
| FAST-LIO ROS 2 port | `~/fastlio_ws/src/FAST_LIO_ROS2` (config `mid360.yaml`) | `run_traj.sh`, `measure_fastlio_cpu.sh` |
| Livox driver | `~/livox_ws/src/livox_ros_driver2` | workspace sourcing only (not needed to replay this bag) |
| PDAL | 2.3.0 | DEM pipelines, `pdal_timing.py` |
| GDAL CLI | 3.4.1 (`gdal-bin`) | hillshade / color-relief rendering |
| Python GDAL bindings | `python3-gdal` (`osgeo`), 3.4.1 | `geo.py`, `validate_dem.py`, `compare_dem.py` |
| NumPy | **must be < 2 for GDAL-touching scripts** (see [Errors](#errors-and-fixes)) | all |
| matplotlib | 3.10.9 (Agg backend, headless) | `gnss_quality.py`, `gnss_stats.py`, `pcd_preview.py` |

`tools/geo.py` deliberately implements WGS84 LLH↔ECEF↔ENU in pure NumPy instead of
using pyproj (not installed). It uses `osgeo.osr` only for the final
LLH→UTM projection.

ROS sourcing order used by the shell scripts:

```bash
source /opt/ros/humble/setup.bash
source /home/sayooj/livox_ws/install/setup.bash
source /home/sayooj/fastlio_ws/install/setup.bash
```

FAST-LIO config used (`~/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml`),
the parts that matter here:

```yaml
lid_topic:  "/livox/points"      # PointCloud2 input, lidar_type: 4 (Mid-360)
imu_topic:  "/livox/imu"
time_sync_en: false              # hardware sync assumed; bag proved it is fine
extrinsic_est_en: true
pcd_save: { pcd_save_en: true, interval: -1 }
map_file_path: "/home/sayooj/Documents/livox_imu_gnss/stadtgarten_seq2_map.pcd"
```

Note `map_file_path` — FAST-LIO writes the saved map straight into this directory
when `/map_save` is called.

---

## The dataset

`stadtgarten_seq2.db3` — rosbag2, sqlite3, uncompressed, 16.8 GB,
**219,926 messages over 934.67 s** (epoch `1762269335594932794` ns =
2025-11-04 15:15:35 UTC). Metadata in `metadata.yaml`.

| Topic | Type | Count | Rate | Notes |
|---|---|---|---|---|
| `/livox/points` | `sensor_msgs/PointCloud2` | 9,216 | ~10 Hz | 20,256 points/frame nominal; `offset_time` spans 0…99.81 ms; **573 frames are empty** (see below) |
| `/livox/lidar` | `livox_ros_driver/msg/CustomMsg` | 9,216 | ~10 Hz | same data in Livox native form; **not used** (FAST-LIO config takes PointCloud2) |
| `/livox/imu` | `sensor_msgs/Imu` | 175,191 | 193.3 Hz median | `|acc|` mean 9.798 m/s², std 0.0042 — sensor was essentially static at t=0 |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | 8,768 | ~10 Hz | 3,541 FIX (40.4%), 5,227 NO_FIX (59.6%) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 17,535 | ~19 Hz | unused |

Facts established by validation (see `logs/bag_stats.out`, `logs/zero_check.out`):

* LiDAR frame dt: median 101.28 ms, **0 non-monotonic stamps, 0 gaps > 150 ms**.
* IMU dt: median 5.17 ms, **0 non-monotonic stamps, 0 gaps > 10 ms**.
* LiDAR spans 934.67 s but IMU spans only 875.95 s — the IMU (and the useful
  part of the recording) **stops ~58.7 s before the LiDAR stream does**.
* All 573 empty `/livox/points` frames occur at t ≥ 876.7 s (minutes 14–15),
  i.e. exactly after the IMU stops. FAST-LIO logs `No point, skip this scan!`
  for them and processed **8,632 of 9,216** scans.
* ~50 % of the points in every frame are `(0,0,0)` padding (77 % in the first
  frames, dropping as the scene fills in). Non-zero range p50 ≈ 1.2 m,
  p95 ≈ 40 m. This is normal for Mid-360 PointCloud2 output; FAST-LIO's
  `blind: 0.5` preprocess handles it.

---

## Pipeline, stage by stage

### Stage 0 — inspect the bag without deserializing everything

```bash
python3 tools/inspect_bag.py stadtgarten_seq2.db3 /livox/points,/livox/imu,/gnss/fix 3
```

Prints the first N messages of the selected topics: PointCloud2 field layout
(offsets, datatypes, per-point `offset_time` monotonicity, `line` ids), IMU
values and covariances, NavSatFix status/covariance. This is how the field
layout assumptions used by `bag_stats.py` and `zero_check.py` were pinned down.

### Stage 1 — full-bag validation

```bash
python3 tools/bag_stats.py    stadtgarten_seq2.db3 > logs/bag_stats.out
python3 tools/zero_check.py   stadtgarten_seq2.db3 > logs/zero_check.out
python3 tools/timebase_check.py stadtgarten_seq2.db3 gnss_fixes.csv
```

Answers, before any processing money is spent:

* do LiDAR/IMU/GNSS share one time epoch? → **yes** (0 / +5.07 / +37.69 ms);
* are stamps monotonic, what is the real rate, where are the gaps? → clean;
* is the GNSS NO_FIX stretch contiguous (sky occlusion) or interleaved? →
  59.6 % NO_FIX overall, but concentrated in blocks;
* what does a point actually look like (zeros, ranges, intensities)?

`timebase_check.py` needs `gnss_fixes.csv` for the status histogram, so run
Stage 2 first if you are starting from a fresh bag.

### Stage 2 — GNSS extraction and quality characterisation

```bash
python3 tools/extract_gnss.py stadtgarten_seq2.db3 gnss_fixes.csv
python3 tools/gnss_quality.py gnss_fixes.csv gnss_quality.png
python3 tools/gnss_stats.py   stadtgarten_seq2.db3          # writes gnss_track.png
```

Why offline: the fixes are already in the bag; replaying them through ROS adds
nothing but QoS/queue-drop risk. `extract_gnss.py` reads only `/gnss/fix` and
skips the 16 GB of clouds.

`gnss_fixes.csv` columns: `stamp_ns, stamp_sec, latitude, longitude, altitude,
status, cov_e, cov_n, cov_u` (cov_* = sqrt of the diagonal = 1σ metres in ENU).

`gnss_quality.py` writes a 4-panel figure (track colored FIX/NO_FIX, status vs
time, reported covariance on log scale, altitude) and prints a per-60 s table
plus the FIX-period jitter (high-pass residual) and whether NO_FIX coordinates
are frozen or still tracking.

### Stage 3 — FAST-LIO replay and absolute-time trajectory

```bash
bash tools/run_traj.sh        # ~16 min, ends with "TRAJ_RUN_DONE"
```

What it does, in order:

1. sources ROS + both workspaces;
2. launches `fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false`
   into `logs/traj_fastlio.log`, waits up to 60 s for `/laser_mapping`;
3. starts `tools/record_odom.py` writing `stadtgarten_seq2_odom.csv`
   (columns `stamp_ns, stamp_sec, x, y, z, qx, qy, qz, qw`, frame `camera_init`),
   and aborts if the recorder did not come up;
4. plays the bag with `ros2 bag play --topics /livox/points /livox/imu /gnss/fix
   --rate 1.0 --read-ahead-queue-size 400`;
5. after playback: 3 s settle, SIGINT the recorder (flushes, prints
   `RECORDER_DONE rows=8632`);
6. calls `/map_save` (`std_srvs/srv/Trigger`) → FAST-LIO writes
   `stadtgarten_seq2_map.pcd` to the path in `mid360.yaml`;
7. SIGINTs the launch, `pkill`s leftovers, reports the row count.

**Why a full re-run instead of reusing an earlier map:** laserMapping's own
`mat_pre.txt` stores only *relative* time and Euler angles, so its rows cannot be
matched to GNSS fix stamps, and its map would come from a different run than the
trajectory. Replaying once and recording `/Odometry` gives absolute
`header.stamp` (same epoch as `/gnss/fix`) plus full quaternions, and the
`/map_save` map is guaranteed to be from that same run.

Expected end state: 8,632 trajectory rows, 995,439-point map,
`logs/traj_mapsave.log` showing `success=True, message='Map saved.'`.

### Stage 4 — rigid alignment to GNSS (LIO world → ENU → UTM)

First, measure what "up" is in the sensor frame from the static IMU segment:

```bash
python3 tools/imu_mount_check.py stadtgarten_seq2.db3 2.0
# -> mean specific force [+3.3130 -0.1190 +9.2202] m/s^2 (|.| = 9.7981)
#    unit up = [+0.3381 -0.0121 +0.9410]
#    -> sensor +z points UP (nominal mounting), tilted 19.76 deg in x, -0.70 deg in y
```

Then the fit:

```bash
python3 tools/align_r_t.py \
    --odom stadtgarten_seq2_odom.csv \
    --gnss gnss_fixes.csv \
    --pcd  stadtgarten_seq2_map.pcd \
    --out  stadtgarten_seq2 \
    --model rigid \
    --up   0.33811,-0.01210,0.94103
```

What happens inside:

1. Load odometry (8,632 poses), GNSS fixes (8,768; 3,541 valid), map (995,439 pts).
2. Build a local ENU frame anchored at the first valid fix
   (`48.781994775 N, 9.172948618 E, 302.572 m`).
3. Cut the GNSS track into **usable fix blocks**: runs of `status >= 0` with no
   gap > 0.25 s and ≥ 10 s long → 4 blocks for this bag.
4. Interpolate GNSS ENU at the LIO stamps inside each block → 3,481 matched pairs.
5. Fit two models and compare: full 6-DOF **rigid** (Umeyama, no scale) and a
   4-DOF **yaw-only** model.
6. Apply `--up`: the world-frame up-vector must map to ENU +U. If it maps to −U
   the script **exits without writing products** (mirrored cloud).
7. Report per-block residuals (this is where time-varying LIO drift shows up).
8. Write the products:
   * `stadtgarten_seq2_matched.csv` — matched pairs + per-pair residual,
   * `stadtgarten_seq2_cloud_utm.csv` — every map point in EPSG:32632 (for PDAL),
   * `stadtgarten_seq2_cloud_geo.pcd` — same, binary float32 PCD,
   * `stadtgarten_seq2_trajectory_utm.csv` — all 8,632 poses in UTM + quaternion,
   * `stadtgarten_seq2_align.md` — the full report (also echoed to stdout).

Result for this bag: rigid RMS 3D **0.380 m** (yaw-only: 17.27 m, unusable),
yaw −112.9°, pitch −21.2°, roll −1.1°, up-maps-to-up angle **1.47° → PASS**,
LIO/GNSS path-length ratio 1.008 over the matched window.

Diagnose the degeneracy explicitly:

```bash
python3 tools/align_diag.py stadtgarten_seq2_odom.csv gnss_fixes.csv
```

Shows that the unconstrained SVD branch is physically valid here (world-up →
+U) and that forcing the constraint costs only ~the 1.8° residual IMU tilt.
For a *flatter* trajectory, or if the fit had chosen the −U branch, this is the
tool that tells you so.

### Stage 5 — previews

```bash
python3 tools/pcd_preview.py stadtgarten_seq2_map.pcd stadtgarten_seq2_map_preview.png
```

Top-down (XY, colored by Z) + side view (XZ, colored by Y) PNG. Useful sanity
check that the cloud is not mirrored/scaled wrong before spending time on PDAL.

### Stage 6 — DEM

```bash
pdal pipeline dem/make_dem.json                 # -> dem/stadtgarten_seq2_dem.tif
pdal pipeline dem/make_dem_nooutlier.json       # -> dem/stadtgarten_seq2_dem_nooutlier.tif

# renders (parameters as produced; regenerate freely)
gdaldem hillshade    dem/stadtgarten_seq2_dem.tif dem/hillshade.tif
gdaldem color-relief dem/stadtgarten_seq2_dem.tif dem/ramp.txt dem/elev_color.tif
gdal_translate -of PNG dem/hillshade.tif    dem/stadtgarten_seq2_hillshade.png
gdal_translate -of PNG dem/elev_color.tif   dem/stadtgarten_seq2_elevation.png
```

Pipeline stages: `readers.text` (the UTM CSV, `override_srs: EPSG:32632`) →
`filters.outlier` (statistical, mean_k 10, multiplier 2.0, tagged class 7) →
`filters.smrf` (cell 1.0, slope 0.3, scalar 1.0, threshold 0.5, only over
non-outliers) → `writers.gdal` (0.5 m cell, radius 1.5 m, IDW, **only
Classification == 2**, i.e. ground, nodata −9999).

Validate against GNSS — this is the independent check, because GNSS set the R+T
but constrains nothing about the surface:

```bash
PYTHONNOUSERSITE=1 python3 tools/validate_dem.py dem/stadtgarten_seq2_dem.tif gnss_fixes.csv
```

Since the antenna rides a fixed height above ground, `DEM(lat,lon) − GNSS_alt`
must be constant. Measured: **mean −0.746 m, std 0.564 m, max |offset| 2.1 m,
constant across 5 time windows → PASS** (implied sensor height 0.75 m above the
ground surface; note the DEM is bare earth, so this is the antenna-above-ground
number as mapped by SMRF, not the physical pole height).

Ablate the expensive outlier stage:

```bash
PYTHONNOUSERSITE=1 python3 tools/compare_dem.py logs/cmp_ref.tif logs/cmp_noout.tif
```

(`cmp_ref.tif` = with outlier filter, `cmp_noout.tif` = without, both built with
identical explicit `bounds` so the grids match — see [Errors](#errors-and-fixes).)

### Stage 7 — onboard-feasibility measurements

```bash
bash tools/measure_fastlio_cpu.sh     # ~16 min: FAST-LIO only, no map save, no recorder
python3 tools/dynamics_profile.py     # speed / yaw-rate percentiles of this dataset
python3 tools/pdal_timing.py          # per-stage + scaling timing of the DEM pipeline
python3 tools/dem_budget.py --incremental   # keep-up budget from those timings
```

`measure_fastlio_cpu.sh` deliberately measures the *odometry* footprint only
(that is what must run in flight); map export and R+T are post-flight steps.

---

## Tool reference — every script in `tools/`

All Python scripts are Python 3 / ROS 2 Humble. "needs ROS" means
`source /opt/ros/humble/setup.bash` first. "needs PYGDAL fix" means prefix with
`PYTHONNOUSERSITE=1` (or have numpy < 2).

### Bag inspection & validation

| Script | Needs | Usage | What it does / outputs |
|---|---|---|---|
| `inspect_bag.py` | ROS | `inspect_bag.py BAG topic1,topic2 [N]` | Dumps first N msgs of selected topics with raw field layout (PointCloud2 fields/offsets/datatypes, per-point values, `offset_time` monotonicity, `line` ids; IMU values+cov; NavSatFix status/cov). Foundational: everything else's parsing assumptions were checked here. |
| `bag_stats.py` | ROS | `bag_stats.py BAG` | Full pass over `/livox/points` + `/livox/imu`: frame count, empty frames, widths, `offset_time` span, zero-xyz fraction, dt histogram/monotonicity/gaps, IMU rate and `|acc|` stats, LiDAR↔IMU alignment. Output saved in `logs/bag_stats.out`. |
| `zero_check.py` | ROS | `zero_check.py BAG` | Deep dive on the (0,0,0) padding: run-length pattern of zero vs valid points in the first 3 frames, range percentiles, intensity range, plus a timeline of empty frames bucketed per minute. Output in `logs/zero_check.out`. |
| `timebase_check.py` | ROS | `timebase_check.py BAG gnss_fixes.csv` | Compares first `header.stamp` and first receive time of `/livox/points`, `/livox/imu`, `/gnss/fix` and their offsets (leap-second/epoch check). Stops after the first message of each (does not stream 16 GB). Also prints the NavSatFix.status histogram from the CSV. |
| `count_topics.py` | ROS | `count_topics.py [DUR_S]` | Live-rate counter over `/livox/points`, `/livox/imu`, `/Odometry`, `/cloud_registered` with RELIABLE QoS depth 200. For live/replay runs, not offline. |
| `lag.py` | ROS | `lag.py` | 8 s probe: message counts and the `points_stamp − odom_stamp` lag between `/livox/points` and `/Odometry`. |

### GNSS

| Script | Needs | Usage | Outputs |
|---|---|---|---|
| `extract_gnss.py` | ROS | `extract_gnss.py BAG_DIR out.csv` | `gnss_fixes.csv` — all 8,768 fixes with `stamp_ns, stamp_sec, lat, lon, alt, status, cov_e, cov_n, cov_u`; prints FIX/SBAS count and median σ_E. |
| `gnss_quality.py` | numpy/mpl | `gnss_quality.py fixes.csv [out.png]` | Console: FIX/NO_FIX split, status transition count, per-60 s table (fix counts, σ_h/σ_v, E/N/U ranges), FIX-period jitter (2-sample predictor residual), median step during NO_FIX (freeze vs still-tracking). Figure: `gnss_quality.png` (track, status vs time, covariance, altitude). |
| `gnss_stats.py` | ROS + mpl | `gnss_stats.py BAG` | Console: E/N/altitude ranges and spans, σ stats, 2D path length, start→end displacement, altitude profile. Figure: **hard-coded output path** `.../livox_imu_gnss/gnss_track.png` colored by altitude. |

### FAST-LIO replay / odometry

| Script | Needs | Usage | Outputs |
|---|---|---|---|
| `run_traj.sh` | ROS | `bash tools/run_traj.sh` | Orchestrates the whole Stage 3 run: launch FAST-LIO → start recorder → play bag → SIGINT recorder → `/map_save` → cleanup. Logs in `logs/traj_*.log`. Produces `stadtgarten_seq2_odom.csv` + `stadtgarten_seq2_map.pcd`. |
| `record_odom.py` | ROS | `record_odom.py out.csv` | Subscribes `/Odometry` (reliable, depth 200) and streams rows `stamp_ns, stamp_sec, x, y, z, qx, qy, qz, qw`. Flushes every 200 rows; prints `RECORDER_DONE rows=N` on SIGINT. Called by `run_traj.sh`. |
| `measure_fastlio_cpu.sh` | ROS | `bash tools/measure_fastlio_cpu.sh` | Wrapper: launch FAST-LIO, wait for the node, run the sampler + replay, then shut down and print FAST-LIO's own `STATS` lines. |
| `measure_fastlio_cpu.py` | ROS | (run via the .sh) | Samples `/proc/<pid>/{stat,status}` of `fastlio_mapping` once per second during replay; drives `ros2 bag play` itself. Writes `tools/fastlio_cpu.csv` (t, utime, stime, rss) and `tools/fastlio_cpu_summary.txt` (mean/steady/thirds CPU %, core-sec/s, RSS start/peak/end). Has a `--wait-only` probe mode. |

### Alignment / georeferencing

| Script | Needs | Usage | Outputs |
|---|---|---|---|
| `geo.py` | numpy (+osgeo for UTM) | library | WGS84 constants, `llh_to_ecef`, `ecef_to_llh` (Bowring + fixed-point, <1e-12 rad), `enu_basis`, `ENUFrame` (to_enu/to_ecef/to_llh), `utm_epsg`, `to_utm`, `llh_to_utm` (via `osgeo.osr`, EPSG 4978/4979 → UTM). Imported by `align_r_t.py`, `align_diag.py`, `validate_dem.py`. |
| `align_r_t.py` | numpy + geo | see Stage 4 | The core Stage-2 tool. Writes `<out>_matched.csv`, `<out>_cloud_utm.csv`, `<out>_cloud_geo.pcd`, `<out>_trajectory_utm.csv`, `<out>_align.md`. Flags: `--model rigid|yaw`, `--block-s` (min fix-block length, default 10), `--up x,y,z` (enables the up→+U sanity check and refuses to export on failure). Reads **binary PCD only**. |
| `align_diag.py` | numpy + geo | `align_diag.py odom.csv gnss.csv` | Diagnoses the planar degeneracy: computes what "up" is in the LIO world frame (IMU-measured, and via `R(q)@u` constancy over the run), then compares the unconstrained SVD fit against a constrained `up → +U` fit (closed-form yaw) and prints a verdict on which branch to use. |
| `imu_mount_check.py` | ROS + numpy | `imu_mount_check.py BAG [WINDOW_S]` | Averages the first ~2 s of `/livox/imu` specific force to recover the sensor's up-direction and mounting tilt; prints whether +z points up / down / sideways. Source of the `--up` vector. |

### DEM

| Script | Needs | Usage | Outputs |
|---|---|---|---|
| `dem/make_dem.json` | PDAL | `pdal pipeline dem/make_dem.json` | `dem/stadtgarten_seq2_dem.tif` — outlier→SMRF→IDW ground grid at 0.5 m. |
| `dem/make_dem_nooutlier.json` | PDAL | `pdal pipeline dem/make_dem_nooutlier.json` | Same without `filters.outlier` (ablation baseline). |
| `validate_dem.py` | numpy + osgeo (**needs PYGDAL fix**) | `validate_dem.py DEM.tif gnss_fixes.csv` | Samples the DEM at every valid GNSS fix, reports `DEM − GNSS altitude` (mean/median/std/min/max), the implied sensor height, per-window drift, and a PASS/SUSPECT verdict. |
| `compare_dem.py` | numpy + osgeo (**needs PYGDAL fix**) | `compare_dem.py REF.tif TEST.tif` | Cell-by-cell difference over cells valid in both: mean/std/p50/p95/p99/max, threshold histogram, and how many cells change validity. **Requires identical grid shape** — build both with the same explicit `bounds` if needed. |
| `pdal_timing.py` | PDAL | `python3 tools/pdal_timing.py` | Times the DEM pipeline two ways: cumulative stage breakdown (read / +outlier / +smrf / +gdal) on the full cloud, and total time on uniform-stride subsets (25/50/100 % — stride, not truncation, so spatial extent is preserved). Writes `tools/pdal_timing.csv`; temp files under `logs/pdal_timing/`. |
| `dem_budget.py` | stdlib | `dem_budget.py [--slowdown 3.0] [--cadence 60,30,10] [--incremental]` | Fits `cost(N) = c0 + b·N` to the timing rows, combines with the map growth rate (1,137 pts/s) and an assumed CPU slowdown, and reports for how long a refresh cadence can keep up. Writes `tools/dem_budget.txt` / `tools/dem_budget_inc.txt`. |

### Visualisation / misc

| Script | Needs | Usage | Outputs |
|---|---|---|---|
| `pcd_preview.py` | numpy + mpl | `pcd_preview.py in.pcd out.png` | Top-down XY (colored by Z) and side XZ (colored by Y) render of a binary PCD. |
| `dynamics_profile.py` | stdlib | `python3 tools/dynamics_profile.py` | Reads `stadtgarten_seq2_odom.csv` only (no bag access, safe to run concurrently) and prints duration, path length, mean speed, and median/p95/p99/max of translation speed and rotation rate. Documents that this dataset is walking pace. |

`tools/__pycache__/` is a build artifact of `geo.py` — safe to delete.
`tools/pdal_timing.csv`, `tools/fastlio_cpu.csv`, `tools/fastlio_cpu_summary.txt`,
`tools/dem_budget*.txt` are outputs of the Stage 7 scripts (kept for reference).

---

## File inventory

| File | Size | What it is |
|---|---|---|
| `stadtgarten_seq2.db3` | 16 GB | **The raw bag.** Irreplaceable. |
| `metadata.yaml` | 2 KB | rosbag2 manifest (topics, counts, epoch). |
| `stadtgarten_seq2_odom.csv` | 950 KB | 8,632 FAST-LIO poses (absolute stamps + quaternion), `camera_init` frame. |
| `stadtgarten_seq2_map.pcd` | 31 MB | 995,439-pt map from the **same run** as the odom CSV (fields x y z intensity nx ny nz curvature). |
| `stadtgarten_seq2_map.pcd.bak` | 31 MB | 992,440-pt map from an **earlier run**. Backup only — do not pair with the current odom CSV. |
| `gnss_fixes.csv` | 860 KB | All 8,768 NavSatFix messages as CSV. |
| `gnss_track.png` / `gnss_quality.png` | — | GNSS track (colored by altitude) / 4-panel quality figure. |
| `stadtgarten_seq2_matched.csv` | 320 KB | 3,481 matched LIO↔GNSS pairs with per-pair ENU residuals and block id. |
| `stadtgarten_seq2_cloud_utm.csv` | 31 MB | 995,439 map points as `x,y,z` in EPSG:32632 — PDAL input. |
| `stadtgarten_seq2_cloud_geo.pcd` | 12 MB | Same points, binary float32 PCD, UTM 32632. |
| `stadtgarten_seq2_trajectory_utm.csv` | 768 KB | 8,632 poses in UTM (`stamp_sec, easting, northing, height, qx..qw`). |
| `stadtgarten_seq2_align.md` | 2.5 KB | The alignment report (fit tables, sanity check, per-block residuals). |
| `stadtgarten_seq2_map_preview.png` | 420 KB | Top/side preview of the LIO map. |
| `dem/stadtgarten_seq2_dem.tif` | 780 KB | 0.5 m bare-earth DEM, 515×380, EPSG:32632, nodata −9999. |
| `dem/stadtgarten_seq2_dem_nooutlier.tif` | 1.4 MB | Ablation DEM without `filters.outlier` (689×527, extends further). |
| `dem/hillshade.tif`, `dem/elev_color.tif`, `dem/ramp.txt` | — | Hillshade, color-relief and its color ramp. |
| `dem/stadtgarten_seq2_{hillshade,elevation}.png` | — | PNG renders of the two above. |
| `tools/*` | — | Scripts + their outputs (see above). |
| `logs/*` | 86 MB | Run logs, validation outputs, comparison rasters, PDAL timing scratch. |

---

## Measured results

**Bag**: 934.67 s, 219,926 msgs. LiDAR 9,216 frames @ ~9.9 Hz, IMU 175,191 @ 193.3 Hz,
GNSS 8,768 @ ~9.4 Hz (40.4 % FIX).

**FAST-LIO**: 8,632 / 9,216 scans processed; the rest are the empty post-876.7 s frames.
CPU **0.349 core-seconds per second of flight** (34.9 % of one core, stable across
thirds: 35.6 / 39.7 / 29.6 %), RSS peak 407 MB, growth +219 MB over the run
(`tools/fastlio_cpu_summary.txt`).

**Dynamics of this dataset** (i.e. what the above was measured on — walking):

| quantity | median | p95 | p99 | max |
|---|---|---|---|---|
| speed (m/s) | 0.145 | 1.491 | 1.663 | 1.969 |
| rotation rate (deg/s) | 9.24 | 64.10 | 103.25 | 215.90 |

Path length 498.7 m over 875.5 s (mean 0.570 m/s), pose interval 101.3 ms.

**Alignment**: 3,481 matched pairs over 4 usable fix blocks.

| model | yaw | pitch | roll | RMS horiz | RMS 3D | max 3D |
|---|---|---|---|---|---|---|
| rigid (chosen) | −112.909° | −21.195° | −1.094° | 0.339 m | **0.380 m** | 1.402 m |
| yaw-only | −111.195° | 0° | 0° | 2.323 m | 17.273 m | 27.776 m |

World-up → ENU angle 1.47° (PASS). LIO vs GNSS path length over the matched
window: 337.8 m vs 335.3 m (ratio 1.008 — no scale error).
Per-block RMS 3D: 0.265 / 0.326 / 0.302 / **0.862 m** (block 4 degrades; that is
drift, not fit error).

**DEM**: 515×380 @ 0.5 m, EPSG:32632, nodata −9999. Validated against 3,541 fixes
(all inside the extent, all hitting real cells):

```
DEM − GNSS altitude : mean −0.746 m, median −1.019 m, std 0.564 m, [−2.099, +0.287]
per-window means    : −0.717 / −0.990 / −0.783 / – / −0.599 m   (no drift)
verdict             : PASS
```

**Outlier-stage ablation** (`logs/cmp_ref.tif` vs `logs/cmp_noout.tif`, identical
grids): valid-cell coverage 26.71 % → 37.07 % without the filter; over the 51,825
common cells the p50 difference is **9 mm**, p95 2.77 m, p99 5.74 m, max 8.87 m;
13.2 % of cells differ by > 0.5 m. Interpretation: the filter mostly removes
canopy/edge cells (hence the coverage change), while the true ground surface it
keeps is nearly identical.

**PDAL timing** (this machine, 995,439 pts):

| stage | cumulative | incremental |
|---|---|---|
| read | 0.967 s | 0.967 s |
| + filters.outlier | 8.878 s | **7.911 s (74 % of total)** |
| + filters.smrf | 9.099 s | 0.221 s |
| + writers.gdal | 10.612 s | 1.513 s |

Scaling with point count (extent-preserving stride): 248,860 pts → 2.70 s,
497,720 → 4.97 s, 995,439 → 10.65 s (≈ linear, 10.75 µs/point).

**Keep-up budget** (map growth 1,137 pts/s, assumed 3.0× CPU slowdown — an
estimate, not measured):

| cadence | full rebuild | incremental (outlier on new pts only) |
|---|---|---|
| 60 s | 27.5 min | **105.9 min** |
| 30 s | 13.8 min | 53.3 min |
| 10 s | 4.7 min | 18.3 min |

Conclusion captured from these numbers: a *full* rebuild cannot cover a long
flight at a tight cadence, but an **incremental** loop (only new points through
the kNN outlier filter) comfortably can, because that stage is 74 % of the cost.

---

## Errors and fixes

Each entry: symptom → cause → fix. All were hit for real during this project.

1. **`AttributeError: _ARRAY_API not found` / `ImportError: numpy.core.multiarray failed to import` in `ReadAsArray()`**
   *Where*: `validate_dem.py`, `compare_dem.py` (any `osgeo.gdal` array read).
   *Cause*: `python3-gdal` in `/usr/lib/python3/dist-packages` is compiled against
   NumPy 1.x, but a user-site NumPy 2.2.6 in `~/.local/lib/python3.10/site-packages`
   takes import priority.
   *Fix*: `PYTHONNOUSERSITE=1 python3 tools/validate_dem.py ...` (uses the distro
   NumPy 1.21.5), or `pip install --user 'numpy<2'`. Note the pure-NumPy tools
   (`align_r_t.py`, `align_diag.py`, `pcd_preview.py`, `gnss_quality.py`) work
   fine with NumPy 2 — only the GDAL array path breaks. Everything else about
   GDAL (`gdal.Open`, geotransforms, `osr`) still works; only `ReadAsArray` is
   affected.

2. **`ModuleNotFoundError: rosbag2_py` (or `rclpy`)**
   *Where*: every bag-reading script.
   *Cause*: ROS 2 not sourced in the shell.
   *Fix*: `source /opt/ros/humble/setup.bash` (plus `~/livox_ws` and
   `~/fastlio_ws/install/setup.bash` if you need the driver/FAST-LIO).
   Use the sourcing block at the top of `tools/run_traj.sh`.

3. **`No point, skip this scan!` warnings flooding the FAST-LIO log**
   *Where*: `logs/run*.log`, `logs/traj_fastlio.log`.
   *Cause*: 573 `/livox/points` frames have `width == 0`. They are not corruption —
   they all occur after t ≥ 876.7 s, i.e. after the IMU stream ends at 875.95 s;
   the LiDAR driver kept publishing empty frames until the bag ended.
   *Fix*: none needed. FAST-LIO skips them harmlessly (8,632 of 9,216 processed).
   If you re-time the run, expect the useful data to end ~876 s.

4. **`rclpy.executors.ExternalShutdownException` + `RCLError: rcl_shutdown already called` traceback at the end of `record_odom.py`**
   *Where*: `logs/traj_recorder.log`.
   *Cause*: SIGINT delivered to rclpy raises `ExternalShutdownException` from
   `spin()`, which is not a `KeyboardInterrupt`, so the `except KeyboardInterrupt`
   in `main()` does not catch it; the subsequent `rclpy.shutdown()` then fails
   because the context is already down.
   *Fix*: cosmetic only — `RECORDER_DONE rows=8632` is printed (and the file
   flushed) *before* the traceback, and `run_traj.sh` ignores the exit code.
   If you want it clean, also catch `ExternalShutdownException` in `main()`.

5. **A mirrored (vertically flipped) georeferenced cloud**
   *Where*: conceptual trap in `align_r_t.py` / `align_diag.py`.
   *Cause*: the trajectory on flat ground is a plane, so the orthogonal
   Procrustes/SVD solution is degenerate — mapping the plane normal to +U or −U
   fits in-plane positions almost equally well, and SVD returns one silently.
   A mirrored cloud still matches the GNSS track.
   *Fix*: pass `--up <imu-measured up>` (from `tools/imu_mount_check.py`) so the
   fit is checked and the export is refused on failure. For a deeper analysis use
   `tools/align_diag.py`, which fits both branches and compares residuals.

6. **`compare_dem.py: shape mismatch: (380, 515) vs (527, 689)`**
   *Where*: comparing `dem/stadtgarten_seq2_dem.tif` against
   `dem/stadtgarten_seq2_dem_nooutlier.tif`.
   *Cause*: `writers.gdal` derives the grid extent from the surviving points, and
   the two pipelines keep different points (the outlier filter trims the fringe),
   so the rasters cover different areas.
   *Fix*: build both variants with the same explicit `"bounds"` — see
   `logs/cmp_ref.json` and `logs/cmp_noout.json` (they pin
   `([512496.955, 512754.455], [5403089.639, 5403279.639])`) — then compare
   `logs/cmp_ref.tif` vs `logs/cmp_noout.tif`.

7. **`ros2 bag play` drops / no odometry recorded (QoS experiments)**
   *Where*: `logs/run1_qos_besteffort_depth5.log`,
   `logs/run2_qos_besteffort_depth50.log`, `logs/run3_final_reliable.log`.
   *Cause*: the `/Odometry` subscriber QoS must match what FAST-LIO publishes
   (reliable, KeepLast(20)); a shallow best-effort subscription loses messages
   when the publisher floods during fast motion/scan bursts.
   *Fix*: subscribe with **RELIABLE** and a generous depth (200) — this is what
   `tools/record_odom.py` uses (`QOS = 200`) and what the final run did.

8. **Empty trajectory / FAST-LIO receives nothing**
   *Where*: `logs/probe_fastlio.log` (STATS all zero).
   *Cause*: playback started before the node was up, or topics not remapped; the
   FAST-LIO config expects `/livox/points` + `/livox/imu` exactly.
   *Fix*: `run_traj.sh` polls `ros2 node list` for `/laser_mapping` (up to 60 s)
   before playing, and plays only
   `--topics /livox/points /livox/imu /gnss/fix`. Verify with
   `grep STATS logs/traj_fastlio.log` — `cb_scan`/`cb_imu`/`processed` should be
   climbing.

9. **`livox_ros_driver/msg/CustomMsg` unavailable when subscribing to `/livox/lidar`**
   *Where*: any tool subscribing to the raw Livox topic.
   *Cause*: the bag stores the CustomMsg type name from the original driver
   (`livox_ros_driver`, v1); this machine only has `livox_ros_driver2` built.
   *Fix*: don't subscribe to `/livox/lidar`. `run_traj.sh` plays only
   `/livox/points` (plain `PointCloud2`), which needs no Livox package, and
   FAST-LIO is configured with `lidar_type: 4` (Mid-360 PointCloud2).

10. **`pdal pipeline` writes a DEM with holes / unexpected extent**
    *Where*: `dem/make_dem.json`.
    *Cause*: `writers.gdal` only gridded `Classification == 2` (ground), and the
    radius (1.5 m) plus the outlier filter leave the fringe unclassified.
    *Fix*: expected behaviour; if more coverage is wanted, drop/loosen
    `filters.outlier (see the ablation: coverage 26.7 % → 37.1 %) or increase
    `radius`. Never widen the grid with `bounds` unless you also want the
    comparison in [Errors #6](#errors-and-fixes) to work.

11. **`pkill` killing the script that is running it**
    *Where*: cleanup step of `tools/run_traj.sh`.
    *Cause*: a `pkill -f fastlio_mapping` pattern matches the shell command line
    itself (which contains that string).
    *Fix*: the bracket trick used in the script:
    `pkill -f 'fastlio[_]mapping'` (and `pkill -x fastlio_mapping` in
    `measure_fastlio_cpu.sh`, exact-commission match).

12. **Everything works but the numbers look wrong at the end of the bag**
    *Where*: any per-timestamp analysis.
    *Cause*: the last ~58 s contain LiDAR frames without IMU and GNSS gaps;
    FAST-LIO produces no poses there, and the GNSS track has NO_FIX stretches.
    *Fix*: rely on `align_r_t.py`'s fix-block logic (only LIO stamps inside a
    valid fix block are used) rather than naive global interpolation. The
    matched window is 1762269336.0 … 1762270030.5 s (694.5 s), not the full bag.

---

## Limitations (read this before trusting any number)

* **Walking dataset.** Mean speed 0.57 m/s, p99 1.66 m/s. The CPU and DEM
  timings are therefore *not* worst-case for survey flight; the map growth rate
  (1,137 pts/s) is an upper bound only because revisit passes add few points to
  an already-voxelized map, not because the motion was aggressive. Record a
  flight-speed bag before concluding anything about onboard feasibility.
* **Single global rigid transform.** LIO drift is not compensated; it shows up
  as per-block residuals (0.27 → 0.86 m RMS across the four blocks, worst in the
  last). A pose-graph / piecewise alignment would be needed for larger areas.
* **Only 40.4 % of GNSS fixes are valid**, in 4 blocks. Any extension of this
  pipeline must expect large NO_FIX stretches and use the same block logic.
* **Vertical accuracy is GNSS-limited.** The DEM-vs-GNSS std of 0.56 m is
  dominated by the receiver's vertical error, not by the LIO map.
* **The 3× CPU slowdown in `dem_budget.py` is an estimate**, explicitly labelled
  as such in the output. Do not quote it as measured.
* **Hard-coded absolute paths** (`/home/sayooj/...`) exist in `run_traj.sh`,
  `measure_fastlio_cpu.sh`, `measure_fastlio_cpu.py`, `dynamics_profile.py`,
  `pdal_timing.py`, `dem_budget.py` (`mid360.yaml` `map_file_path`) and
  `gnss_stats.py` (output path of `gnss_track.png`). Relocating the project
  means editing those.

---

## Rules for AI agents

* **Do not delete or overwrite** `stadtgarten_seq2.db3`, `metadata.yaml`, or any
  `stadtgarten_seq2_*` product without explicit user confirmation — the bag is
  16 GB and cannot be re-recorded from here.
* **Regenerate, don't patch.** If a product looks wrong, re-run the stage that
  produced it. Products are cheap; the bag is not. Do not hand-edit CSV/PCD/TIF
  outputs.
* **Never mix runs.** `stadtgarten_seq2_map.pcd` + `stadtgarten_seq2_odom.csv` are
  a matched pair from one replay. If you re-run `tools/run_traj.sh`, both are
  replaced together and the whole Stage 4+ must be redone (the R+T changes).
* **Always pass `--up`** to `align_r_t.py` (value from `imu_mount_check.py`), and
  check the sanity section of the resulting `_align.md` before using any
  georeferenced product. A report saying FAIL means the cloud is mirrored.
* **Prefix GDAL array work with `PYTHONNOUSERSITE=1`** (numpy 2 shadowing issue,
  [Errors #1](#errors-and-fixes)).
* **Source ROS before anything touching a bag**, and filter bag reads by topic;
  never deserialize all 219,926 messages.
* **Treat `logs/` as scratch.** Safe to clean, except `bag_stats.out`,
  `zero_check.out`, `cmp_*.tif/json` and `pdal_timing/` which document past
  validation runs referenced by this README.
* **A full `run_traj.sh` takes ~16 minutes** (it plays the bag at rate 1.0); the
  CPU measurement script also takes ~16 minutes. Budget for that, and do not
  start a second FAST-LIO replay concurrently — they write the same map file.
* **Verification bar for any change**: after modifying a script, the chain
  `align_r_t.py … --up …` must still report PASS on up-mapping and an RMS 3D in
  the same ballpark (≈0.38 m), and `validate_dem.py` must still return a
  near-constant offset (std < 1 m).
* **If a number in this README disagrees with a fresh run**, trust the fresh run
  and update the README — but check first that you did not accidentally use
  `stadtgarten_seq2_map.pcd.bak` or an unfiltered NumPy/GDAL mix.
# livox_imu_gnss
