#!/usr/bin/env python3
"""Shared geodesy helpers — WGS84 LLH <-> ECEF <-> local ENU, plus projection.

Everything here is pure numpy except `utm_epsg()` / `to_utm()`, which use the
OSR bindings that arrived with gdal-bin (python3-gdal).

Why implement ECEF ourselves instead of calling pyproj: pyproj is not
installed and would need a pip bootstrap; the WGS84 closed forms are exact
(the geographic->geocentric direction is closed-form, and the inverse uses an
iterative method that converges to <1e-9 rad, i.e. well under a millimetre).
"""
import math

import numpy as np

# --- WGS84 ellipsoid -------------------------------------------------------
A = 6378137.0                      # semi-major axis (m)
F = 1.0 / 298.257223563            # flattening
E2 = F * (2.0 - F)                 # first eccentricity squared  = 6.69437999e-3
B = A * (1.0 - F)                  # semi-minor axis (m)


def llh_to_ecef(lat_deg, lon_deg, alt):
    """Geodetic (deg, deg, m) -> ECEF (m). Vectorised over the first axis.

    Exact closed form.
    """
    lat = np.radians(np.asarray(lat_deg, dtype=np.float64))
    lon = np.radians(np.asarray(lon_deg, dtype=np.float64))
    alt = np.asarray(alt, dtype=np.float64)
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    sin_lon, cos_lon = np.sin(lon), np.cos(lon)
    N = A / np.sqrt(1.0 - E2 * sin_lat * sin_lat)
    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1.0 - E2) + alt) * sin_lat
    return np.column_stack([x, y, z])


def ecef_to_llh(ecef):
    """ECEF (m) -> geodetic (deg, deg, m). Iterative; <1e-12 rad after ~5 iters."""
    xyz = np.atleast_2d(np.asarray(ecef, dtype=np.float64))
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    # Bowring's starting guess, then fixed-point refinement
    lat = np.arctan2(z, p * (1.0 - E2))
    for _ in range(8):
        sin_lat = np.sin(lat)
        N = A / np.sqrt(1.0 - E2 * sin_lat * sin_lat)
        lat_new = np.arctan2(z + E2 * N * sin_lat, p)
        if np.max(np.abs(lat_new - lat)) < 1e-14:
            lat = lat_new
            break
        lat = lat_new
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    N = A / np.sqrt(1.0 - E2 * sin_lat * sin_lat)
    alt = np.where(cos_lat > 1e-10,
                   p / np.maximum(cos_lat, 1e-300) - N,
                   np.abs(z) / np.maximum(np.abs(sin_lat), 1e-300) - N * (1.0 - E2))
    return np.column_stack([np.degrees(lat), np.degrees(lon), alt])


def enu_basis(lat_deg, lon_deg):
    """Rotation matrix taking ECEF-relative vectors into the local ENU frame
    at (lat, lon). Rows are the E, N, U unit vectors expressed in ECEF.

    East  = (-sin(lon),           cos(lon),          0        )
    North = (-sin(lat)cos(lon),  -sin(lat)sin(lon),  cos(lat) )
    Up    = ( cos(lat)cos(lon),   cos(lat)sin(lon),  sin(lat) )
    """
    lat = math.radians(float(lat_deg))
    lon = math.radians(float(lon_deg))
    sL, cL = math.sin(lat), math.cos(lat)
    sO, cO = math.sin(lon), math.cos(lon)
    return np.array([
        [    -sO,      cO,   0.0],      # EAST
        [-sL * cO, -sL * sO,  cL],      # NORTH
        [ cL * cO,  cL * sO,  sL],      # UP
    ])


class ENUFrame:
    """Local east-north-up frame anchored at a WGS84 position."""

    def __init__(self, lat_deg, lon_deg, alt):
        self.lat0 = float(lat_deg)
        self.lon0 = float(lon_deg)
        self.alt0 = float(alt)
        self.ecef0 = llh_to_ecef(self.lat0, self.lon0, self.alt0)
        self.R = enu_basis(self.lat0, self.lon0)   # ECEF -> ENU

    def to_enu(self, lat_deg, lon_deg, alt):
        ecef = llh_to_ecef(lat_deg, lon_deg, alt)
        return (ecef - self.ecef0) @ self.R.T

    def to_ecef(self, enu):
        enu = np.asarray(enu, dtype=np.float64)
        return self.ecef0 + enu @ self.R

    def to_llh(self, enu):
        return ecef_to_llh(self.to_ecef(enu))

    def __repr__(self):
        return (f"ENUFrame(anchor lat={self.lat0:.9f} lon={self.lon0:.9f} "
                f"alt={self.alt0:.3f})")


def utm_epsg(lat_deg, lon_deg):
    """EPSG code of the WGS84 UTM zone containing (lat, lon)."""
    zone = int(math.floor((float(lon_deg) + 180.0) / 6.0)) + 1
    if lat_deg >= 0:
        return 32600 + zone, zone
    return 32700 + zone, zone


def to_utm(ecef):
    """ECEF -> (Easting, Northing, height) in the natural UTM zone, via OSR."""
    from osgeo import osr
    src = osr.SpatialReference()
    src.ImportFromEPSG(4978)          # WGS84 geocentric
    src.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    lat, lon, _ = ecef_to_llh(ecef[:3])[0]
    epsg, _zone = utm_epsg(lat, lon)
    dst = osr.SpatialReference()
    dst.ImportFromEPSG(epsg)
    dst.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    ct = osr.CoordinateTransformation(src, dst)
    ecef = np.atleast_2d(np.asarray(ecef, dtype=np.float64))
    out = ct.TransformPoints([tuple(p) for p in ecef])
    return np.array([[p[0], p[1], p[2]] for p in out]), epsg


def llh_to_utm(lat_deg, lon_deg, alt, epsg=None):
    """Geodetic -> UTM via OSR (used for the GNSS track)."""
    from osgeo import osr
    lat = np.asarray(lat_deg, dtype=np.float64)
    lon = np.asarray(lon_deg, dtype=np.float64)
    alt = np.asarray(alt, dtype=np.float64)
    if epsg is None:
        epsg, _ = utm_epsg(float(np.mean(lat)), float(np.mean(lon)))
    src = osr.SpatialReference()
    src.ImportFromEPSG(4979)          # WGS84 geographic 3D
    src.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    dst = osr.SpatialReference()
    dst.ImportFromEPSG(epsg)
    dst.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    ct = osr.CoordinateTransformation(src, dst)
    pts = np.column_stack([lon, lat, alt])   # traditional GIS order = lon,lat,z
    out = ct.TransformPoints([tuple(p) for p in pts])
    return np.array([[p[0], p[1], p[2]] for p in out]), epsg
