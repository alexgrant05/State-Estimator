"""Minimal WGS84 conversions used by the receiver-neutral GNSS interface."""

from __future__ import annotations

import numpy as np

WGS84_A_M = 6_378_137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
    latitude = np.radians(latitude_deg)
    longitude = np.radians(longitude_deg)
    sin_lat = np.sin(latitude)
    radius = WGS84_A_M / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (radius + altitude_m) * np.cos(latitude) * np.cos(longitude),
            (radius + altitude_m) * np.cos(latitude) * np.sin(longitude),
            (radius * (1.0 - WGS84_E2) + altitude_m) * sin_lat,
        ],
        dtype=np.float64,
    )


def ecef_from_enu_rotation(latitude_deg: float, longitude_deg: float) -> np.ndarray:
    latitude = np.radians(latitude_deg)
    longitude = np.radians(longitude_deg)
    return np.array(
        [
            [-np.sin(longitude), -np.sin(latitude) * np.cos(longitude), np.cos(latitude) * np.cos(longitude)],
            [np.cos(longitude), -np.sin(latitude) * np.sin(longitude), np.cos(latitude) * np.sin(longitude)],
            [0.0, np.cos(latitude), np.sin(latitude)],
        ],
        dtype=np.float64,
    )


def enu_to_ecef(vector_enu: np.ndarray, latitude_deg: float, longitude_deg: float) -> np.ndarray:
    return ecef_from_enu_rotation(latitude_deg, longitude_deg) @ np.asarray(vector_enu, dtype=np.float64)


def ecef_to_enu(vector_ecef: np.ndarray, latitude_deg: float, longitude_deg: float) -> np.ndarray:
    return ecef_from_enu_rotation(latitude_deg, longitude_deg).T @ np.asarray(vector_ecef, dtype=np.float64)
