# Copyright 2026 WHEELTEC innovation workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Deterministic projection, observation, and rasterization helpers."""

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    """Axis-aligned output grid metadata."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def validate(self) -> None:
        """Raise when the grid cannot be rasterized safely."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError('grid width and height must be positive')
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError('grid resolution must be positive')
        if not math.isfinite(self.origin_x) or not math.isfinite(self.origin_y):
            raise ValueError('grid origin must be finite')


@dataclass(frozen=True)
class RiskProfile:
    """Risk value and physical radius for a semantic class."""

    value: int
    radius_m: float

    def validate(self) -> None:
        """Raise when a risk profile is outside supported bounds."""
        if self.value < 1 or self.value > 100:
            raise ValueError('risk value must be in [1, 100]')
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError('risk radius must be positive')


def scale_risk_profile(
    profile: RiskProfile,
    radius_scale: float,
    value_scale: float = 1.0,
) -> RiskProfile:
    """Return a validated profile with reproducible value/radius multipliers."""
    profile.validate()
    if not math.isfinite(radius_scale) or radius_scale <= 0.0:
        raise ValueError('risk radius scale must be positive')
    if not math.isfinite(value_scale) or value_scale <= 0.0:
        raise ValueError('risk value scale must be positive')
    scaled = RiskProfile(
        int(round(profile.value * value_scale)),
        profile.radius_m * radius_scale,
    )
    scaled.validate()
    return scaled


@dataclass
class RiskObservation:
    """Map-frame semantic observation with deterministic expiration."""

    label: str
    x: float
    y: float
    radius_m: float
    risk_value: int
    observed_at: float
    decay_start: float
    expires_at: float

    def value_at(self, now: float) -> int:
        """Return held or linearly decayed risk at the requested time."""
        if now >= self.expires_at:
            return 0
        if now <= self.decay_start:
            return self.risk_value
        decay_duration = self.expires_at - self.decay_start
        if decay_duration <= 0.0:
            return 0
        remaining = (self.expires_at - now) / decay_duration
        return max(1, int(round(self.risk_value * remaining)))


class ObservationStore:
    """Bounded spatially merged collection of transient observations."""

    def __init__(self, merge_distance_m: float, max_observations: int = 200):
        if merge_distance_m < 0.0:
            raise ValueError('merge distance must be nonnegative')
        if max_observations <= 0:
            raise ValueError('max observations must be positive')
        self._merge_distance_m = merge_distance_m
        self._max_observations = max_observations
        self._observations: list[RiskObservation] = []

    def add(
        self,
        label: str,
        x: float,
        y: float,
        profile: RiskProfile,
        now: float,
        hold_sec: float,
        decay_sec: float,
    ) -> None:
        """Insert or refresh a nearby observation of the same class."""
        profile.validate()
        if hold_sec < 0.0 or decay_sec <= 0.0:
            raise ValueError('hold must be nonnegative and decay positive')
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(now):
            raise ValueError('observation position and time must be finite')
        candidate = RiskObservation(
            label=label,
            x=x,
            y=y,
            radius_m=profile.radius_m,
            risk_value=profile.value,
            observed_at=now,
            decay_start=now + hold_sec,
            expires_at=now + hold_sec + decay_sec,
        )
        merge_limit_sq = self._merge_distance_m ** 2
        nearest_index = None
        nearest_distance_sq = math.inf
        for index, observation in enumerate(self._observations):
            if observation.label != label:
                continue
            distance_sq = (
                (observation.x - x) ** 2 + (observation.y - y) ** 2
            )
            if distance_sq <= merge_limit_sq and distance_sq < nearest_distance_sq:
                nearest_index = index
                nearest_distance_sq = distance_sq

        if nearest_index is None:
            self._observations.append(candidate)
        else:
            self._observations[nearest_index] = candidate
        self._observations.sort(key=lambda item: item.observed_at, reverse=True)
        del self._observations[self._max_observations:]

    def active(self, now: float) -> list[RiskObservation]:
        """Prune expired entries and return a snapshot of active entries."""
        self._observations = [
            item for item in self._observations if item.expires_at > now
        ]
        return list(self._observations)


def decode_depth_image(
    data: object,
    encoding: str,
    width: int,
    height: int,
    step: int,
    is_bigendian: bool,
) -> np.ndarray:
    """Decode ROS depth bytes into a meter-valued float array."""
    normalized_encoding = encoding.upper()
    if normalized_encoding in ('16UC1', 'MONO16'):
        byte_order = '>' if is_bigendian else '<'
        dtype = np.dtype(byte_order + 'u2')
        scale = 0.001
    elif normalized_encoding == '32FC1':
        byte_order = '>' if is_bigendian else '<'
        dtype = np.dtype(byte_order + 'f4')
        scale = 1.0
    else:
        raise ValueError(f'unsupported depth encoding: {encoding}')

    if width <= 0 or height <= 0:
        raise ValueError('depth dimensions must be positive')
    minimum_step = width * dtype.itemsize
    if step < minimum_step:
        raise ValueError('depth row step is smaller than image width')
    if len(data) < step * height:
        raise ValueError('depth data is shorter than declared dimensions')

    view = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=data,
        strides=(step, dtype.itemsize),
    )
    return view.astype(np.float32) * scale


def median_depth(
    depth_m: np.ndarray,
    u: float,
    v: float,
    window_radius: int,
    minimum_m: float,
    maximum_m: float,
) -> float | None:
    """Return the robust local depth around one image coordinate."""
    if depth_m.ndim != 2:
        raise ValueError('depth image must be two-dimensional')
    if window_radius < 0:
        raise ValueError('window radius must be nonnegative')
    center_x = int(round(u))
    center_y = int(round(v))
    if (
        center_x < 0
        or center_y < 0
        or center_x >= depth_m.shape[1]
        or center_y >= depth_m.shape[0]
    ):
        return None
    min_x = max(0, center_x - window_radius)
    max_x = min(depth_m.shape[1], center_x + window_radius + 1)
    min_y = max(0, center_y - window_radius)
    max_y = min(depth_m.shape[0], center_y + window_radius + 1)
    samples = depth_m[min_y:max_y, min_x:max_x]
    valid = samples[
        np.isfinite(samples)
        & (samples >= minimum_m)
        & (samples <= maximum_m)
    ]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def project_pixel(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float, float]:
    """Project a pixel into the optical camera coordinate frame."""
    if (
        not all(math.isfinite(value) for value in (fx, fy, cx, cy))
        or fx <= 0.0
        or fy <= 0.0
    ):
        raise ValueError('camera focal lengths must be positive')
    if not math.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError('depth must be positive')
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return x, y, depth_m


def transform_point(
    point: tuple[float, float, float],
    translation: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Apply a normalized quaternion transform to a three-dimensional point."""
    qx, qy, qz, qw = rotation
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError('transform quaternion must be nonzero')
    qx, qy, qz, qw = (value / norm for value in rotation)
    px, py, pz = point

    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)
    rotated_x = px + qw * tx + (qy * tz - qz * ty)
    rotated_y = py + qw * ty + (qz * tx - qx * tz)
    rotated_z = pz + qw * tz + (qx * ty - qy * tx)
    return (
        rotated_x + translation[0],
        rotated_y + translation[1],
        rotated_z + translation[2],
    )


def rasterize_observations(
    spec: GridSpec,
    observations: Iterable[RiskObservation],
    now: float,
) -> np.ndarray:
    """Rasterize circular observations with maximum-risk overlap."""
    spec.validate()
    grid = np.zeros((spec.height, spec.width), dtype=np.uint8)
    for observation in observations:
        risk_value = min(100, observation.value_at(now))
        if risk_value <= 0 or observation.radius_m <= 0.0:
            continue
        center_x = (observation.x - spec.origin_x) / spec.resolution
        center_y = (observation.y - spec.origin_y) / spec.resolution
        radius_cells = observation.radius_m / spec.resolution
        min_x = max(0, int(math.floor(center_x - radius_cells)))
        max_x = min(spec.width - 1, int(math.ceil(center_x + radius_cells)))
        min_y = max(0, int(math.floor(center_y - radius_cells)))
        max_y = min(spec.height - 1, int(math.ceil(center_y + radius_cells)))
        if min_x > max_x or min_y > max_y:
            continue

        y_indices, x_indices = np.ogrid[min_y:max_y + 1, min_x:max_x + 1]
        inside = (
            (x_indices - center_x) ** 2 + (y_indices - center_y) ** 2
            <= radius_cells ** 2
        )
        region = grid[min_y:max_y + 1, min_x:max_x + 1]
        region[inside] = np.maximum(region[inside], risk_value)
    return grid
