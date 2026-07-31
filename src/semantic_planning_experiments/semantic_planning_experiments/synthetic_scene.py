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

"""Deterministic synthetic RGB-D scene helpers for paper experiments."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SyntheticSceneSpec:
    """Validated parameters for deterministic object detections and depth."""

    width: int
    height: int
    depth_m: float
    depth_noise_std_m: float
    depth_invalid_fraction: float
    random_seed: int
    detection_class_id: str
    person_count: int
    center_x_fraction: float
    center_y_fraction: float
    person_spacing_y_fraction: float
    bbox_width_fraction: float
    bbox_height_fraction: float
    detection_publish_every_n_frames: int
    detection_timestamp_offset_sec: float

    def validate(self) -> None:
        """Raise when a scene cannot produce bounded deterministic messages."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError('synthetic image dimensions must be positive')
        if not math.isfinite(self.depth_m) or self.depth_m <= 0.0:
            raise ValueError('synthetic depth must be positive')
        if (
            not math.isfinite(self.depth_noise_std_m)
            or self.depth_noise_std_m < 0.0
        ):
            raise ValueError('depth_noise_std_m must be nonnegative')
        if not 0.0 <= self.depth_invalid_fraction < 1.0:
            raise ValueError('depth_invalid_fraction must be in [0, 1)')
        if self.person_count < 1 or self.person_count > 20:
            raise ValueError('person_count must be in [1, 20]')
        if (
            not self.detection_class_id
            or self.detection_class_id != self.detection_class_id.strip()
        ):
            raise ValueError('detection_class_id must be a nonempty trimmed label')
        for name, value in (
            ('center_x_fraction', self.center_x_fraction),
            ('center_y_fraction', self.center_y_fraction),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} must be in [0, 1]')
        if (
            not math.isfinite(self.person_spacing_y_fraction)
            or self.person_spacing_y_fraction < 0.0
        ):
            raise ValueError(
                'person_spacing_y_fraction must be nonnegative'
            )
        for name, value in (
            ('bbox_width_fraction', self.bbox_width_fraction),
            ('bbox_height_fraction', self.bbox_height_fraction),
        ):
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f'{name} must be in (0, 1]')
        if self.detection_publish_every_n_frames < 1:
            raise ValueError(
                'detection_publish_every_n_frames must be positive'
            )
        if not math.isfinite(self.detection_timestamp_offset_sec):
            raise ValueError(
                'detection_timestamp_offset_sec must be finite'
            )
        centers = self.person_center_y_fractions()
        half_height = self.bbox_height_fraction * 0.5
        if any(
            center - half_height < 0.0 or center + half_height > 1.0
            for center in centers
        ):
            raise ValueError('person bounding boxes exceed image height')
        half_width = self.bbox_width_fraction * 0.5
        if (
            self.center_x_fraction - half_width < 0.0
            or self.center_x_fraction + half_width > 1.0
        ):
            raise ValueError('person bounding boxes exceed image width')

    def person_center_y_fractions(self) -> list[float]:
        """Return symmetric vertical centers for the requested person count."""
        midpoint = (self.person_count - 1) * 0.5
        return [
            self.center_y_fraction
            + (index - midpoint) * self.person_spacing_y_fraction
            for index in range(self.person_count)
        ]

    def detection_centers_px(self) -> list[tuple[float, float]]:
        """Return deterministic detection centers in image pixels."""
        self.validate()
        x = self.width * self.center_x_fraction
        return [
            (x, self.height * center_y)
            for center_y in self.person_center_y_fractions()
        ]

    def depth_bytes(self) -> bytes:
        """Return one seeded, fixed 16UC1 depth frame."""
        self.validate()
        random = np.random.default_rng(self.random_seed)
        depth = np.full(
            (self.height, self.width),
            self.depth_m,
            dtype=np.float64,
        )
        if self.depth_noise_std_m > 0.0:
            depth += random.normal(
                0.0,
                self.depth_noise_std_m,
                size=depth.shape,
            )
        depth = np.maximum(depth, 0.001)
        depth_mm = np.rint(depth * 1000.0).astype('<u2')
        if self.depth_invalid_fraction > 0.0:
            invalid = random.random(depth.shape) < self.depth_invalid_fraction
            depth_mm[invalid] = 0
        return depth_mm.tobytes()
