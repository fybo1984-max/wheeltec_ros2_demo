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

"""Map loading and path metrics for semantic planning experiments."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import yaml


@dataclass(frozen=True)
class MaskGrid:
    """Occupied semantic cells and their map geometry."""

    occupied: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    yaml_path: Path
    image_path: Path

    @property
    def height(self) -> int:
        """Return the mask height in cells."""
        return int(self.occupied.shape[0])

    @property
    def width(self) -> int:
        """Return the mask width in cells."""
        return int(self.occupied.shape[1])

    def contains(self, x: float, y: float) -> bool:
        """Return whether a world point lies in a semantic cell."""
        cell_x = math.floor((x - self.origin_x) / self.resolution)
        cell_y = math.floor((y - self.origin_y) / self.resolution)
        if cell_x < 0 or cell_y < 0:
            return False
        if cell_x >= self.width or cell_y >= self.height:
            return False
        image_row = self.height - 1 - cell_y
        return bool(self.occupied[image_row, cell_x])

    def occupied_centers(self) -> np.ndarray:
        """Return semantic cell centers as an N-by-2 world-coordinate array."""
        rows, columns = np.nonzero(self.occupied)
        if rows.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        x = self.origin_x + (columns.astype(np.float64) + 0.5) * self.resolution
        grid_y = self.height - 1 - rows
        y = self.origin_y + (grid_y.astype(np.float64) + 0.5) * self.resolution
        return np.column_stack((x, y))


def sha256_file(path: Path) -> str:
    """Return a file SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _read_p5(path: Path) -> np.ndarray:
    with path.open('rb') as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                raise ValueError(f'incomplete PGM header: {path}')
            tokens.extend(line.split(b'#', 1)[0].split())
        magic, width_token, height_token, maximum_token = tokens[:4]
        if magic != b'P5':
            raise ValueError(f'only binary P5 PGM is supported: {path}')
        width = int(width_token)
        height = int(height_token)
        maximum = int(maximum_token)
        if width <= 0 or height <= 0 or maximum <= 0 or maximum > 255:
            raise ValueError(f'invalid 8-bit PGM geometry: {path}')
        pixels = stream.read()
    expected = width * height
    if len(pixels) != expected:
        raise ValueError(
            f'PGM pixel count is {len(pixels)}, expected {expected}: {path}'
        )
    return np.frombuffer(pixels, dtype=np.uint8).reshape((height, width))


def load_mask_grid(yaml_path: Path) -> MaskGrid:
    """Load a Nav2 trinary map YAML as a semantic mask."""
    yaml_path = yaml_path.expanduser().resolve()
    metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    if not isinstance(metadata, dict):
        raise ValueError(f'map YAML must contain a mapping: {yaml_path}')
    if metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('semantic path metrics currently require trinary mode')

    image_path = Path(str(metadata['image']))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    pixels = _read_p5(image_path)

    resolution = float(metadata['resolution'])
    origin = metadata['origin']
    if resolution <= 0.0 or len(origin) < 3:
        raise ValueError(f'invalid map geometry: {yaml_path}')
    if abs(float(origin[2])) > 1.0e-9:
        raise ValueError('rotated semantic masks are not supported')

    normalized = pixels.astype(np.float64) / 255.0
    occupancy_probability = (
        normalized if int(metadata.get('negate', 0)) else 1.0 - normalized
    )
    occupied = occupancy_probability > float(metadata['occupied_thresh'])
    return MaskGrid(
        occupied=occupied,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        yaml_path=yaml_path,
        image_path=image_path,
    )


def _segment_samples(
    points: Sequence[tuple[float, float]],
    maximum_step: float,
) -> Iterable[tuple[float, float, float]]:
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length == 0.0:
            continue
        pieces = max(1, math.ceil(length / maximum_step))
        piece_length = length / pieces
        for index in range(pieces):
            fraction = (index + 0.5) / pieces
            yield (
                start[0] + fraction * dx,
                start[1] + fraction * dy,
                piece_length,
            )


def path_metrics(
    points: Sequence[tuple[float, float]],
    mask: MaskGrid,
) -> dict:
    """Measure path length, semantic crossing, and mask-boundary clearance."""
    if len(points) < 2:
        raise ValueError('a path needs at least two points')

    path_length = sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(points, points[1:])
    )
    samples = list(_segment_samples(points, mask.resolution * 0.5))
    crossing_length = sum(
        weight for x, y, weight in samples if mask.contains(x, y)
    )

    semantic_centers = mask.occupied_centers()
    if semantic_centers.size == 0:
        minimum_clearance = None
    elif crossing_length > 0.0:
        minimum_clearance = 0.0
    else:
        path_samples = np.asarray(
            [(x, y) for x, y, _ in samples] + list(points),
            dtype=np.float64,
        )
        minimum_squared = math.inf
        for offset in range(0, len(path_samples), 256):
            chunk = path_samples[offset:offset + 256]
            differences = chunk[:, np.newaxis, :] - semantic_centers[np.newaxis, :, :]
            squared = np.sum(differences * differences, axis=2)
            minimum_squared = min(minimum_squared, float(np.min(squared)))
        center_distance = math.sqrt(minimum_squared)
        half_diagonal = mask.resolution / math.sqrt(2.0)
        minimum_clearance = max(0.0, center_distance - half_diagonal)

    return {
        'path_length_m': path_length,
        'semantic_crossing_length_m': crossing_length,
        'semantic_crossing_ratio': (
            crossing_length / path_length if path_length > 0.0 else 0.0
        ),
        'minimum_semantic_clearance_m': minimum_clearance,
        'pose_count': len(points),
    }


def metric_delta(baseline: dict, semantic: dict) -> dict:
    """Return semantic-minus-baseline values for comparable metrics."""
    result = {}
    for key in (
        'path_length_m',
        'semantic_crossing_length_m',
        'semantic_crossing_ratio',
        'minimum_semantic_clearance_m',
    ):
        first = baseline.get(key)
        second = semantic.get(key)
        result[key] = None if first is None or second is None else second - first
    return result
