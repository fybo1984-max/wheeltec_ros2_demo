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

from pathlib import Path

import numpy as np
import pytest

from semantic_planning_experiments.metrics import (
    MaskGrid,
    load_mask_grid,
    metric_delta,
    occupancy_data,
    path_metrics,
    sha256_file,
)


def test_load_mask_grid_uses_nav2_image_coordinates(tmp_path: Path):
    image = tmp_path / 'mask.pgm'
    image.write_bytes(b'P5\n3 2\n255\n' + bytes([255, 0, 255, 255, 255, 255]))
    yaml_path = tmp_path / 'mask.yaml'
    yaml_path.write_text(
        '\n'.join([
            'image: mask.pgm',
            'mode: trinary',
            'resolution: 1.0',
            'origin: [10.0, 20.0, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.25',
        ]),
        encoding='utf-8',
    )

    mask = load_mask_grid(yaml_path)

    assert mask.contains(11.5, 21.5)
    assert not mask.contains(11.5, 20.5)
    assert not mask.contains(9.5, 21.5)


def test_path_metrics_distinguishes_crossing_and_detour(tmp_path: Path):
    occupied = np.zeros((4, 4), dtype=bool)
    occupied[1:3, 1:3] = True
    mask = MaskGrid(
        occupied=occupied,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        yaml_path=tmp_path / 'mask.yaml',
        image_path=tmp_path / 'mask.pgm',
    )

    crossing = path_metrics([(0.5, 2.5), (3.5, 2.5)], mask)
    detour = path_metrics(
        [(0.5, 2.5), (0.5, 0.25), (3.5, 0.25), (3.5, 2.5)],
        mask,
    )

    assert crossing['path_length_m'] == pytest.approx(3.0)
    assert crossing['semantic_crossing_length_m'] == pytest.approx(2.0)
    assert crossing['semantic_crossing_ratio'] == pytest.approx(2.0 / 3.0)
    assert crossing['minimum_semantic_clearance_m'] == 0.0
    assert detour['semantic_crossing_length_m'] == 0.0
    assert detour['minimum_semantic_clearance_m'] > 0.0


def test_occupancy_data_flips_image_rows_into_ros_grid_order(tmp_path: Path):
    occupied = np.asarray([
        [False, True, False],
        [True, False, False],
    ])
    mask = MaskGrid(
        occupied=occupied,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        yaml_path=tmp_path / 'mask.yaml',
        image_path=tmp_path / 'mask.pgm',
    )

    assert occupancy_data(mask) == [100, 0, 0, 0, 100, 0]


def test_empty_mask_has_no_clearance_value(tmp_path: Path):
    mask = MaskGrid(
        occupied=np.zeros((2, 2), dtype=bool),
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        yaml_path=tmp_path / 'mask.yaml',
        image_path=tmp_path / 'mask.pgm',
    )

    metrics = path_metrics([(0.0, 0.0), (1.0, 0.0)], mask)

    assert metrics['semantic_crossing_length_m'] == 0.0
    assert metrics['minimum_semantic_clearance_m'] is None


def test_metric_delta_handles_numeric_and_missing_values():
    baseline = {
        'path_length_m': 10.0,
        'semantic_crossing_length_m': 2.0,
        'semantic_crossing_ratio': 0.2,
        'minimum_semantic_clearance_m': None,
    }
    semantic = {
        'path_length_m': 12.0,
        'semantic_crossing_length_m': 0.0,
        'semantic_crossing_ratio': 0.0,
        'minimum_semantic_clearance_m': 0.5,
    }

    delta = metric_delta(baseline, semantic)

    assert delta['path_length_m'] == 2.0
    assert delta['semantic_crossing_length_m'] == -2.0
    assert delta['semantic_crossing_ratio'] == -0.2
    assert delta['minimum_semantic_clearance_m'] is None


def test_sha256_file(tmp_path: Path):
    value = tmp_path / 'value'
    value.write_bytes(b'abc')

    assert sha256_file(value) == (
        'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad'
    )
