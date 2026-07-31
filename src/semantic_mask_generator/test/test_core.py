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

import math

import numpy as np
import pytest

from semantic_mask_generator.core import decode_depth_image
from semantic_mask_generator.core import GridSpec
from semantic_mask_generator.core import median_depth
from semantic_mask_generator.core import ObservationStore
from semantic_mask_generator.core import project_pixel
from semantic_mask_generator.core import rasterize_observations
from semantic_mask_generator.core import RiskObservation
from semantic_mask_generator.core import RiskProfile
from semantic_mask_generator.core import scale_risk_profile
from semantic_mask_generator.core import transform_point


def test_decode_16_bit_depth_with_row_padding():
    rows = np.array(
        [
            [1000, 2000, 9999],
            [3000, 4000, 9999],
        ],
        dtype='<u2',
    )
    decoded = decode_depth_image(
        rows.tobytes(),
        '16UC1',
        width=2,
        height=2,
        step=6,
        is_bigendian=False,
    )
    np.testing.assert_allclose(decoded, [[1.0, 2.0], [3.0, 4.0]])


def test_decode_32_bit_depth():
    values = np.array([[1.25, 2.5]], dtype='<f4')
    decoded = decode_depth_image(
        values.tobytes(),
        '32FC1',
        width=2,
        height=1,
        step=8,
        is_bigendian=False,
    )
    np.testing.assert_allclose(decoded, values)


def test_decode_rejects_unsupported_encoding():
    with pytest.raises(ValueError, match='unsupported'):
        decode_depth_image(b'\x00', '8UC1', 1, 1, 1, False)


def test_median_depth_filters_zero_nan_and_out_of_range():
    depth = np.array(
        [
            [0.0, math.nan, 9.0],
            [1.0, 2.0, 3.0],
            [0.1, 4.0, math.inf],
        ],
        dtype=np.float32,
    )
    assert median_depth(depth, 1, 1, 1, 0.3, 8.0) == 2.5
    assert median_depth(depth, 20, 20, 1, 0.3, 8.0) is None


def test_pixel_projection_uses_camera_intrinsics():
    point = project_pixel(420.0, 290.0, 2.0, 500.0, 500.0, 320.0, 240.0)
    assert point == pytest.approx((0.4, 0.2, 2.0))

    with pytest.raises(ValueError, match='focal lengths'):
        project_pixel(1.0, 1.0, 2.0, math.nan, 500.0, 0.0, 0.0)


def test_quaternion_transform_rotates_then_translates():
    half_sqrt = math.sqrt(0.5)
    point = transform_point(
        (1.0, 0.0, 0.0),
        (2.0, 3.0, 0.0),
        (0.0, 0.0, half_sqrt, half_sqrt),
    )
    assert point == pytest.approx((2.0, 4.0, 0.0))


def test_observation_store_merges_nearby_same_class():
    store = ObservationStore(merge_distance_m=0.5)
    profile = RiskProfile(value=100, radius_m=1.0)
    store.add('person', 1.0, 1.0, profile, 10.0, 1.0, 1.0)
    store.add('person', 1.2, 1.0, profile, 10.5, 1.0, 1.0)
    active = store.active(10.5)
    assert len(active) == 1
    assert active[0].x == 1.2


def test_risk_profile_value_and_radius_scales_are_validated():
    profile = scale_risk_profile(
        RiskProfile(value=100, radius_m=1.2),
        1.5,
        0.65,
    )
    assert profile.value == 65
    assert profile.radius_m == pytest.approx(1.8)

    with pytest.raises(ValueError, match='scale'):
        scale_risk_profile(profile, 0.0)
    with pytest.raises(ValueError, match='value scale'):
        scale_risk_profile(profile, 1.0, 0.0)
    with pytest.raises(ValueError, match='risk value'):
        scale_risk_profile(
            RiskProfile(value=100, radius_m=1.2),
            1.0,
            1.01,
        )


def test_observation_holds_then_decays_and_expires():
    observation = RiskObservation(
        label='person',
        x=0.0,
        y=0.0,
        radius_m=1.0,
        risk_value=100,
        observed_at=0.0,
        decay_start=1.0,
        expires_at=3.0,
    )
    assert observation.value_at(0.5) == 100
    assert observation.value_at(2.0) == 50
    assert observation.value_at(3.0) == 0


def test_rasterization_uses_maximum_overlap_and_map_bounds():
    spec = GridSpec(10, 10, 1.0, 0.0, 0.0)
    observations = [
        RiskObservation('person', 5.0, 5.0, 2.0, 100, 0.0, 5.0, 6.0),
        RiskObservation('box', 5.0, 5.0, 1.0, 60, 0.0, 5.0, 6.0),
        RiskObservation('outside', -20.0, -20.0, 1.0, 90, 0.0, 5.0, 6.0),
    ]
    grid = rasterize_observations(spec, observations, now=1.0)
    assert grid.shape == (10, 10)
    assert grid[5, 5] == 100
    assert grid[0, 0] == 0
    assert np.count_nonzero(grid) == 13
