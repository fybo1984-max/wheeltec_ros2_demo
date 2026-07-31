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

from dataclasses import replace

import numpy as np
import pytest

from semantic_planning_experiments.synthetic_scene import SyntheticSceneSpec


def _scene() -> SyntheticSceneSpec:
    return SyntheticSceneSpec(
        width=64,
        height=48,
        depth_m=2.0,
        depth_noise_std_m=0.0,
        depth_invalid_fraction=0.0,
        random_seed=42,
        detection_class_id='person',
        person_count=1,
        center_x_fraction=0.5,
        center_y_fraction=0.5,
        person_spacing_y_fraction=0.2,
        bbox_width_fraction=0.25,
        bbox_height_fraction=0.25,
        detection_publish_every_n_frames=1,
        detection_timestamp_offset_sec=0.0,
    )


def test_scene_generates_symmetric_person_centers():
    scene = replace(_scene(), person_count=3)

    assert scene.person_center_y_fractions() == pytest.approx([0.3, 0.5, 0.7])
    np.testing.assert_allclose(
        scene.detection_centers_px(),
        [
            (32.0, 14.4),
            (32.0, 24.0),
            (32.0, 33.6),
        ],
    )


def test_seeded_noisy_depth_is_repeatable():
    scene = replace(_scene(), depth_noise_std_m=0.1)

    first = scene.depth_bytes()
    second = scene.depth_bytes()
    other_seed = replace(scene, random_seed=43).depth_bytes()

    assert first == second
    assert first != other_seed


def test_invalid_depth_fraction_produces_bounded_sparse_frame():
    scene = replace(_scene(), depth_invalid_fraction=0.5)

    depth = np.frombuffer(scene.depth_bytes(), dtype='<u2')

    invalid_fraction = np.count_nonzero(depth == 0) / depth.size
    assert 0.4 < invalid_fraction < 0.6
    assert np.all(depth[depth != 0] == 2000)


@pytest.mark.parametrize(
    'scene, message',
    [
        (replace(_scene(), person_count=0), 'person_count'),
        (
            replace(_scene(), depth_invalid_fraction=1.0),
            'depth_invalid_fraction',
        ),
        (
            replace(_scene(), center_y_fraction=0.95),
            'bounding boxes',
        ),
        (
            replace(_scene(), detection_publish_every_n_frames=0),
            'publish_every',
        ),
        (
            replace(_scene(), detection_class_id=' '),
            'detection_class_id',
        ),
    ],
)
def test_scene_rejects_invalid_injection_parameters(scene, message):
    with pytest.raises(ValueError, match=message):
        scene.validate()
