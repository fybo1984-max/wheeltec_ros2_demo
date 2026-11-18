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

import pytest

from semantic_planning_experiments.latency_audit import (
    associate_events,
    summarize_events,
)


def _event(stage, receipt_sec, header_stamp=None):
    return {
        'stage': stage,
        'receipt_ros_ns': int(receipt_sec * 1e9),
        'receipt_monotonic_ns': int(receipt_sec * 1e9),
        'header_stamp_ns': header_stamp,
    }


def test_image_detection_requires_matching_source_header():
    events = [
        _event('image', 10.00, 100),
        _event('image', 10.10, 200),
        _event('detection', 10.25, 100),
        _event('detection', 10.30, 999),
    ]

    gaps = associate_events(
        events,
        'image',
        'detection',
        'matching_header_stamp',
        1.0,
    )

    assert gaps == pytest.approx([0.25])


def test_nearest_preceding_association_rejects_future_and_stale_events():
    events = [
        _event('mask', 5.0),
        _event('mask', 9.8),
        _event('costmap', 9.9),
        _event('costmap', 4.0),
        _event('costmap', 12.0),
    ]

    gaps = associate_events(
        events,
        'mask',
        'costmap',
        'nearest_preceding_receipt',
        1.0,
    )

    assert gaps == pytest.approx([0.1])


def test_receipt_gap_uses_monotonic_clock_when_ros_time_jumps():
    source = _event('mask', 5.0)
    target = _event('costmap', 5.2)
    source['receipt_ros_ns'] = 50_000_000_000
    target['receipt_ros_ns'] = 10_000_000_000

    gaps = associate_events(
        [source, target],
        'mask',
        'costmap',
        'nearest_preceding_receipt',
        1.0,
    )

    assert gaps == pytest.approx([0.2])


def test_summary_marks_only_header_matched_pair_as_latency_claimable():
    events = [
        _event('image', 1.0, 77),
        _event('detection', 1.2, 77),
        _event('mask', 1.3),
        _event('costmap', 1.4),
    ]

    summary = summarize_events(events, 1.0)
    pairs = summary['pairwise_receipt_associations']

    assert summary['stage_counts']['image'] == 1
    assert summary['stage_counts']['cmd_vel'] == 0
    assert pairs[0]['association_method'] == 'matching_header_stamp'
    assert pairs[0]['latency_claim_allowed']
    assert pairs[0]['receipt_gap_sec']['median'] == pytest.approx(0.2)
    assert not pairs[1]['latency_claim_allowed']


@pytest.mark.parametrize(
    ('method', 'gap'),
    [
        ('unknown', 1.0),
        ('nearest_preceding_receipt', 0.0),
        ('nearest_preceding_receipt', float('nan')),
    ],
)
def test_invalid_association_configuration_fails_visibly(method, gap):
    with pytest.raises(ValueError):
        associate_events([], 'image', 'detection', method, gap)
