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

import pytest

from semantic_planning_experiments.paired_statistics import (
    summarize_paired_differences,
)


def test_paired_statistics_report_interval_effect_and_exact_test():
    result = summarize_paired_differences([1.0, 2.0, 3.0])

    interval = result['confidence_interval_95']
    test = result['two_sided_paired_permutation_test']
    assert result['mean'] == pytest.approx(2.0)
    assert result['sample_stddev'] == pytest.approx(1.0)
    assert result['cohen_dz'] == pytest.approx(2.0)
    assert interval['lower'] == pytest.approx(-0.4841377, abs=1e-6)
    assert interval['upper'] == pytest.approx(4.4841377, abs=1e-6)
    assert interval['degrees_freedom'] == 2
    assert test['method'] == 'exact_sign_flip'
    assert test['permutation_count'] == 8
    assert test['p_value'] == pytest.approx(0.25)


def test_constant_differences_keep_effect_size_undefined():
    result = summarize_paired_differences([2.0, 2.0, 2.0])

    assert result['confidence_interval_95']['lower'] == pytest.approx(2.0)
    assert result['confidence_interval_95']['upper'] == pytest.approx(2.0)
    assert result['cohen_dz'] is None
    assert result['two_sided_paired_permutation_test']['p_value'] == 0.25


def test_zero_differences_have_unit_p_value():
    result = summarize_paired_differences([0.0, 0.0, 0.0])

    assert result['mean'] == 0.0
    assert result['cohen_dz'] is None
    assert result['two_sided_paired_permutation_test']['p_value'] == 1.0


def test_single_pair_has_no_interval_or_standardized_effect():
    result = summarize_paired_differences([2.0])

    assert result['sample_stddev'] is None
    assert result['standard_error'] is None
    assert result['confidence_interval_95'] is None
    assert result['cohen_dz'] is None
    assert result['two_sided_paired_permutation_test']['p_value'] == 1.0


def test_large_sample_monte_carlo_is_reproducible():
    values = [float(index - 8) for index in range(17)]

    first = summarize_paired_differences(values)
    second = summarize_paired_differences(values)
    first_test = first['two_sided_paired_permutation_test']
    second_test = second['two_sided_paired_permutation_test']

    assert first_test == second_test
    assert first_test['method'] == 'monte_carlo_sign_flip'
    assert first_test['permutation_count'] == 100000
    assert first_test['seed'] == 20260801


@pytest.mark.parametrize('values', [[], [math.nan], [math.inf], [True]])
def test_invalid_paired_differences_are_rejected(values):
    with pytest.raises(ValueError, match='paired differences'):
        summarize_paired_differences(values)
