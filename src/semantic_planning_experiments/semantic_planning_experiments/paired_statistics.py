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

"""Dependency-free paired statistics for semantic planning experiments."""

import math
import random
import statistics


_EXACT_SIGN_FLIP_LIMIT = 16
_MONTE_CARLO_SAMPLES = 100000
_MONTE_CARLO_SEED = 20260801
_T_CRITICAL_975 = (
    0.0,
    12.7062047364321,
    4.30265272991127,
    3.18244630528426,
    2.7764451051978,
    2.57058183661474,
    2.44691184879168,
    2.3646242510103,
    2.30600413503337,
    2.26215716274099,
    2.22813885196494,
    2.20098516008295,
    2.17881282966342,
    2.16036865646101,
    2.14478668791693,
    2.13144954555932,
    2.11990529922101,
    2.10981557783318,
    2.10092204024096,
    2.09302405440826,
    2.08596344726584,
    2.07961384472766,
    2.07387306790401,
    2.06865761041904,
    2.06389856162802,
    2.05953855275329,
    2.05552943864287,
    2.05183051648028,
    2.04840714179524,
    2.0452296421327,
    2.04227245630124,
)


def _t_critical_975(degrees_freedom: int) -> float:
    if degrees_freedom < len(_T_CRITICAL_975):
        return _T_CRITICAL_975[degrees_freedom]
    z_value = 1.959963984540054
    inverse_df = 1.0 / degrees_freedom
    z2 = z_value * z_value
    first = z_value * (z2 + 1.0) * inverse_df / 4.0
    second = (
        z_value
        * (5.0 * z2 * z2 + 16.0 * z2 + 3.0)
        * inverse_df
        * inverse_df
        / 96.0
    )
    third = (
        z_value
        * (
            3.0 * z2 * z2 * z2
            + 19.0 * z2 * z2
            + 17.0 * z2
            - 15.0
        )
        * inverse_df
        * inverse_df
        * inverse_df
        / 384.0
    )
    return z_value + first + second + third


def _exact_sign_flip(values: list[float]) -> dict:
    observed = abs(sum(values))
    tolerance = 1e-12 * max(1.0, observed)
    total = 1 << len(values)
    extreme = 0
    for pattern in range(total):
        signed_sum = sum(
            value if pattern & (1 << index) else -value
            for index, value in enumerate(values)
        )
        if abs(signed_sum) + tolerance >= observed:
            extreme += 1
    return {
        'method': 'exact_sign_flip',
        'p_value': extreme / total,
        'permutation_count': total,
        'seed': None,
    }


def _monte_carlo_sign_flip(values: list[float]) -> dict:
    observed = abs(sum(values))
    tolerance = 1e-12 * max(1.0, observed)
    generator = random.Random(_MONTE_CARLO_SEED)
    extreme = 0
    for _ in range(_MONTE_CARLO_SAMPLES):
        signed_sum = sum(
            value if generator.getrandbits(1) else -value
            for value in values
        )
        if abs(signed_sum) + tolerance >= observed:
            extreme += 1
    return {
        'method': 'monte_carlo_sign_flip',
        'p_value': (extreme + 1) / (_MONTE_CARLO_SAMPLES + 1),
        'permutation_count': _MONTE_CARLO_SAMPLES,
        'seed': _MONTE_CARLO_SEED,
    }


def summarize_paired_differences(values: list[float]) -> dict:
    """Return descriptive and paired inferential statistics."""
    if not values:
        raise ValueError('paired differences must not be empty')
    if any(isinstance(value, bool) for value in values):
        raise ValueError('paired differences must be numeric')
    try:
        numeric = [float(value) for value in values]
    except (TypeError, ValueError) as error:
        raise ValueError('paired differences must be numeric') from error
    if any(not math.isfinite(value) for value in numeric):
        raise ValueError('paired differences must be finite')

    count = len(numeric)
    mean = statistics.fmean(numeric)
    population_stddev = statistics.pstdev(numeric)
    sample_stddev = statistics.stdev(numeric) if count > 1 else None
    standard_error = (
        sample_stddev / math.sqrt(count)
        if sample_stddev is not None
        else None
    )
    if standard_error is None:
        confidence_interval = None
    else:
        degrees_freedom = count - 1
        half_width = _t_critical_975(degrees_freedom) * standard_error
        confidence_interval = {
            'method': 'student_t',
            'confidence_level': 0.95,
            'degrees_freedom': degrees_freedom,
            'lower': mean - half_width,
            'upper': mean + half_width,
        }
    cohen_dz = (
        mean / sample_stddev
        if sample_stddev is not None and sample_stddev > 0.0
        else None
    )
    sign_flip = (
        _exact_sign_flip(numeric)
        if count <= _EXACT_SIGN_FLIP_LIMIT
        else _monte_carlo_sign_flip(numeric)
    )
    return {
        'count': count,
        'mean': mean,
        'population_stddev': population_stddev,
        'sample_stddev': sample_stddev,
        'standard_error': standard_error,
        'confidence_interval_95': confidence_interval,
        'cohen_dz': cohen_dz,
        'two_sided_paired_permutation_test': sign_flip,
    }
