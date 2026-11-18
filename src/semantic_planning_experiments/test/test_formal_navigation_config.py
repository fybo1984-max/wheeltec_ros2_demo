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

"""Regression checks for the forward-only formal navigation contract."""

from pathlib import Path

import yaml


def test_formal_and_controller_free_configs_are_forward_only():
    """Keep physical and controller-free method comparisons kinematically aligned."""
    source_root = Path(__file__).resolve().parents[2]
    formal_path = (
        source_root / 'wheeltec_robot_nav2' / 'param' / 'wheeltec_params'
        / 'param_senior_akm.yaml'
    )
    planner_path = (
        source_root / 'semantic_planning_experiments' / 'config'
        / 'planner_ab.yaml'
    )
    formal = yaml.safe_load(formal_path.read_text(encoding='utf-8'))
    planner = yaml.safe_load(planner_path.read_text(encoding='utf-8'))

    controller = formal['controller_server']['ros__parameters']['FollowPath']
    global_planner = formal['planner_server']['ros__parameters']['GridBased']
    offline_planner = planner['planner_server']['ros__parameters']['GridBased']

    assert controller['vx_min'] == 0.0
    assert controller['enforce_path_inversion'] is False
    assert global_planner['motion_model_for_search'] == 'DUBIN'
    assert offline_planner['motion_model_for_search'] == 'DUBIN'
