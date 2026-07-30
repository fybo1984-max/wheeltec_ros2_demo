// Copyright 2026 Tiago Alves
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modified 2026 by WHEELTEC innovation workspace for ROS 2 Humble.

#ifndef SEMANTIC_COSTMAP_PLUGIN__MASK_LAYER_HPP_
#define SEMANTIC_COSTMAP_PLUGIN__MASK_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "semantic_costmap_plugin/fuzzy_inference_engine.hpp"
#include "semantic_costmap_plugin/mask_inflator.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace semantic_costmap_plugin
{

class MaskLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  MaskLayer();
  ~MaskLayer() override;

  void onInitialize() override;
  void matchSize() override;
  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  bool loadMask();
  void computeMaskBounds();
  void applyMask(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j);
  rcl_interfaces::msg::SetParametersResult onParametersChanged(
    const std::vector<rclcpp::Parameter> & parameters);

  nav_msgs::msg::OccupancyGrid mask_;
  std::string map_yaml_path_;
  unsigned char mask_cost_value_;
  bool map_loaded_;
  bool fuzzy_recalculation_needed_;
  int task_urgency_;
  double avoidance_level_;
  double inverse_mask_resolution_;
  bool mask_bounds_cached_;
  int mask_min_i_;
  int mask_min_j_;
  int mask_max_i_;
  int mask_max_j_;
  unsigned int publish_divisor_;
  unsigned int update_count_;
  std::unique_ptr<FuzzyInferenceEngine> fuzzy_engine_;
  std::unique_ptr<MaskInflator> inflator_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr cost_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace semantic_costmap_plugin

#endif  // SEMANTIC_COSTMAP_PLUGIN__MASK_LAYER_HPP_
